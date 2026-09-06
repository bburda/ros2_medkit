# Copyright 2026 bburda
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

# PlannedStop.msg - a window of wall-clock time during which faults are expected.
#
# An operator declares the window; the fault manager stores it and serves it.
# Nothing in the fault pipeline consults a window: a fault whose cycle starts
# inside one is confirmed, healed, cleared, captured, published and audited
# exactly as any other fault. The window only lets a reader tell the two apart.
#
# Field names are starts_at / ends_at rather than from / to because a message
# field named "from" is a Python keyword and rosidl cannot generate a binding
# for it. The REST representation of this same window uses "from" and "to".

# Server-assigned identifier, unique within one fault manager
string id

# Start of the window (wall clock, UTC). A fault whose first_occurred lies in
# [starts_at, ends_at] is expected.
builtin_interfaces/Time starts_at

# End of the window (wall clock, UTC). Always strictly greater than starts_at.
builtin_interfaces/Time ends_at

# Why the plant is stopping: a changeover, a maintenance weekend, a line move
string reason

# Who declared it: the authenticated client id, or "anonymous" when the
# declaration arrived without authentication
string declared_by

# When the declaration was recorded (wall clock, UTC). Retention orders by this
# field, not by starts_at, because a window declared after the fact describes a
# stop that already happened.
builtin_interfaces/Time declared_at

# True when an operator cut the window short, which moved ends_at to the moment
# of that request. False for a window that ran or is running to its declared end.
bool ended_early

# True on the copy returned when a window was CANCELLED: stopped before it ever
# started, and therefore removed rather than shortened. Never true on a stored
# window - a cancelled one is gone - so a listing never carries it.
bool cancelled
