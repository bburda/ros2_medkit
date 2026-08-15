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
# Snapshot.msg - Unified snapshot type for freeze frames and rosbag recordings.
#
# Used in EnvironmentData to represent captured diagnostic data. Supports two types:
# - "freeze_frame": Single message capture with JSON-serialized data
# - "rosbag": Recording file metadata with bulk-data download reference

# Snapshot type: "freeze_frame" or "rosbag"
string type

# Human-readable name for this snapshot
string name

# --- Freeze frame fields (empty for rosbag type) ---

# JSON-serialized message data
string data

# ROS topic name (e.g., "/motor/temperature")
string topic

# Message type (e.g., "sensor_msgs/msg/Temperature")
string message_type

# --- Rosbag fields (empty for freeze_frame type) ---

# Identifier for bulk-data download. For a rosbag this is the RECORDING id (the
# bag directory basename), not the fault code: one fault can hold several
# recordings and each needs its own address.
string bulk_data_id

# File size in bytes
uint64 size_bytes

# Recording duration in seconds
float64 duration_sec

# Recording format: "sqlite3" or "mcap"
string format

# --- Common metadata ---

# Capture timestamp (nanoseconds since epoch)
int64 captured_at_ns

# Snapshot type constants
string TYPE_FREEZE_FRAME = "freeze_frame"
string TYPE_ROSBAG = "rosbag"

# Rosbag format constants
string FORMAT_SQLITE3 = "sqlite3"
string FORMAT_MCAP = "mcap"
