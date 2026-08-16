// Copyright 2026 bburda
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "ros2_medkit_gateway/ros2/conversions/fault_msg_conversions.hpp"

#include <builtin_interfaces/msg/time.hpp>

#include "ros2_medkit_gateway/core/http/http_utils.hpp"

namespace ros2_medkit_gateway::ros2::conversions {

namespace {

double to_seconds(const builtin_interfaces::msg::Time & t) {
  return t.sec + static_cast<double>(t.nanosec) / 1e9;
}

}  // namespace

nlohmann::json fault_to_json(const ros2_medkit_msgs::msg::Fault & fault) {
  nlohmann::json j;
  j["fault_code"] = fault.fault_code;
  j["severity"] = fault.severity;
  j["description"] = fault.description;
  j["first_occurred"] = to_seconds(fault.first_occurred);
  j["last_occurred"] = to_seconds(fault.last_occurred);
  // Omitted while zero: the fault never reported a PASSED event.
  if (fault.last_passed.sec != 0 || fault.last_passed.nanosec != 0u) {
    j["last_passed"] = to_seconds(fault.last_passed);
  }
  j["occurrence_count"] = fault.occurrence_count;
  j["status"] = fault.status;
  j["reporting_sources"] = fault.reporting_sources;

  switch (fault.severity) {
    case ros2_medkit_msgs::msg::Fault::SEVERITY_INFO:
      j["severity_label"] = "INFO";
      break;
    case ros2_medkit_msgs::msg::Fault::SEVERITY_WARN:
      j["severity_label"] = "WARN";
      break;
    case ros2_medkit_msgs::msg::Fault::SEVERITY_ERROR:
      j["severity_label"] = "ERROR";
      break;
    case ros2_medkit_msgs::msg::Fault::SEVERITY_CRITICAL:
      j["severity_label"] = "CRITICAL";
      break;
    default:
      j["severity_label"] = "UNKNOWN";
      break;
  }

  return j;
}

nlohmann::json environment_data_to_json(const ros2_medkit_msgs::msg::EnvironmentData & env_data) {
  using nlohmann::json;

  json snapshots = json::array();
  for (const auto & s : env_data.snapshots) {
    json snap;
    snap["type"] = s.type;
    snap["name"] = s.name;

    if (s.type == "freeze_frame") {
      // Preserve the parsed payload (including primary-value extraction hint
      // via "message_type") so the handler can extract the primary value
      // without re-parsing. We pass the raw string + message_type so the
      // handler keeps full control over primary-field extraction.
      snap["snapshot_type"] = "freeze_frame";
      snap["data"] = s.data;
      snap["topic"] = s.topic;
      snap["message_type"] = s.message_type;
      snap["captured_at_ns"] = s.captured_at_ns;
    } else if (s.type == "rosbag") {
      snap["snapshot_type"] = "rosbag";
      // Carries a RECORDING id now, so it is named after what it is. Calling it
      // "fault_code" while it holds something else is how the next reader builds a
      // URL out of the wrong value.
      snap["bulk_data_id"] = s.bulk_data_id;
      snap["size_bytes"] = s.size_bytes;
      snap["duration_sec"] = s.duration_sec;
      snap["format"] = s.format;
      // The fault manager has always set this; only the freeze-frame branch
      // forwarded it, so every rosbag reached the UI with no capture time and
      // rendered "N/A". With one recording per fault that was cosmetic. With
      // several it is the only thing that tells the occurrences apart.
      snap["captured_at_ns"] = s.captured_at_ns;
      // bulk_data_uri intentionally omitted - handler appends per-request URL
    }

    snapshots.push_back(snap);
  }

  json out;
  out["extended_data_records"] = {
      {"first_occurrence", format_timestamp_ns(env_data.extended_data_records.first_occurrence_ns)},
      {"last_occurrence", format_timestamp_ns(env_data.extended_data_records.last_occurrence_ns)}};
  out["snapshots"] = snapshots;
  return out;
}

}  // namespace ros2_medkit_gateway::ros2::conversions
