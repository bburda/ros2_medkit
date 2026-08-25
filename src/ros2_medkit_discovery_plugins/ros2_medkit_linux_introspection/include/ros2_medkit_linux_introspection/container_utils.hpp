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

#pragma once

#include "ros2_medkit_linux_introspection/cgroup_reader.hpp"

#include <nlohmann/json.hpp>

namespace ros2_medkit_linux_introspection {

/// Convert CgroupInfo to JSON for the container plugin HTTP response.
///
/// The numeric fields (memory_limit_bytes, cpu_quota_us, cpu_period_us) are
/// present only when a value was read, so a client that reads no limit still
/// has to know why. That is what memory_limit_state and cpu_quota_state carry:
/// "limited", "unlimited", "unreadable" or "unavailable", always present.
///
/// The CPU limit is the quota and its period together, so cpu_quota_state
/// covers both and cpu_period_us has no state of its own.
inline nlohmann::json cgroup_info_to_json(const CgroupInfo & info) {
  nlohmann::json j;
  j["container_id"] = info.container_id;
  j["runtime"] = info.container_runtime;

  j["memory_limit_state"] = limit_state_to_string(info.memory_limit.state());
  if (info.memory_limit.value()) {
    j["memory_limit_bytes"] = *info.memory_limit.value();
  }

  j["cpu_quota_state"] = limit_state_to_string(info.cpu_quota.state());
  if (info.cpu_quota.value()) {
    j["cpu_quota_us"] = *info.cpu_quota.value();
  }
  if (info.cpu_period_us) {
    j["cpu_period_us"] = *info.cpu_period_us;
  }
  return j;
}

}  // namespace ros2_medkit_linux_introspection
