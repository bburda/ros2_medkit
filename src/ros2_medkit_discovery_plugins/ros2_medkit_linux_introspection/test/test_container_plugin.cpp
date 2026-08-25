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

#include <gtest/gtest.h>
#include "ros2_medkit_linux_introspection/container_utils.hpp"

using namespace ros2_medkit_linux_introspection;

// @verifies REQ_INTEROP_003
TEST(ContainerPlugin, CgroupInfoToJsonAllFields) {
  CgroupInfo info;
  info.container_id = "a1b2c3d4e5f6a1b2c3d4e5f6a1b2c3d4e5f6a1b2c3d4e5f6a1b2c3d4e5f6a1b2";
  info.container_runtime = "docker";
  info.memory_limit = CgroupLimit<uint64_t>::limited(uint64_t{1073741824});
  info.cpu_quota = CgroupLimit<int64_t>::limited(100000);
  info.cpu_period_us = 100000;

  auto j = cgroup_info_to_json(info);
  EXPECT_EQ(j["container_id"], info.container_id);
  EXPECT_EQ(j["runtime"], "docker");
  EXPECT_EQ(j["memory_limit_bytes"], 1073741824u);
  EXPECT_EQ(j["memory_limit_state"], "limited");
  EXPECT_EQ(j["cpu_quota_us"], 100000);
  EXPECT_EQ(j["cpu_quota_state"], "limited");
  EXPECT_EQ(j["cpu_period_us"], 100000);
}

// @verifies REQ_INTEROP_003
TEST(ContainerPlugin, CgroupInfoToJsonMissingOptionals) {
  CgroupInfo info;
  info.container_id = "deadbeef12345678deadbeef12345678deadbeef12345678deadbeef12345678";
  info.container_runtime = "containerd";
  // Leave the limits at their default state

  auto j = cgroup_info_to_json(info);
  EXPECT_EQ(j["container_id"], info.container_id);
  EXPECT_EQ(j["runtime"], "containerd");
  EXPECT_FALSE(j.contains("memory_limit_bytes"));
  EXPECT_FALSE(j.contains("cpu_quota_us"));
  EXPECT_FALSE(j.contains("cpu_period_us"));
  EXPECT_EQ(j["memory_limit_state"], "unavailable");
  EXPECT_EQ(j["cpu_quota_state"], "unavailable");
}

// An unlimited container and one whose limit files could not be read both carry
// no number, and the state is the only thing that tells them apart.
// @verifies REQ_INTEROP_003
TEST(ContainerPlugin, CgroupInfoToJsonUnlimitedIsNotUnreadable) {
  CgroupInfo unlimited;
  unlimited.memory_limit = CgroupLimit<uint64_t>::unlimited();
  unlimited.cpu_quota = CgroupLimit<int64_t>::unlimited();
  unlimited.cpu_period_us = 100000;

  CgroupInfo unreadable;
  unreadable.memory_limit = CgroupLimit<uint64_t>::unreadable();
  unreadable.cpu_quota = CgroupLimit<int64_t>::unreadable();

  auto j_unlimited = cgroup_info_to_json(unlimited);
  auto j_unreadable = cgroup_info_to_json(unreadable);

  EXPECT_FALSE(j_unlimited.contains("memory_limit_bytes"));
  EXPECT_FALSE(j_unreadable.contains("memory_limit_bytes"));
  EXPECT_EQ(j_unlimited["memory_limit_state"], "unlimited");
  EXPECT_EQ(j_unreadable["memory_limit_state"], "unreadable");
  EXPECT_EQ(j_unlimited["cpu_quota_state"], "unlimited");
  EXPECT_EQ(j_unreadable["cpu_quota_state"], "unreadable");
  EXPECT_NE(j_unlimited["memory_limit_state"], j_unreadable["memory_limit_state"]);
}

// @verifies REQ_INTEROP_003
TEST(ContainerPlugin, NotContainerizedSkipped) {
  EXPECT_FALSE(is_containerized(1, "/nonexistent_root"));
}
