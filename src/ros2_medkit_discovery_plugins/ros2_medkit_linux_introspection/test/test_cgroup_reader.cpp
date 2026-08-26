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

#include <filesystem>
#include <fstream>
#include <gtest/gtest.h>
#include <string>
#include <sys/stat.h>
#include <sys/types.h>
#include "ros2_medkit_linux_introspection/cgroup_reader.hpp"

namespace fs = std::filesystem;
using namespace ros2_medkit_linux_introspection;

namespace {

constexpr const char * kDockerId = "aabb112233445566aabb112233445566aabb112233445566aabb112233445566";

// cgroup v1 spells "no memory limit" as a sentinel near the top of the value
// range rather than a keyword; the reader treats anything from 2^62 up as no
// limit, so both sides of that boundary are pinned here.
constexpr uint64_t kV1UnlimitedSentinel = 9223372036854771712ULL;
constexpr uint64_t kV1UnlimitedBoundary = 4611686018427387904ULL;  // 2^62

// Builds a synthetic /proc and /sys tree so a layout can be exercised without
// the host having to be in it.
class CgroupTree : public ::testing::Test {
 protected:
  void SetUp() override {
    const auto * info = ::testing::UnitTest::GetInstance()->current_test_info();
    root_ = fs::temp_directory_path() /
            ("medkit_cgroup_" + std::string(info->test_suite_name()) + "_" + std::string(info->name()));
    fs::remove_all(root_);
    fs::create_directories(root_);
  }

  void TearDown() override {
    fs::remove_all(root_);
  }

  void write_proc_cgroup(pid_t pid, const std::string & content) {
    auto dir = root_ / "proc" / std::to_string(pid);
    fs::create_directories(dir);
    std::ofstream f(dir / "cgroup");
    f << content;
  }

  void write_file(const std::string & relative, const std::string & content) {
    auto path = root_ / relative;
    fs::create_directories(path.parent_path());
    std::ofstream f(path);
    f << content;
  }

  std::string root() const {
    return root_.string();
  }

  CgroupInfo read(pid_t pid = 42) {
    auto result = read_cgroup_info(pid, root());
    EXPECT_TRUE(result.has_value()) << (result ? "" : result.error());
    return result ? *result : CgroupInfo{};
  }

  fs::path root_;
};

std::string docker_v2_line() {
  return std::string("0::/docker/") + kDockerId + "\n";
}

}  // namespace

// @verifies REQ_INTEROP_003
TEST(CgroupReader, ExtractDockerContainerId) {
  std::string path =
      "/system.slice/"
      "docker-a1b2c3d4e5f6a1b2c3d4e5f6a1b2c3d4e5f6a1b2c3d4e5f6a1b2c3d4e5f6a1b2.scope";
  auto id = extract_container_id(path);
  EXPECT_EQ(id, "a1b2c3d4e5f6a1b2c3d4e5f6a1b2c3d4e5f6a1b2c3d4e5f6a1b2c3d4e5f6a1b2");
}

// @verifies REQ_INTEROP_003
TEST(CgroupReader, ExtractPodmanContainerId) {
  std::string path =
      "/user.slice/user-1000.slice/user@1000.service/"
      "libpod-aabbccddee112233aabbccddee112233aabbccddee112233aabbccddee112233.scope";
  auto id = extract_container_id(path);
  EXPECT_EQ(id, "aabbccddee112233aabbccddee112233aabbccddee112233aabbccddee112233");
}

// @verifies REQ_INTEROP_003
TEST(CgroupReader, ExtractContainerdContainerId) {
  std::string path =
      "/system.slice/containerd.service/"
      "cri-containerd-deadbeef12345678deadbeef12345678deadbeef12345678deadbeef12345678.scope";
  auto id = extract_container_id(path);
  EXPECT_EQ(id, "deadbeef12345678deadbeef12345678deadbeef12345678deadbeef12345678");
}

// @verifies REQ_INTEROP_003
TEST(CgroupReader, ExtractNoContainerId) {
  EXPECT_TRUE(extract_container_id("/user.slice/user-1000.slice/session-1.scope").empty());
  EXPECT_TRUE(extract_container_id("/").empty());
}

// @verifies REQ_INTEROP_003
TEST(CgroupReader, ExtractDockerOldStyleContainerId) {
  std::string path = "/docker/a1b2c3d4e5f6a1b2c3d4e5f6a1b2c3d4e5f6a1b2c3d4e5f6a1b2c3d4e5f6a1b2";
  auto id = extract_container_id(path);
  EXPECT_EQ(id, "a1b2c3d4e5f6a1b2c3d4e5f6a1b2c3d4e5f6a1b2c3d4e5f6a1b2c3d4e5f6a1b2");
}

// @verifies REQ_INTEROP_003
TEST(CgroupReader, DetectRuntime) {
  EXPECT_EQ(detect_runtime("/system.slice/docker-abc123.scope"), "docker");
  EXPECT_EQ(detect_runtime("/user.slice/libpod-abc123.scope"), "podman");
  EXPECT_EQ(detect_runtime("/system.slice/cri-containerd-abc123.scope"), "containerd");
  EXPECT_TRUE(detect_runtime("/user.slice/session-1.scope").empty());
}

// @verifies REQ_INTEROP_003
TEST(CgroupReader, LimitStateStrings) {
  EXPECT_EQ(limit_state_to_string(LimitState::kLimited), "limited");
  EXPECT_EQ(limit_state_to_string(LimitState::kUnlimited), "unlimited");
  EXPECT_EQ(limit_state_to_string(LimitState::kUnreadable), "unreadable");
  EXPECT_EQ(limit_state_to_string(LimitState::kUnavailable), "unavailable");
}

// @verifies REQ_INTEROP_003
TEST(CgroupReader, CgroupLimitHoldsValueOnlyWhenLimited) {
  EXPECT_EQ(CgroupLimit<uint64_t>().state(), LimitState::kUnavailable);
  EXPECT_FALSE(CgroupLimit<uint64_t>().value().has_value());
  EXPECT_FALSE(CgroupLimit<uint64_t>::unlimited().value().has_value());
  EXPECT_FALSE(CgroupLimit<uint64_t>::unreadable().value().has_value());
  EXPECT_FALSE(CgroupLimit<uint64_t>::unavailable().value().has_value());

  auto limited = CgroupLimit<uint64_t>::limited(uint64_t{4096});
  EXPECT_EQ(limited.state(), LimitState::kLimited);
  ASSERT_TRUE(limited.value().has_value());
  EXPECT_EQ(*limited.value(), 4096u);
}

// --- Layout sweep: cgroup v2 -------------------------------------------------

// cgroupns=host reports the path of the container's cgroup inside the whole
// hierarchy, and the limit files sit under the mount point joined with it.
// @verifies REQ_INTEROP_003
TEST_F(CgroupTree, V2HostNamespaceJoinedPath) {
  write_proc_cgroup(42, docker_v2_line());
  write_file(std::string("sys/fs/cgroup/docker/") + kDockerId + "/memory.max", "536870912\n");
  write_file(std::string("sys/fs/cgroup/docker/") + kDockerId + "/cpu.max", "50000 100000\n");

  // The bare mount point holds nothing, exactly as on a host where the root
  // cgroup carries no limit files.
  ASSERT_FALSE(fs::exists(root_ / "sys/fs/cgroup/memory.max"));

  auto info = read();
  EXPECT_EQ(info.container_id, kDockerId);
  EXPECT_EQ(info.container_runtime, "docker");
  EXPECT_EQ(info.memory_limit.state(), LimitState::kLimited);
  ASSERT_TRUE(info.memory_limit.value().has_value());
  EXPECT_EQ(*info.memory_limit.value(), 536870912u);
  EXPECT_EQ(info.cpu_quota.state(), LimitState::kLimited);
  ASSERT_TRUE(info.cpu_quota.value().has_value());
  EXPECT_EQ(*info.cpu_quota.value(), 50000);
  ASSERT_TRUE(info.cpu_period_us.has_value());
  EXPECT_EQ(*info.cpu_period_us, 100000);
}

// cgroupns=private mounts the container's own cgroup at the mount point and
// reports "/" for it.
// @verifies REQ_INTEROP_003
TEST_F(CgroupTree, V2PrivateNamespaceRootPath) {
  write_proc_cgroup(42, "0::/\n");
  write_file("sys/fs/cgroup/memory.max", "536870912\n");
  write_file("sys/fs/cgroup/cpu.max", "50000 100000\n");

  auto info = read();
  EXPECT_EQ(info.cgroup_path, "/");
  EXPECT_EQ(info.memory_limit.state(), LimitState::kLimited);
  EXPECT_EQ(*info.memory_limit.value(), 536870912u);
  EXPECT_EQ(*info.cpu_quota.value(), 50000);
  EXPECT_EQ(*info.cpu_period_us, 100000);
}

// The case the joined-path-only reader missed: a private namespace whose
// reported path is not "/", with the limit files at the bare mount point.
// @verifies REQ_INTEROP_003
TEST_F(CgroupTree, V2PrivateNamespaceNonRootPath) {
  write_proc_cgroup(42, "0::/system.slice/ros_node.service\n");
  write_file("sys/fs/cgroup/memory.max", "268435456\n");
  write_file("sys/fs/cgroup/cpu.max", "25000 100000\n");

  ASSERT_FALSE(fs::exists(root_ / "sys/fs/cgroup/system.slice/ros_node.service"));

  auto info = read();
  EXPECT_EQ(info.cgroup_path, "/system.slice/ros_node.service");
  EXPECT_EQ(info.memory_limit.state(), LimitState::kLimited);
  ASSERT_TRUE(info.memory_limit.value().has_value());
  EXPECT_EQ(*info.memory_limit.value(), 268435456u);
  EXPECT_EQ(info.cpu_quota.state(), LimitState::kLimited);
  EXPECT_EQ(*info.cpu_quota.value(), 25000);
}

// When the reported path does resolve under the mount point, that cgroup is the
// one the process is in and its limits win over the mount point's.
// @verifies REQ_INTEROP_003
TEST_F(CgroupTree, V2PrivateNamespaceNestedPathPrefersOwnCgroup) {
  write_proc_cgroup(42, "0::/system.slice/ros_node.service\n");
  write_file("sys/fs/cgroup/memory.max", "536870912\n");
  write_file("sys/fs/cgroup/system.slice/ros_node.service/memory.max", "268435456\n");

  auto info = read();
  ASSERT_TRUE(info.memory_limit.value().has_value());
  EXPECT_EQ(*info.memory_limit.value(), 268435456u);
}

// --- Containerization when the cgroup path hides the id ----------------------

// cgroupns=private reports "/" for the cgroup, so the path carries no container
// id at all. The limits are still there and the process is still in a
// container - the runtime marker is what says so.
// @verifies REQ_INTEROP_003
TEST_F(CgroupTree, PrivateNamespaceDetectedByDockerMarker) {
  write_proc_cgroup(42, "0::/\n");
  write_file("proc/42/root/.dockerenv", "");
  write_file("sys/fs/cgroup/memory.max", "536870912\n");
  write_file("sys/fs/cgroup/cpu.max", "100000 100000\n");

  EXPECT_TRUE(is_containerized(42, root()));

  auto info = read();
  EXPECT_TRUE(info.containerized);
  EXPECT_TRUE(info.container_id.empty());
  EXPECT_EQ(info.container_runtime, "docker");
  EXPECT_EQ(info.memory_limit.state(), LimitState::kLimited);
  ASSERT_TRUE(info.memory_limit.value().has_value());
  EXPECT_EQ(*info.memory_limit.value(), 536870912u);
  EXPECT_EQ(info.cpu_quota.state(), LimitState::kLimited);
  EXPECT_EQ(*info.cpu_quota.value(), 100000);
  EXPECT_EQ(*info.cpu_period_us, 100000);
}

// @verifies REQ_INTEROP_003
TEST_F(CgroupTree, PrivateNamespaceDetectedByPodmanMarker) {
  write_proc_cgroup(42, "0::/\n");
  write_file("proc/42/root/run/.containerenv", "");

  EXPECT_TRUE(is_containerized(42, root()));
  auto info = read();
  EXPECT_TRUE(info.containerized);
  EXPECT_EQ(info.container_runtime, "podman");
}

// With no marker file, an overlay mounted as the process's root still says the
// process is in a container.
// @verifies REQ_INTEROP_003
TEST_F(CgroupTree, PrivateNamespaceDetectedByOverlayRoot) {
  write_proc_cgroup(42, "0::/\n");
  write_file("proc/42/mountinfo",
             "1700 461 0:154 / / rw,relatime - overlay overlay rw,lowerdir=/a,upperdir=/b\n"
             "1706 1705 0:25 / /sys/fs/cgroup ro,nosuid,nodev,noexec,relatime - cgroup2 cgroup rw\n");

  EXPECT_TRUE(is_containerized(42, root()));
  auto info = read();
  EXPECT_TRUE(info.containerized);
  // The overlay says "a container" but names no runtime.
  EXPECT_TRUE(info.container_runtime.empty());
}

// A host root filesystem is not an overlay, and no marker file is there.
// @verifies REQ_INTEROP_003
TEST_F(CgroupTree, HostProcessIsNotContainerized) {
  write_proc_cgroup(42, "0::/user.slice/user-1000.slice/session-1.scope\n");
  write_file("proc/42/mountinfo",
             "25 1 259:2 / / rw,relatime - ext4 /dev/nvme0n1p2 rw\n"
             "26 25 0:25 / /sys/fs/cgroup rw,nosuid,nodev,noexec,relatime - cgroup2 cgroup rw\n");

  EXPECT_FALSE(is_containerized(42, root()));
  auto info = read();
  EXPECT_FALSE(info.containerized);
  EXPECT_TRUE(info.container_id.empty());
}

// An overlay mounted somewhere other than the root says nothing about the
// process being in a container.
// @verifies REQ_INTEROP_003
TEST_F(CgroupTree, OverlayBelowRootIsNotAContainer) {
  write_proc_cgroup(42, "0::/user.slice/user-1000.slice/session-1.scope\n");
  write_file("proc/42/mountinfo",
             "25 1 259:2 / / rw,relatime - ext4 /dev/nvme0n1p2 rw\n"
             "77 25 0:99 / /var/lib/docker/overlay2/x rw,relatime - overlay overlay rw\n");

  EXPECT_FALSE(is_containerized(42, root()));
}

// The marker belongs to the inspected process, not to whoever is reading. A
// gateway that is itself in a container must not report every process it can
// see as containerized.
// @verifies REQ_INTEROP_003
TEST_F(CgroupTree, MarkerAtTheReadersRootDoesNotContainerizeAHostProcess) {
  write_proc_cgroup(42, "0::/user.slice/user-1000.slice/session-1.scope\n");
  write_file(".dockerenv", "");
  write_file("run/.containerenv", "");
  write_file("proc/42/mountinfo", "25 1 259:2 / / rw,relatime - ext4 /dev/nvme0n1p2 rw\n");

  EXPECT_FALSE(is_containerized(42, root()));
  auto info = read();
  EXPECT_FALSE(info.containerized);
  EXPECT_TRUE(info.container_runtime.empty());
}

// An overlay root is how whole distributions boot, so on its own it says
// nothing. Only a cgroup path rewritten to the namespace root makes it a
// container signal.
// @verifies REQ_INTEROP_003
TEST_F(CgroupTree, OverlayRootWithAHostCgroupPathIsNotAContainer) {
  write_proc_cgroup(42, "0::/user.slice/user-1000.slice/session-1.scope\n");
  write_file("proc/42/mountinfo", "1700 461 0:154 / / rw,relatime - overlay overlay rw\n");

  EXPECT_FALSE(is_containerized(42, root()));
  EXPECT_FALSE(read().containerized);
}

// When mounts are stacked at "/", the visible root is the last one listed.
// @verifies REQ_INTEROP_003
TEST_F(CgroupTree, StackedOverlayRootIsStillDetected) {
  write_proc_cgroup(42, "0::/\n");
  write_file("proc/42/mountinfo",
             "25 1 259:2 / / rw,relatime - ext4 /dev/nvme0n1p2 rw\n"
             "99 25 0:154 / / rw,relatime - overlay overlay rw\n");

  EXPECT_TRUE(is_containerized(42, root()));
  EXPECT_TRUE(read().containerized);
}

// A container id in the path stays the answer, and keeps naming the runtime.
// @verifies REQ_INTEROP_003
TEST_F(CgroupTree, CgroupPathIdWinsOverMarker) {
  write_proc_cgroup(42, docker_v2_line());
  write_file("proc/42/root/run/.containerenv", "");

  auto info = read();
  EXPECT_TRUE(info.containerized);
  EXPECT_EQ(info.container_id, kDockerId);
  EXPECT_EQ(info.container_runtime, "docker");
}

// --- Layout sweep: cgroup v1 -------------------------------------------------

// @verifies REQ_INTEROP_003
TEST_F(CgroupTree, V1HostNamespaceJoinedPath) {
  write_proc_cgroup(42, std::string("12:memory:/docker/") + kDockerId + "\n" + "11:cpu,cpuacct:/docker/" + kDockerId +
                            "\n" + "10:pids:/docker/" + kDockerId + "\n");
  write_file(std::string("sys/fs/cgroup/memory/docker/") + kDockerId + "/memory.limit_in_bytes", "536870912\n");
  write_file(std::string("sys/fs/cgroup/cpu,cpuacct/docker/") + kDockerId + "/cpu.cfs_quota_us", "50000\n");
  write_file(std::string("sys/fs/cgroup/cpu,cpuacct/docker/") + kDockerId + "/cpu.cfs_period_us", "100000\n");

  auto info = read();
  EXPECT_EQ(info.container_id, kDockerId);
  EXPECT_EQ(info.container_runtime, "docker");
  EXPECT_EQ(info.memory_limit.state(), LimitState::kLimited);
  ASSERT_TRUE(info.memory_limit.value().has_value());
  EXPECT_EQ(*info.memory_limit.value(), 536870912u);
  EXPECT_EQ(info.cpu_quota.state(), LimitState::kLimited);
  ASSERT_TRUE(info.cpu_quota.value().has_value());
  EXPECT_EQ(*info.cpu_quota.value(), 50000);
  ASSERT_TRUE(info.cpu_period_us.has_value());
  EXPECT_EQ(*info.cpu_period_us, 100000);
}

// Docker on cgroup v1 bind-mounts the container's own cgroup directory over
// /sys/fs/cgroup/<controller>, so inside the container the reported path leads
// nowhere and the files are at the mount point itself.
// @verifies REQ_INTEROP_003
TEST_F(CgroupTree, V1InsideContainerBareMountPoint) {
  write_proc_cgroup(42, std::string("12:memory:/docker/") + kDockerId + "\n" + "11:cpu,cpuacct:/docker/" + kDockerId +
                            "\n");
  write_file("sys/fs/cgroup/memory/memory.limit_in_bytes", "536870912\n");
  write_file("sys/fs/cgroup/cpu,cpuacct/cpu.cfs_quota_us", "50000\n");
  write_file("sys/fs/cgroup/cpu,cpuacct/cpu.cfs_period_us", "100000\n");

  auto info = read();
  EXPECT_EQ(info.container_id, kDockerId);
  ASSERT_TRUE(info.memory_limit.value().has_value());
  EXPECT_EQ(*info.memory_limit.value(), 536870912u);
  ASSERT_TRUE(info.cpu_quota.value().has_value());
  EXPECT_EQ(*info.cpu_quota.value(), 50000);
}

// The root of a v1 hierarchy always reads as unlimited. When the reported path
// leads to a cgroup that does carry a limit, that limit is the answer - a
// fallback location must not be able to report a limited container as free.
// @verifies REQ_INTEROP_003
TEST_F(CgroupTree, V1RootFilesDoNotMaskContainerLimit) {
  write_proc_cgroup(42, std::string("12:memory:/docker/") + kDockerId + "\n" + "11:cpu,cpuacct:/docker/" + kDockerId +
                            "\n");
  write_file("sys/fs/cgroup/memory/memory.limit_in_bytes", std::to_string(kV1UnlimitedSentinel) + "\n");
  write_file("sys/fs/cgroup/cpu,cpuacct/cpu.cfs_quota_us", "-1\n");
  write_file("sys/fs/cgroup/cpu,cpuacct/cpu.cfs_period_us", "100000\n");
  write_file(std::string("sys/fs/cgroup/memory/docker/") + kDockerId + "/memory.limit_in_bytes", "536870912\n");
  write_file(std::string("sys/fs/cgroup/cpu,cpuacct/docker/") + kDockerId + "/cpu.cfs_quota_us", "50000\n");
  write_file(std::string("sys/fs/cgroup/cpu,cpuacct/docker/") + kDockerId + "/cpu.cfs_period_us", "100000\n");

  auto info = read();
  EXPECT_EQ(info.memory_limit.state(), LimitState::kLimited);
  ASSERT_TRUE(info.memory_limit.value().has_value());
  EXPECT_EQ(*info.memory_limit.value(), 536870912u);
  EXPECT_EQ(info.cpu_quota.state(), LimitState::kLimited);
  ASSERT_TRUE(info.cpu_quota.value().has_value());
  EXPECT_EQ(*info.cpu_quota.value(), 50000);
}

// The plain "cpu" mount point is a symlink to the co-mounted directory on most
// distributions, and only one of the two names is guaranteed to be there.
// @verifies REQ_INTEROP_003
TEST_F(CgroupTree, V1CpuControllerMountedWithoutCpuacct) {
  write_proc_cgroup(42, "12:memory:/\n11:cpu:/\n");
  write_file("sys/fs/cgroup/cpu/cpu.cfs_quota_us", "50000\n");
  write_file("sys/fs/cgroup/cpu/cpu.cfs_period_us", "100000\n");

  auto info = read();
  ASSERT_TRUE(info.cpu_quota.value().has_value());
  EXPECT_EQ(*info.cpu_quota.value(), 50000);
  EXPECT_EQ(*info.cpu_period_us, 100000);
}

// @verifies REQ_INTEROP_003
TEST_F(CgroupTree, V1IsContainerized) {
  write_proc_cgroup(42, std::string("12:memory:/docker/") + kDockerId + "\n" + "11:cpu,cpuacct:/docker/" + kDockerId +
                            "\n");
  EXPECT_TRUE(is_containerized(42, root()));

  write_proc_cgroup(43, "12:memory:/user.slice/user-1000.slice/session-1.scope\n");
  EXPECT_FALSE(is_containerized(43, root()));
}

// --- Layout sweep: hybrid ----------------------------------------------------

// A hybrid host mounts both hierarchies. The unified line is the bare root
// there, so the container is named only on the v1 hierarchies.
// @verifies REQ_INTEROP_003
TEST_F(CgroupTree, HybridUnifiedRootFallsBackToV1Path) {
  write_proc_cgroup(42, std::string("0::/\n") + "12:memory:/docker/" + kDockerId + "\n" + "11:cpu,cpuacct:/docker/" +
                            kDockerId + "\n");
  write_file(std::string("sys/fs/cgroup/memory/docker/") + kDockerId + "/memory.limit_in_bytes", "536870912\n");
  write_file(std::string("sys/fs/cgroup/cpu,cpuacct/docker/") + kDockerId + "/cpu.cfs_quota_us", "50000\n");
  write_file(std::string("sys/fs/cgroup/cpu,cpuacct/docker/") + kDockerId + "/cpu.cfs_period_us", "100000\n");

  auto info = read();
  EXPECT_EQ(info.cgroup_path, std::string("/docker/") + kDockerId);
  EXPECT_EQ(info.container_id, kDockerId);
  ASSERT_TRUE(info.memory_limit.value().has_value());
  EXPECT_EQ(*info.memory_limit.value(), 536870912u);
  ASSERT_TRUE(info.cpu_quota.value().has_value());
  EXPECT_EQ(*info.cpu_quota.value(), 50000);
}

// The hybrid layout keeps the legacy controllers at the top level and mounts
// the unified hierarchy beside them.
// @verifies REQ_INTEROP_003
TEST_F(CgroupTree, HybridUnifiedMountedBesideLegacyControllers) {
  write_proc_cgroup(42, docker_v2_line() + "12:memory:/docker/" + kDockerId + "\n");
  write_file(std::string("sys/fs/cgroup/unified/docker/") + kDockerId + "/memory.max", "268435456\n");
  write_file(std::string("sys/fs/cgroup/unified/docker/") + kDockerId + "/cpu.max", "25000 100000\n");

  auto info = read();
  EXPECT_EQ(info.container_id, kDockerId);
  ASSERT_TRUE(info.memory_limit.value().has_value());
  EXPECT_EQ(*info.memory_limit.value(), 268435456u);
  ASSERT_TRUE(info.cpu_quota.value().has_value());
  EXPECT_EQ(*info.cpu_quota.value(), 25000);
  EXPECT_EQ(*info.cpu_period_us, 100000);
}

// --- Unlimited vs unreadable vs absent ---------------------------------------

// @verifies REQ_INTEROP_003
TEST_F(CgroupTree, V2UnlimitedIsNotAbsent) {
  write_proc_cgroup(42, "0::/\n");
  write_file("sys/fs/cgroup/memory.max", "max\n");
  write_file("sys/fs/cgroup/cpu.max", "max 100000\n");

  auto info = read();
  EXPECT_EQ(info.memory_limit.state(), LimitState::kUnlimited);
  EXPECT_FALSE(info.memory_limit.value().has_value());
  EXPECT_EQ(info.cpu_quota.state(), LimitState::kUnlimited);
  EXPECT_FALSE(info.cpu_quota.value().has_value());
  // The period is still a fact about the cgroup even with no quota on it.
  ASSERT_TRUE(info.cpu_period_us.has_value());
  EXPECT_EQ(*info.cpu_period_us, 100000);
}

// @verifies REQ_INTEROP_003
TEST_F(CgroupTree, V2UnreadableIsNotUnlimited) {
  write_proc_cgroup(42, "0::/\n");
  write_file("sys/fs/cgroup/memory.max", "not-a-number\n");
  write_file("sys/fs/cgroup/cpu.max", "not-a-number either\n");

  auto info = read();
  EXPECT_EQ(info.memory_limit.state(), LimitState::kUnreadable);
  EXPECT_FALSE(info.memory_limit.value().has_value());
  EXPECT_EQ(info.cpu_quota.state(), LimitState::kUnreadable);
  EXPECT_FALSE(info.cpu_period_us.has_value());
}

// @verifies REQ_INTEROP_003
TEST_F(CgroupTree, AbsentLimitFilesAreUnavailable) {
  write_proc_cgroup(42, "0::/user.slice/user-1000.slice/session-1.scope\n");

  auto info = read();
  EXPECT_TRUE(info.container_id.empty());
  EXPECT_TRUE(info.container_runtime.empty());
  EXPECT_EQ(info.memory_limit.state(), LimitState::kUnavailable);
  EXPECT_EQ(info.cpu_quota.state(), LimitState::kUnavailable);
  EXPECT_FALSE(info.cpu_period_us.has_value());
}

// The three outcomes have to stay distinguishable from one another, which is
// the whole reason the state is reported separately from the value.
// @verifies REQ_INTEROP_003
TEST_F(CgroupTree, UnlimitedUnreadableAndAbsentAreThreeResults) {
  write_proc_cgroup(42, "0::/\n");
  write_file("sys/fs/cgroup/memory.max", "max\n");
  auto unlimited = read().memory_limit.state();

  write_file("sys/fs/cgroup/memory.max", "garbage\n");
  auto unreadable = read().memory_limit.state();

  fs::remove(root_ / "sys/fs/cgroup/memory.max");
  auto absent = read().memory_limit.state();

  EXPECT_EQ(unlimited, LimitState::kUnlimited);
  EXPECT_EQ(unreadable, LimitState::kUnreadable);
  EXPECT_EQ(absent, LimitState::kUnavailable);
  EXPECT_NE(unlimited, unreadable);
  EXPECT_NE(unreadable, absent);
  EXPECT_NE(unlimited, absent);
}

// --- Malformed contents ------------------------------------------------------

// @verifies REQ_INTEROP_003
TEST_F(CgroupTree, EmptyLimitFilesAreUnreadable) {
  write_proc_cgroup(42, "0::/\n");
  write_file("sys/fs/cgroup/memory.max", "");
  write_file("sys/fs/cgroup/cpu.max", "");

  auto info = read();
  EXPECT_EQ(info.memory_limit.state(), LimitState::kUnreadable);
  EXPECT_EQ(info.cpu_quota.state(), LimitState::kUnreadable);
}

// A quota with no period is not a bandwidth limit: the period is what the next
// caller would divide by.
// @verifies REQ_INTEROP_003
TEST_F(CgroupTree, CpuMaxWithoutPeriodIsUnreadable) {
  write_proc_cgroup(42, "0::/\n");
  write_file("sys/fs/cgroup/cpu.max", "50000\n");

  auto info = read();
  EXPECT_EQ(info.cpu_quota.state(), LimitState::kUnreadable);
  EXPECT_FALSE(info.cpu_quota.value().has_value());
  EXPECT_FALSE(info.cpu_period_us.has_value());
}

// @verifies REQ_INTEROP_003
TEST_F(CgroupTree, CpuMaxWithZeroPeriodIsUnreadable) {
  write_proc_cgroup(42, "0::/\n");
  write_file("sys/fs/cgroup/cpu.max", "50000 0\n");

  auto info = read();
  EXPECT_EQ(info.cpu_quota.state(), LimitState::kUnreadable);
  EXPECT_FALSE(info.cpu_period_us.has_value());
}

// @verifies REQ_INTEROP_003
TEST_F(CgroupTree, V1ZeroPeriodIsUnreadable) {
  write_proc_cgroup(42, "11:cpu,cpuacct:/\n");
  write_file("sys/fs/cgroup/cpu,cpuacct/cpu.cfs_quota_us", "50000\n");
  write_file("sys/fs/cgroup/cpu,cpuacct/cpu.cfs_period_us", "0\n");

  auto info = read();
  EXPECT_EQ(info.cpu_quota.state(), LimitState::kUnreadable);
  EXPECT_FALSE(info.cpu_period_us.has_value());
}

// @verifies REQ_INTEROP_003
TEST_F(CgroupTree, V1QuotaWithoutPeriodFileIsUnreadable) {
  write_proc_cgroup(42, "11:cpu,cpuacct:/\n");
  write_file("sys/fs/cgroup/cpu,cpuacct/cpu.cfs_quota_us", "50000\n");

  auto info = read();
  EXPECT_EQ(info.cpu_quota.state(), LimitState::kUnreadable);
}

// @verifies REQ_INTEROP_003
TEST_F(CgroupTree, ValueBeyondTypeRangeIsUnreadable) {
  write_proc_cgroup(42, "0::/\n");
  write_file("sys/fs/cgroup/memory.max", "999999999999999999999999999999\n");
  write_file("sys/fs/cgroup/cpu.max", "999999999999999999999999999999 100000\n");

  auto info = read();
  EXPECT_EQ(info.memory_limit.state(), LimitState::kUnreadable);
  EXPECT_EQ(info.cpu_quota.state(), LimitState::kUnreadable);
}

// A negative byte count is an error value, not a very large limit.
// @verifies REQ_INTEROP_003
TEST_F(CgroupTree, NegativeMemoryLimitIsUnreadable) {
  write_proc_cgroup(42, "0::/\n");
  write_file("sys/fs/cgroup/memory.max", "-1\n");

  auto info = read();
  EXPECT_EQ(info.memory_limit.state(), LimitState::kUnreadable);
  EXPECT_FALSE(info.memory_limit.value().has_value());
}

// @verifies REQ_INTEROP_003
TEST_F(CgroupTree, TrailingGarbageAfterNumberIsUnreadable) {
  write_proc_cgroup(42, "0::/\n");
  write_file("sys/fs/cgroup/memory.max", "536870912bytes\n");

  auto info = read();
  EXPECT_EQ(info.memory_limit.state(), LimitState::kUnreadable);
}

// @verifies REQ_INTEROP_003
TEST_F(CgroupTree, V1UnlimitedSentinelBoundary) {
  write_proc_cgroup(42, "12:memory:/\n");

  write_file("sys/fs/cgroup/memory/memory.limit_in_bytes", std::to_string(kV1UnlimitedSentinel) + "\n");
  EXPECT_EQ(read().memory_limit.state(), LimitState::kUnlimited);

  write_file("sys/fs/cgroup/memory/memory.limit_in_bytes", std::to_string(kV1UnlimitedBoundary) + "\n");
  EXPECT_EQ(read().memory_limit.state(), LimitState::kUnlimited);

  write_file("sys/fs/cgroup/memory/memory.limit_in_bytes", std::to_string(kV1UnlimitedBoundary - 1) + "\n");
  auto info = read();
  EXPECT_EQ(info.memory_limit.state(), LimitState::kLimited);
  ASSERT_TRUE(info.memory_limit.value().has_value());
  EXPECT_EQ(*info.memory_limit.value(), kV1UnlimitedBoundary - 1);
}

// @verifies REQ_INTEROP_003
TEST_F(CgroupTree, V1QuotaMinusOneIsUnlimited) {
  write_proc_cgroup(42, "11:cpu,cpuacct:/\n");
  write_file("sys/fs/cgroup/cpu,cpuacct/cpu.cfs_quota_us", "-1\n");
  write_file("sys/fs/cgroup/cpu,cpuacct/cpu.cfs_period_us", "100000\n");

  auto info = read();
  EXPECT_EQ(info.cpu_quota.state(), LimitState::kUnlimited);
  EXPECT_FALSE(info.cpu_quota.value().has_value());
  ASSERT_TRUE(info.cpu_period_us.has_value());
  EXPECT_EQ(*info.cpu_period_us, 100000);
}

// @verifies REQ_INTEROP_003
TEST_F(CgroupTree, MalformedProcCgroupLinesAreIgnored) {
  write_proc_cgroup(42, std::string("this line has no colons\n") + "5:only:two:colons:is:fine:/docker/" + kDockerId +
                            "\n" + "\n" + "0::" + "\n");

  // The "0::" line carries no path and is dropped; the v1 line still resolves.
  auto info = read();
  EXPECT_EQ(info.cgroup_path, std::string("two:colons:is:fine:/docker/") + kDockerId);
}

// A fallback location that says "no limit" must not overwrite an unreadable
// answer from the process's own cgroup. Reporting no limit for a limit nobody
// could read is the failure this reader exists to remove.
// @verifies REQ_INTEROP_003
TEST_F(CgroupTree, UnreadableOwnCgroupIsNotMaskedByAnUnlimitedFallback) {
  write_proc_cgroup(42, std::string("12:memory:/docker/") + kDockerId + "\n");
  write_file(std::string("sys/fs/cgroup/memory/docker/") + kDockerId + "/memory.limit_in_bytes", "garbage\n");
  write_file("sys/fs/cgroup/memory/memory.limit_in_bytes", std::to_string(kV1UnlimitedSentinel) + "\n");

  auto info = read();
  EXPECT_EQ(info.memory_limit.state(), LimitState::kUnreadable);
  EXPECT_FALSE(info.memory_limit.value().has_value());
}

// cpu.max holds exactly two fields. Anything after them is not this file.
// @verifies REQ_INTEROP_003
TEST_F(CgroupTree, CpuMaxWithATrailingTokenIsUnreadable) {
  write_proc_cgroup(42, "0::/\n");
  write_file("sys/fs/cgroup/cpu.max", "50000 100000 garbage\n");

  auto info = read();
  EXPECT_EQ(info.cpu_quota.state(), LimitState::kUnreadable);
  EXPECT_FALSE(info.cpu_quota.value().has_value());
  EXPECT_FALSE(info.cpu_period_us.has_value());
}

// A limit file that is there but yields nothing readable is a failed read, not
// an absent limit. A permission-based fixture cannot be used here because root
// bypasses it, and the tests run as root in CI.
// @verifies REQ_INTEROP_003
TEST_F(CgroupTree, PresentButUnreadableLimitFileIsNotReportedAsAbsent) {
  write_proc_cgroup(42, "0::/\n");
  fs::create_directories(root_ / "sys/fs/cgroup/memory.max");

  auto info = read();
  EXPECT_EQ(info.memory_limit.state(), LimitState::kUnreadable);
  EXPECT_NE(info.memory_limit.state(), LimitState::kUnavailable);
}

// The container id can sit on a hierarchy other than the ones the limits come
// from, so every reported path is searched for one.
// @verifies REQ_INTEROP_003
TEST_F(CgroupTree, ContainerIdIsFoundOnANonPreferredHierarchy) {
  write_proc_cgroup(42, std::string("12:memory:/\n11:cpu,cpuacct:/\n10:pids:/docker/") + kDockerId + "\n");

  auto info = read();
  EXPECT_EQ(info.container_id, kDockerId);
  EXPECT_EQ(info.container_runtime, "docker");
  EXPECT_TRUE(info.containerized);
}

// cgroup v1 does not require a controller to be mounted at a directory named
// after it, so the mount points are read from the process's mountinfo.
// @verifies REQ_INTEROP_003
TEST_F(CgroupTree, V1ControllerAtACustomMountPoint) {
  write_proc_cgroup(42, std::string("12:memory:/docker/") + kDockerId + "\n");
  write_file("proc/42/mountinfo",
             "25 1 259:2 / / rw,relatime - ext4 /dev/nvme0n1p2 rw\n"
             "30 25 0:26 / /sys/fs/cgroup/odd rw,relatime - cgroup cgroup rw,memory\n");
  write_file(std::string("sys/fs/cgroup/odd/docker/") + kDockerId + "/memory.limit_in_bytes", "536870912\n");

  auto info = read();
  EXPECT_EQ(info.memory_limit.state(), LimitState::kLimited);
  ASSERT_TRUE(info.memory_limit.value().has_value());
  EXPECT_EQ(*info.memory_limit.value(), 536870912u);
}

// A mount that exposes only part of the hierarchy shows a cgroup path with
// that subtree stripped off the front.
// @verifies REQ_INTEROP_003
TEST_F(CgroupTree, V1MountExposingASubtreeStripsItFromThePath) {
  write_proc_cgroup(42, "12:memory:/docker/parent/child\n");
  write_file("proc/42/mountinfo",
             "25 1 259:2 / / rw,relatime - ext4 /dev/nvme0n1p2 rw\n"
             "30 25 0:26 /docker/parent /sys/fs/cgroup/memory rw,relatime - cgroup cgroup rw,memory\n");
  write_file("sys/fs/cgroup/memory/child/memory.limit_in_bytes", "268435456\n");

  auto info = read();
  EXPECT_EQ(info.memory_limit.state(), LimitState::kLimited);
  ASSERT_TRUE(info.memory_limit.value().has_value());
  EXPECT_EQ(*info.memory_limit.value(), 268435456u);
}

// A controller lives on one hierarchy. The other one reporting no limit must
// not outrank the real limit the process actually has.
// @verifies REQ_INTEROP_003
TEST_F(CgroupTree, UnlimitedUnifiedDoesNotOutrankALimitedLegacyController) {
  write_proc_cgroup(42, std::string("0::/\n12:memory:/docker/") + kDockerId + "\n");
  write_file("sys/fs/cgroup/memory.max", "max\n");
  write_file(std::string("sys/fs/cgroup/memory/docker/") + kDockerId + "/memory.limit_in_bytes", "536870912\n");

  auto info = read();
  EXPECT_EQ(info.memory_limit.state(), LimitState::kLimited);
  ASSERT_TRUE(info.memory_limit.value().has_value());
  EXPECT_EQ(*info.memory_limit.value(), 536870912u);
}

// @verifies REQ_INTEROP_003
TEST_F(CgroupTree, UnlimitedUnifiedCpuDoesNotOutrankALimitedLegacyQuota) {
  write_proc_cgroup(42, std::string("0::/\n11:cpu,cpuacct:/docker/") + kDockerId + "\n");
  write_file("sys/fs/cgroup/cpu.max", "max 100000\n");
  write_file(std::string("sys/fs/cgroup/cpu,cpuacct/docker/") + kDockerId + "/cpu.cfs_quota_us", "50000\n");
  write_file(std::string("sys/fs/cgroup/cpu,cpuacct/docker/") + kDockerId + "/cpu.cfs_period_us", "100000\n");

  auto info = read();
  EXPECT_EQ(info.cpu_quota.state(), LimitState::kLimited);
  ASSERT_TRUE(info.cpu_quota.value().has_value());
  EXPECT_EQ(*info.cpu_quota.value(), 50000);
}

// A mount root only matches on a component boundary: /docker/parent must not
// swallow /docker/parent2, which is a different cgroup.
// @verifies REQ_INTEROP_003
TEST_F(CgroupTree, MountRootMatchesOnlyWholePathComponents) {
  write_proc_cgroup(42, "12:memory:/docker/parent2/child\n");
  write_file("proc/42/mountinfo",
             "25 1 259:2 / / rw,relatime - ext4 /dev/nvme0n1p2 rw\n"
             "30 25 0:26 /docker/parent /sys/fs/cgroup/memory rw,relatime - cgroup cgroup rw,memory\n");
  // What the bad prefix strip would have built, holding a limit that belongs to
  // a cgroup this mount does not expose.
  write_file("sys/fs/cgroup/memory2/child/memory.limit_in_bytes", "999999999\n");

  auto info = read();
  EXPECT_NE(info.memory_limit.state(), LimitState::kLimited);
}

// @verifies REQ_INTEROP_003
TEST_F(CgroupTree, SystemdNspawnMarkerIsDetected) {
  write_proc_cgroup(42, "0::/\n");
  write_file("proc/42/root/run/systemd/container", "systemd-nspawn\n");

  EXPECT_TRUE(is_containerized(42, root()));
  auto info = read();
  EXPECT_TRUE(info.containerized);
  EXPECT_TRUE(info.container_runtime.empty());
}

// The unified hierarchy can be mounted somewhere other than /sys/fs/cgroup, and
// can expose only part of itself, exactly like a legacy controller.
// @verifies REQ_INTEROP_003
TEST_F(CgroupTree, UnifiedHierarchyAtACustomMountPointWithASubtreeRoot) {
  write_proc_cgroup(42, "0::/tenant/workload\n");
  write_file("proc/42/mountinfo",
             "25 1 259:2 / / rw,relatime - ext4 /dev/nvme0n1p2 rw\n"
             "30 25 0:26 /tenant /custom/unified rw,relatime - cgroup2 cgroup rw\n");
  write_file("custom/unified/workload/memory.max", "268435456\n");

  auto info = read();
  EXPECT_EQ(info.memory_limit.state(), LimitState::kLimited);
  ASSERT_TRUE(info.memory_limit.value().has_value());
  EXPECT_EQ(*info.memory_limit.value(), 268435456u);
}

// A path whose parent is a regular file fails to open with ENOTDIR, not
// ENOENT. That is a file we could not read, not a limit that is not there, and
// unlike a permission fixture it behaves the same when the tests run as root.
// @verifies REQ_INTEROP_003
TEST_F(CgroupTree, OpenFailureOtherThanMissingIsUnreadable) {
  write_proc_cgroup(42, std::string("0::/docker/") + kDockerId + "\n");
  // "docker" is a file, so ".../docker/<id>/memory.max" cannot be walked.
  write_file("sys/fs/cgroup/docker", "not a directory\n");

  auto info = read();
  EXPECT_EQ(info.memory_limit.state(), LimitState::kUnreadable);
  EXPECT_NE(info.memory_limit.state(), LimitState::kUnavailable);
}

// A first line that never ends inside the bytes we are willing to read is not a
// limit. Parsing the prefix would turn "536870912<spaces>garbage" into a valid
// 512 MiB limit.
// @verifies REQ_INTEROP_003
TEST_F(CgroupTree, LimitFileWithoutALineEndingWithinTheCapIsUnreadable) {
  write_proc_cgroup(42, "0::/\n");
  write_file("sys/fs/cgroup/memory.max", "536870912" + std::string(8192, ' ') + "garbage\n");

  auto info = read();
  EXPECT_EQ(info.memory_limit.state(), LimitState::kUnreadable);
  EXPECT_FALSE(info.memory_limit.value().has_value());
}

// A limit file with no trailing newline is still a complete first line.
// @verifies REQ_INTEROP_003
TEST_F(CgroupTree, LimitFileWithoutATrailingNewlineIsRead) {
  write_proc_cgroup(42, "0::/\n");
  write_file("sys/fs/cgroup/memory.max", "536870912");

  auto info = read();
  EXPECT_EQ(info.memory_limit.state(), LimitState::kLimited);
  ASSERT_TRUE(info.memory_limit.value().has_value());
  EXPECT_EQ(*info.memory_limit.value(), 536870912u);
}

// A mount root is the same subtree whether or not it is written with a
// trailing slash.
// @verifies REQ_INTEROP_003
TEST_F(CgroupTree, MountRootWithATrailingSlashStillMatches) {
  write_proc_cgroup(42, "12:memory:/docker/parent/child\n");
  write_file("proc/42/mountinfo",
             "25 1 259:2 / / rw,relatime - ext4 /dev/nvme0n1p2 rw\n"
             "30 25 0:26 /docker/parent/ /sys/fs/cgroup/memory rw,relatime - cgroup cgroup rw,memory\n");
  write_file("sys/fs/cgroup/memory/child/memory.limit_in_bytes", "268435456\n");

  auto info = read();
  EXPECT_EQ(info.memory_limit.state(), LimitState::kLimited);
  ASSERT_TRUE(info.memory_limit.value().has_value());
  EXPECT_EQ(*info.memory_limit.value(), 268435456u);
}

// The hierarchy that owns the controller answers, even when the other one has
// a readable file. Choosing by which answer looks better would report a limit
// belonging to a cgroup this process is not in.
// @verifies REQ_INTEROP_003
TEST_F(CgroupTree, LegacyOwnedControllerAnswersEvenWhenUnifiedIsReadable) {
  write_proc_cgroup(42, std::string("0::/\n12:memory:/docker/") + kDockerId + "\n");
  write_file("sys/fs/cgroup/memory.max", "999999999\n");
  write_file(std::string("sys/fs/cgroup/memory/docker/") + kDockerId + "/memory.limit_in_bytes", "garbage\n");

  auto info = read();
  // The legacy controller owns memory here, and its file cannot be parsed.
  EXPECT_EQ(info.memory_limit.state(), LimitState::kUnreadable);
  EXPECT_FALSE(info.memory_limit.value().has_value());
}

// @verifies REQ_INTEROP_003
TEST_F(CgroupTree, LegacyOwnedControllerReportsUnlimitedOverAUnifiedNumber) {
  write_proc_cgroup(42, std::string("0::/\n12:memory:/docker/") + kDockerId + "\n");
  write_file("sys/fs/cgroup/memory.max", "999999999\n");
  write_file(std::string("sys/fs/cgroup/memory/docker/") + kDockerId + "/memory.limit_in_bytes",
             std::to_string(kV1UnlimitedSentinel) + "\n");

  auto info = read();
  EXPECT_EQ(info.memory_limit.state(), LimitState::kUnlimited);
}

// A limit path that turns out to be a FIFO with no writer must not park the
// caller. Opening one without O_NONBLOCK blocks until a writer appears, which
// for an HTTP worker means never. If this ever regresses the test does not
// fail, it hangs, and its ctest timeout reports it.
// @verifies REQ_INTEROP_003
TEST_F(CgroupTree, LimitPathThatIsAFifoDoesNotBlock) {
  write_proc_cgroup(42, "0::/\n");
  fs::create_directories(root_ / "sys/fs/cgroup");
  auto fifo = (root_ / "sys/fs/cgroup/memory.max").string();
  ASSERT_EQ(::mkfifo(fifo.c_str(), 0600), 0);

  auto info = read();
  // It is there and it gave us nothing, which is a failed read.
  EXPECT_EQ(info.memory_limit.state(), LimitState::kUnreadable);
}

// The first line can be longer than one read returns. Stopping at the first
// chunk would take the prefix for the whole line.
// @verifies REQ_INTEROP_003
TEST_F(CgroupTree, FirstLineLongerThanOneReadIsAccumulated) {
  write_proc_cgroup(42, "0::/\n");
  // Leading blanks push the number past any single-chunk read; trim drops them.
  write_file("sys/fs/cgroup/memory.max", std::string(1024, ' ') + "536870912\n");

  auto info = read();
  EXPECT_EQ(info.memory_limit.state(), LimitState::kLimited);
  ASSERT_TRUE(info.memory_limit.value().has_value());
  EXPECT_EQ(*info.memory_limit.value(), 536870912u);
}

// @verifies REQ_INTEROP_003
TEST_F(CgroupTree, MissingCgroupFileIsAnError) {
  fs::create_directories(root_ / "proc" / "42");

  auto result = read_cgroup_info(42, root());
  ASSERT_FALSE(result.has_value());
  EXPECT_FALSE(result.error().empty());
}
