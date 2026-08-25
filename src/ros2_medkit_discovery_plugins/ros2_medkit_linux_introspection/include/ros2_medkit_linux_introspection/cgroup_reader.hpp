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

#include <cstdint>
#include <optional>
#include <string>
#include <sys/types.h>
#include <tl/expected.hpp>

namespace ros2_medkit_linux_introspection {

/// Outcome of reading one cgroup limit.
///
/// "No limit is in force" and "the limit could not be read" are different facts
/// about a container: only the first one means the container may use the whole
/// machine, so they are reported as different states rather than both as an
/// absent value.
enum class LimitState {
  kUnavailable,  ///< No file for this limit exists in any supported cgroup layout
  kUnreadable,   ///< A file exists but its contents could not be parsed
  kUnlimited,    ///< The cgroup reports no limit ("max" on v2, -1 on v1)
  kLimited,      ///< A numeric limit is in force
};

/// Wire form of a LimitState: "unavailable", "unreadable", "unlimited" or "limited".
std::string limit_state_to_string(LimitState state);

/// One cgroup limit together with the outcome of reading it.
///
/// The value is present if and only if the state is LimitState::kLimited, an
/// invariant the named constructors are the only way to establish.
template <typename T>
class CgroupLimit {
 public:
  CgroupLimit() = default;

  static CgroupLimit limited(T value) {
    CgroupLimit limit;
    limit.state_ = LimitState::kLimited;
    limit.value_ = value;
    return limit;
  }

  static CgroupLimit unlimited() {
    return CgroupLimit(LimitState::kUnlimited);
  }

  static CgroupLimit unreadable() {
    return CgroupLimit(LimitState::kUnreadable);
  }

  static CgroupLimit unavailable() {
    return CgroupLimit(LimitState::kUnavailable);
  }

  LimitState state() const {
    return state_;
  }

  const std::optional<T> & value() const {
    return value_;
  }

 private:
  explicit CgroupLimit(LimitState state) : state_(state) {
  }

  LimitState state_{LimitState::kUnavailable};
  std::optional<T> value_;
};

struct CgroupInfo {
  std::string cgroup_path;
  std::string container_id;       // 64-char hex from cgroup path, or empty
  std::string container_runtime;  // "docker", "podman", "containerd", or empty

  /// Whether the process runs inside a container.
  ///
  /// Not the same as having a container id: under ``cgroupns=private`` the
  /// reported cgroup path is the namespace root and carries no id, so the path
  /// alone cannot answer the question and a runtime marker does instead. A
  /// container is therefore reported with an empty ``container_id``.
  bool containerized{false};

  /// Memory limit in bytes, from ``memory.max`` (v2) or ``memory.limit_in_bytes`` (v1).
  CgroupLimit<uint64_t> memory_limit;

  /// CPU bandwidth quota in microseconds of CPU time per period, from ``cpu.max``
  /// (v2) or ``cpu.cfs_quota_us`` (v1).
  ///
  /// This is the CFS bandwidth limit and nothing else. It does not reflect the
  /// set of CPUs a container is pinned to (``--cpuset-cpus``); that is visible
  /// only through sched_getaffinity(). A caller that wants the effective CPU
  /// budget needs both numbers.
  CgroupLimit<int64_t> cpu_quota;

  /// Period the quota is measured over, in microseconds. It is read together
  /// with the quota and carries no state of its own, so it is set whenever the
  /// CPU limit was read at all - including when the quota itself is unlimited.
  std::optional<int64_t> cpu_period_us;
};

/// Detect if PID runs inside a container.
///
/// Prefers the container id carried by the cgroup path. When the cgroup
/// namespace hides that id, falls back to the markers a runtime leaves behind:
/// ``/.dockerenv``, ``/run/.containerenv``, or an overlay filesystem mounted as
/// the process's root.
bool is_containerized(pid_t pid, const std::string & root = "/");

/// Read cgroup info for a PID.
///
/// Handles the unified (v2) and legacy (v1) hierarchies as well as the hybrid
/// layout that mounts both, and both cgroup namespace modes: the interface
/// files are looked for at the bare mount point, where ``cgroupns=private``
/// puts the process's own cgroup, and at the mount point joined with the
/// reported path, where ``cgroupns=host`` puts it.
///
/// Fails only when the cgroup of the process cannot be determined at all. A
/// limit that could not be read is reported through its LimitState, not as a
/// missing limit.
tl::expected<CgroupInfo, std::string> read_cgroup_info(pid_t pid, const std::string & root = "/");

/// Extract container ID from a cgroup path string
/// Supports Docker (/docker/<hash>), podman (/libpod-<hash>.scope), containerd
/// (/cri-containerd-<hash>.scope)
std::string extract_container_id(const std::string & cgroup_path);

/// Detect container runtime from cgroup path
std::string detect_runtime(const std::string & cgroup_path);

}  // namespace ros2_medkit_linux_introspection
