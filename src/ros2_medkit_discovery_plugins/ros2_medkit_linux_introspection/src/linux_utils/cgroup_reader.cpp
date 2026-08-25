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

#include "ros2_medkit_linux_introspection/cgroup_reader.hpp"

#include <cctype>
#include <filesystem>
#include <fstream>
#include <regex>
#include <sstream>
#include <utility>
#include <vector>

namespace ros2_medkit_linux_introspection {

namespace {

// cgroup v1 has no "max" keyword: an unlimited memory controller reports a
// sentinel near the largest representable byte count, and its exact value
// depends on the page size, so anything in that region counts as no limit.
constexpr uint64_t kV1MemoryUnlimited = uint64_t{1} << 62;

const char * const kWhitespace = " \t\r\n";

std::string trim(const std::string & text) {
  auto begin = text.find_first_not_of(kWhitespace);
  if (begin == std::string::npos) {
    return {};
  }
  auto end = text.find_last_not_of(kWhitespace);
  return text.substr(begin, end - begin + 1);
}

// Returns nullopt when the file cannot be opened, and the (possibly empty)
// first line when it can - the caller has to tell "no such limit file" from
// "the limit file holds something we cannot parse".
std::optional<std::string> read_first_line(const std::string & path) {
  std::ifstream f(path);
  if (!f.is_open()) {
    return std::nullopt;
  }
  std::string line;
  std::getline(f, line);
  return line;
}

std::optional<uint64_t> parse_u64(const std::string & text) {
  // std::stoull accepts a leading minus and wraps it around, which would turn
  // an error value into a plausible byte count.
  if (text.empty() || text.front() == '-') {
    return std::nullopt;
  }
  try {
    size_t consumed = 0;
    auto value = std::stoull(text, &consumed);
    if (consumed != text.size()) {
      return std::nullopt;
    }
    return static_cast<uint64_t>(value);
  } catch (const std::exception &) {
    return std::nullopt;
  }
}

std::optional<int64_t> parse_i64(const std::string & text) {
  if (text.empty()) {
    return std::nullopt;
  }
  try {
    size_t consumed = 0;
    auto value = std::stoll(text, &consumed);
    if (consumed != text.size()) {
      return std::nullopt;
    }
    return static_cast<int64_t>(value);
  } catch (const std::exception &) {
    return std::nullopt;
  }
}

// The cgroup a process belongs to, as reported by /proc/{pid}/cgroup. A host
// mounts the unified hierarchy, the legacy per-controller ones, or both.
struct CgroupPaths {
  std::string unified;                                  // path from the "0::" line
  std::vector<std::pair<std::string, std::string>> v1;  // controller -> path, in file order
};

CgroupPaths read_cgroup_paths(pid_t pid, const std::string & root) {
  CgroupPaths paths;
  std::ifstream f(root + "/proc/" + std::to_string(pid) + "/cgroup");
  std::string line;
  while (std::getline(f, line)) {
    // Both layouts write "<hierarchy-id>:<controllers>:<path>". The unified
    // hierarchy is the one with an empty controller list; every other line
    // belongs to a v1 hierarchy and names the controllers mounted on it.
    auto first = line.find(':');
    if (first == std::string::npos) {
      continue;
    }
    auto second = line.find(':', first + 1);
    if (second == std::string::npos) {
      continue;
    }
    auto controllers = line.substr(first + 1, second - first - 1);
    auto path = trim(line.substr(second + 1));
    if (path.empty()) {
      continue;
    }
    if (controllers.empty()) {
      paths.unified = path;
      continue;
    }
    // A v1 hierarchy can carry several co-mounted controllers, e.g. "cpu,cpuacct".
    size_t start = 0;
    while (start <= controllers.size()) {
      auto comma = controllers.find(',', start);
      auto name = comma == std::string::npos ? controllers.substr(start) : controllers.substr(start, comma - start);
      if (!name.empty()) {
        paths.v1.emplace_back(name, path);
      }
      if (comma == std::string::npos) {
        break;
      }
      start = comma + 1;
    }
  }
  return paths;
}

// A container's root filesystem is an overlay mount; a host's is not. This is
// per-process, so it still answers for a PID other than our own.
bool has_overlay_root(pid_t pid, const std::string & root) {
  std::ifstream f(root + "/proc/" + std::to_string(pid) + "/mountinfo");
  std::string line;
  while (std::getline(f, line)) {
    // "<id> <parent> <maj:min> <mount root> <mount point> <options> [tags] - <fstype> <source> ..."
    auto separator = line.find(" - ");
    if (separator == std::string::npos) {
      continue;
    }
    std::istringstream head(line.substr(0, separator));
    std::string field;
    std::string mount_point;
    if (!(head >> field >> field >> field >> field >> mount_point)) {
      continue;
    }
    if (mount_point != "/") {
      continue;
    }
    std::istringstream tail(line.substr(separator + 3));
    std::string fstype;
    if (!(tail >> fstype)) {
      continue;
    }
    return fstype == "overlay";
  }
  return false;
}

// What a container runtime leaves behind when the cgroup path cannot name it.
struct ContainerMarker {
  bool present{false};
  std::string runtime;  // empty when the marker does not identify a runtime
};

ContainerMarker detect_container_marker(pid_t pid, const std::string & root) {
  std::error_code ec;
  if (std::filesystem::exists(root + "/.dockerenv", ec)) {
    return {true, "docker"};
  }
  if (std::filesystem::exists(root + "/run/.containerenv", ec)) {
    return {true, "podman"};
  }
  if (has_overlay_root(pid, root)) {
    return {true, {}};
  }
  return {};
}

std::optional<std::string> controller_path(const CgroupPaths & paths, const std::string & controller) {
  for (const auto & [name, path] : paths.v1) {
    if (name == controller) {
      return path;
    }
  }
  return std::nullopt;
}

// The single path reported to callers and searched for a container id. On a
// hybrid host the unified line is often the bare root while the container is
// named only on the v1 hierarchies, so a root unified path yields to those.
std::string select_reported_path(const CgroupPaths & paths) {
  if (!paths.unified.empty() && paths.unified != "/") {
    return paths.unified;
  }
  for (const auto * preferred : {"memory", "cpu", "cpuacct"}) {
    if (auto path = controller_path(paths, preferred); path && *path != "/") {
      return *path;
    }
  }
  if (!paths.unified.empty()) {
    return paths.unified;
  }
  if (!paths.v1.empty()) {
    return paths.v1.front().second;
  }
  return {};
}

// Where a controller's interface files can sit, most specific first. Under
// cgroupns=host the mount point holds the whole hierarchy and the reported path
// leads to the process's own cgroup; under cgroupns=private that cgroup is the
// mount point itself and the reported path resolves nowhere. Trying both covers
// either mode without having to detect which one is in use.
std::vector<std::string> candidate_dirs(const std::string & mount, const std::string & cgroup_path) {
  std::vector<std::string> dirs;
  if (!cgroup_path.empty() && cgroup_path != "/") {
    dirs.push_back(mount + cgroup_path);
  }
  dirs.push_back(mount);
  return dirs;
}

std::vector<std::string> unified_mounts(const std::string & root) {
  // The hybrid layout keeps the legacy controllers at the top level and mounts
  // the unified hierarchy beside them.
  return {root + "/sys/fs/cgroup", root + "/sys/fs/cgroup/unified"};
}

std::vector<std::string> v1_mounts(const std::string & root, const std::string & controller) {
  std::vector<std::string> mounts{root + "/sys/fs/cgroup/" + controller};
  if (controller == "cpu") {
    // Distributions co-mount cpu with cpuacct and leave "cpu" as a symlink to
    // that directory, which not every image preserves.
    mounts.push_back(root + "/sys/fs/cgroup/cpu,cpuacct");
  }
  return mounts;
}

int rank(LimitState state) {
  switch (state) {
    case LimitState::kLimited:
      return 3;
    case LimitState::kUnlimited:
      return 2;
    case LimitState::kUnreadable:
      return 1;
    case LimitState::kUnavailable:
      return 0;
  }
  return 0;
}

// Merges what two candidate locations said about the same limit, keeping the
// most informative answer and, on a tie, the more specific location. A location
// that reports no limit does not end the search: the root of a cgroup v1
// hierarchy always reads as unlimited and would otherwise mask a limit set on
// the container's own cgroup further down.
template <typename T>
CgroupLimit<T> better_of(const CgroupLimit<T> & first, const CgroupLimit<T> & second) {
  return rank(second.state()) > rank(first.state()) ? second : first;
}

// The CPU limit is the quota together with the period it is measured over; the
// period alone says nothing, so the two travel as one reading.
struct CpuLimit {
  CgroupLimit<int64_t> quota;
  std::optional<int64_t> period_us;
};

CpuLimit better_of(const CpuLimit & first, const CpuLimit & second) {
  return rank(second.quota.state()) > rank(first.quota.state()) ? second : first;
}

CgroupLimit<uint64_t> read_memory_v2(const std::string & dir) {
  auto content = read_first_line(dir + "/memory.max");
  if (!content) {
    return CgroupLimit<uint64_t>::unavailable();
  }
  auto text = trim(*content);
  if (text == "max") {
    return CgroupLimit<uint64_t>::unlimited();
  }
  auto value = parse_u64(text);
  if (!value) {
    return CgroupLimit<uint64_t>::unreadable();
  }
  return CgroupLimit<uint64_t>::limited(*value);
}

CgroupLimit<uint64_t> read_memory_v1(const std::string & dir) {
  auto content = read_first_line(dir + "/memory.limit_in_bytes");
  if (!content) {
    return CgroupLimit<uint64_t>::unavailable();
  }
  auto value = parse_u64(trim(*content));
  if (!value) {
    return CgroupLimit<uint64_t>::unreadable();
  }
  if (*value >= kV1MemoryUnlimited) {
    return CgroupLimit<uint64_t>::unlimited();
  }
  return CgroupLimit<uint64_t>::limited(*value);
}

CpuLimit read_cpu_v2(const std::string & dir) {
  auto content = read_first_line(dir + "/cpu.max");
  if (!content) {
    return {CgroupLimit<int64_t>::unavailable(), std::nullopt};
  }

  // "<quota> <period>", where the quota may be the keyword "max".
  std::istringstream ss(*content);
  std::string quota_text;
  std::string period_text;
  if (!(ss >> quota_text) || !(ss >> period_text)) {
    return {CgroupLimit<int64_t>::unreadable(), std::nullopt};
  }

  auto period = parse_i64(period_text);
  if (!period || *period <= 0) {
    return {CgroupLimit<int64_t>::unreadable(), std::nullopt};
  }
  if (quota_text == "max") {
    return {CgroupLimit<int64_t>::unlimited(), *period};
  }
  auto quota = parse_i64(quota_text);
  if (!quota || *quota <= 0) {
    return {CgroupLimit<int64_t>::unreadable(), std::nullopt};
  }
  return {CgroupLimit<int64_t>::limited(*quota), *period};
}

CpuLimit read_cpu_v1(const std::string & dir) {
  auto quota_content = read_first_line(dir + "/cpu.cfs_quota_us");
  auto period_content = read_first_line(dir + "/cpu.cfs_period_us");
  if (!quota_content && !period_content) {
    return {CgroupLimit<int64_t>::unavailable(), std::nullopt};
  }

  std::optional<int64_t> period;
  if (period_content) {
    period = parse_i64(trim(*period_content));
    if (period && *period <= 0) {
      period.reset();
    }
  }
  if (!quota_content || !period) {
    return {CgroupLimit<int64_t>::unreadable(), std::nullopt};
  }

  auto quota = parse_i64(trim(*quota_content));
  if (!quota) {
    return {CgroupLimit<int64_t>::unreadable(), std::nullopt};
  }
  if (*quota == -1) {
    return {CgroupLimit<int64_t>::unlimited(), period};
  }
  if (*quota <= 0) {
    return {CgroupLimit<int64_t>::unreadable(), std::nullopt};
  }
  return {CgroupLimit<int64_t>::limited(*quota), period};
}

}  // namespace

std::string limit_state_to_string(LimitState state) {
  switch (state) {
    case LimitState::kLimited:
      return "limited";
    case LimitState::kUnlimited:
      return "unlimited";
    case LimitState::kUnreadable:
      return "unreadable";
    case LimitState::kUnavailable:
      return "unavailable";
  }
  return "unavailable";
}

std::string extract_container_id(const std::string & cgroup_path) {
  // Docker: /docker-<64hex>.scope or /docker/<64hex>
  // Podman: /libpod-<64hex>.scope
  // Containerd: /cri-containerd-<64hex>.scope
  static const std::regex re("(?:docker-|libpod-|cri-containerd-|docker/)([0-9a-f]{64})(?:\\.scope)?");
  std::smatch match;
  if (std::regex_search(cgroup_path, match, re)) {
    return match[1].str();
  }
  return {};
}

std::string detect_runtime(const std::string & cgroup_path) {
  if (cgroup_path.find("docker") != std::string::npos) {
    return "docker";
  }
  if (cgroup_path.find("libpod") != std::string::npos) {
    return "podman";
  }
  if (cgroup_path.find("cri-containerd") != std::string::npos) {
    return "containerd";
  }
  return {};
}

bool is_containerized(pid_t pid, const std::string & root) {
  auto paths = read_cgroup_paths(pid, root);
  if (!extract_container_id(select_reported_path(paths)).empty()) {
    return true;
  }
  return detect_container_marker(pid, root).present;
}

tl::expected<CgroupInfo, std::string> read_cgroup_info(pid_t pid, const std::string & root) {
  auto paths = read_cgroup_paths(pid, root);
  auto cgroup_path = select_reported_path(paths);
  if (cgroup_path.empty()) {
    return tl::make_unexpected("Cannot read cgroup for PID " + std::to_string(pid));
  }

  CgroupInfo info;
  info.cgroup_path = cgroup_path;
  info.container_id = extract_container_id(cgroup_path);
  info.container_runtime = detect_runtime(cgroup_path);
  info.containerized = !info.container_id.empty();
  if (!info.containerized) {
    auto marker = detect_container_marker(pid, root);
    info.containerized = marker.present;
    if (info.container_runtime.empty()) {
      info.container_runtime = marker.runtime;
    }
  }

  CpuLimit cpu;

  if (!paths.unified.empty()) {
    for (const auto & mount : unified_mounts(root)) {
      for (const auto & dir : candidate_dirs(mount, paths.unified)) {
        info.memory_limit = better_of(info.memory_limit, read_memory_v2(dir));
        cpu = better_of(cpu, read_cpu_v2(dir));
      }
    }
  }

  if (auto memory_path = controller_path(paths, "memory")) {
    for (const auto & mount : v1_mounts(root, "memory")) {
      for (const auto & dir : candidate_dirs(mount, *memory_path)) {
        info.memory_limit = better_of(info.memory_limit, read_memory_v1(dir));
      }
    }
  }

  if (auto cpu_path = controller_path(paths, "cpu")) {
    for (const auto & mount : v1_mounts(root, "cpu")) {
      for (const auto & dir : candidate_dirs(mount, *cpu_path)) {
        cpu = better_of(cpu, read_cpu_v1(dir));
      }
    }
  }

  info.cpu_quota = cpu.quota;
  info.cpu_period_us = cpu.period_us;
  return info;
}

}  // namespace ros2_medkit_linux_introspection
