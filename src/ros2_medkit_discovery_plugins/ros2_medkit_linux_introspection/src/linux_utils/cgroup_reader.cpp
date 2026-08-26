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
#include <cerrno>
#include <fcntl.h>
#include <filesystem>
#include <fstream>
#include <regex>
#include <sstream>
#include <unistd.h>
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

// The outcome of looking at one limit file. "Not there" and "there but it
// would not open" are different answers: the second is a failed read, and
// reporting it as an absent limit is the confusion this reader exists to
// remove.
struct LimitFile {
  bool exists{false};
  bool opened{false};
  std::string line;
};

LimitFile read_limit_file(const std::string & path) {
  LimitFile result;
  // errno on the single open call separates "no such file" from "present but
  // it would not open". Opening and then asking whether the path exists is two
  // steps, and a file that disappears between them reads as absent when it was
  // really a failed read.
  int fd = ::open(path.c_str(), O_RDONLY | O_CLOEXEC);
  if (fd < 0) {
    result.exists = (errno != ENOENT);
    return result;
  }
  result.exists = true;
  result.opened = true;

  std::string buffer(4096, '\0');
  auto count = ::read(fd, buffer.data(), buffer.size());
  ::close(fd);
  if (count > 0) {
    buffer.resize(static_cast<size_t>(count));
    auto newline = buffer.find('\n');
    result.line = newline == std::string::npos ? buffer : buffer.substr(0, newline);
  }
  return result;
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

// Whether the process's own root filesystem is an overlay mount. Read from the
// process's mountinfo, so it answers for a PID other than our own. The last
// entry mounted at "/" is the visible one when mounts are stacked.
bool has_overlay_root(pid_t pid, const std::string & root) {
  std::ifstream f(root + "/proc/" + std::to_string(pid) + "/mountinfo");
  std::string line;
  bool overlay = false;
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
    overlay = (fstype == "overlay");
  }
  return overlay;
}

// What a container runtime leaves behind when the cgroup path cannot name it.
struct ContainerMarker {
  bool present{false};
  std::string runtime;  // empty when the marker does not identify a runtime
};

// The marker files describe the filesystem of the INSPECTED process, so they
// are read through its own root rather than the gateway's - a gateway that is
// itself containerized would otherwise answer for every PID it can see.
ContainerMarker detect_container_marker(pid_t pid, const std::string & root, const std::string & cgroup_path) {
  const auto process_root = root + "/proc/" + std::to_string(pid) + "/root";
  std::error_code ec;
  if (std::filesystem::exists(process_root + "/.dockerenv", ec)) {
    return {true, "docker"};
  }
  if (std::filesystem::exists(process_root + "/run/.containerenv", ec)) {
    return {true, "podman"};
  }
  if (std::filesystem::exists(process_root + "/run/systemd/container", ec)) {
    return {true, {}};
  }

  // An overlay root on its own proves nothing: whole distributions boot that
  // way. It only says "container" together with a cgroup path that has been
  // rewritten to the namespace root, which a process outside a cgroup
  // namespace never reports.
  if (cgroup_path == "/" && has_overlay_root(pid, root)) {
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
  // A path that names a container wins, whichever hierarchy carries it. The
  // controllers the limits come from are not always the ones holding the id:
  // a partly delegated v1 host can name the container on "pids" alone.
  if (!extract_container_id(paths.unified).empty()) {
    return paths.unified;
  }
  for (const auto & [name, path] : paths.v1) {
    (void)name;
    if (!extract_container_id(path).empty()) {
      return path;
    }
  }
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

// A cgroup filesystem as the inspected process sees it.
struct CgroupMount {
  std::string mount_point;
  std::string mount_root;  // the subtree of the hierarchy this mount exposes
  std::string options;     // super options, which name the v1 controllers
  bool unified{false};
};

// cgroup v1 does not require a controller to be mounted at a directory named
// after it, and co-mounts may list controllers in any order, so the mount
// points have to be read rather than guessed.
std::vector<CgroupMount> read_cgroup_mounts(pid_t pid, const std::string & root) {
  std::vector<CgroupMount> mounts;
  std::ifstream f(root + "/proc/" + std::to_string(pid) + "/mountinfo");
  std::string line;
  while (std::getline(f, line)) {
    auto separator = line.find(" - ");
    if (separator == std::string::npos) {
      continue;
    }
    std::istringstream head(line.substr(0, separator));
    std::string field;
    std::string mount_root;
    std::string mount_point;
    if (!(head >> field >> field >> field >> mount_root >> mount_point)) {
      continue;
    }
    std::istringstream tail(line.substr(separator + 3));
    std::string fstype;
    std::string source;
    std::string options;
    if (!(tail >> fstype >> source >> options)) {
      continue;
    }
    if (fstype != "cgroup" && fstype != "cgroup2") {
      continue;
    }
    mounts.push_back({root + mount_point, mount_root, options, fstype == "cgroup2"});
  }
  return mounts;
}

bool mount_carries_controller(const CgroupMount & mount, const std::string & controller) {
  // Super options are comma separated and include the controller names.
  size_t start = 0;
  while (start <= mount.options.size()) {
    auto comma = mount.options.find(',', start);
    auto name = comma == std::string::npos ? mount.options.substr(start) : mount.options.substr(start, comma - start);
    if (name == controller) {
      return true;
    }
    if (comma == std::string::npos) {
      break;
    }
    start = comma + 1;
  }
  return false;
}

// Where a cgroup path is visible under a mount that exposes only part of the
// hierarchy: the exposed subtree is stripped off the front of the path.
void append_mount_dirs(std::vector<std::string> & dirs, const CgroupMount & mount, const std::string & cgroup_path) {
  if (mount.mount_root != "/" && !mount.mount_root.empty() && cgroup_path.rfind(mount.mount_root, 0) == 0) {
    auto relative = cgroup_path.substr(mount.mount_root.size());
    // The prefix has to end on a component boundary, or /docker/parent matches
    // /docker/parent2/child and names a cgroup this mount does not expose.
    if (relative.empty() || relative.front() == '/') {
      dirs.push_back(relative.empty() ? mount.mount_point : mount.mount_point + relative);
    }
  }
  if (!cgroup_path.empty() && cgroup_path != "/") {
    dirs.push_back(mount.mount_point + cgroup_path);
  }
  dirs.push_back(mount.mount_point);
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

// Keeps the answer from the most specific candidate that had a file at all.
// Candidates are visited most specific first, so a location with nothing there
// falls through to the next one, while a location that HAS the file decides -
// including when what it holds cannot be parsed. Letting an unlimited reading
// from a fallback location override an unreadable one from the process's own
// cgroup would report "no limit" for a limit nobody managed to read, which is
// the confusion this reader exists to remove.
template <typename T>
CgroupLimit<T> better_of(const CgroupLimit<T> & first, const CgroupLimit<T> & second) {
  return first.state() == LimitState::kUnavailable ? second : first;
}

// The CPU limit is the quota together with the period it is measured over; the
// period alone says nothing, so the two travel as one reading.
struct CpuLimit {
  CgroupLimit<int64_t> quota;
  std::optional<int64_t> period_us;
};

CpuLimit better_of(const CpuLimit & first, const CpuLimit & second) {
  return first.quota.state() == LimitState::kUnavailable ? second : first;
}

// Merges the answers from two different hierarchies. A controller is mounted on
// the unified hierarchy or on a legacy one, never both, so the hierarchy that
// does not carry it reports no limit rather than nothing at all. Preferring a
// real limit keeps that empty answer from deciding.
template <typename T>
CgroupLimit<T> across_hierarchies(const CgroupLimit<T> & first, const CgroupLimit<T> & second) {
  if (second.state() == LimitState::kLimited && first.state() != LimitState::kLimited) {
    return second;
  }
  return first.state() == LimitState::kUnavailable ? second : first;
}

CpuLimit across_hierarchies(const CpuLimit & first, const CpuLimit & second) {
  if (second.quota.state() == LimitState::kLimited && first.quota.state() != LimitState::kLimited) {
    return second;
  }
  return first.quota.state() == LimitState::kUnavailable ? second : first;
}

CgroupLimit<uint64_t> read_memory_v2(const std::string & dir) {
  auto file = read_limit_file(dir + "/memory.max");
  if (!file.exists) {
    return CgroupLimit<uint64_t>::unavailable();
  }
  if (!file.opened) {
    return CgroupLimit<uint64_t>::unreadable();
  }
  auto text = trim(file.line);
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
  auto file = read_limit_file(dir + "/memory.limit_in_bytes");
  if (!file.exists) {
    return CgroupLimit<uint64_t>::unavailable();
  }
  if (!file.opened) {
    return CgroupLimit<uint64_t>::unreadable();
  }
  auto value = parse_u64(trim(file.line));
  if (!value) {
    return CgroupLimit<uint64_t>::unreadable();
  }
  if (*value >= kV1MemoryUnlimited) {
    return CgroupLimit<uint64_t>::unlimited();
  }
  return CgroupLimit<uint64_t>::limited(*value);
}

CpuLimit read_cpu_v2(const std::string & dir) {
  auto file = read_limit_file(dir + "/cpu.max");
  if (!file.exists) {
    return {CgroupLimit<int64_t>::unavailable(), std::nullopt};
  }
  if (!file.opened) {
    return {CgroupLimit<int64_t>::unreadable(), std::nullopt};
  }

  // The format is exactly two fields, "<quota> <period>", where the quota may
  // be the keyword "max". Anything after them is not this file.
  std::istringstream ss(file.line);
  std::string quota_text;
  std::string period_text;
  std::string trailing;
  if (!(ss >> quota_text) || !(ss >> period_text) || (ss >> trailing)) {
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
  auto quota_file = read_limit_file(dir + "/cpu.cfs_quota_us");
  auto period_file = read_limit_file(dir + "/cpu.cfs_period_us");
  if (!quota_file.exists && !period_file.exists) {
    return {CgroupLimit<int64_t>::unavailable(), std::nullopt};
  }

  std::optional<int64_t> period;
  if (period_file.opened) {
    period = parse_i64(trim(period_file.line));
    if (period && *period <= 0) {
      period.reset();
    }
  }
  if (!quota_file.opened || !period) {
    return {CgroupLimit<int64_t>::unreadable(), std::nullopt};
  }

  auto quota = parse_i64(trim(quota_file.line));
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
  auto cgroup_path = select_reported_path(paths);
  if (!extract_container_id(cgroup_path).empty()) {
    return true;
  }
  return detect_container_marker(pid, root, cgroup_path).present;
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
    auto marker = detect_container_marker(pid, root, cgroup_path);
    info.containerized = marker.present;
    if (info.container_runtime.empty()) {
      info.container_runtime = marker.runtime;
    }
  }

  // The conventional mount points come first because they are the ones a
  // gateway can reach when it does not share the target's mount namespace.
  // The mounts read from the target's own mountinfo are tried after them, so a
  // host that puts its controllers somewhere else is still covered and a
  // cross-namespace path that resolves to nothing here simply does not match.
  const auto mounts = read_cgroup_mounts(pid, root);

  CgroupLimit<uint64_t> memory_v2;
  CgroupLimit<uint64_t> memory_v1;
  CpuLimit cpu_v2;
  CpuLimit cpu_v1;

  if (!paths.unified.empty()) {
    std::vector<std::string> dirs;
    for (const auto & mount : unified_mounts(root)) {
      for (const auto & dir : candidate_dirs(mount, paths.unified)) {
        dirs.push_back(dir);
      }
    }
    for (const auto & mount : mounts) {
      if (mount.unified) {
        append_mount_dirs(dirs, mount, paths.unified);
      }
    }
    for (const auto & dir : dirs) {
      memory_v2 = better_of(memory_v2, read_memory_v2(dir));
      cpu_v2 = better_of(cpu_v2, read_cpu_v2(dir));
    }
  }

  if (auto memory_path = controller_path(paths, "memory")) {
    std::vector<std::string> dirs;
    for (const auto & mount : v1_mounts(root, "memory")) {
      for (const auto & dir : candidate_dirs(mount, *memory_path)) {
        dirs.push_back(dir);
      }
    }
    for (const auto & mount : mounts) {
      if (!mount.unified && mount_carries_controller(mount, "memory")) {
        append_mount_dirs(dirs, mount, *memory_path);
      }
    }
    for (const auto & dir : dirs) {
      memory_v1 = better_of(memory_v1, read_memory_v1(dir));
    }
  }

  if (auto cpu_path = controller_path(paths, "cpu")) {
    std::vector<std::string> dirs;
    for (const auto & mount : v1_mounts(root, "cpu")) {
      for (const auto & dir : candidate_dirs(mount, *cpu_path)) {
        dirs.push_back(dir);
      }
    }
    for (const auto & mount : mounts) {
      if (!mount.unified && mount_carries_controller(mount, "cpu")) {
        append_mount_dirs(dirs, mount, *cpu_path);
      }
    }
    for (const auto & dir : dirs) {
      cpu_v1 = better_of(cpu_v1, read_cpu_v1(dir));
    }
  }

  info.memory_limit = across_hierarchies(memory_v2, memory_v1);
  auto cpu = across_hierarchies(cpu_v2, cpu_v1);
  info.cpu_quota = cpu.quota;
  info.cpu_period_us = cpu.period_us;
  return info;
}

}  // namespace ros2_medkit_linux_introspection
