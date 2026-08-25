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

#include "ros2_medkit_linux_introspection/proc_reader.hpp"

#include <algorithm>
#include <cinttypes>
#include <cstdlib>
#include <cstring>
#include <dirent.h>
#include <filesystem>
#include <fstream>
#include <memory>
#include <mutex>
#include <sstream>
#include <sys/stat.h>
#include <unistd.h>
#include <vector>

namespace fs = std::filesystem;

namespace ros2_medkit_linux_introspection {

namespace {

struct DirCloser {
  void operator()(DIR * d) const {
    if (d) {
      closedir(d);
    }
  }
};
using UniqueDir = std::unique_ptr<DIR, DirCloser>;

std::string read_file_contents(const std::string & path) {
  std::ifstream f(path);
  if (!f.is_open()) {
    return {};
  }
  // Read through the stream buffer rather than a pair of istreambuf_iterators:
  // the iterator form makes GCC lose track of the buffer pointers and report a
  // null dereference in <streambuf> that cannot happen on an open stream.
  std::ostringstream contents;
  contents << f.rdbuf();
  return contents.str();
}

// Parse VmRSS and VmSize from /proc/{pid}/status
void parse_status_file(const std::string & path, uint64_t & rss_bytes, uint64_t & vm_size_bytes) {
  std::ifstream f(path);
  std::string line;
  while (std::getline(f, line)) {
    if (line.rfind("VmRSS:", 0) == 0) {
      uint64_t val = 0;
      if (sscanf(line.c_str(), "VmRSS: %" SCNu64, &val) == 1) {  // NOLINT(runtime/printf)
        rss_bytes = val * 1024;                                  // kB to bytes
      }
    } else if (line.rfind("VmSize:", 0) == 0) {
      uint64_t val = 0;
      if (sscanf(line.c_str(), "VmSize: %" SCNu64, &val) == 1) {  // NOLINT(runtime/printf)
        vm_size_bytes = val * 1024;
      }
    }
  }
}

// Parse cmdline from /proc/{pid}/cmdline (null-separated arguments)
std::string parse_cmdline(const std::string & path) {
  auto content = read_file_contents(path);
  // Replace null bytes with spaces for display
  std::replace(content.begin(), content.end(), '\0', ' ');
  // Trim trailing space
  if (!content.empty() && content.back() == ' ') {
    content.pop_back();
  }
  return content;
}

}  // namespace

// Parse node name and namespace from /proc/{pid}/cmdline
// Not in anonymous namespace - used by both find_pid_for_node and PidCache::refresh
static bool parse_ros_args(const std::string & cmdline_path, std::string & node_name, std::string & node_namespace) {
  auto content = read_file_contents(cmdline_path);
  // cmdline is null-separated
  std::vector<std::string> args;
  std::string current;
  for (char c : content) {
    if (c == '\0') {
      if (!current.empty()) {
        args.push_back(std::move(current));
      }
      current.clear();
    } else {
      current += c;
    }
  }
  if (!current.empty()) {
    args.push_back(std::move(current));
  }

  for (const auto & arg : args) {
    if (arg.rfind("__node:=", 0) == 0) {
      node_name = arg.substr(8);
    } else if (arg.rfind("__ns:=", 0) == 0) {
      node_namespace = arg.substr(6);
    }
  }
  return !node_name.empty();
}

tl::expected<ProcessInfo, std::string> read_process_info(pid_t pid, const std::string & root) {
  auto proc_dir = root + "/proc/" + std::to_string(pid);

  if (!fs::exists(proc_dir)) {
    return tl::make_unexpected("Process " + std::to_string(pid) + " not found");
  }

  ProcessInfo info;
  info.pid = pid;

  // Parse /proc/{pid}/stat
  auto stat_content = read_file_contents(proc_dir + "/stat");
  if (stat_content.empty()) {
    return tl::make_unexpected("Cannot read " + proc_dir + "/stat");
  }

  // Find the closing paren of (comm) to skip process names with spaces
  auto comm_end = stat_content.rfind(')');
  if (comm_end == std::string::npos || comm_end + 2 >= stat_content.size()) {
    return tl::make_unexpected("Malformed stat file for PID " + std::to_string(pid));
  }

  // Fields after (comm): state ppid pgrp session tty_nr tpgid flags
  //   minflt cminflt majflt cmajflt utime stime cutime cstime priority nice
  //   num_threads itrealvalue starttime vsize rss ...
  std::istringstream ss(stat_content.substr(comm_end + 2));
  std::string state;
  int64_t ppid = 0;
  int64_t pgrp = 0;
  int64_t session = 0;
  int64_t tty_nr = 0;
  int64_t tpgid = 0;
  uint64_t flags = 0;
  uint64_t minflt = 0;
  uint64_t cminflt = 0;
  uint64_t majflt = 0;
  uint64_t cmajflt = 0;
  uint64_t utime = 0;
  uint64_t stime = 0;
  uint64_t cutime = 0;
  uint64_t cstime = 0;
  int64_t priority = 0;
  int64_t nice = 0;
  uint32_t num_threads = 0;
  int64_t itrealvalue = 0;
  uint64_t starttime = 0;

  ss >> state >> ppid >> pgrp >> session >> tty_nr >> tpgid >> flags;
  ss >> minflt >> cminflt >> majflt >> cmajflt;
  ss >> utime >> stime >> cutime >> cstime;
  ss >> priority >> nice >> num_threads >> itrealvalue >> starttime;

  if (ss.fail()) {
    return tl::make_unexpected("Truncated stat file for PID " + std::to_string(pid));
  }

  // Suppress unused variable warnings for positional fields we don't need
  (void)pgrp;
  (void)session;
  (void)tty_nr;
  (void)tpgid;
  (void)flags;
  (void)minflt;
  (void)cminflt;
  (void)majflt;
  (void)cmajflt;
  (void)cutime;
  (void)cstime;
  (void)priority;
  (void)nice;
  (void)itrealvalue;

  info.ppid = static_cast<pid_t>(ppid);
  info.state = state;
  info.cpu_user_ticks = utime;
  info.cpu_system_ticks = stime;
  info.num_threads = num_threads;
  info.start_time_ticks = starttime;

  // Parse /proc/{pid}/status for VmRSS, VmSize
  parse_status_file(proc_dir + "/status", info.rss_bytes, info.vm_size_bytes);

  // Parse /proc/{pid}/cmdline
  info.cmdline = parse_cmdline(proc_dir + "/cmdline");

  // Read /proc/{pid}/exe symlink
  std::error_code ec;
  auto exe = fs::read_symlink(proc_dir + "/exe", ec);
  if (!ec) {
    info.exe_path = exe.string();
  }

  return info;
}

tl::expected<pid_t, std::string> find_pid_for_node(const std::string & node_name, const std::string & node_namespace,
                                                   const std::string & root) {
  auto proc_dir = root + "/proc";
  UniqueDir dir(opendir(proc_dir.c_str()));
  if (!dir) {
    return tl::make_unexpected("Cannot open " + proc_dir);
  }

  struct dirent * entry = nullptr;
  while ((entry = readdir(dir.get())) != nullptr) {
    if (entry->d_type != DT_DIR && entry->d_type != DT_UNKNOWN) {
      continue;
    }
    if (entry->d_type == DT_UNKNOWN) {
      struct stat st {};
      auto full_path = proc_dir + "/" + entry->d_name;
      if (stat(full_path.c_str(), &st) != 0 || !S_ISDIR(st.st_mode)) {
        continue;
      }
    }
    bool all_digits = true;
    for (const char * p = entry->d_name; *p; ++p) {
      if (*p < '0' || *p > '9') {
        all_digits = false;
        break;
      }
    }
    if (!all_digits) {
      continue;
    }

    auto cmdline_path = proc_dir + "/" + entry->d_name + "/cmdline";
    std::string found_node;
    std::string found_ns;
    if (parse_ros_args(cmdline_path, found_node, found_ns)) {
      if (found_node == node_name && (node_namespace.empty() || found_ns == node_namespace)) {
        char * end = nullptr;
        long pid_val = std::strtol(entry->d_name, &end, 10);
        if (end == entry->d_name || *end != '\0' || pid_val <= 0) {
          continue;
        }
        return static_cast<pid_t>(pid_val);
      }
    }
  }

  return tl::make_unexpected("No process found for node " + node_namespace + "/" + node_name);
}

tl::expected<double, std::string> read_system_uptime(const std::string & root) {
  auto path = root + "/proc/uptime";
  std::ifstream f(path);
  if (!f.is_open()) {
    return tl::make_unexpected("Cannot open " + path);
  }
  double uptime = 0.0;
  if (!(f >> uptime)) {
    return tl::make_unexpected("Cannot parse " + path);
  }
  return uptime;
}

// --- PidCache implementation ---

PidCache::PidCache(std::chrono::steady_clock::duration ttl) : ttl_(ttl) {
}

std::optional<pid_t> PidCache::lookup(const std::string & node_fqn, const std::string & root) {
  {
    std::shared_lock lock(mutex_);
    auto now = std::chrono::steady_clock::now();
    if (now - last_refresh_ < ttl_) {
      auto it = node_to_pid_.find(node_fqn);
      if (it != node_to_pid_.end()) {
        return it->second;
      }
      return std::nullopt;
    }
  }
  // TTL expired - refresh
  refresh(root);

  std::shared_lock lock(mutex_);
  auto it = node_to_pid_.find(node_fqn);
  if (it != node_to_pid_.end()) {
    return it->second;
  }
  return std::nullopt;
}

void PidCache::refresh(const std::string & root) {
  std::unique_lock lock(mutex_);

  // Double-check TTL under exclusive lock to avoid redundant scans
  auto now = std::chrono::steady_clock::now();
  if (now - last_refresh_ < ttl_) {
    return;
  }

  node_to_pid_.clear();
  auto proc_dir = root + "/proc";
  UniqueDir dir(opendir(proc_dir.c_str()));
  if (!dir) {
    last_refresh_ = std::chrono::steady_clock::now();
    return;
  }

  struct dirent * entry = nullptr;
  while ((entry = readdir(dir.get())) != nullptr) {
    if (entry->d_type != DT_DIR && entry->d_type != DT_UNKNOWN) {
      continue;
    }
    if (entry->d_type == DT_UNKNOWN) {
      struct stat st {};
      auto full_path = proc_dir + "/" + entry->d_name;
      if (stat(full_path.c_str(), &st) != 0 || !S_ISDIR(st.st_mode)) {
        continue;
      }
    }
    bool all_digits = true;
    for (const char * p = entry->d_name; *p; ++p) {
      if (*p < '0' || *p > '9') {
        all_digits = false;
        break;
      }
    }
    if (!all_digits) {
      continue;
    }

    auto cmdline_path = proc_dir + "/" + entry->d_name + "/cmdline";
    std::string node_name;
    std::string node_ns;
    if (parse_ros_args(cmdline_path, node_name, node_ns)) {
      std::string fqn;
      if (node_ns != "/" && !node_ns.empty()) {
        fqn = node_ns;
      }
      fqn += "/";
      fqn += node_name;
      char * end = nullptr;
      long pid_val = std::strtol(entry->d_name, &end, 10);
      if (end != entry->d_name && *end == '\0' && pid_val > 0) {
        node_to_pid_[fqn] = static_cast<pid_t>(pid_val);
      }
    }
  }
  last_refresh_ = std::chrono::steady_clock::now();
}

size_t PidCache::size() const {
  std::shared_lock lock(mutex_);
  return node_to_pid_.size();
}

}  // namespace ros2_medkit_linux_introspection
