// Copyright 2025 mfaferek93
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

#include "ros2_medkit_fault_manager/fault_storage.hpp"

#include <algorithm>
#include <filesystem>

namespace ros2_medkit_fault_manager {

int32_t clamp_debounce_counter(int32_t counter, const DebounceConfig & config) {
  // Manual min/max rather than std::clamp: well-defined even if a bad config has lo > hi
  // (sanitize_debounce_config normally prevents that, but the storage layer must never UB).
  if (counter < config.confirmation_threshold) {
    return config.confirmation_threshold;
  }
  if (counter > config.healing_threshold) {
    return config.healing_threshold;
  }
  return counter;
}

std::string compute_debounce_status(int32_t counter, const std::string & current_status,
                                    const DebounceConfig & config) {
  namespace msg = ros2_medkit_msgs::msg;
  if (counter <= config.confirmation_threshold) {
    return msg::Fault::STATUS_CONFIRMED;
  }
  if (config.healing_enabled && counter >= config.healing_threshold) {
    return msg::Fault::STATUS_HEALED;
  }
  // Hysteresis latch: a confirmed or healed fault stays put until the counter reaches the opposite
  // threshold (handled above). A single opposite-direction report must not flip it to a pending state.
  if (current_status == msg::Fault::STATUS_CONFIRMED || current_status == msg::Fault::STATUS_HEALED) {
    return current_status;
  }
  if (counter < 0) {
    return msg::Fault::STATUS_PREFAILED;
  }
  if (counter > 0) {
    return msg::Fault::STATUS_PREPASSED;
  }
  return current_status;  // counter == 0 keeps the current status (avoids flapping at the boundary)
}

bool sanitize_debounce_config(DebounceConfig & config) {
  bool valid = true;
  if (config.confirmation_threshold >= 0) {
    config.confirmation_threshold = -1;
    valid = false;
  }
  // healing_threshold == 0 is valid: it means "heal on a single PASSED event" (the counter reaches
  // the threshold at 0). Only a negative threshold is rejected.
  if (config.healing_threshold < 0) {
    config.healing_threshold = 3;
    valid = false;
  }
  return valid;
}

ros2_medkit_msgs::msg::Fault FaultState::to_msg() const {
  ros2_medkit_msgs::msg::Fault msg;
  msg.fault_code = fault_code;
  msg.severity = severity;
  msg.description = description;
  msg.first_occurred = first_occurred;
  msg.last_occurred = last_occurred;
  msg.last_passed = last_passed_time;
  msg.occurrence_count = occurrence_count;
  msg.status = status;

  // Convert set to vector
  msg.reporting_sources.reserve(reporting_sources.size());
  for (const auto & source : reporting_sources) {
    msg.reporting_sources.push_back(source);
  }

  return msg;
}

void InMemoryFaultStorage::set_debounce_config(const DebounceConfig & config) {
  std::lock_guard<std::mutex> lock(mutex_);
  config_ = config;
}

DebounceConfig InMemoryFaultStorage::get_debounce_config() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return config_;
}

void InMemoryFaultStorage::update_status(FaultState & state, const DebounceConfig & config) {
  // CLEARED faults are handled in report_fault_event() before this is called.
  // Delegate to the shared state machine so both backends behave identically.
  state.status = compute_debounce_status(state.debounce_counter, state.status, config);
}

bool InMemoryFaultStorage::report_fault_event(const std::string & fault_code, uint8_t event_type, uint8_t severity,
                                              const std::string & description, const std::string & source_id,
                                              const rclcpp::Time & timestamp, const DebounceConfig & config) {
  std::lock_guard<std::mutex> lock(mutex_);

  const bool is_failed = (event_type == EventType::EVENT_FAILED);

  auto it = faults_.find(fault_code);
  if (it == faults_.end()) {
    // New fault - only create entry for FAILED events
    if (!is_failed) {
      return false;  // PASSED event for non-existent fault is ignored
    }

    FaultState state;
    state.fault_code = fault_code;
    state.severity = severity;
    state.description = description;
    state.first_occurred = timestamp;
    state.last_occurred = timestamp;
    state.last_failed_time = timestamp;
    state.occurrence_count = 1;
    state.debounce_counter = -1;  // First FAILED event
    state.reporting_sources.insert(source_id);

    // CRITICAL severity bypasses debounce and confirms immediately
    if (config.critical_immediate_confirm && severity == ros2_medkit_msgs::msg::Fault::SEVERITY_CRITICAL) {
      state.status = ros2_medkit_msgs::msg::Fault::STATUS_CONFIRMED;
    } else {
      // Set status based on debounce counter vs threshold
      update_status(state, config);
    }

    faults_.emplace(fault_code, std::move(state));
    return true;
  }

  // Existing fault - update
  auto & state = it->second;

  // CLEARED faults can be reactivated by FAILED events
  if (state.status == ros2_medkit_msgs::msg::Fault::STATUS_CLEARED) {
    if (!is_failed) {
      // PASSED events for CLEARED faults are ignored
      return false;
    }
    // FAILED event reactivates the fault - reset debounce counter and first-seen:
    // this is a new outage cycle, not a continuation of the one that just cleared.
    state.debounce_counter = -1;
    state.first_occurred = timestamp;
    state.last_failed_time = timestamp;
    state.last_occurred = timestamp;
    state.reporting_sources.insert(source_id);
    if (state.occurrence_count < std::numeric_limits<uint32_t>::max()) {
      ++state.occurrence_count;
    }
    if (severity > state.severity) {
      state.severity = severity;
    }
    if (!description.empty()) {
      state.description = description;
    }
    // Check for immediate CRITICAL confirmation
    if (config.critical_immediate_confirm && severity == ros2_medkit_msgs::msg::Fault::SEVERITY_CRITICAL) {
      state.status = ros2_medkit_msgs::msg::Fault::STATUS_CONFIRMED;
    } else {
      update_status(state, config);
    }
    return true;  // Reactivation treated as new occurrence for event publishing
  }

  if (is_failed) {
    // last_occurred tracks occurrences only. A PASSED event is the fault ENDING, not
    // occurring; bumping it there makes a long-stale CONFIRMED fault look freshly
    // active to operators. The PASSED instant is kept in last_passed_time.
    state.last_occurred = timestamp;
    state.last_failed_time = timestamp;

    // occurrence_count is NOT bumped here: this is a still-active fault being
    // re-reported (level-triggered poller, or debounce building toward
    // confirmation), the same continuous occurrence. It only increments on a
    // genuine edge - new fault (above) or reactivation after CLEARED (above).

    // Decrement towards confirmation, clamped to the thresholds (a long FAILED burst can't
    // run the counter off to INT32_MIN and delay later healing).
    state.debounce_counter = clamp_debounce_counter(state.debounce_counter - 1, config);

    // Add source if not already present
    state.reporting_sources.insert(source_id);

    // Update severity if higher
    if (severity > state.severity) {
      state.severity = severity;
    }

    // Update description if provided
    if (!description.empty()) {
      state.description = description;
    }

    // Check for immediate confirmation of CRITICAL
    if (config.critical_immediate_confirm && severity == ros2_medkit_msgs::msg::Fault::SEVERITY_CRITICAL) {
      state.status = ros2_medkit_msgs::msg::Fault::STATUS_CONFIRMED;
      return false;
    }
  } else {
    // PASSED event
    state.last_passed_time = timestamp;

    // Increment towards healing, clamped to the thresholds (a heal heartbeat can't run the
    // counter off to INT32_MAX and then delay a real fault from confirming).
    state.debounce_counter = clamp_debounce_counter(state.debounce_counter + 1, config);
  }

  // Update status based on debounce counter
  update_status(state, config);

  return false;
}

std::vector<ros2_medkit_msgs::msg::Fault>
InMemoryFaultStorage::list_faults(bool filter_by_severity, uint8_t severity,
                                  const std::vector<std::string> & statuses) const {
  std::lock_guard<std::mutex> lock(mutex_);

  // Determine which statuses to include
  std::set<std::string> status_filter;
  if (statuses.empty()) {
    // Default: only CONFIRMED faults (excludes PREFAILED, CLEARED, HEALED)
    status_filter.insert(ros2_medkit_msgs::msg::Fault::STATUS_CONFIRMED);
  } else {
    for (const auto & s : statuses) {
      // Only add valid statuses (invalid ones are silently ignored)
      if (s == ros2_medkit_msgs::msg::Fault::STATUS_PREFAILED || s == ros2_medkit_msgs::msg::Fault::STATUS_PREPASSED ||
          s == ros2_medkit_msgs::msg::Fault::STATUS_CONFIRMED || s == ros2_medkit_msgs::msg::Fault::STATUS_HEALED ||
          s == ros2_medkit_msgs::msg::Fault::STATUS_CLEARED) {
        status_filter.insert(s);
      }
    }
    // If all provided statuses were invalid, default to CONFIRMED for consistency
    if (status_filter.empty()) {
      status_filter.insert(ros2_medkit_msgs::msg::Fault::STATUS_CONFIRMED);
    }
  }

  std::vector<ros2_medkit_msgs::msg::Fault> result;
  result.reserve(faults_.size());

  for (const auto & [code, state] : faults_) {
    // Filter by status
    if (status_filter.find(state.status) == status_filter.end()) {
      continue;
    }

    // Filter by severity if requested
    if (filter_by_severity && state.severity != severity) {
      continue;
    }

    result.push_back(state.to_msg());
  }

  return result;
}

std::optional<ros2_medkit_msgs::msg::Fault> InMemoryFaultStorage::get_fault(const std::string & fault_code) const {
  std::lock_guard<std::mutex> lock(mutex_);

  auto it = faults_.find(fault_code);
  if (it == faults_.end()) {
    return std::nullopt;
  }

  return it->second.to_msg();
}

bool InMemoryFaultStorage::clear_fault(const std::string & fault_code) {
  std::lock_guard<std::mutex> lock(mutex_);

  auto it = faults_.find(fault_code);
  if (it == faults_.end()) {
    return false;
  }

  // Delete associated snapshots when fault is cleared
  snapshots_.erase(std::remove_if(snapshots_.begin(), snapshots_.end(),
                                  [&fault_code](const SnapshotData & s) {
                                    return s.fault_code == fault_code;
                                  }),
                   snapshots_.end());

  it->second.status = ros2_medkit_msgs::msg::Fault::STATUS_CLEARED;
  return true;
}

size_t InMemoryFaultStorage::size() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return faults_.size();
}

bool InMemoryFaultStorage::contains(const std::string & fault_code) const {
  std::lock_guard<std::mutex> lock(mutex_);
  return faults_.find(fault_code) != faults_.end();
}

std::vector<std::string> InMemoryFaultStorage::check_time_based_confirmation(const rclcpp::Time & current_time) {
  std::lock_guard<std::mutex> lock(mutex_);

  std::vector<std::string> confirmed;
  if (config_.auto_confirm_after_sec <= 0.0) {
    return confirmed;  // Time-based confirmation disabled
  }

  const double threshold_ns = config_.auto_confirm_after_sec * 1e9;

  for (auto & [code, state] : faults_) {
    if (state.status == ros2_medkit_msgs::msg::Fault::STATUS_PREFAILED) {
      const int64_t age_ns = (current_time - state.last_failed_time).nanoseconds();
      if (static_cast<double>(age_ns) >= threshold_ns) {
        state.status = ros2_medkit_msgs::msg::Fault::STATUS_CONFIRMED;
        confirmed.push_back(code);
      }
    }
  }

  return confirmed;
}

void InMemoryFaultStorage::set_max_snapshots_per_fault(size_t max_count) {
  std::lock_guard<std::mutex> lock(mutex_);
  max_snapshots_per_fault_ = max_count;
}

void InMemoryFaultStorage::store_snapshot(const SnapshotData & snapshot) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (max_snapshots_per_fault_ > 0) {
    size_t count = 0;
    for (const auto & s : snapshots_) {
      if (s.fault_code == snapshot.fault_code) {
        ++count;
      }
    }
    if (count >= max_snapshots_per_fault_) {
      // Silent rejection: storage layer has no logger. Callers should log if needed.
      return;  // Reject new - keep first N inserted snapshots
    }
  }
  snapshots_.push_back(snapshot);
}

std::vector<SnapshotData> InMemoryFaultStorage::get_snapshots(const std::string & fault_code,
                                                              const std::string & topic_filter) const {
  std::lock_guard<std::mutex> lock(mutex_);

  std::vector<SnapshotData> result;
  for (const auto & snapshot : snapshots_) {
    if (snapshot.fault_code == fault_code) {
      if (topic_filter.empty() || snapshot.topic == topic_filter) {
        result.push_back(snapshot);
      }
    }
  }
  return result;
}

void InMemoryFaultStorage::store_freeze_frame(const FreezeFrameData & frame) {
  std::lock_guard<std::mutex> lock(mutex_);
  freeze_frames_[frame.fault_code] = frame;
}

std::optional<FreezeFrameData> InMemoryFaultStorage::get_freeze_frame(const std::string & fault_code) const {
  std::lock_guard<std::mutex> lock(mutex_);
  auto it = freeze_frames_.find(fault_code);
  if (it == freeze_frames_.end()) {
    return std::nullopt;
  }
  return it->second;
}

void InMemoryFaultStorage::store_rosbag_file(const RosbagFileInfo & info) {
  std::lock_guard<std::mutex> lock(mutex_);

  // Delete existing bag file if present (prevent orphaned files on re-confirm)
  auto it = rosbag_files_.find(info.fault_code);
  if (it != rosbag_files_.end()) {
    if (it->second.file_path != info.file_path &&
        !path_shared_with_other_fault(it->second.file_path, info.fault_code)) {
      std::error_code ec;
      std::filesystem::remove_all(it->second.file_path, ec);
      // Ignore errors - file may already be deleted
    }
  }

  rosbag_files_[info.fault_code] = info;
}

std::optional<RosbagFileInfo> InMemoryFaultStorage::get_rosbag_file(const std::string & fault_code) const {
  std::lock_guard<std::mutex> lock(mutex_);

  auto it = rosbag_files_.find(fault_code);
  if (it == rosbag_files_.end()) {
    return std::nullopt;
  }
  return it->second;
}

bool InMemoryFaultStorage::delete_rosbag_file(const std::string & fault_code) {
  std::lock_guard<std::mutex> lock(mutex_);

  auto it = rosbag_files_.find(fault_code);
  if (it == rosbag_files_.end()) {
    return false;
  }

  const std::string file_path = it->second.file_path;
  rosbag_files_.erase(it);

  // Try to delete the actual file, unless a sibling fault still points at it
  if (!path_shared_with_other_fault(file_path, fault_code)) {
    std::error_code ec;
    std::filesystem::remove_all(file_path, ec);
    // Ignore errors - file may already be deleted
  }
  return true;
}

bool InMemoryFaultStorage::path_shared_with_other_fault(const std::string & file_path,
                                                        const std::string & fault_code) const {
  return std::any_of(rosbag_files_.begin(), rosbag_files_.end(), [&](const auto & entry) {
    return entry.first != fault_code && entry.second.file_path == file_path;
  });
}

size_t InMemoryFaultStorage::get_total_rosbag_storage_bytes() const {
  std::lock_guard<std::mutex> lock(mutex_);

  // Sum per bag, not per fault: one recording can back a burst of correlated
  // faults, and double-counting it would evict bags that still fit the quota.
  // Rows sharing a path can disagree on size while a recording is being
  // finalised, so take the largest - matching the SQLite backend's MAX() and
  // never under-reporting what is on disk.
  std::map<std::string, size_t> bytes_per_path;
  for (const auto & [code, info] : rosbag_files_) {
    auto & bytes = bytes_per_path[info.file_path];
    bytes = std::max(bytes, info.size_bytes);
  }

  size_t total = 0;
  for (const auto & [path, bytes] : bytes_per_path) {
    total += bytes;
  }
  return total;
}

std::vector<RosbagFileInfo> InMemoryFaultStorage::get_all_rosbag_files() const {
  std::lock_guard<std::mutex> lock(mutex_);

  std::vector<RosbagFileInfo> result;
  result.reserve(rosbag_files_.size());
  for (const auto & [code, info] : rosbag_files_) {
    result.push_back(info);
  }

  // Sort by creation time (oldest first)
  std::sort(result.begin(), result.end(), [](const RosbagFileInfo & a, const RosbagFileInfo & b) {
    return a.created_at_ns < b.created_at_ns;
  });

  return result;
}

std::vector<RosbagFileInfo> InMemoryFaultStorage::list_rosbags_for_entity(const std::string & entity_fqn) const {
  std::lock_guard<std::mutex> lock(mutex_);

  std::vector<RosbagFileInfo> result;

  for (const auto & [fault_code, rosbag_info] : rosbag_files_) {
    // Check if any of the fault's reporting sources contain this entity
    auto fault_it = faults_.find(fault_code);
    if (fault_it != faults_.end()) {
      const auto & fault_state = fault_it->second;
      if (fault_state.reporting_sources.find(entity_fqn) != fault_state.reporting_sources.end()) {
        result.push_back(rosbag_info);
      }
    }
  }

  return result;
}

std::vector<ros2_medkit_msgs::msg::Fault> InMemoryFaultStorage::get_all_faults() const {
  std::lock_guard<std::mutex> lock(mutex_);

  std::vector<ros2_medkit_msgs::msg::Fault> result;
  result.reserve(faults_.size());

  for (const auto & [code, state] : faults_) {
    result.push_back(state.to_msg());
  }

  return result;
}

std::vector<std::string> InMemoryFaultStorage::reclassify_healed_as_cleared() {
  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<std::string> reclassified;
  for (auto & [code, state] : faults_) {
    if (state.status == ros2_medkit_msgs::msg::Fault::STATUS_HEALED) {
      state.status = ros2_medkit_msgs::msg::Fault::STATUS_CLEARED;
      reclassified.push_back(code);
    }
  }
  return reclassified;
}

}  // namespace ros2_medkit_fault_manager
