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
#include <set>
#include <tuple>

namespace ros2_medkit_fault_manager {

std::string rosbag_recording_id(const std::string & file_path) {
  std::filesystem::path p(file_path);
  if (!p.has_filename()) {
    p = p.parent_path();  // tolerate a trailing slash
  }
  return p.filename().string();
}

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

bool is_near_miss(bool is_failed_event, const std::string & resulting_status) {
  return is_failed_event && resulting_status != ros2_medkit_msgs::msg::Fault::STATUS_CONFIRMED;
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

    if (is_near_miss(true, state.status)) {
      record_near_miss(state, config, severity, source_id, timestamp);
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
    if (is_near_miss(true, state.status)) {
      record_near_miss(state, config, severity, source_id, timestamp);
    }
    return true;  // Reactivation treated as new occurrence for event publishing
  }

  // Bring a counter left outside this config's band back into range before applying the report.
  // Per-entity threshold overrides mean two sources of the same fault code can be evaluated
  // against different bands, so a stored value clamped to one source's ceiling can sit above
  // another's. The SQLite backend clamps on read for the same reason, and the two backends have to
  // agree on the counter they record and on the status it produces.
  state.debounce_counter = clamp_debounce_counter(state.debounce_counter, config);

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

  if (is_near_miss(is_failed, state.status)) {
    record_near_miss(state, config, severity, source_id, timestamp);
  }

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

  // Acknowledging a fault drops its value snapshots, unless a history was asked
  // for: with recordings retained past a clear, deleting the readings that go with
  // them leaves a fault holding bags whose matching values are gone.
  if (!retain_snapshots_on_clear_) {
    snapshots_.erase(std::remove_if(snapshots_.begin(), snapshots_.end(),
                                    [&fault_code](const SnapshotData & s) {
                                      return s.fault_code == fault_code;
                                    }),
                     snapshots_.end());
  }

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

void InMemoryFaultStorage::set_retain_snapshots_on_clear(bool retain) {
  std::lock_guard<std::mutex> lock(mutex_);
  retain_snapshots_on_clear_ = retain;
}

bool InMemoryFaultStorage::retains_snapshots_on_clear() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return retain_snapshots_on_clear_;
}

void InMemoryFaultStorage::store_snapshot(const SnapshotData & snapshot) {
  store_snapshots({snapshot});
}

void InMemoryFaultStorage::store_snapshots(const std::vector<SnapshotData> & snapshots) {
  if (snapshots.empty()) {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);

  const std::string fault_code = snapshots.front().fault_code;

  // Built beside the live vector and swapped in, the shape store_rosbag_files
  // uses: a capture is all-or-nothing, so a throw partway must not leave half of
  // it stored.
  auto updated = snapshots_;
  updated.insert(updated.end(), snapshots.begin(), snapshots.end());

  if (max_snapshots_per_fault_ > 0) {
    // Evict whole capture sets, oldest first, until this fault fits.
    //
    // The old rule counted rows and rejected the NEW row once full, so a capture
    // that straddled the cap was stored in part: some topics present, the rest
    // silently absent, indistinguishable from "that topic was not publishing".
    // Keep-newest also stops this cap from opposing the rosbag one.
    const auto rows_for_fault = [&updated, &fault_code]() {
      return static_cast<size_t>(std::count_if(updated.begin(), updated.end(), [&fault_code](const SnapshotData & s) {
        return s.fault_code == fault_code;
      }));
    };

    // One capture can exceed the cap on its own, so the newest set is exempt from
    // eviction: half a freeze frame reads exactly like "those topics were silent".
    // SqliteFaultStorage carries the same exemption as `AND capture_id <> ?2`, and
    // the two backends have to agree or the evidence depends on storage_type.
    int64_t newest = 0;
    bool have_newest = false;
    for (const auto & s : updated) {
      if (s.fault_code == fault_code && (!have_newest || s.capture_id > newest)) {
        newest = s.capture_id;
        have_newest = true;
      }
    }

    while (have_newest && rows_for_fault() > max_snapshots_per_fault_) {
      bool found = false;
      int64_t oldest = 0;
      for (const auto & s : updated) {
        if (s.fault_code == fault_code && s.capture_id != newest && (!found || s.capture_id < oldest)) {
          oldest = s.capture_id;
          found = true;
        }
      }
      if (!found) {
        break;  // only the newest capture is left and it is over the cap on its own
      }
      updated.erase(std::remove_if(updated.begin(), updated.end(),
                                   [&fault_code, oldest](const SnapshotData & s) {
                                     return s.fault_code == fault_code && s.capture_id == oldest;
                                   }),
                    updated.end());
    }
  }

  snapshots_.swap(updated);
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
  // Newest capture first, matching SqliteFaultStorage's ORDER BY. Insertion order
  // would put the OLDEST set first here, and the reader folds rows into a per-topic
  // map, so the values a client sees would depend on storage_type.
  std::stable_sort(result.begin(), result.end(), [](const SnapshotData & a, const SnapshotData & b) {
    return std::tie(b.capture_id, b.captured_at_ns) < std::tie(a.capture_id, a.captured_at_ns);
  });
  return result;
}

int64_t InMemoryFaultStorage::get_max_capture_id() const {
  std::lock_guard<std::mutex> lock(mutex_);

  int64_t max_id = 0;
  for (const auto & snapshot : snapshots_) {
    max_id = std::max(max_id, snapshot.capture_id);
  }
  return max_id;
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

void InMemoryFaultStorage::record_near_miss(const FaultState & state, const DebounceConfig & config, uint8_t severity,
                                            const std::string & source_id, const rclcpp::Time & timestamp) {
  auto & series = near_misses_[state.fault_code];

  NearMissRecord record;
  record.fault_code = state.fault_code;
  record.occurred_at_ns = timestamp.nanoseconds();
  record.debounce_counter = state.debounce_counter;
  record.confirmation_threshold = config.confirmation_threshold;
  record.severity = severity;
  record.source_id = source_id;
  record.resulting_status = state.status;
  series.push_back(std::move(record));

  // Evict oldest-first: a series frozen at boot says nothing about a trend.
  if (max_near_misses_per_fault_ > 0 && series.size() > max_near_misses_per_fault_) {
    using DiffType = std::vector<NearMissRecord>::difference_type;
    const auto excess = static_cast<DiffType>(series.size() - max_near_misses_per_fault_);
    series.erase(series.begin(), series.begin() + excess);
  }
}

size_t InMemoryFaultStorage::set_max_near_misses_per_fault(size_t max_count) {
  std::lock_guard<std::mutex> lock(mutex_);
  max_near_misses_per_fault_ = max_count;

  if (max_count == 0) {
    return 0;  // Unlimited
  }

  // Apply the bound to what is already stored, so a series built under a larger bound does not
  // stay over the new one until the next near miss for that code happens to arrive.
  using DiffType = std::vector<NearMissRecord>::difference_type;
  size_t evicted = 0;
  for (auto & [code, series] : near_misses_) {
    (void)code;
    if (series.size() > max_count) {
      const size_t excess = series.size() - max_count;
      series.erase(series.begin(), series.begin() + static_cast<DiffType>(excess));
      evicted += excess;
    }
  }
  return evicted;
}

std::vector<NearMissRecord> InMemoryFaultStorage::get_near_misses(const std::string & fault_code) const {
  std::lock_guard<std::mutex> lock(mutex_);
  auto it = near_misses_.find(fault_code);
  if (it == near_misses_.end()) {
    return {};
  }
  return it->second;
}

void InMemoryFaultStorage::set_max_rosbags_per_fault(size_t max_count) {
  std::lock_guard<std::mutex> lock(mutex_);
  max_rosbags_per_fault_ = max_count;
}

void InMemoryFaultStorage::store_rosbag_file(const RosbagFileInfo & info) {
  // Through the batch path deliberately: with a per-fault cap a single store is
  // insert + trim, and both must publish together.
  store_rosbag_files({info});
}

void InMemoryFaultStorage::store_rosbag_files(const std::vector<RosbagFileInfo> & infos) {
  if (infos.empty()) {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);

  // Built beside the live vector and swapped in, because swap cannot throw. The
  // caller reads a throw here as "no row was written" and removes the recording, so
  // a batch that stored half a burst and then threw would leave those rows naming a
  // bag that is gone. Copying costs one allocation per stored recording, not per
  // message. std::vector::swap is noexcept for the default allocator, exactly as
  // std::map::swap was.
  auto updated = rosbag_files_;
  uint64_t next_seq = rosbag_seq_;
  std::set<std::string> evicted;

  for (const auto & in : infos) {
    RosbagFileInfo row = in;
    if (row.recording_id.empty()) {
      row.recording_id = rosbag_recording_id(row.file_path);
    }

    // Upsert on the (fault, recording) LINK - the same grain as the SQLite unique
    // index. Re-storing the same link refreshes it; a link to a different recording
    // appends, which is the feature.
    auto it = std::find_if(updated.begin(), updated.end(), [&row](const RosbagRow & r) {
      return r.info.fault_code == row.fault_code && r.info.file_path == row.file_path;
    });
    if (it != updated.end()) {
      it->info = row;
    } else {
      updated.push_back(RosbagRow{row, ++next_seq});
    }

    if (max_rosbags_per_fault_ == 0) {
      continue;
    }

    // Keep the newest N recordings of this fault, oldest evicted first - the same
    // direction as evict_bags_over_quota, so the two eviction owners never need a
    // tiebreak. At N = 1 this is the pre-#620 behaviour exactly.
    std::vector<size_t> mine;
    for (size_t i = 0; i < updated.size(); ++i) {
      if (updated[i].info.fault_code == row.fault_code) {
        mine.push_back(i);
      }
    }
    if (mine.size() <= max_rosbags_per_fault_) {
      continue;
    }
    std::sort(mine.begin(), mine.end(), [&updated](size_t a, size_t b) {
      return std::tie(updated[a].info.created_at_ns, updated[a].seq) <
             std::tie(updated[b].info.created_at_ns, updated[b].seq);
    });
    std::vector<size_t> doomed(mine.begin(),
                               mine.begin() + static_cast<std::ptrdiff_t>(mine.size() - max_rosbags_per_fault_));
    std::sort(doomed.rbegin(), doomed.rend());  // descending, so erase stays valid
    for (size_t idx : doomed) {
      evicted.insert(updated[idx].info.file_path);
      updated.erase(updated.begin() + static_cast<std::ptrdiff_t>(idx));
    }
  }

  rosbag_files_.swap(updated);
  rosbag_seq_ = next_seq;

  // Unlinked only once every row is in, the way the SQLite backend unlinks after its
  // COMMIT: a throw above must leave the old rows pointing at bags that still exist.
  // Referencing is decided on the finished batch, not on the state before it - two
  // faults of one burst can share the bag being evicted, and checking row by row
  // beforehand would find it still held by a sibling that a later iteration then
  // evicts, leaking the directory.
  for (const auto & path : evicted) {
    if (path_referenced(path)) {
      continue;
    }
    std::error_code ec;
    std::filesystem::remove_all(path, ec);
    // Ignore errors - file may already be deleted
  }
}

std::optional<RosbagFileInfo> InMemoryFaultStorage::get_rosbag_file(const std::string & fault_code) const {
  std::lock_guard<std::mutex> lock(mutex_);

  // Newest, matching the SQLite backend's ORDER BY created_at_ns DESC, id DESC.
  const RosbagRow * best = nullptr;
  for (const auto & row : rosbag_files_) {
    if (row.info.fault_code != fault_code) {
      continue;
    }
    if (best == nullptr || std::tie(best->info.created_at_ns, best->seq) < std::tie(row.info.created_at_ns, row.seq)) {
      best = &row;
    }
  }
  if (best == nullptr) {
    return std::nullopt;
  }
  return best->info;
}

std::vector<RosbagFileInfo> InMemoryFaultStorage::get_rosbag_files(const std::string & fault_code) const {
  std::lock_guard<std::mutex> lock(mutex_);

  std::vector<const RosbagRow *> mine;
  for (const auto & row : rosbag_files_) {
    if (row.info.fault_code == fault_code) {
      mine.push_back(&row);
    }
  }
  std::sort(mine.begin(), mine.end(), [](const RosbagRow * a, const RosbagRow * b) {
    return std::tie(b->info.created_at_ns, b->seq) < std::tie(a->info.created_at_ns, a->seq);  // newest first
  });

  std::vector<RosbagFileInfo> result;
  result.reserve(mine.size());
  for (const auto * row : mine) {
    result.push_back(row->info);
  }
  return result;
}

std::vector<RosbagFileInfo>
InMemoryFaultStorage::get_rosbag_files_by_recording(const std::string & recording_id) const {
  std::lock_guard<std::mutex> lock(mutex_);

  std::vector<RosbagFileInfo> result;
  for (const auto & row : rosbag_files_) {
    if (row.info.recording_id == recording_id) {
      result.push_back(row.info);
    }
  }
  std::sort(result.begin(), result.end(), [](const RosbagFileInfo & a, const RosbagFileInfo & b) {
    return a.fault_code < b.fault_code;
  });
  return result;
}

bool InMemoryFaultStorage::delete_rosbag_file(const std::string & fault_code) {
  std::lock_guard<std::mutex> lock(mutex_);

  // ALL recordings of this fault. Used by auto_cleanup on clear, where dropping the
  // fault's whole black-box history is the intent.
  std::set<std::string> touched;
  const size_t before = rosbag_files_.size();
  for (auto it = rosbag_files_.begin(); it != rosbag_files_.end();) {
    if (it->info.fault_code == fault_code) {
      touched.insert(it->info.file_path);
      it = rosbag_files_.erase(it);
    } else {
      ++it;
    }
  }
  if (rosbag_files_.size() == before) {
    return false;
  }

  for (const auto & path : touched) {
    if (path_referenced(path)) {
      continue;  // a sibling fault of the burst still holds it
    }
    std::error_code ec;
    std::filesystem::remove_all(path, ec);
    // Ignore errors - file may already be deleted
  }
  return true;
}

size_t InMemoryFaultStorage::delete_rosbag_recording(const std::string & recording_id) {
  // Mirrors SqliteFaultStorage: an empty id addresses a set, not a recording.
  if (recording_id.empty()) {
    return 0;
  }

  std::lock_guard<std::mutex> lock(mutex_);

  std::set<std::string> touched;
  size_t removed = 0;
  for (auto it = rosbag_files_.begin(); it != rosbag_files_.end();) {
    if (it->info.recording_id == recording_id) {
      touched.insert(it->info.file_path);
      it = rosbag_files_.erase(it);
      ++removed;
    } else {
      ++it;
    }
  }

  for (const auto & path : touched) {
    if (path_referenced(path)) {
      continue;
    }
    std::error_code ec;
    std::filesystem::remove_all(path, ec);
  }
  return removed;
}

bool InMemoryFaultStorage::path_referenced(const std::string & file_path) const {
  return std::any_of(rosbag_files_.begin(), rosbag_files_.end(), [&](const RosbagRow & row) {
    return row.info.file_path == file_path;
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
  for (const auto & row : rosbag_files_) {
    auto & bytes = bytes_per_path[row.info.file_path];
    bytes = std::max(bytes, row.info.size_bytes);
  }

  size_t total = 0;
  for (const auto & [path, bytes] : bytes_per_path) {
    total += bytes;
  }
  return total;
}

std::vector<RosbagFileInfo> InMemoryFaultStorage::get_all_rosbag_files() const {
  std::lock_guard<std::mutex> lock(mutex_);

  std::vector<const RosbagRow *> rows;
  rows.reserve(rosbag_files_.size());
  for (const auto & row : rosbag_files_) {
    rows.push_back(&row);
  }

  // Oldest first, seq breaking the created_at_ns ties a burst guarantees - the
  // SQLite side orders by created_at_ns ASC, id ASC for the same reason.
  std::sort(rows.begin(), rows.end(), [](const RosbagRow * a, const RosbagRow * b) {
    return std::tie(a->info.created_at_ns, a->seq) < std::tie(b->info.created_at_ns, b->seq);
  });

  std::vector<RosbagFileInfo> result;
  result.reserve(rows.size());
  for (const auto * row : rows) {
    result.push_back(row->info);
  }
  return result;
}

std::vector<RosbagFileInfo> InMemoryFaultStorage::list_rosbags_for_entity(const std::string & entity_fqn) const {
  std::lock_guard<std::mutex> lock(mutex_);

  std::vector<const RosbagRow *> rows;
  for (const auto & row : rosbag_files_) {
    // Check if any of the fault's reporting sources contain this entity
    auto fault_it = faults_.find(row.info.fault_code);
    if (fault_it == faults_.end()) {
      continue;
    }
    const auto & fault_state = fault_it->second;
    if (fault_state.reporting_sources.find(entity_fqn) != fault_state.reporting_sources.end()) {
      rows.push_back(&row);
    }
  }

  std::sort(rows.begin(), rows.end(), [](const RosbagRow * a, const RosbagRow * b) {
    return std::tie(b->info.created_at_ns, b->seq) < std::tie(a->info.created_at_ns, a->seq);  // newest first
  });

  std::vector<RosbagFileInfo> result;
  result.reserve(rows.size());
  for (const auto * row : rows) {
    result.push_back(row->info);
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

  // Reclassified rows must match CLEARED semantics, snapshots included - the SQLite backend drops
  // them here, and a backend that kept them would answer a snapshot query differently for the
  // same sequence of calls. The near-miss series is retained, as it is on clear_fault.
  if (!retain_snapshots_on_clear_ && !reclassified.empty()) {
    const std::set<std::string> affected(reclassified.begin(), reclassified.end());
    snapshots_.erase(std::remove_if(snapshots_.begin(), snapshots_.end(),
                                    [&affected](const SnapshotData & s) {
                                      return affected.count(s.fault_code) > 0;
                                    }),
                     snapshots_.end());
  }

  return reclassified;
}

// ---------------------------------------------------------------------------
// Planned-stop windows
// ---------------------------------------------------------------------------

bool InMemoryFaultStorage::declare_planned_stop(const PlannedStopWindow & window) {
  std::lock_guard<std::mutex> lock(mutex_);

  const bool exists = std::any_of(planned_stops_.begin(), planned_stops_.end(), [&window](const PlannedStopWindow & w) {
    return w.id == window.id;
  });
  if (exists) {
    return false;
  }

  planned_stops_.push_back(window);
  prune_planned_stops_locked(window.declared_at_ns, window.id);
  return true;
}

EndPlannedStopResult InMemoryFaultStorage::end_planned_stop(const std::string & id, int64_t at_ns) {
  std::lock_guard<std::mutex> lock(mutex_);

  auto it = std::find_if(planned_stops_.begin(), planned_stops_.end(), [&id](const PlannedStopWindow & w) {
    return w.id == id;
  });
  if (it == planned_stops_.end()) {
    return EndPlannedStopResult{EndPlannedStopOutcome::NotFound, {}};
  }
  if (at_ns < it->starts_at_ns) {
    // It never started: cancelled, not ended early.
    PlannedStopWindow cancelled = *it;
    cancelled.cancelled = true;
    planned_stops_.erase(it);
    return EndPlannedStopResult{EndPlannedStopOutcome::Cancelled, cancelled};
  }
  if (it->ended_early || !it->active_at(at_ns)) {
    return EndPlannedStopResult{EndPlannedStopOutcome::AlreadyEnded, *it};
  }

  it->ends_at_ns = at_ns;
  it->ended_early = true;
  return EndPlannedStopResult{EndPlannedStopOutcome::Ended, *it};
}

std::optional<PlannedStopWindow> InMemoryFaultStorage::get_planned_stop(const std::string & id) const {
  std::lock_guard<std::mutex> lock(mutex_);

  auto it = std::find_if(planned_stops_.begin(), planned_stops_.end(), [&id](const PlannedStopWindow & w) {
    return w.id == id;
  });
  if (it == planned_stops_.end()) {
    return std::nullopt;
  }
  return *it;
}

std::vector<PlannedStopWindow> InMemoryFaultStorage::list_planned_stops() const {
  std::lock_guard<std::mutex> lock(mutex_);

  std::vector<PlannedStopWindow> out(planned_stops_.begin(), planned_stops_.end());
  // Newest declaration first, ties broken by id so two windows declared in the
  // same nanosecond do not order differently from one call to the next.
  std::stable_sort(out.begin(), out.end(), [](const PlannedStopWindow & a, const PlannedStopWindow & b) {
    if (a.declared_at_ns != b.declared_at_ns) {
      return a.declared_at_ns > b.declared_at_ns;
    }
    return a.id > b.id;
  });
  return out;
}

size_t InMemoryFaultStorage::set_max_planned_stops(size_t max_count, int64_t now_ns) {
  std::lock_guard<std::mutex> lock(mutex_);
  max_planned_stops_ = max_count;
  return prune_planned_stops_locked(now_ns);
}

size_t InMemoryFaultStorage::prune_planned_stops_locked(int64_t now_ns, const std::string & exempt_id) {
  if (max_planned_stops_ == 0 || planned_stops_.size() <= max_planned_stops_) {
    return 0;
  }

  // Prune candidates: windows that have already ended, oldest declaration first.
  std::vector<size_t> candidates;
  candidates.reserve(planned_stops_.size());
  for (size_t i = 0; i < planned_stops_.size(); ++i) {
    if (!planned_stops_[i].active_at(now_ns) && planned_stops_[i].id != exempt_id) {
      candidates.push_back(i);
    }
  }
  std::sort(candidates.begin(), candidates.end(), [this](size_t a, size_t b) {
    if (planned_stops_[a].declared_at_ns != planned_stops_[b].declared_at_ns) {
      return planned_stops_[a].declared_at_ns < planned_stops_[b].declared_at_ns;
    }
    return planned_stops_[a].id < planned_stops_[b].id;
  });

  const size_t over = planned_stops_.size() - max_planned_stops_;
  const size_t to_drop = std::min(over, candidates.size());
  if (to_drop == 0) {
    return 0;
  }

  std::set<std::string> doomed;
  for (size_t i = 0; i < to_drop; ++i) {
    doomed.insert(planned_stops_[candidates[i]].id);
  }
  planned_stops_.erase(std::remove_if(planned_stops_.begin(), planned_stops_.end(),
                                      [&doomed](const PlannedStopWindow & w) {
                                        return doomed.count(w.id) > 0;
                                      }),
                       planned_stops_.end());
  return to_drop;
}

}  // namespace ros2_medkit_fault_manager
