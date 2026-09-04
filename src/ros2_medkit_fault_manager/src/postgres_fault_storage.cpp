// Copyright 2026 gstavrinos
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

#include "ros2_medkit_fault_manager/postgres_fault_storage.hpp"

#include "rcutils/logging_macros.h"
#include "ros2_medkit_msgs/msg/fault.hpp"

namespace ros2_medkit_fault_manager {

PgFaultStorage::PgFaultStorage(const std::string & conn_info) : conn_info_(conn_info) {
  try {
    db_conn_ = std::make_unique<pqxx::connection>(conn_info);
  } catch (const std::exception & e) {
    throw std::runtime_error(std::string("Failed to connect to PostgreSQL: ") + e.what());
  }
  initialize_schema();
}

PgFaultStorage::~PgFaultStorage() {
  if (!db_conn_->is_open()) {
    return;
  }
  try {
    db_conn_->close();
  } catch (...) {
  }
}

void PgFaultStorage::set_debounce_config(const DebounceConfig & config) {
  std::lock_guard<std::mutex> lock(mutex_);
  config_ = config;
}

DebounceConfig PgFaultStorage::get_debounce_config() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return config_;
}

void PgFaultStorage::initialize_schema() {
  pqxx::work tx(*db_conn_);
  try {
    tx.exec(R"(
      CREATE TABLE IF NOT EXISTS faults (
        fault_code TEXT PRIMARY KEY,
        severity INTEGER NOT NULL,
        description TEXT NOT NULL,
        first_occurred_ns BIGINT NOT NULL,
        last_occurred_ns BIGINT NOT NULL,
        occurrence_count BIGINT NOT NULL,
        status TEXT NOT NULL,
        reporting_sources TEXT NOT NULL,
        debounce_counter INTEGER NOT NULL DEFAULT 0,
        last_failed_ns BIGINT NOT NULL DEFAULT 0,
        last_passed_ns BIGINT NOT NULL DEFAULT 0,
        confirmed_at_ns BIGINT NOT NULL DEFAULT 0
      );
    )");

    // NOTE: Since PostgreSQL support came after the 'confirmed_at_ns' column
    // was introduced, we should not check for backwards compatibility

    // Migration: releases that advanced last_occurred_ns on PASSED events left
    // inflated rows behind, and a latched CONFIRMED fault that only ever heals
    // would keep the wrong timestamp forever. last_failed_ns holds the true
    // last occurrence; last_occurred_ns can only exceed it via that old bug.
    tx.exec(
        "UPDATE faults SET last_occurred_ns = last_failed_ns WHERE last_failed_ns > 0 AND last_occurred_ns > "
        "last_failed_ns");

    // Create snapshots table for storing topic data captured when faults are confirmed
    // Migration: snapshots gained capture_id, which groups the rows of one capture.
    // Without it the per-fault cap could not tell where a capture ended and trimmed
    // by row, storing a confirmation's values in part. Rows written before it read
    // as capture 0 - one legacy set, which is how they behaved anyway.
    // NOTE: The sqlite version adds another column (capture_id) on a separate step. We are doing it on creation, since
    // no previous version exists to ensure compatibility right from the start
    tx.exec(R"(
      CREATE TABLE IF NOT EXISTS snapshots (
        id BIGINT PRIMARY KEY GENERATED ALWAYS AS IDENTITY,
        fault_code TEXT NOT NULL,
        topic TEXT NOT NULL,
        message_type TEXT NOT NULL,
        data TEXT NOT NULL,
        captured_at_ns BIGINT NOT NULL,
        capture_id BIGINT NOT NULL DEFAULT 0
      );
      CREATE INDEX IF NOT EXISTS idx_snapshots_fault_code ON snapshots(fault_code);
      CREATE INDEX IF NOT EXISTS idx_snapshots_fault_topic ON snapshots(fault_code, topic))");

    // Create freeze_frames table: one compact JSON dict of captured topic values per fault
    // code. Unlike snapshots, freeze frames are keyed by fault_code and are NOT removed on
    // clear_fault, so the confirmed-state record is retained after acknowledgement.
    tx.exec(R"(
      CREATE TABLE IF NOT EXISTS freeze_frames (
        fault_code TEXT PRIMARY KEY,
        data TEXT NOT NULL,
        captured_at_ns BIGINT NOT NULL
      );
    )");

    // NOTE: The capture_id migration check is not required since the PostgreSQL integration came after it

    // Create near_misses table: append-only series of FAILED reports that moved the debounce
    // counter without confirming the fault. One row per qualifying report, never updated in
    // place, and NOT removed on clear_fault - acknowledging a fault cycle must not erase how
    // often that code approached confirmation. Bounded per fault code by the caller-supplied
    // limit, evicting the oldest rows first.
    tx.exec(R"(
      CREATE TABLE IF NOT EXISTS near_misses (
        id BIGINT PRIMARY KEY GENERATED ALWAYS AS IDENTITY,
        fault_code TEXT NOT NULL,
        occurred_at_ns BIGINT NOT NULL,
        debounce_counter INTEGER NOT NULL,
        confirmation_threshold INTEGER NOT NULL,
        severity INTEGER NOT NULL,
        source_id TEXT NOT NULL,
        resulting_status TEXT NOT NULL DEFAULT ''
      );
      CREATE INDEX IF NOT EXISTS idx_near_misses_fault_code ON near_misses(fault_code, id);
    )");

    // NOTE: The resulting_status migration check is not required since the PostgreSQL integration came after it

    // Create rosbag_files table. One row = one LINK (a fault claiming a recording):
    // several faults of a burst link to one bag, and one fault links to several bags
    // over time. Bytes belong to file_path, not to the row.
    //
    // House rule, learned the hard way here: uniqueness is expressed with
    // CREATE UNIQUE INDEX, never as a column constraint. The original
    // `fault_code TEXT NOT NULL UNIQUE` could not be dropped with ALTER TABLE and
    // forced the full table rebuild in the sqlite version.
    // NOTE: Since the PostgreSQL integration came after the UNIQUE fault_code was removed, the migration steps are
    // omitted and all indices are created without the conflicts
    tx.exec(R"(
      CREATE TABLE IF NOT EXISTS rosbag_files (
        id BIGINT PRIMARY KEY GENERATED ALWAYS AS IDENTITY,
        fault_code TEXT NOT NULL,
        recording_id TEXT NOT NULL DEFAULT '',
        file_path TEXT NOT NULL,
        format TEXT NOT NULL,
        duration_sec DOUBLE PRECISION NOT NULL,
        size_bytes BIGINT NOT NULL,
        created_at_ns BIGINT NOT NULL
      );
      CREATE INDEX IF NOT EXISTS idx_rosbag_files_fault_code ON rosbag_files(fault_code);
      CREATE INDEX IF NOT EXISTS idx_rosbag_files_created_at ON rosbag_files(created_at_ns);
      CREATE INDEX IF NOT EXISTS idx_rosbag_files_fault_created ON rosbag_files(fault_code, created_at_ns, id);
      CREATE INDEX IF NOT EXISTS idx_rosbag_files_recording ON rosbag_files(recording_id);
      CREATE INDEX IF NOT EXISTS idx_rosbag_files_path ON rosbag_files(file_path);
      CREATE UNIQUE INDEX IF NOT EXISTS idx_rosbag_files_fault_path ON rosbag_files(fault_code, file_path);
    )");

    tx.commit();
  } catch (const std::exception & e) {
    tx.abort();
    throw std::runtime_error("Schema initialization failed: " + std::string(e.what()));
  }
}

// NOTE: Skipping a huge portion of functions in the sqlite implementation that are solely used to ensure compatibility
// between versions that came before the PostgreSQL integration

std::vector<std::string> PgFaultStorage::parse_json_array(const std::string & json_str) {
  std::vector<std::string> result;

  // Simple JSON array parser for ["a", "b", "c"] format
  if (json_str.size() < 2 || json_str.front() != '[' || json_str.back() != ']') {
    if (!json_str.empty()) {
      RCUTILS_LOG_WARN_NAMED("postgres_fault_storage", "Malformed JSON array in database: '%s'", json_str.c_str());
    }
    return result;
  }

  std::string content = json_str.substr(1, json_str.size() - 2);
  if (content.empty()) {
    return result;
  }

  size_t pos = 0;
  while (pos < content.size()) {
    // Skip whitespace
    while (pos < content.size() && std::isspace(static_cast<unsigned char>(content[pos]))) {
      ++pos;
    }
    if (pos >= content.size()) {
      break;
    }

    // Expect opening quote
    if (content[pos] != '"') {
      break;
    }
    ++pos;

    // Find closing quote (handle escape sequences)
    std::string value;
    while (pos < content.size() && content[pos] != '"') {
      if (content[pos] == '\\' && pos + 1 < content.size()) {
        ++pos;
        char escaped = content[pos];
        switch (escaped) {
          case '"':
            value.push_back('"');
            break;
          case '\\':
            value.push_back('\\');
            break;
          case '/':
            value.push_back('/');
            break;
          case 'b':
            value.push_back('\b');
            break;
          case 'f':
            value.push_back('\f');
            break;
          case 'n':
            value.push_back('\n');
            break;
          case 'r':
            value.push_back('\r');
            break;
          case 't':
            value.push_back('\t');
            break;
          default:
            // Unknown escape sequence: preserve character as-is
            value.push_back(escaped);
            break;
        }
        ++pos;
        continue;
      }
      value.push_back(content[pos]);
      ++pos;
    }

    if (pos < content.size()) {
      ++pos;  // Skip closing quote
    }

    result.push_back(value);

    // Skip whitespace and comma
    while (pos < content.size() && (std::isspace(static_cast<unsigned char>(content[pos])) || content[pos] == ',')) {
      ++pos;
    }
  }

  return result;
}

std::string PgFaultStorage::serialize_json_array(const std::vector<std::string> & vec) {
  std::ostringstream oss;
  oss << '[';
  for (size_t i = 0; i < vec.size(); ++i) {
    if (i > 0) {
      oss << ',';
    }
    oss << '"';
    // Escape special characters per JSON specification
    for (char c : vec[i]) {
      switch (c) {
        case '"':
          oss << "\\\"";
          break;
        case '\\':
          oss << "\\\\";
          break;
        case '\b':
          oss << "\\b";
          break;
        case '\f':
          oss << "\\f";
          break;
        case '\n':
          oss << "\\n";
          break;
        case '\r':
          oss << "\\r";
          break;
        case '\t':
          oss << "\\t";
          break;
        default:
          oss << c;
          break;
      }
    }
    oss << '"';
  }
  oss << ']';
  return oss.str();
}

bool PgFaultStorage::report_fault_event(const std::string & fault_code, uint8_t event_type, uint8_t severity,
                                        const std::string & description, const std::string & source_id,
                                        const rclcpp::Time & timestamp, const DebounceConfig & config) {
  std::lock_guard<std::mutex> lock(mutex_);

  // Only a FAILED report can write two rows (fault row and near-miss row) and
  // both have to land together: written separately, a failure on the second would
  // leave the debounce counter already advanced, so the caller's retry would
  // advance it a second time and the near miss it retried for would still be
  // missing from the series.
  //
  // The SQLite backend has to opt into a transaction for this (BEGIN IMMEDIATE)
  // In the PostgreSQL implementation the lock used is a transaction lock, so no extra handling is required apart from
  // updating the tables properly
  pqxx::work tx(*db_conn_);
  try {
    const bool is_new_occurrence =
        report_fault_event_locked(fault_code, event_type, severity, description, source_id, timestamp, config, tx);
    tx.commit();
    return is_new_occurrence;
  } catch (const std::exception & e) {
    tx.abort();
    throw std::runtime_error(std::string("report_fault_event PostgreSQL error: ") + e.what());
  }
}

bool PgFaultStorage::report_fault_event_locked(const std::string & fault_code, uint8_t event_type, uint8_t severity,
                                               const std::string & description, const std::string & source_id,
                                               const rclcpp::Time & timestamp, const DebounceConfig & config,
                                               pqxx::work & tx) {
  int64_t timestamp_ns = timestamp.nanoseconds();
  const bool is_failed = (event_type == EventType::EVENT_FAILED);

  // NOTE: Do I need to SELECT here with FOR UPDATE to secure the selected row?
  // Check if fault exists
  auto res = tx.exec_params(
      "SELECT severity, occurrence_count, reporting_sources, status, debounce_counter, confirmed_at_ns, "
      "first_occurred_ns FROM faults WHERE fault_code = $1",
      fault_code);

  if (res.affected_rows() > 0) {
    // Fault exists - update it
    int existing_severity = res[0]["severity"].as<int>();
    int64_t existing_count = res[0]["occurrence_count"].as<int64_t>();
    std::string sources_json = res[0]["reporting_sources"].as<std::string>();
    std::string current_status = res[0]["status"].as<std::string>();
    int32_t debounce_counter = res[0]["debounce_counter"].as<int32_t>();
    int64_t confirmed_at_ns = res[0]["confirmed_at_ns"].as<int64_t>();
    int64_t first_occurred_ns = res[0]["first_occurred_ns"].as<int64_t>();

    // Bring a runaway counter persisted by an older build (the bug this fixes) back into range on
    // first touch; this also keeps the +1/-1 below overflow-safe. The counter is local to this call.
    debounce_counter = clamp_debounce_counter(debounce_counter, config);

    // CLEARED faults can be reactivated by FAILED events
    bool is_reactivation = false;
    if (current_status == ros2_medkit_msgs::msg::Fault::STATUS_CLEARED) {
      if (!is_failed) {
        // PASSED events for CLEARED faults are ignored
        return false;
      }
      // FAILED event reactivates - reset debounce counter to 0 so FAILED branch
      // decrements it to -1, then reuse the existing FAILED logic below. Also
      // reset first_occurred: this is a new outage cycle, not a continuation
      // of the one that just cleared.
      debounce_counter = 0;
      first_occurred_ns = timestamp_ns;
      is_reactivation = true;
    }

    if (is_failed) {
      // FAILED event
      // Parse existing sources and add new one
      std::vector<std::string> sources = parse_json_array(sources_json);
      std::set<std::string> sources_set(sources.begin(), sources.end());
      sources_set.insert(source_id);
      sources.assign(sources_set.begin(), sources_set.end());

      // Escalate severity if new severity is higher
      int new_severity = std::max(existing_severity, static_cast<int>(severity));

      // Increment count with saturation - only on a genuine new occurrence (reactivation
      // after CLEARED). A still-active fault being re-reported (level-triggered poller,
      // or debounce building toward confirmation) is the same continuous occurrence.
      int64_t new_count = existing_count;
      if (is_reactivation && new_count < std::numeric_limits<uint32_t>::max()) {
        ++new_count;
      }

      // Decrement towards confirmation, clamped to the thresholds.
      debounce_counter = clamp_debounce_counter(debounce_counter - 1, config);

      // CRITICAL bypasses debounce; otherwise the shared state machine decides (with hysteresis).
      std::string new_status;
      if (config.critical_immediate_confirm && severity == ros2_medkit_msgs::msg::Fault::SEVERITY_CRITICAL) {
        new_status = ros2_medkit_msgs::msg::Fault::STATUS_CONFIRMED;
      } else {
        new_status = compute_debounce_status(debounce_counter, current_status, config);
      }

      // Record the confirmation instant on the transition into CONFIRMED (also
      // on a reactivation that re-confirms); an already-confirmed fault keeps
      // its original timestamp.
      if (new_status == ros2_medkit_msgs::msg::Fault::STATUS_CONFIRMED &&
          current_status != ros2_medkit_msgs::msg::Fault::STATUS_CONFIRMED) {
        confirmed_at_ns = timestamp_ns;
      }

      if (description.empty()) {
        tx.exec_params(
            "UPDATE faults SET severity = $1, last_occurred_ns = $2, last_failed_ns = $3, occurrence_count = $4, "
            "reporting_sources = $5, status = $6, debounce_counter = $7, confirmed_at_ns = $8, "
            "first_occurred_ns = $9 WHERE fault_code = $10",
            new_severity, timestamp_ns, timestamp_ns, new_count, serialize_json_array(sources), new_status,
            debounce_counter, confirmed_at_ns, first_occurred_ns, fault_code);
      } else {
        tx.exec_params(
            "UPDATE faults SET severity = $1, description = $2, last_occurred_ns = $3, last_failed_ns = $4, "
            "occurrence_count = $5, reporting_sources = $6, status = $7, debounce_counter = $8, "
            "confirmed_at_ns = $9, first_occurred_ns = $10 WHERE fault_code = $11",
            new_severity, description, timestamp_ns, timestamp_ns, new_count, serialize_json_array(sources), new_status,
            debounce_counter, confirmed_at_ns, first_occurred_ns, fault_code);
      }

      if (is_near_miss(true, new_status)) {
        record_near_miss_locked(fault_code, timestamp_ns, debounce_counter, config, severity, source_id, new_status,
                                tx);
      }
    } else {
      // PASSED event - increment towards healing, clamped to the thresholds.
      debounce_counter = clamp_debounce_counter(debounce_counter + 1, config);

      std::string new_status = compute_debounce_status(debounce_counter, current_status, config);

      // last_occurred_ns is deliberately NOT touched: a PASSED event is the fault
      // ENDING, not occurring. Bumping it made a long-stale CONFIRMED fault look
      // freshly active. The PASSED instant is recorded in last_passed_ns.
      tx.exec_params("UPDATE faults SET last_passed_ns = $1, status = $2, debounce_counter = $3 WHERE fault_code = $4",
                     timestamp_ns, new_status, debounce_counter, fault_code);
    }

    return is_reactivation;  // Reactivation treated as new occurrence for event publishing
  }

  // New fault - only create for FAILED events
  if (!is_failed) {
    return false;  // PASSED event for non-existent fault is ignored
  }

  // Determine initial status based on debounce logic (shared with the in-memory backend).
  std::string initial_status;
  constexpr int32_t initial_counter = -1;  // First FAILED event sets counter to -1
  if (config.critical_immediate_confirm && severity == ros2_medkit_msgs::msg::Fault::SEVERITY_CRITICAL) {
    initial_status = ros2_medkit_msgs::msg::Fault::STATUS_CONFIRMED;
  } else {
    initial_status = compute_debounce_status(initial_counter, "", config);
  }

  const bool confirmed_now = initial_status == ros2_medkit_msgs::msg::Fault::STATUS_CONFIRMED;
  tx.exec_params(
      "INSERT INTO faults (fault_code, severity, description, first_occurred_ns, last_occurred_ns, occurrence_count, "
      "status, reporting_sources, debounce_counter, last_failed_ns, last_passed_ns, confirmed_at_ns) "
      "VALUES ($1, $2, $3, $4, $5, $6, $7, $8, $9, $10, $11, $12)",
      fault_code, static_cast<int>(severity), description, timestamp_ns, timestamp_ns, 1 /* occurrence_count = 1 */,
      initial_status, serialize_json_array({source_id}), initial_counter, timestamp_ns, 0,
      confirmed_now ? timestamp_ns : 0);

  if (is_near_miss(true, initial_status)) {
    record_near_miss_locked(fault_code, timestamp_ns, initial_counter, config, severity, source_id, initial_status, tx);
  }

  return true;  // New fault created
}

std::vector<ros2_medkit_msgs::msg::Fault> PgFaultStorage::list_faults(bool filter_by_severity, uint8_t severity,
                                                                      const std::vector<std::string> & statuses) const {
  std::lock_guard<std::mutex> lock(mutex_);
  pqxx::work tx(*db_conn_);
  try {
    // Determine which statuses to include
    std::set<std::string> status_filter;
    if (statuses.empty()) {
      status_filter.insert(ros2_medkit_msgs::msg::Fault::STATUS_CONFIRMED);
    } else {
      for (const auto & s : statuses) {
        if (s == ros2_medkit_msgs::msg::Fault::STATUS_PREFAILED ||
            s == ros2_medkit_msgs::msg::Fault::STATUS_PREPASSED ||
            s == ros2_medkit_msgs::msg::Fault::STATUS_CONFIRMED || s == ros2_medkit_msgs::msg::Fault::STATUS_HEALED ||
            s == ros2_medkit_msgs::msg::Fault::STATUS_CLEARED) {
          status_filter.insert(s);
        }
      }
      if (status_filter.empty()) {
        status_filter.insert(ros2_medkit_msgs::msg::Fault::STATUS_CONFIRMED);
      }
    }

    std::vector<std::string> status_filter_vector(status_filter.begin(), status_filter.end());
    // Build query
    std::string sql =
        "SELECT fault_code, severity, description, first_occurred_ns, last_occurred_ns, occurrence_count, status, "
        "reporting_sources, last_passed_ns FROM faults WHERE status = ANY($1) ";
    if (filter_by_severity) {
      sql += " AND severity = $2";
    }

    auto res = filter_by_severity ? tx.exec_params(sql, status_filter_vector, static_cast<int>(severity))
                                  : tx.exec_params(sql, status_filter_vector);
    tx.commit();

    std::vector<ros2_medkit_msgs::msg::Fault> result;
    for (const auto & r : res) {
      ros2_medkit_msgs::msg::Fault fault;
      fault.fault_code = r["fault_code"].as<std::string>();
      fault.severity = static_cast<uint8_t>(r["severity"].as<int>());
      fault.description = r["description"].as<std::string>();
      fault.first_occurred = rclcpp::Time(r["first_occurred_ns"].as<int64_t>(), RCL_SYSTEM_TIME);
      fault.last_occurred = rclcpp::Time(r["last_occurred_ns"].as<int64_t>(), RCL_SYSTEM_TIME);
      fault.occurrence_count = static_cast<uint32_t>(r["occurrence_count"].as<int64_t>());
      fault.status = r["status"].as<std::string>();
      fault.reporting_sources = parse_json_array(r["reporting_sources"].as<std::string>());
      fault.last_passed = rclcpp::Time(r["last_passed_ns"].as<int64_t>(), RCL_SYSTEM_TIME);
      result.push_back(fault);
    }
    return result;
  } catch (const std::exception & e) {
    tx.abort();
    throw std::runtime_error(std::string("list_faults PostgreSQL error: ") + e.what());
  }
}

std::optional<ros2_medkit_msgs::msg::Fault> PgFaultStorage::get_fault(const std::string & fault_code) const {
  std::lock_guard<std::mutex> lock(mutex_);
  pqxx::work tx(*db_conn_);
  try {
    auto res = tx.exec_params(
        "SELECT fault_code, severity, description, first_occurred_ns, last_occurred_ns, occurrence_count, status, "
        "reporting_sources, last_passed_ns FROM faults WHERE fault_code = $1",
        fault_code);
    if (res.affected_rows() == 0) {
      tx.commit();
      return std::nullopt;
    }
    auto r = res[0];
    ros2_medkit_msgs::msg::Fault fault;
    fault.fault_code = r["fault_code"].as<std::string>();
    fault.severity = static_cast<uint8_t>(r["severity"].as<int>());
    fault.description = r["description"].as<std::string>();
    fault.first_occurred = rclcpp::Time(r["first_occurred_ns"].as<int64_t>(), RCL_SYSTEM_TIME);
    fault.last_occurred = rclcpp::Time(r["last_occurred_ns"].as<int64_t>(), RCL_SYSTEM_TIME);
    fault.occurrence_count = static_cast<uint32_t>(r["occurrence_count"].as<int64_t>());
    fault.status = r["status"].as<std::string>();
    fault.reporting_sources = parse_json_array(r["reporting_sources"].as<std::string>());
    fault.last_passed = rclcpp::Time(r["last_passed_ns"].as<int64_t>(), RCL_SYSTEM_TIME);
    tx.commit();
    return fault;
  } catch (const std::exception & e) {
    tx.abort();
    throw std::runtime_error(std::string("get_fault PostgreSQL error: ") + e.what());
  }
}

bool PgFaultStorage::clear_fault(const std::string & fault_code) {
  std::lock_guard<std::mutex> lock(mutex_);
  pqxx::work tx(*db_conn_);

  try {
    // The near_misses rows for this code are deliberately left alone. Clearing acknowledges one
    // fault cycle; the record of how often the code approached confirmation spans cycles and
    // cannot be reconstructed once deleted.

    // Acknowledging a fault drops its value snapshots, unless a history was asked
    // for: with recordings retained past a clear, deleting the readings that go with
    // them leaves a fault holding bags whose matching values are gone.
    if (!retain_snapshots_on_clear_) {
      tx.exec_params("DELETE FROM snapshots WHERE fault_code = $1", fault_code);
    }

    auto res = tx.exec_params("UPDATE faults SET status = $1 WHERE fault_code = $2",
                              ros2_medkit_msgs::msg::Fault::STATUS_CLEARED, fault_code);
    const bool changed = res.affected_rows() > 0;
    tx.commit();
    return changed;
  } catch (const std::exception & e) {
    tx.abort();
    throw std::runtime_error(std::string("clear_fault PostgreSQL error: ") + e.what());
  }
}

std::vector<std::string> PgFaultStorage::reclassify_healed_as_cleared() {
  std::lock_guard<std::mutex> lock(mutex_);
  pqxx::work tx(*db_conn_);

  try {
    // Collect the codes that will flip first so the caller can audit each one. The
    // SELECT predicate mirrors the UPDATE exactly, and both run under the same lock,
    // so the returned list matches the rows actually reclassified below.
    auto res =
        tx.exec_params("SELECT fault_code FROM faults WHERE status = $1", ros2_medkit_msgs::msg::Fault::STATUS_HEALED);
    std::vector<std::string> reclassified;
    for (const auto & r : res) {
      reclassified.push_back(r["fault_code"].as<std::string>());
    }
    if (reclassified.empty()) {
      tx.commit();
      return reclassified;
    }
    // Drop snapshots for the affected faults so a reclassified row matches CLEARED semantics.
    // clear_fault is not the only place that takes a fault's readings, so retain_snapshots_on_clear_
    // has to reach here too: otherwise the setting holds until the next restart and then the
    // reclassification deletes exactly what it was set to keep.
    if (!retain_snapshots_on_clear_) {
      tx.exec_params("DELETE FROM snapshots WHERE fault_code IN (SELECT fault_code FROM faults WHERE status = $1)",
                     ros2_medkit_msgs::msg::Fault::STATUS_HEALED);
    }
    tx.exec_params("UPDATE faults SET status = $1 WHERE status = $2", ros2_medkit_msgs::msg::Fault::STATUS_CLEARED,
                   ros2_medkit_msgs::msg::Fault::STATUS_HEALED);
    tx.commit();
    return reclassified;
  } catch (const std::exception & e) {
    tx.abort();
    throw std::runtime_error(std::string("reclassify_healed_as_cleared PostgreSQL error: ") + e.what());
  }
}

size_t PgFaultStorage::size() const {
  std::lock_guard<std::mutex> lock(mutex_);
  pqxx::work tx(*db_conn_);
  try {
    auto res = tx.exec("SELECT COUNT(*) AS sz FROM faults");
    tx.commit();
    return res.empty() ? 0 : static_cast<size_t>(res[0]["sz"].as<int64_t>());
  } catch (const std::exception & e) {
    tx.abort();
    throw std::runtime_error(std::string("size PostgreSQL error: ") + e.what());
  }
}

bool PgFaultStorage::contains(const std::string & fault_code) const {
  std::lock_guard<std::mutex> lock(mutex_);
  pqxx::work tx(*db_conn_);

  try {
    auto res = tx.exec_params("SELECT 1 FROM faults WHERE fault_code = $1 LIMIT 1", fault_code);
    tx.commit();
    return res.affected_rows() > 0;
  } catch (const std::exception & e) {
    tx.abort();
    throw std::runtime_error(std::string("contains PostgreSQL error: ") + e.what());
  }
}

std::vector<std::string> PgFaultStorage::check_time_based_confirmation(const rclcpp::Time & current_time) {
  std::lock_guard<std::mutex> lock(mutex_);

  std::vector<std::string> confirmed;
  if (config_.auto_confirm_after_sec <= 0.0) {
    return confirmed;  // Time-based confirmation disabled
  }

  pqxx::work tx(*db_conn_);
  try {
    int64_t current_ns = current_time.nanoseconds();
    int64_t cutoff_ns = current_ns - static_cast<int64_t>(config_.auto_confirm_after_sec * 1e9);

    // Collect the codes that will flip first so the caller can audit each one. The
    // SELECT predicate mirrors the UPDATE exactly, and both run under the same lock,
    // so the returned list matches the rows actually confirmed below.
    auto res = tx.exec_params(
        "SELECT fault_code FROM faults WHERE status = $1 AND last_failed_ns <= $2 AND last_failed_ns > 0 ",
        ros2_medkit_msgs::msg::Fault::STATUS_PREFAILED, cutoff_ns);
    for (const auto & r : res) {
      confirmed.push_back(r["fault_code"].as<std::string>());
    }

    if (!confirmed.empty()) {
      tx.exec_params("UPDATE faults SET status = $1, confirmed_at_ns = $2 WHERE status = $3 AND last_failed_ns <= $4 ",
                     ros2_medkit_msgs::msg::Fault::STATUS_CONFIRMED, current_ns,
                     ros2_medkit_msgs::msg::Fault::STATUS_PREFAILED, cutoff_ns);
    }
    tx.commit();
    return confirmed;
  } catch (const std::exception & e) {
    tx.abort();
    throw std::runtime_error(std::string("check_time_based_confirmation PostgreSQL error: ") + e.what());
  }
}

void PgFaultStorage::set_max_snapshots_per_fault(size_t max_count) {
  std::lock_guard<std::mutex> lock(mutex_);
  max_snapshots_per_fault_ = max_count;
}

void PgFaultStorage::set_retain_snapshots_on_clear(bool retain) {
  std::lock_guard<std::mutex> lock(mutex_);
  retain_snapshots_on_clear_ = retain;
}

bool PgFaultStorage::retains_snapshots_on_clear() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return retain_snapshots_on_clear_;
}

void PgFaultStorage::store_snapshot(const SnapshotData & snapshot) {
  store_snapshots({snapshot});
}

void PgFaultStorage::store_snapshots(const std::vector<SnapshotData> & snapshots) {
  if (snapshots.empty()) {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);

  const std::string & fault_code = snapshots.front().fault_code;

  // One transaction for the whole capture: a capture is all-or-nothing, and the
  // old row-at-a-time path could leave a confirmation's values half stored.
  pqxx::work tx(*db_conn_);
  try {
    for (const auto & snapshot : snapshots) {
      tx.exec_params(
          "INSERT INTO snapshots (fault_code, topic, message_type, data, captured_at_ns, capture_id) VALUES ($1, $2, "
          "$3, $4, $5, $6)",
          snapshot.fault_code, snapshot.topic, snapshot.message_type, snapshot.data, snapshot.captured_at_ns,
          snapshot.capture_id);
    }
    if (max_snapshots_per_fault_ > 0) {
      // Trim whole capture sets, oldest first, until the fault fits. The old rule
      // counted rows and rejected the NEW row once full, so a capture straddling
      // the cap was stored in part - some topics present, the rest silently gone,
      // indistinguishable from "that topic was not publishing". Keep-newest also
      // stops this cap from opposing the rosbag one.
      //
      // The newest capture is never trimmed: if it alone exceeds the cap, the cap
      // is smaller than this fault's topic count and tearing it would be the very
      // thing being fixed.
      auto res =
          tx.exec_params("SELECT MAX(capture_id) AS max_capture_id FROM snapshots WHERE fault_code = $1", fault_code);
      int64_t newest_capture = 0;
      if (res.affected_rows() > 0) {
        newest_capture = res[0]["max_capture"].as<int64_t>();
      }
      while (true) {
        auto count_res = tx.exec_params("SELECT COUNT(*) AS sz FROM snapshots WHERE fault_code = $1", fault_code);
        if (count_res.affected_rows() == 0 || count_res[0]["sz"].as<size_t>() <= max_snapshots_per_fault_) {
          break;
        }
        auto trim_res = tx.exec_params(
            "DELETE FROM snapshots WHERE fault_code = $1 AND capture_id = "
            "(SELECT MIN(capture_id) FROM snapshots WHERE fault_code = $1) "
            "AND capture_id <> $2",
            fault_code, newest_capture);
        if (trim_res.affected_rows() == 0) {
          break;
        }
      }
    }
    tx.commit();
  } catch (const std::exception & e) {
    tx.abort();
    throw std::runtime_error(std::string("store_snapshots PostgreSQL error: ") + e.what());
  }
}

std::vector<SnapshotData> PgFaultStorage::get_snapshots(const std::string & fault_code,
                                                        const std::string & topic_filter) const {
  std::lock_guard<std::mutex> lock(mutex_);

  std::vector<SnapshotData> result;

  pqxx::work tx(*db_conn_);
  try {
    std::string sql =
        "SELECT fault_code, topic, message_type, data, captured_at_ns, capture_id FROM snapshots WHERE fault_code "
        "= "
        "$1";
    // capture_id before the timestamp: the rows of one capture are written seconds
    // apart under load and their timestamps interleave with a neighbouring capture's,
    // so ordering by time alone splits a set the reader then cannot regroup.
    sql += " ORDER BY capture_id DESC, captured_at_ns DESC";
    auto res = topic_filter.empty() ? tx.exec_params(sql, fault_code)
                                    : tx.exec_params(sql + " AND topic = $2", fault_code, topic_filter);
    tx.commit();
    for (const auto & r : res) {
      SnapshotData snapshot;
      snapshot.fault_code = r["fault_code"].as<std::string>();
      snapshot.topic = r["topic"].as<std::string>();
      snapshot.message_type = r["message_type"].as<std::string>();
      snapshot.data = r["data"].as<std::string>();
      snapshot.captured_at_ns = r["captured_at_ns"].as<int64_t>();
      snapshot.capture_id = r["capture_id"].as<int64_t>();
      result.push_back(snapshot);
    }
    return result;
  } catch (const std::exception & e) {
    tx.abort();
    throw std::runtime_error(std::string("get_snapshots PostgreSQL error: ") + e.what());
  }
}

int64_t PgFaultStorage::get_max_capture_id() const {
  std::lock_guard<std::mutex> lock(mutex_);
  pqxx::work tx(*db_conn_);
  try {
    // Global, not per fault: the counter that mints these is global, and seeding it
    // below any id already on disk is what lets a restart evict the capture it just
    // wrote. NULL on an empty table reads back as 0.
    auto res = tx.exec("SELECT COALESCE(MAX(capture_id), 0) AS max_capture FROM snapshots");
    tx.commit();
    return res.empty() ? 0 : res[0]["max_capture"].as<int64_t>();
  } catch (const std::exception & e) {
    tx.abort();
    throw std::runtime_error(std::string("get_max_capture_id PostgreSQL error: ") + e.what());
  }
}

void PgFaultStorage::store_freeze_frame(const FreezeFrameData & frame) {
  std::lock_guard<std::mutex> lock(mutex_);
  pqxx::work tx(*db_conn_);

  try {
    // Keyed by fault_code (PRIMARY KEY): a re-confirm replaces the previous frame.
    tx.exec_params(
        "INSERT INTO freeze_frames (fault_code, data, captured_at_ns) VALUES ($1, $2, $3) ON CONFLICT(fault_code) "
        "DO "
        "UPDATE SET data = $2, captured_at_ns = $3 ",
        frame.fault_code, frame.data, frame.captured_at_ns);
    tx.commit();
  } catch (const std::exception & e) {
    tx.abort();
    throw std::runtime_error(std::string("store_freeze_frame PostgreSQL error: ") + e.what());
  }
}

std::optional<FreezeFrameData> PgFaultStorage::get_freeze_frame(const std::string & fault_code) const {
  std::lock_guard<std::mutex> lock(mutex_);
  pqxx::work tx(*db_conn_);

  try {
    auto res =
        tx.exec_params("SELECT fault_code, data, captured_at_ns FROM freeze_frames WHERE fault_code = $1", fault_code);
    if (res.affected_rows() == 0) {
      tx.commit();
      return std::nullopt;
    }
    FreezeFrameData frame;
    frame.fault_code = res[0]["fault_code"].as<std::string>();
    frame.data = res[0]["data"].as<std::string>();
    frame.captured_at_ns = res[0]["captured_at_ns"].as<int64_t>();
    tx.commit();
    return frame;
  } catch (const std::exception & e) {
    tx.abort();
    throw std::runtime_error(std::string("get_freeze_frame PostgreSQL error: ") + e.what());
  }
}

size_t PgFaultStorage::set_max_near_misses_per_fault(size_t max_count) {
  std::lock_guard<std::mutex> lock(mutex_);
  max_near_misses_per_fault_ = max_count;

  // 0 and any bound past what PostgreSQL can hold both mean "keep everything". Binding SIZE_MAX
  // straight into an int64 makes it -1, and every row then compares as beyond the bound, so the
  // idiomatic spelling of "no limit" would empty the table.
  if (max_count == 0 || max_count > static_cast<size_t>(std::numeric_limits<int64_t>::max())) {
    return 0;  // Unlimited
  }

  pqxx::work tx(*db_conn_);
  // Apply the bound to what is already in the database. Without this, a database that grew under
  // a larger bound (or none) stays over the new bound until each fault code happens to record
  // another near miss - and a code that never does keeps its rows for good.
  try {
    auto res = tx.exec_params(
        "DELETE FROM near_misses WHERE id IN ("
        "SELECT id FROM (SELECT id, ROW_NUMBER() OVER "
        "(PARTITION BY fault_code ORDER BY id DESC) AS rn FROM near_misses) "
        "WHERE rn > $1)",
        static_cast<int64_t>(max_count));
    tx.commit();
    const auto dropped = res.affected_rows();
    // Returned rather than logged: the storage layer has no logger, and a bound applied by mistake
    // deletes history that cannot be recovered, so the caller has to be able to report it.
    return static_cast<size_t>(dropped);
  } catch (const std::exception & e) {
    tx.abort();
    throw std::runtime_error(std::string("set_max_near_misses_per_fault PostgreSQL error: ") + e.what());
  }
}

void PgFaultStorage::record_near_miss_locked(const std::string & fault_code, int64_t occurred_at_ns,
                                             int32_t debounce_counter, const DebounceConfig & config, uint8_t severity,
                                             const std::string & source_id, const std::string & resulting_status,
                                             pqxx::work & tx) {
  tx.exec_params(
      "INSERT INTO near_misses (fault_code, occurred_at_ns, debounce_counter, confirmation_threshold, severity, "
      "source_id, resulting_status) VALUES ($1, $2, $3, $4, $5, $6, $7)",
      fault_code, occurred_at_ns, debounce_counter, config.confirmation_threshold, static_cast<int>(severity),
      source_id, resulting_status);

  if (max_near_misses_per_fault_ == 0 ||
      max_near_misses_per_fault_ > static_cast<size_t>(std::numeric_limits<int64_t>::max())) {
    return;  // Unlimited
  }

  // Evict oldest-first, keeping the newest max_near_misses_per_fault_ rows - the same direction as
  // the snapshot and rosbag caps. A series frozen at boot answers nothing about whether the rate
  // of near misses is changing.
  //
  // "Oldest" means earliest ARRIVAL (id), not earliest occurred_at_ns. Reporters carry their own
  // clocks, so a report can arrive with a timestamp behind one already stored; ordering eviction
  // by timestamp would then drop the row that was just appended and make the two backends, which
  // append in arrival order, disagree on the same input.
  tx.exec_params(
      "DELETE FROM near_misses WHERE fault_code = $1 AND id NOT IN "
      "(SELECT id FROM near_misses WHERE fault_code = $1 ORDER BY id DESC LIMIT $2)",
      fault_code, static_cast<int64_t>(max_near_misses_per_fault_));
}

std::vector<NearMissRecord> PgFaultStorage::get_near_misses(const std::string & fault_code) const {
  std::lock_guard<std::mutex> lock(mutex_);
  pqxx::work tx(*db_conn_);

  try {
    auto res = tx.exec_params(
        "SELECT fault_code, occurred_at_ns, debounce_counter, confirmation_threshold, "
        " severity, source_id, resulting_status FROM near_misses WHERE fault_code = $1 "
        "ORDER BY id ASC",
        fault_code);

    std::vector<NearMissRecord> result;
    for (const auto & r : res) {
      NearMissRecord record;
      record.fault_code = r["fault_code"].as<std::string>();
      record.occurred_at_ns = r["occurred_at_ns"].as<int64_t>();
      record.debounce_counter = r["debounce_counter"].as<int32_t>();
      record.confirmation_threshold = r["confirmation_threshold"].as<int32_t>();
      record.severity = static_cast<uint8_t>(r["severity"].as<int>());
      record.source_id = r["source_id"].as<std::string>();
      record.resulting_status = r["resulting_status"].as<std::string>();
      result.push_back(std::move(record));
    }
    tx.commit();
    return result;
  } catch (const std::exception & e) {
    tx.abort();
    throw std::runtime_error(std::string("get_near_misses PostgreSQL error: ") + e.what());
  }
}

void PgFaultStorage::set_max_rosbags_per_fault(size_t max_count) {
  std::lock_guard<std::mutex> lock(mutex_);
  max_rosbags_per_fault_ = max_count;
}

void PgFaultStorage::store_rosbag_file(const RosbagFileInfo & info) {
  // Routed through the batch path deliberately: with a per-fault cap a single
  // store is insert + trim, i.e. several statements that must share one
  // transaction and one post-commit unlink pass.
  store_rosbag_files({info});
}

void PgFaultStorage::store_rosbag_files(const std::vector<RosbagFileInfo> & infos) {
  if (infos.empty()) {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);

  // One transaction for the whole burst: a crash mid-store must not leave some
  // faults of the shared recording without their lookup row. Evicted bags are
  // unlinked only after COMMIT - a ROLLBACK (PostgreSQL transaction abort) resurrects the rows, which must keep
  // pointing at bags that still exist.
  std::vector<std::string> evicted;
  {
    // NOTE: The transaction object is scoped here, to allow for path_referenced() to use its own. The initial lock is
    // used universally
    pqxx::work tx(*db_conn_);
    try {
      for (const auto & info : infos) {
        auto paths = store_rosbag_file_locked(info, tx);
        evicted.insert(evicted.end(), std::make_move_iterator(paths.begin()), std::make_move_iterator(paths.end()));
      }
      tx.commit();
    } catch (const std::exception & e) {
      tx.abort();
      throw std::runtime_error(std::string("store_rosbag_files PostgreSQL error: ") + e.what());
    }
  }

  // Referencing is re-checked on the committed state, not on the state each
  // eviction saw: two faults of one burst can link the same bag, and a row-by-row
  // check inside the loop would find it still held by a sibling that a later
  // iteration then evicts, leaking the directory.
  const std::set<std::string> unique_paths(evicted.begin(), evicted.end());
  for (const auto & path : unique_paths) {
    if (path_referenced(path)) {
      continue;
    }
    std::error_code ec;
    std::filesystem::remove_all(path, ec);
  }
}

std::vector<std::string> PgFaultStorage::store_rosbag_file_locked(const RosbagFileInfo & info, pqxx::work & tx) {
  RosbagFileInfo row = info;
  if (row.recording_id.empty()) {
    row.recording_id = rosbag_recording_id(row.file_path);
  }

  // Upserts on idx_rosbag_files_fault_path, i.e. on the (fault, recording) link.
  // Re-storing the SAME link refreshes it; a link to a DIFFERENT recording is a new
  // row now, which is the feature. Nothing is unlinked here - byte lifetime is the
  // cap's business below, and the caller's, after the commit.
  //
  // ON CONFLICT DO UPDATE keeps the row's identity, so a refresh does not move the
  // link to the end of the id order. Every read below breaks created_at_ns ties by
  // id, and the in-memory backend keeps its sequence number across a refresh, so a
  // delete-and-reinsert would put the two backends in a different order for a
  // re-stored row inside a tie group.
  tx.exec_params(
      "INSERT INTO rosbag_files "
      "(fault_code, recording_id, file_path, format, duration_sec, size_bytes, created_at_ns) "
      "VALUES ($1, $2, $3, $4, $5, $6, $7) "
      "ON CONFLICT (fault_code, file_path) DO UPDATE SET "
      "recording_id = EXCLUDED.recording_id, format = EXCLUDED.format, "
      "duration_sec = EXCLUDED.duration_sec, size_bytes = EXCLUDED.size_bytes, "
      "created_at_ns = EXCLUDED.created_at_ns",
      row.fault_code, row.recording_id, row.file_path, row.format, row.duration_sec,
      static_cast<int64_t>(row.size_bytes), row.created_at_ns);

  if (max_rosbags_per_fault_ == 0) {
    return {};  // unlimited per fault; only the global byte quota bounds this
  }

  // Keep the newest N recordings of this fault. Oldest-first eviction, the same
  // direction as evict_bags_over_quota, so the two eviction owners never need a
  // tiebreak. At N = 1 this reproduces the pre-#620 behaviour exactly: the new
  // recording replaces the old and the old bag is unlinked.
  //
  // NOTE: DELETE -> RETURNING is used to merge the SQLite SELECT+DELETE pair into one statement to minimise
  // transactions with the database and eliminate synchronization issues
  auto res = tx.exec_params(
      "DELETE FROM rosbag_files WHERE fault_code = $1 AND id NOT IN "
      "(SELECT id FROM rosbag_files WHERE fault_code = $1 ORDER BY created_at_ns DESC, id DESC LIMIT $2) "
      "RETURNING file_path",
      row.fault_code, static_cast<int64_t>(max_rosbags_per_fault_));

  std::set<std::string> evicted;
  for (const auto & r : res) {
    evicted.insert(r["file_path"].as<std::string>());
  }
  return {evicted.begin(), evicted.end()};
}

namespace {

/// Shared projection so every rosbag read decodes the same column order.
RosbagFileInfo read_rosbag_row(const pqxx::row & r) {
  RosbagFileInfo info;
  info.fault_code = r["fault_code"].as<std::string>();
  info.recording_id = r["recording_id"].as<std::string>();
  info.file_path = r["file_path"].as<std::string>();
  info.format = r["format"].as<std::string>();
  info.duration_sec = r["duration_sec"].as<double>();
  info.size_bytes = static_cast<size_t>(r["size_bytes"].as<int64_t>());
  info.created_at_ns = r["created_at_ns"].as<int64_t>();
  return info;
}

constexpr const char * kRosbagColumns =
    "fault_code, recording_id, file_path, format, duration_sec, size_bytes, created_at_ns";

}  // namespace

std::vector<RosbagFileInfo> PgFaultStorage::get_rosbag_files(const std::string & fault_code) const {
  std::lock_guard<std::mutex> lock(mutex_);
  pqxx::work tx(*db_conn_);

  try {
    auto res = tx.exec_params(std::string("SELECT ") + kRosbagColumns +
                                  " FROM rosbag_files WHERE fault_code = $1 ORDER BY created_at_ns DESC, id DESC",
                              fault_code);
    tx.commit();

    std::vector<RosbagFileInfo> result;
    for (const auto & r : res) {
      result.push_back(read_rosbag_row(r));
    }
    return result;
  } catch (const std::exception & e) {
    tx.abort();
    throw std::runtime_error(std::string("get_rosbag_files PostgreSQL error: ") + e.what());
  }
}

std::vector<RosbagFileInfo> PgFaultStorage::get_rosbag_files_by_recording(const std::string & recording_id) const {
  std::lock_guard<std::mutex> lock(mutex_);
  pqxx::work tx(*db_conn_);

  try {
    auto res = tx.exec_params(std::string("SELECT ") + kRosbagColumns +
                                  " FROM rosbag_files WHERE recording_id = $1 ORDER BY fault_code ASC",
                              recording_id);
    tx.commit();

    std::vector<RosbagFileInfo> result;
    for (const auto & r : res) {
      result.push_back(read_rosbag_row(r));
    }
    return result;
  } catch (const std::exception & e) {
    tx.abort();
    throw std::runtime_error(std::string("get_rosbag_files_by_recording PostgreSQL error: ") + e.what());
  }
}

size_t PgFaultStorage::delete_rosbag_recording(const std::string & recording_id) {
  // An empty id is not a recording that happens to be unnamed, it is a row whose
  // backfill did not finish. Matching on it would take every such row of every
  // fault with one DELETE, and evict_bags_over_quota calls this with whatever the
  // row held.
  if (recording_id.empty()) {
    return 0;
  }

  std::lock_guard<std::mutex> lock(mutex_);

  std::set<std::string> paths;
  size_t removed = 0;
  {
    // NOTE: The transaction object is scoped here, to allow for path_referenced() to use its own. The initial lock is
    pqxx::work tx(*db_conn_);
    try {
      // NOTE: DELETE -> RETURNING is used to merge the SQLite SELECT+DELETE pair into one statement to minimise
      // transactions with the database and eliminate synchronization issues
      auto res = tx.exec_params("DELETE FROM rosbag_files WHERE recording_id = $1 RETURNING file_path", recording_id);
      for (const auto & r : res) {
        paths.insert(r["file_path"].as<std::string>());
      }
      removed = res.affected_rows() > 0;
      tx.commit();
    } catch (const std::exception & e) {
      tx.abort();
      throw std::runtime_error(std::string("delete_rosbag_recording PostgreSQL error: ") + e.what());
    }
  }

  for (const auto & path : paths) {
    if (path_referenced(path)) {
      continue;  // another recording writes into the same directory - leave it
    }
    std::error_code ec;
    std::filesystem::remove_all(path, ec);
  }
  return removed;
}

std::optional<RosbagFileInfo> PgFaultStorage::get_rosbag_file(const std::string & fault_code) const {
  std::lock_guard<std::mutex> lock(mutex_);
  pqxx::work tx(*db_conn_);

  try {
    // ORDER BY is load-bearing now that a fault can hold several recordings. Without
    // it PostgreSQL may return any matching row, so the fault detail and the download
    // would serve an arbitrary recording - non-deterministically, which no test
    // catches reliably. id breaks the tie because a burst stamps one created_at_ns
    // across all its rows.
    auto res = tx.exec_params(
        "SELECT fault_code, recording_id, file_path, format, duration_sec, size_bytes, created_at_ns "
        "FROM rosbag_files WHERE fault_code = $1 "
        "ORDER BY created_at_ns DESC, id DESC LIMIT 1",
        fault_code);
    tx.commit();
    if (res.empty()) {
      return std::nullopt;
    }
    RosbagFileInfo info;
    info.fault_code = res[0]["fault_code"].as<std::string>();
    info.recording_id = res[0]["recording_id"].as<std::string>();
    info.file_path = res[0]["file_path"].as<std::string>();
    info.format = res[0]["format"].as<std::string>();
    info.duration_sec = res[0]["duration_sec"].as<double>();
    info.size_bytes = static_cast<size_t>(res[0]["size_bytes"].as<int64_t>());
    info.created_at_ns = res[0]["created_at_ns"].as<int64_t>();
    return info;
  } catch (const std::exception & e) {
    tx.abort();
    throw std::runtime_error(std::string("get_rosbag_file PostgreSQL error: ") + e.what());
  }
}

bool PgFaultStorage::delete_rosbag_file(const std::string & fault_code) {
  std::lock_guard<std::mutex> lock(mutex_);

  std::set<std::string> paths;
  bool deleted = false;
  // NOTE: The transaction object is scoped here, to allow for path_referenced() to use its own. The initial lock is
  // used universally
  {
    pqxx::work tx(*db_conn_);
    try {
      // Row first, file after. Deleting the bag before its row would, on a failed
      // DELETE, leave a surviving row pointing at a directory that is gone -
      // unreadable for good, and still charged against the storage quota, which sums
      // rows. This way the worst case is an orphaned directory instead.
      // NOTE: DELETE -> RETURNING is used to merge the SQLite SELECT+DELETE pair into one statement to minimise
      auto res = tx.exec_params("DELETE FROM rosbag_files WHERE fault_code = $1 RETURNING file_path", fault_code);
      // Every path, not the first one: a fault holds as many recordings as its cap
      // allows, and stepping (sqlite term) once would unlink one bag and leak the rest - rows gone,
      // directories left behind, uncounted by a quota that sums rows.
      for (const auto & r : res) {
        paths.insert(r["file_path"].as<std::string>());
      }

      deleted = res.affected_rows() > 0;
      tx.commit();
    } catch (const std::exception & e) {
      tx.abort();
      throw std::runtime_error(std::string("delete_rosbag_file PostgreSQL error: ") + e.what());
    }
  }

  // Unlink only once no fault references the bag any more. These rows are already
  // gone, so path_referenced() sees exactly the siblings of a shared recording.
  for (const auto & path : paths) {
    if (!path_referenced(path)) {
      std::error_code ec;
      std::filesystem::remove_all(path, ec);
      // Ignore errors - file may already be deleted
    }
  }

  return deleted;
}

size_t PgFaultStorage::delete_rosbag_files(const std::vector<std::string> & fault_codes) {
  if (fault_codes.empty()) {
    return 0;
  }
  std::lock_guard<std::mutex> lock(mutex_);

  // Rows go first, in one transaction, and the files only after the commit: a
  // crash mid-delete leaves at worst an orphaned directory, never a row whose
  // bag is already gone.
  std::set<std::string> paths;
  size_t deleted = 0;
  // NOTE: The transaction object is scoped here, to allow for path_referenced() to use its own. The initial lock is
  {
    pqxx::work tx(*db_conn_);
    try {
      for (const auto & code : fault_codes) {
        // RETURNING gives every path this code held, not just the first: one fault
        // code can name several recordings now, and the sweep must be able to
        // reclaim every one of their bags.
        // NOTE: DELETE -> RETURNING is used to merge the SQLite SELECT+DELETE pair into one statement to minimise
        auto res = tx.exec_params("DELETE FROM rosbag_files WHERE fault_code = $1 RETURNING file_path", code);
        for (const auto & r : res) {
          paths.insert(r["file_path"].as<std::string>());
        }
        if (res.affected_rows() > 0) {
          ++deleted;
        }
      }
      tx.commit();
    } catch (const std::exception & e) {
      tx.abort();
      throw std::runtime_error(std::string("delete_rosbag_files PostgreSQL error: ") + e.what());
    }
  }

  for (const auto & path : paths) {
    if (!path_referenced(path)) {
      std::error_code ec;
      std::filesystem::remove_all(path, ec);
      // Ignore errors - file may already be deleted
    }
  }
  return deleted;
}

bool PgFaultStorage::path_referenced(const std::string & file_path) const {
  pqxx::work tx(*db_conn_);
  try {
    auto res = tx.exec_params("SELECT COUNT(*) FROM rosbag_files WHERE file_path = $1", file_path);
    tx.commit();
    return res.affected_rows();
  } catch (const std::exception & e) {
    tx.abort();
    throw std::runtime_error(std::string("path_referenced PostgreSQL error: ") + e.what());
  }
}

size_t PgFaultStorage::get_total_rosbag_storage_bytes() const {
  std::lock_guard<std::mutex> lock(mutex_);
  pqxx::work tx(*db_conn_);

  // Sum per bag, not per fault: one recording can back a burst of correlated
  // faults, and double-counting it would evict bags that still fit the quota.
  try {
    auto res = tx.exec(
        "SELECT COALESCE(SUM(size_bytes),0) AS sz FROM (SELECT MAX(size_bytes) AS size_bytes FROM rosbag_files "
        "GROUP BY file_path)");
    tx.commit();
    return res.empty() ? 0 : res[0]["sz"].as<size_t>();
  } catch (const std::exception & e) {
    tx.abort();
    throw std::runtime_error(std::string("get_total_rosbag_storage_bytes PostgreSQL error: ") + e.what());
  }
}

std::vector<RosbagFileInfo> PgFaultStorage::get_all_rosbag_files() const {
  std::lock_guard<std::mutex> lock(mutex_);
  pqxx::work tx(*db_conn_);
  std::vector<RosbagFileInfo> result;

  try {
    auto res = tx.exec(
        "SELECT fault_code, recording_id, file_path, format, duration_sec, size_bytes, created_at_ns "
        "FROM rosbag_files ORDER BY created_at_ns ASC, id ASC");
    tx.commit();

    for (const auto & r : res) {
      RosbagFileInfo info;
      info.fault_code = r["fault_code"].as<std::string>();
      info.recording_id = r["recording_id"].as<std::string>();
      info.file_path = r["file_path"].as<std::string>();
      info.format = r["format"].as<std::string>();
      info.duration_sec = r["duration_sec"].as<double>();
      info.size_bytes = static_cast<size_t>(r["size_bytes"].as<int64_t>());
      info.created_at_ns = r["created_at_ns"].as<int64_t>();
      result.push_back(info);
    }

    return result;
  } catch (const std::exception & e) {
    tx.abort();
    throw std::runtime_error(std::string("get_all_rosbag_files PostgreSQL error: ") + e.what());
  }
}

std::vector<RosbagFileInfo> PgFaultStorage::list_rosbags_for_entity(const std::string & entity_fqn) const {
  std::lock_guard<std::mutex> lock(mutex_);
  pqxx::work tx(*db_conn_);

  std::vector<RosbagFileInfo> result;

  // Join rosbag_files with faults table and filter by reporting_sources containing entity_fqn.
  // Use json_each() for proper JSON array querying instead of LIKE, which treats
  // '_' as a single-char wildcard and would produce false positives on ROS names.
  // NOTE: Use jsonb as an equivalent of json_each on sqlite
  try {
    auto res = tx.exec_params(
        "SELECT r.fault_code, r.recording_id, r.file_path, r.format, r.duration_sec, r.size_bytes, r.created_at_ns "
        "FROM rosbag_files r "
        "JOIN faults f ON r.fault_code = f.fault_code "
        "JOIN jsonb_array_elements_text(f.reporting_sources::jsonb) AS j(value) ON j.value = $1 "
        "ORDER BY r.created_at_ns DESC, r.id DESC",
        entity_fqn);
    tx.commit();

    for (const auto & r : res) {
      RosbagFileInfo info;
      info.fault_code = r["fault_code"].as<std::string>();
      info.recording_id = r["recording_id"].as<std::string>();
      info.file_path = r["file_path"].as<std::string>();
      info.format = r["format"].as<std::string>();
      info.duration_sec = r["duration_sec"].as<double>();
      info.size_bytes = static_cast<size_t>(r["size_bytes"].as<int64_t>());
      info.created_at_ns = r["created_at_ns"].as<int64_t>();
      result.push_back(info);
    }

    return result;
  } catch (const std::exception & e) {
    tx.abort();
    throw std::runtime_error(std::string("list_rosbags_for_entity PostgreSQL error: ") + e.what());
  }
}

std::vector<ros2_medkit_msgs::msg::Fault> PgFaultStorage::get_all_faults() const {
  std::lock_guard<std::mutex> lock(mutex_);
  pqxx::work tx(*db_conn_);
  try {
    auto res = tx.exec(
        "SELECT fault_code, severity, description, first_occurred_ns, last_occurred_ns, occurrence_count, status, "
        "reporting_sources, last_passed_ns FROM faults");
    std::vector<ros2_medkit_msgs::msg::Fault> result;
    for (const auto & r : res) {
      ros2_medkit_msgs::msg::Fault fault;
      fault.fault_code = r["fault_code"].as<std::string>();
      fault.severity = static_cast<uint8_t>(r["severity"].as<int>());
      fault.description = r["description"].as<std::string>();
      fault.first_occurred = rclcpp::Time(r["first_occurred_ns"].as<int64_t>(), RCL_SYSTEM_TIME);
      fault.last_occurred = rclcpp::Time(r["last_occurred_ns"].as<int64_t>(), RCL_SYSTEM_TIME);
      fault.occurrence_count = static_cast<uint32_t>(r["occurrence_count"].as<int64_t>());
      fault.status = r["status"].as<std::string>();
      fault.reporting_sources = parse_json_array(r["reporting_sources"].as<std::string>());
      fault.last_passed = rclcpp::Time(r["last_passed_ns"].as<int64_t>(), RCL_SYSTEM_TIME);
      result.push_back(fault);
    }
    tx.commit();
    return result;
  } catch (const std::exception & e) {
    tx.abort();
    throw std::runtime_error(std::string("get_all_faults PostgreSQL error: ") + e.what());
  }
}

}  // namespace ros2_medkit_fault_manager
