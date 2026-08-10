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

#include "ros2_medkit_fault_manager/sqlite_fault_storage.hpp"

#include <filesystem>
#include <limits>
#include <set>
#include <sstream>
#include <stdexcept>

#include "rcutils/logging_macros.h"
#include "ros2_medkit_msgs/msg/fault.hpp"

namespace ros2_medkit_fault_manager {

namespace {

/// RAII wrapper for SQLite statements
class SqliteStatement {
 public:
  SqliteStatement(sqlite3 * db, const char * sql) : db_(db) {
    if (sqlite3_prepare_v2(db, sql, -1, &stmt_, nullptr) != SQLITE_OK) {
      throw std::runtime_error(std::string("Failed to prepare statement: ") + sqlite3_errmsg(db));
    }
  }

  ~SqliteStatement() {
    if (stmt_) {
      sqlite3_finalize(stmt_);
    }
  }

  SqliteStatement(const SqliteStatement &) = delete;
  SqliteStatement & operator=(const SqliteStatement &) = delete;
  SqliteStatement(SqliteStatement &&) = delete;
  SqliteStatement & operator=(SqliteStatement &&) = delete;

  sqlite3_stmt * get() const {
    return stmt_;
  }

  void bind_text(int index, const std::string & value) {
    const auto size = value.size();
    if (size > static_cast<std::size_t>(std::numeric_limits<int>::max())) {
      throw std::runtime_error("Failed to bind text: value size exceeds SQLite int length limit");
    }
    const auto length = static_cast<int>(size);
    if (sqlite3_bind_text(stmt_, index, value.c_str(), length, SQLITE_TRANSIENT) != SQLITE_OK) {
      throw std::runtime_error(std::string("Failed to bind text: ") + sqlite3_errmsg(db_));
    }
  }

  void bind_int(int index, int value) {
    if (sqlite3_bind_int(stmt_, index, value) != SQLITE_OK) {
      throw std::runtime_error(std::string("Failed to bind int: ") + sqlite3_errmsg(db_));
    }
  }

  void bind_int64(int index, int64_t value) {
    if (sqlite3_bind_int64(stmt_, index, value) != SQLITE_OK) {
      throw std::runtime_error(std::string("Failed to bind int64: ") + sqlite3_errmsg(db_));
    }
  }

  int step() {
    return sqlite3_step(stmt_);
  }

  void reset() {
    sqlite3_reset(stmt_);
    sqlite3_clear_bindings(stmt_);
  }

  std::string column_text(int index) {
    const auto * text = reinterpret_cast<const char *>(sqlite3_column_text(stmt_, index));
    return text ? std::string(text) : std::string();
  }

  int column_int(int index) {
    return sqlite3_column_int(stmt_, index);
  }

  int64_t column_int64(int index) {
    return sqlite3_column_int64(stmt_, index);
  }

 private:
  sqlite3 * db_;
  sqlite3_stmt * stmt_{nullptr};
};

}  // namespace

SqliteFaultStorage::SqliteFaultStorage(const std::string & db_path) : db_path_(db_path) {
  int flags = SQLITE_OPEN_READWRITE | SQLITE_OPEN_CREATE | SQLITE_OPEN_FULLMUTEX;
  if (sqlite3_open_v2(db_path.c_str(), &db_, flags, nullptr) != SQLITE_OK) {
    std::string error = db_ ? sqlite3_errmsg(db_) : "Unknown error";
    if (db_) {
      sqlite3_close(db_);
      db_ = nullptr;
    }
    throw std::runtime_error("Failed to open database '" + db_path + "': " + error);
  }

  // Enable WAL mode for better concurrent performance
  char * err_msg = nullptr;
  if (sqlite3_exec(db_, "PRAGMA journal_mode=WAL;", nullptr, nullptr, &err_msg) != SQLITE_OK) {
    std::string error = err_msg ? err_msg : "Unknown error";
    sqlite3_free(err_msg);
    sqlite3_close(db_);
    db_ = nullptr;
    throw std::runtime_error("Failed to enable WAL mode: " + error);
  }

  // Set busy timeout to handle concurrent access
  sqlite3_busy_timeout(db_, 5000);

  initialize_schema();
}

SqliteFaultStorage::~SqliteFaultStorage() {
  if (db_) {
    sqlite3_close(db_);
  }
}

void SqliteFaultStorage::set_debounce_config(const DebounceConfig & config) {
  std::lock_guard<std::mutex> lock(mutex_);
  config_ = config;
}

DebounceConfig SqliteFaultStorage::get_debounce_config() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return config_;
}

void SqliteFaultStorage::initialize_schema() {
  const char * create_faults_table_sql = R"(
    CREATE TABLE IF NOT EXISTS faults (
      fault_code TEXT PRIMARY KEY,
      severity INTEGER NOT NULL,
      description TEXT NOT NULL,
      first_occurred_ns INTEGER NOT NULL,
      last_occurred_ns INTEGER NOT NULL,
      occurrence_count INTEGER NOT NULL,
      status TEXT NOT NULL,
      reporting_sources TEXT NOT NULL,
      debounce_counter INTEGER NOT NULL DEFAULT 0,
      last_failed_ns INTEGER NOT NULL DEFAULT 0,
      last_passed_ns INTEGER NOT NULL DEFAULT 0,
      confirmed_at_ns INTEGER NOT NULL DEFAULT 0
    );
  )";

  char * err_msg = nullptr;
  if (sqlite3_exec(db_, create_faults_table_sql, nullptr, nullptr, &err_msg) != SQLITE_OK) {
    std::string error = err_msg ? err_msg : "Unknown error";
    sqlite3_free(err_msg);
    throw std::runtime_error("Failed to create faults table: " + error);
  }

  // Migration: databases created before confirmed_at_ns existed keep their rows;
  // the column arrives as 0 (= confirmation time unknown). Consumers (the
  // compliance timeline) treat 0 as "not recorded".
  {
    bool has_confirmed_at = false;
    SqliteStatement info(db_, "PRAGMA table_info(faults)");
    while (info.step() == SQLITE_ROW) {
      if (info.column_text(1) == "confirmed_at_ns") {
        has_confirmed_at = true;
        break;
      }
    }
    if (!has_confirmed_at) {
      if (sqlite3_exec(db_, "ALTER TABLE faults ADD COLUMN confirmed_at_ns INTEGER NOT NULL DEFAULT 0", nullptr,
                       nullptr, &err_msg) != SQLITE_OK) {
        std::string error = err_msg ? err_msg : "Unknown error";
        sqlite3_free(err_msg);
        throw std::runtime_error("Failed to add confirmed_at_ns column: " + error);
      }
    }
  }

  // Migration: releases that advanced last_occurred_ns on PASSED events left
  // inflated rows behind, and a latched CONFIRMED fault that only ever heals
  // would keep the wrong timestamp forever. last_failed_ns holds the true
  // last occurrence; last_occurred_ns can only exceed it via that old bug.
  if (sqlite3_exec(db_,
                   "UPDATE faults SET last_occurred_ns = last_failed_ns "
                   "WHERE last_failed_ns > 0 AND last_occurred_ns > last_failed_ns",
                   nullptr, nullptr, &err_msg) != SQLITE_OK) {
    std::string error = err_msg ? err_msg : "Unknown error";
    sqlite3_free(err_msg);
    throw std::runtime_error("Failed to repair last_occurred_ns rows: " + error);
  }

  // Create snapshots table for storing topic data captured when faults are confirmed
  const char * create_snapshots_table_sql = R"(
    CREATE TABLE IF NOT EXISTS snapshots (
      id INTEGER PRIMARY KEY AUTOINCREMENT,
      fault_code TEXT NOT NULL,
      topic TEXT NOT NULL,
      message_type TEXT NOT NULL,
      data TEXT NOT NULL,
      captured_at_ns INTEGER NOT NULL
    );
    CREATE INDEX IF NOT EXISTS idx_snapshots_fault_code ON snapshots(fault_code);
    CREATE INDEX IF NOT EXISTS idx_snapshots_fault_topic ON snapshots(fault_code, topic);
  )";

  if (sqlite3_exec(db_, create_snapshots_table_sql, nullptr, nullptr, &err_msg) != SQLITE_OK) {
    std::string error = err_msg ? err_msg : "Unknown error";
    sqlite3_free(err_msg);
    throw std::runtime_error("Failed to create snapshots table: " + error);
  }

  // Create freeze_frames table: one compact JSON dict of captured topic values per fault
  // code. Unlike snapshots, freeze frames are keyed by fault_code and are NOT removed on
  // clear_fault, so the confirmed-state record is retained after acknowledgement.
  const char * create_freeze_frames_table_sql = R"(
    CREATE TABLE IF NOT EXISTS freeze_frames (
      fault_code TEXT PRIMARY KEY,
      data TEXT NOT NULL,
      captured_at_ns INTEGER NOT NULL
    );
  )";

  if (sqlite3_exec(db_, create_freeze_frames_table_sql, nullptr, nullptr, &err_msg) != SQLITE_OK) {
    std::string error = err_msg ? err_msg : "Unknown error";
    sqlite3_free(err_msg);
    throw std::runtime_error("Failed to create freeze_frames table: " + error);
  }

  // Create rosbag_files table for storing time-window bag file metadata
  const char * create_rosbag_files_table_sql = R"(
    CREATE TABLE IF NOT EXISTS rosbag_files (
      id INTEGER PRIMARY KEY AUTOINCREMENT,
      fault_code TEXT NOT NULL UNIQUE,
      file_path TEXT NOT NULL,
      format TEXT NOT NULL,
      duration_sec REAL NOT NULL,
      size_bytes INTEGER NOT NULL,
      created_at_ns INTEGER NOT NULL
    );
    CREATE INDEX IF NOT EXISTS idx_rosbag_files_fault_code ON rosbag_files(fault_code);
    CREATE INDEX IF NOT EXISTS idx_rosbag_files_created_at ON rosbag_files(created_at_ns);
  )";

  if (sqlite3_exec(db_, create_rosbag_files_table_sql, nullptr, nullptr, &err_msg) != SQLITE_OK) {
    std::string error = err_msg ? err_msg : "Unknown error";
    sqlite3_free(err_msg);
    throw std::runtime_error("Failed to create rosbag_files table: " + error);
  }
}

std::vector<std::string> SqliteFaultStorage::parse_json_array(const std::string & json_str) {
  std::vector<std::string> result;

  // Simple JSON array parser for ["a", "b", "c"] format
  if (json_str.size() < 2 || json_str.front() != '[' || json_str.back() != ']') {
    if (!json_str.empty()) {
      RCUTILS_LOG_WARN_NAMED("sqlite_fault_storage", "Malformed JSON array in database: '%s'", json_str.c_str());
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

std::string SqliteFaultStorage::serialize_json_array(const std::vector<std::string> & vec) {
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

bool SqliteFaultStorage::report_fault_event(const std::string & fault_code, uint8_t event_type, uint8_t severity,
                                            const std::string & description, const std::string & source_id,
                                            const rclcpp::Time & timestamp, const DebounceConfig & config) {
  std::lock_guard<std::mutex> lock(mutex_);

  int64_t timestamp_ns = timestamp.nanoseconds();
  const bool is_failed = (event_type == EventType::EVENT_FAILED);

  // Check if fault exists
  SqliteStatement check_stmt(db_,
                             "SELECT severity, occurrence_count, reporting_sources, status, debounce_counter, "
                             "confirmed_at_ns, first_occurred_ns FROM faults WHERE fault_code = ?");
  check_stmt.bind_text(1, fault_code);

  if (check_stmt.step() == SQLITE_ROW) {
    // Fault exists - update it
    int existing_severity = check_stmt.column_int(0);
    int64_t existing_count = check_stmt.column_int64(1);
    std::string sources_json = check_stmt.column_text(2);
    std::string current_status = check_stmt.column_text(3);
    int32_t debounce_counter = check_stmt.column_int(4);
    int64_t confirmed_at_ns = check_stmt.column_int64(5);
    int64_t first_occurred_ns = check_stmt.column_int64(6);

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

      // Update with new values
      SqliteStatement update_stmt(
          db_, description.empty() ? "UPDATE faults SET severity = ?, last_occurred_ns = ?, last_failed_ns = ?, "
                                     "occurrence_count = ?, "
                                     "reporting_sources = ?, status = ?, debounce_counter = ?, confirmed_at_ns = ?, "
                                     "first_occurred_ns = ? WHERE fault_code = ?"
                                   : "UPDATE faults SET severity = ?, description = ?, last_occurred_ns = ?, "
                                     "last_failed_ns = ?, "
                                     "occurrence_count = ?, reporting_sources = ?, status = ?, debounce_counter = ?, "
                                     "confirmed_at_ns = ?, first_occurred_ns = ? WHERE fault_code = ?");

      if (description.empty()) {
        update_stmt.bind_int(1, new_severity);
        update_stmt.bind_int64(2, timestamp_ns);
        update_stmt.bind_int64(3, timestamp_ns);
        update_stmt.bind_int64(4, new_count);
        update_stmt.bind_text(5, serialize_json_array(sources));
        update_stmt.bind_text(6, new_status);
        update_stmt.bind_int(7, debounce_counter);
        update_stmt.bind_int64(8, confirmed_at_ns);
        update_stmt.bind_int64(9, first_occurred_ns);
        update_stmt.bind_text(10, fault_code);
      } else {
        update_stmt.bind_int(1, new_severity);
        update_stmt.bind_text(2, description);
        update_stmt.bind_int64(3, timestamp_ns);
        update_stmt.bind_int64(4, timestamp_ns);
        update_stmt.bind_int64(5, new_count);
        update_stmt.bind_text(6, serialize_json_array(sources));
        update_stmt.bind_text(7, new_status);
        update_stmt.bind_int(8, debounce_counter);
        update_stmt.bind_int64(9, confirmed_at_ns);
        update_stmt.bind_int64(10, first_occurred_ns);
        update_stmt.bind_text(11, fault_code);
      }

      if (update_stmt.step() != SQLITE_DONE) {
        throw std::runtime_error(std::string("Failed to update fault: ") + sqlite3_errmsg(db_));
      }
    } else {
      // PASSED event - increment towards healing, clamped to the thresholds.
      debounce_counter = clamp_debounce_counter(debounce_counter + 1, config);

      std::string new_status = compute_debounce_status(debounce_counter, current_status, config);

      // last_occurred_ns is deliberately NOT touched: a PASSED event is the fault
      // ENDING, not occurring. Bumping it made a long-stale CONFIRMED fault look
      // freshly active. The PASSED instant is recorded in last_passed_ns.
      SqliteStatement update_stmt(
          db_, "UPDATE faults SET last_passed_ns = ?, status = ?, debounce_counter = ? WHERE fault_code = ?");
      update_stmt.bind_int64(1, timestamp_ns);
      update_stmt.bind_text(2, new_status);
      update_stmt.bind_int(3, debounce_counter);
      update_stmt.bind_text(4, fault_code);

      if (update_stmt.step() != SQLITE_DONE) {
        throw std::runtime_error(std::string("Failed to update fault: ") + sqlite3_errmsg(db_));
      }
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

  // New fault - insert with debounce_counter = -1
  SqliteStatement insert_stmt(db_,
                              "INSERT INTO faults (fault_code, severity, description, first_occurred_ns, "
                              "last_occurred_ns, occurrence_count, status, reporting_sources, "
                              "debounce_counter, last_failed_ns, last_passed_ns, confirmed_at_ns) "
                              "VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)");

  const bool confirmed_now = initial_status == ros2_medkit_msgs::msg::Fault::STATUS_CONFIRMED;
  insert_stmt.bind_text(1, fault_code);
  insert_stmt.bind_int(2, static_cast<int>(severity));
  insert_stmt.bind_text(3, description);
  insert_stmt.bind_int64(4, timestamp_ns);
  insert_stmt.bind_int64(5, timestamp_ns);
  insert_stmt.bind_int(6, 1);  // occurrence_count = 1
  insert_stmt.bind_text(7, initial_status);
  insert_stmt.bind_text(8, serialize_json_array({source_id}));
  insert_stmt.bind_int(9, -1);               // debounce_counter = -1 for first FAILED
  insert_stmt.bind_int64(10, timestamp_ns);  // last_failed_ns
  insert_stmt.bind_int64(11, 0);             // last_passed_ns (never passed)
  insert_stmt.bind_int64(12, confirmed_now ? timestamp_ns : 0);

  if (insert_stmt.step() != SQLITE_DONE) {
    throw std::runtime_error(std::string("Failed to insert fault: ") + sqlite3_errmsg(db_));
  }

  return true;  // New fault created
}

std::vector<ros2_medkit_msgs::msg::Fault>
SqliteFaultStorage::list_faults(bool filter_by_severity, uint8_t severity,
                                const std::vector<std::string> & statuses) const {
  std::lock_guard<std::mutex> lock(mutex_);

  // Determine which statuses to include
  std::set<std::string> status_filter;
  if (statuses.empty()) {
    status_filter.insert(ros2_medkit_msgs::msg::Fault::STATUS_CONFIRMED);
  } else {
    for (const auto & s : statuses) {
      if (s == ros2_medkit_msgs::msg::Fault::STATUS_PREFAILED || s == ros2_medkit_msgs::msg::Fault::STATUS_PREPASSED ||
          s == ros2_medkit_msgs::msg::Fault::STATUS_CONFIRMED || s == ros2_medkit_msgs::msg::Fault::STATUS_HEALED ||
          s == ros2_medkit_msgs::msg::Fault::STATUS_CLEARED) {
        status_filter.insert(s);
      }
    }
    if (status_filter.empty()) {
      status_filter.insert(ros2_medkit_msgs::msg::Fault::STATUS_CONFIRMED);
    }
  }

  // Build query
  std::string sql =
      "SELECT fault_code, severity, description, first_occurred_ns, last_occurred_ns, "
      "occurrence_count, status, reporting_sources, last_passed_ns FROM faults WHERE status IN (";
  for (size_t i = 0; i < status_filter.size(); ++i) {
    if (i > 0) {
      sql += ", ";
    }
    sql += "?";
  }
  sql += ")";

  if (filter_by_severity) {
    sql += " AND severity = ?";
  }

  SqliteStatement stmt(db_, sql.c_str());

  int param_index = 1;
  for (const auto & s : status_filter) {
    stmt.bind_text(param_index++, s);
  }
  if (filter_by_severity) {
    stmt.bind_int(param_index, static_cast<int>(severity));
  }

  std::vector<ros2_medkit_msgs::msg::Fault> result;
  while (stmt.step() == SQLITE_ROW) {
    ros2_medkit_msgs::msg::Fault fault;
    fault.fault_code = stmt.column_text(0);
    fault.severity = static_cast<uint8_t>(stmt.column_int(1));
    fault.description = stmt.column_text(2);

    int64_t first_ns = stmt.column_int64(3);
    int64_t last_ns = stmt.column_int64(4);
    fault.first_occurred = rclcpp::Time(first_ns, RCL_SYSTEM_TIME);
    fault.last_occurred = rclcpp::Time(last_ns, RCL_SYSTEM_TIME);

    fault.occurrence_count = static_cast<uint32_t>(stmt.column_int64(5));
    fault.status = stmt.column_text(6);
    fault.reporting_sources = parse_json_array(stmt.column_text(7));
    fault.last_passed = rclcpp::Time(stmt.column_int64(8), RCL_SYSTEM_TIME);

    result.push_back(fault);
  }

  return result;
}

std::optional<ros2_medkit_msgs::msg::Fault> SqliteFaultStorage::get_fault(const std::string & fault_code) const {
  std::lock_guard<std::mutex> lock(mutex_);

  SqliteStatement stmt(db_,
                       "SELECT fault_code, severity, description, first_occurred_ns, last_occurred_ns, "
                       "occurrence_count, status, reporting_sources, last_passed_ns FROM faults WHERE fault_code = ?");
  stmt.bind_text(1, fault_code);

  if (stmt.step() != SQLITE_ROW) {
    return std::nullopt;
  }

  ros2_medkit_msgs::msg::Fault fault;
  fault.fault_code = stmt.column_text(0);
  fault.severity = static_cast<uint8_t>(stmt.column_int(1));
  fault.description = stmt.column_text(2);

  int64_t first_ns = stmt.column_int64(3);
  int64_t last_ns = stmt.column_int64(4);
  fault.first_occurred = rclcpp::Time(first_ns, RCL_SYSTEM_TIME);
  fault.last_occurred = rclcpp::Time(last_ns, RCL_SYSTEM_TIME);

  fault.occurrence_count = static_cast<uint32_t>(stmt.column_int64(5));
  fault.status = stmt.column_text(6);
  fault.reporting_sources = parse_json_array(stmt.column_text(7));
  fault.last_passed = rclcpp::Time(stmt.column_int64(8), RCL_SYSTEM_TIME);

  return fault;
}

bool SqliteFaultStorage::clear_fault(const std::string & fault_code) {
  std::lock_guard<std::mutex> lock(mutex_);

  // Delete associated snapshots when fault is cleared
  SqliteStatement delete_snapshots(db_, "DELETE FROM snapshots WHERE fault_code = ?");
  delete_snapshots.bind_text(1, fault_code);
  if (delete_snapshots.step() != SQLITE_DONE) {
    throw std::runtime_error(std::string("Failed to delete snapshots: ") + sqlite3_errmsg(db_));
  }

  SqliteStatement stmt(db_, "UPDATE faults SET status = ? WHERE fault_code = ?");
  stmt.bind_text(1, ros2_medkit_msgs::msg::Fault::STATUS_CLEARED);
  stmt.bind_text(2, fault_code);

  if (stmt.step() != SQLITE_DONE) {
    throw std::runtime_error(std::string("Failed to clear fault: ") + sqlite3_errmsg(db_));
  }

  return sqlite3_changes(db_) > 0;
}

std::vector<std::string> SqliteFaultStorage::reclassify_healed_as_cleared() {
  std::lock_guard<std::mutex> lock(mutex_);

  // Collect the codes that will flip first so the caller can audit each one. The
  // SELECT predicate mirrors the UPDATE exactly, and both run under the same lock,
  // so the returned list matches the rows actually reclassified below.
  std::vector<std::string> reclassified;
  {
    SqliteStatement select_stmt(db_, "SELECT fault_code FROM faults WHERE status = ?");
    select_stmt.bind_text(1, ros2_medkit_msgs::msg::Fault::STATUS_HEALED);
    while (select_stmt.step() == SQLITE_ROW) {
      reclassified.push_back(select_stmt.column_text(0));
    }
  }

  if (reclassified.empty()) {
    return reclassified;
  }

  // Drop snapshots for the affected faults so a reclassified row matches CLEARED semantics.
  SqliteStatement del(db_,
                      "DELETE FROM snapshots WHERE fault_code IN (SELECT fault_code FROM faults WHERE status = ?)");
  del.bind_text(1, ros2_medkit_msgs::msg::Fault::STATUS_HEALED);
  if (del.step() != SQLITE_DONE) {
    throw std::runtime_error(std::string("Failed to delete snapshots: ") + sqlite3_errmsg(db_));
  }

  SqliteStatement stmt(db_, "UPDATE faults SET status = ? WHERE status = ?");
  stmt.bind_text(1, ros2_medkit_msgs::msg::Fault::STATUS_CLEARED);
  stmt.bind_text(2, ros2_medkit_msgs::msg::Fault::STATUS_HEALED);
  if (stmt.step() != SQLITE_DONE) {
    throw std::runtime_error(std::string("Failed to reclassify HEALED faults: ") + sqlite3_errmsg(db_));
  }
  return reclassified;
}

size_t SqliteFaultStorage::size() const {
  std::lock_guard<std::mutex> lock(mutex_);

  SqliteStatement stmt(db_, "SELECT COUNT(*) FROM faults");

  if (stmt.step() != SQLITE_ROW) {
    return 0;
  }

  return static_cast<size_t>(stmt.column_int64(0));
}

bool SqliteFaultStorage::contains(const std::string & fault_code) const {
  std::lock_guard<std::mutex> lock(mutex_);

  SqliteStatement stmt(db_, "SELECT 1 FROM faults WHERE fault_code = ? LIMIT 1");
  stmt.bind_text(1, fault_code);

  return stmt.step() == SQLITE_ROW;
}

std::vector<std::string> SqliteFaultStorage::check_time_based_confirmation(const rclcpp::Time & current_time) {
  std::lock_guard<std::mutex> lock(mutex_);

  std::vector<std::string> confirmed;
  if (config_.auto_confirm_after_sec <= 0.0) {
    return confirmed;  // Time-based confirmation disabled
  }

  int64_t current_ns = current_time.nanoseconds();
  int64_t threshold_ns = static_cast<int64_t>(config_.auto_confirm_after_sec * 1e9);
  int64_t cutoff_ns = current_ns - threshold_ns;

  // Collect the codes that will flip first so the caller can audit each one. The
  // SELECT predicate mirrors the UPDATE exactly, and both run under the same lock,
  // so the returned list matches the rows actually confirmed below.
  {
    SqliteStatement select_stmt(
        db_, "SELECT fault_code FROM faults WHERE status = ? AND last_failed_ns <= ? AND last_failed_ns > 0");
    select_stmt.bind_text(1, ros2_medkit_msgs::msg::Fault::STATUS_PREFAILED);
    select_stmt.bind_int64(2, cutoff_ns);
    while (select_stmt.step() == SQLITE_ROW) {
      confirmed.push_back(select_stmt.column_text(0));
    }
  }

  if (confirmed.empty()) {
    return confirmed;
  }

  SqliteStatement update_stmt(db_,
                              "UPDATE faults SET status = ?, confirmed_at_ns = ? "
                              "WHERE status = ? AND last_failed_ns <= ? AND last_failed_ns > 0");
  update_stmt.bind_text(1, ros2_medkit_msgs::msg::Fault::STATUS_CONFIRMED);
  update_stmt.bind_int64(2, current_ns);
  update_stmt.bind_text(3, ros2_medkit_msgs::msg::Fault::STATUS_PREFAILED);
  update_stmt.bind_int64(4, cutoff_ns);

  if (update_stmt.step() != SQLITE_DONE) {
    throw std::runtime_error(std::string("Failed to confirm faults: ") + sqlite3_errmsg(db_));
  }

  return confirmed;
}

void SqliteFaultStorage::set_max_snapshots_per_fault(size_t max_count) {
  std::lock_guard<std::mutex> lock(mutex_);
  max_snapshots_per_fault_ = max_count;
}

void SqliteFaultStorage::store_snapshot(const SnapshotData & snapshot) {
  std::lock_guard<std::mutex> lock(mutex_);

  // Check snapshot limit per fault (reject-new strategy: keep earliest)
  if (max_snapshots_per_fault_ > 0) {
    SqliteStatement count_stmt(db_, "SELECT COUNT(*) FROM snapshots WHERE fault_code = ?");
    count_stmt.bind_text(1, snapshot.fault_code);
    if (count_stmt.step() == SQLITE_ROW &&
        count_stmt.column_int64(0) >= static_cast<int64_t>(max_snapshots_per_fault_)) {
      // Silent rejection: storage layer has no logger. Callers should log if needed.
      return;  // Reject new - keep first N inserted snapshots
    }
  }

  SqliteStatement stmt(db_,
                       "INSERT INTO snapshots (fault_code, topic, message_type, data, captured_at_ns) "
                       "VALUES (?, ?, ?, ?, ?)");

  stmt.bind_text(1, snapshot.fault_code);
  stmt.bind_text(2, snapshot.topic);
  stmt.bind_text(3, snapshot.message_type);
  stmt.bind_text(4, snapshot.data);
  stmt.bind_int64(5, snapshot.captured_at_ns);

  if (stmt.step() != SQLITE_DONE) {
    throw std::runtime_error(std::string("Failed to store snapshot: ") + sqlite3_errmsg(db_));
  }
}

std::vector<SnapshotData> SqliteFaultStorage::get_snapshots(const std::string & fault_code,
                                                            const std::string & topic_filter) const {
  std::lock_guard<std::mutex> lock(mutex_);

  std::vector<SnapshotData> result;

  std::string sql =
      "SELECT fault_code, topic, message_type, data, captured_at_ns FROM snapshots WHERE fault_code "
      "= ?";
  if (!topic_filter.empty()) {
    sql += " AND topic = ?";
  }
  sql += " ORDER BY captured_at_ns DESC";

  SqliteStatement stmt(db_, sql.c_str());
  stmt.bind_text(1, fault_code);
  if (!topic_filter.empty()) {
    stmt.bind_text(2, topic_filter);
  }

  while (stmt.step() == SQLITE_ROW) {
    SnapshotData snapshot;
    snapshot.fault_code = stmt.column_text(0);
    snapshot.topic = stmt.column_text(1);
    snapshot.message_type = stmt.column_text(2);
    snapshot.data = stmt.column_text(3);
    snapshot.captured_at_ns = stmt.column_int64(4);
    result.push_back(snapshot);
  }

  return result;
}

void SqliteFaultStorage::store_freeze_frame(const FreezeFrameData & frame) {
  std::lock_guard<std::mutex> lock(mutex_);

  // Keyed by fault_code (PRIMARY KEY): a re-confirm replaces the previous frame.
  SqliteStatement stmt(db_,
                       "INSERT OR REPLACE INTO freeze_frames (fault_code, data, captured_at_ns) "
                       "VALUES (?, ?, ?)");
  stmt.bind_text(1, frame.fault_code);
  stmt.bind_text(2, frame.data);
  stmt.bind_int64(3, frame.captured_at_ns);

  if (stmt.step() != SQLITE_DONE) {
    throw std::runtime_error(std::string("Failed to store freeze frame: ") + sqlite3_errmsg(db_));
  }
}

std::optional<FreezeFrameData> SqliteFaultStorage::get_freeze_frame(const std::string & fault_code) const {
  std::lock_guard<std::mutex> lock(mutex_);

  SqliteStatement stmt(db_, "SELECT fault_code, data, captured_at_ns FROM freeze_frames WHERE fault_code = ?");
  stmt.bind_text(1, fault_code);

  if (stmt.step() != SQLITE_ROW) {
    return std::nullopt;
  }

  FreezeFrameData frame;
  frame.fault_code = stmt.column_text(0);
  frame.data = stmt.column_text(1);
  frame.captured_at_ns = stmt.column_int64(2);
  return frame;
}

void SqliteFaultStorage::exec_or_throw(const char * sql) {
  char * err_msg = nullptr;
  if (sqlite3_exec(db_, sql, nullptr, nullptr, &err_msg) != SQLITE_OK) {
    std::string msg = err_msg ? err_msg : "unknown error";
    sqlite3_free(err_msg);
    throw std::runtime_error(std::string("Failed to run '") + sql + "': " + msg);
  }
}

void SqliteFaultStorage::store_rosbag_file(const RosbagFileInfo & info) {
  std::lock_guard<std::mutex> lock(mutex_);
  const auto replaced = store_rosbag_file_locked(info);
  if (replaced) {
    std::error_code ec;
    std::filesystem::remove_all(*replaced, ec);
  }
}

void SqliteFaultStorage::store_rosbag_files(const std::vector<RosbagFileInfo> & infos) {
  if (infos.empty()) {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);

  // One transaction for the whole burst: a crash mid-store must not leave some
  // faults of the shared recording without their lookup row. Replaced bags are
  // unlinked only after COMMIT - a ROLLBACK resurrects the old rows, which must
  // keep pointing at bags that still exist.
  std::vector<std::string> replaced;
  exec_or_throw("BEGIN IMMEDIATE");
  try {
    for (const auto & info : infos) {
      if (auto old_path = store_rosbag_file_locked(info)) {
        replaced.push_back(*std::move(old_path));
      }
    }
    exec_or_throw("COMMIT");
  } catch (...) {
    sqlite3_exec(db_, "ROLLBACK", nullptr, nullptr, nullptr);
    throw;
  }
  for (const auto & path : replaced) {
    std::error_code ec;
    std::filesystem::remove_all(path, ec);
  }
}

std::optional<std::string> SqliteFaultStorage::store_rosbag_file_locked(const RosbagFileInfo & info) {
  // A re-confirm replaces the row; report the orphaned old bag to the caller,
  // which unlinks it once the row change is durable.
  std::optional<std::string> replaced;
  {
    SqliteStatement query_stmt(db_, "SELECT file_path FROM rosbag_files WHERE fault_code = ?");
    query_stmt.bind_text(1, info.fault_code);
    if (query_stmt.step() == SQLITE_ROW) {
      std::string old_path = query_stmt.column_text(0);
      if (old_path != info.file_path && !path_shared_with_other_fault(old_path, info.fault_code)) {
        replaced = std::move(old_path);
      }
    }
  }

  // Use INSERT OR REPLACE to handle updates (fault_code is UNIQUE)
  SqliteStatement stmt(db_,
                       "INSERT OR REPLACE INTO rosbag_files "
                       "(fault_code, file_path, format, duration_sec, size_bytes, created_at_ns) "
                       "VALUES (?, ?, ?, ?, ?, ?)");

  stmt.bind_text(1, info.fault_code);
  stmt.bind_text(2, info.file_path);
  stmt.bind_text(3, info.format);
  // Bind duration_sec as a double using sqlite3_bind_double directly
  if (sqlite3_bind_double(stmt.get(), 4, info.duration_sec) != SQLITE_OK) {
    throw std::runtime_error(std::string("Failed to bind duration_sec: ") + sqlite3_errmsg(db_));
  }
  stmt.bind_int64(5, static_cast<int64_t>(info.size_bytes));
  stmt.bind_int64(6, info.created_at_ns);

  if (stmt.step() != SQLITE_DONE) {
    throw std::runtime_error(std::string("Failed to store rosbag file: ") + sqlite3_errmsg(db_));
  }
  return replaced;
}

std::optional<RosbagFileInfo> SqliteFaultStorage::get_rosbag_file(const std::string & fault_code) const {
  std::lock_guard<std::mutex> lock(mutex_);

  SqliteStatement stmt(db_,
                       "SELECT fault_code, file_path, format, duration_sec, size_bytes, created_at_ns "
                       "FROM rosbag_files WHERE fault_code = ?");
  stmt.bind_text(1, fault_code);

  if (stmt.step() != SQLITE_ROW) {
    return std::nullopt;
  }

  RosbagFileInfo info;
  info.fault_code = stmt.column_text(0);
  info.file_path = stmt.column_text(1);
  info.format = stmt.column_text(2);
  info.duration_sec = sqlite3_column_double(stmt.get(), 3);
  info.size_bytes = static_cast<size_t>(stmt.column_int64(4));
  info.created_at_ns = stmt.column_int64(5);

  return info;
}

bool SqliteFaultStorage::delete_rosbag_file(const std::string & fault_code) {
  std::lock_guard<std::mutex> lock(mutex_);

  // First get the file path so we can delete the actual file
  std::string file_path;
  {
    SqliteStatement select_stmt(db_, "SELECT file_path FROM rosbag_files WHERE fault_code = ?");
    select_stmt.bind_text(1, fault_code);
    if (select_stmt.step() == SQLITE_ROW) {
      file_path = select_stmt.column_text(0);
    }
  }

  // Try to delete the actual file/directory, unless a sibling fault still
  // points at it. Checked before the DELETE so the exclusion is explicit.
  if (!file_path.empty() && !path_shared_with_other_fault(file_path, fault_code)) {
    std::error_code ec;
    std::filesystem::remove_all(file_path, ec);
    // Ignore errors - file may already be deleted
  }

  SqliteStatement delete_stmt(db_, "DELETE FROM rosbag_files WHERE fault_code = ?");
  delete_stmt.bind_text(1, fault_code);

  if (delete_stmt.step() != SQLITE_DONE) {
    throw std::runtime_error(std::string("Failed to delete rosbag file record: ") + sqlite3_errmsg(db_));
  }

  return sqlite3_changes(db_) > 0;
}

size_t SqliteFaultStorage::delete_rosbag_files(const std::vector<std::string> & fault_codes) {
  if (fault_codes.empty()) {
    return 0;
  }
  std::lock_guard<std::mutex> lock(mutex_);

  // Rows go first, in one transaction, and the file only after the commit: a
  // crash mid-delete leaves at worst an orphaned directory, never a row whose
  // bag is already gone.
  std::set<std::string> paths;
  size_t deleted = 0;
  exec_or_throw("BEGIN IMMEDIATE");
  try {
    for (const auto & code : fault_codes) {
      {
        SqliteStatement select_stmt(db_, "SELECT file_path FROM rosbag_files WHERE fault_code = ?");
        select_stmt.bind_text(1, code);
        if (select_stmt.step() == SQLITE_ROW) {
          paths.insert(select_stmt.column_text(0));
        }
      }
      SqliteStatement delete_stmt(db_, "DELETE FROM rosbag_files WHERE fault_code = ?");
      delete_stmt.bind_text(1, code);
      if (delete_stmt.step() != SQLITE_DONE) {
        throw std::runtime_error(std::string("Failed to delete rosbag file record: ") + sqlite3_errmsg(db_));
      }
      if (sqlite3_changes(db_) > 0) {
        ++deleted;
      }
    }
    exec_or_throw("COMMIT");
  } catch (...) {
    sqlite3_exec(db_, "ROLLBACK", nullptr, nullptr, nullptr);
    throw;
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

bool SqliteFaultStorage::path_shared_with_other_fault(const std::string & file_path,
                                                      const std::string & fault_code) const {
  SqliteStatement stmt(db_, "SELECT COUNT(*) FROM rosbag_files WHERE file_path = ? AND fault_code != ?");
  stmt.bind_text(1, file_path);
  stmt.bind_text(2, fault_code);
  return stmt.step() == SQLITE_ROW && stmt.column_int64(0) > 0;
}

bool SqliteFaultStorage::path_referenced(const std::string & file_path) const {
  SqliteStatement stmt(db_, "SELECT COUNT(*) FROM rosbag_files WHERE file_path = ?");
  stmt.bind_text(1, file_path);
  return stmt.step() == SQLITE_ROW && stmt.column_int64(0) > 0;
}

size_t SqliteFaultStorage::get_total_rosbag_storage_bytes() const {
  std::lock_guard<std::mutex> lock(mutex_);

  // Sum per bag, not per fault: one recording can back a burst of correlated
  // faults, and double-counting it would evict bags that still fit the quota.
  SqliteStatement stmt(
      db_,
      "SELECT COALESCE(SUM(size_bytes), 0) FROM (SELECT MAX(size_bytes) AS size_bytes FROM rosbag_files "
      "GROUP BY file_path)");

  if (stmt.step() != SQLITE_ROW) {
    return 0;
  }

  return static_cast<size_t>(stmt.column_int64(0));
}

std::vector<RosbagFileInfo> SqliteFaultStorage::get_all_rosbag_files() const {
  std::lock_guard<std::mutex> lock(mutex_);

  std::vector<RosbagFileInfo> result;

  SqliteStatement stmt(db_,
                       "SELECT fault_code, file_path, format, duration_sec, size_bytes, created_at_ns "
                       "FROM rosbag_files ORDER BY created_at_ns ASC");

  while (stmt.step() == SQLITE_ROW) {
    RosbagFileInfo info;
    info.fault_code = stmt.column_text(0);
    info.file_path = stmt.column_text(1);
    info.format = stmt.column_text(2);
    info.duration_sec = sqlite3_column_double(stmt.get(), 3);
    info.size_bytes = static_cast<size_t>(stmt.column_int64(4));
    info.created_at_ns = stmt.column_int64(5);
    result.push_back(info);
  }

  return result;
}

std::vector<RosbagFileInfo> SqliteFaultStorage::list_rosbags_for_entity(const std::string & entity_fqn) const {
  std::lock_guard<std::mutex> lock(mutex_);

  std::vector<RosbagFileInfo> result;

  // Join rosbag_files with faults table and filter by reporting_sources containing entity_fqn.
  // Use json_each() for proper JSON array querying instead of LIKE, which treats
  // '_' as a single-char wildcard and would produce false positives on ROS names.
  SqliteStatement stmt(db_,
                       "SELECT r.fault_code, r.file_path, r.format, r.duration_sec, r.size_bytes, "
                       "r.created_at_ns "
                       "FROM rosbag_files r "
                       "JOIN faults f ON r.fault_code = f.fault_code "
                       "JOIN json_each(f.reporting_sources) j ON j.value = ?");

  stmt.bind_text(1, entity_fqn);

  while (stmt.step() == SQLITE_ROW) {
    RosbagFileInfo info;
    info.fault_code = stmt.column_text(0);
    info.file_path = stmt.column_text(1);
    info.format = stmt.column_text(2);
    info.duration_sec = sqlite3_column_double(stmt.get(), 3);
    info.size_bytes = static_cast<size_t>(stmt.column_int64(4));
    info.created_at_ns = stmt.column_int64(5);
    result.push_back(info);
  }

  return result;
}

std::vector<ros2_medkit_msgs::msg::Fault> SqliteFaultStorage::get_all_faults() const {
  std::lock_guard<std::mutex> lock(mutex_);

  SqliteStatement stmt(db_,
                       "SELECT fault_code, severity, description, first_occurred_ns, last_occurred_ns, "
                       "occurrence_count, status, reporting_sources, last_passed_ns FROM faults");

  std::vector<ros2_medkit_msgs::msg::Fault> result;
  while (stmt.step() == SQLITE_ROW) {
    ros2_medkit_msgs::msg::Fault fault;
    fault.fault_code = stmt.column_text(0);
    fault.severity = static_cast<uint8_t>(stmt.column_int(1));
    fault.description = stmt.column_text(2);

    int64_t first_ns = stmt.column_int64(3);
    int64_t last_ns = stmt.column_int64(4);
    fault.first_occurred = rclcpp::Time(first_ns, RCL_SYSTEM_TIME);
    fault.last_occurred = rclcpp::Time(last_ns, RCL_SYSTEM_TIME);

    fault.occurrence_count = static_cast<uint32_t>(stmt.column_int64(5));
    fault.status = stmt.column_text(6);
    fault.reporting_sources = parse_json_array(stmt.column_text(7));
    fault.last_passed = rclcpp::Time(stmt.column_int64(8), RCL_SYSTEM_TIME);

    result.push_back(fault);
  }

  return result;
}

}  // namespace ros2_medkit_fault_manager
