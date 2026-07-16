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

#include "ros2_medkit_fault_manager/fault_manager_node.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cctype>
#include <cinttypes>
#include <filesystem>
#include <fstream>
#include <nlohmann/json.hpp>
#include <sstream>

#include "ros2_medkit_fault_manager/correlation/config_parser.hpp"
#include "ros2_medkit_fault_manager/sqlite_fault_storage.hpp"
#include "ros2_medkit_fault_manager/time_utils.hpp"
#include "ros2_medkit_msgs/msg/cluster_info.hpp"
#include "ros2_medkit_msgs/msg/environment_data.hpp"
#include "ros2_medkit_msgs/msg/extended_data_records.hpp"
#include "ros2_medkit_msgs/msg/muted_fault_info.hpp"
#include "ros2_medkit_msgs/msg/snapshot.hpp"

namespace ros2_medkit_fault_manager {

namespace {

/// Maximum allowed length for fault_code
constexpr size_t kMaxFaultCodeLength = 128;

/// Validate fault_code format
/// @param fault_code The fault code to validate
/// @return Empty string if valid, error message if invalid
std::string validate_fault_code(const std::string & fault_code) {
  if (fault_code.empty()) {
    return "fault_code cannot be empty";
  }

  if (fault_code.length() > kMaxFaultCodeLength) {
    return "fault_code exceeds maximum length of " + std::to_string(kMaxFaultCodeLength);
  }

  // Allow only alphanumeric, underscore, hyphen, and dot
  // This prevents path traversal (../) and header injection (\r\n, ")
  for (char c : fault_code) {
    if (!std::isalnum(static_cast<unsigned char>(c)) && c != '_' && c != '-' && c != '.') {
      return "fault_code contains invalid character '" + std::string(1, c) +
             "'. Only alphanumeric, underscore, hyphen, and dot are allowed";
    }
  }

  // Prevent path traversal patterns
  if (fault_code.find("..") != std::string::npos) {
    return "fault_code cannot contain '..'";
  }

  return "";  // Valid
}

}  // namespace

FaultManagerNode::FaultManagerNode(const rclcpp::NodeOptions & options) : Node("fault_manager", options) {
  // Declare and get parameters
  storage_type_ = declare_parameter<std::string>("storage_type", "sqlite");
  database_path_ = declare_parameter<std::string>("database_path", "/var/lib/ros2_medkit/faults.db");

  auto confirmation_threshold_param = declare_parameter<int>("confirmation_threshold", -1);
  if (confirmation_threshold_param > 0) {
    RCLCPP_WARN(get_logger(),
                "confirmation_threshold should be <= 0 (0 or -1 = immediate confirmation), got %d. Using %d.",
                static_cast<int>(confirmation_threshold_param), static_cast<int>(-confirmation_threshold_param));
    confirmation_threshold_param = -confirmation_threshold_param;
  }
  confirmation_threshold_ = static_cast<int32_t>(confirmation_threshold_param);

  // Healing parameters
  healing_enabled_ = declare_parameter<bool>("healing_enabled", false);
  auto healing_threshold_param = declare_parameter<int>("healing_threshold", 3);
  if (healing_threshold_param < 0) {
    RCLCPP_WARN(get_logger(), "healing_threshold should be >= 0, got %d. Using %d.",
                static_cast<int>(healing_threshold_param), static_cast<int>(-healing_threshold_param));
    healing_threshold_param = -healing_threshold_param;
  }
  healing_threshold_ = static_cast<int32_t>(healing_threshold_param);

  // Time-based auto-confirmation parameter
  auto_confirm_after_sec_ = declare_parameter<double>("auto_confirm_after_sec", 0.0);
  if (auto_confirm_after_sec_ < 0.0) {
    RCLCPP_WARN(get_logger(), "auto_confirm_after_sec should be >= 0, got %.2f. Disabling.", auto_confirm_after_sec_);
    auto_confirm_after_sec_ = 0.0;
  }

  // Capture cooldown parameters (gates both snapshot and rosbag capture)
  snapshot_recapture_cooldown_sec_ = declare_parameter<double>("snapshots.recapture_cooldown_sec", 60.0);
  if (snapshot_recapture_cooldown_sec_ < 0.0) {
    RCLCPP_WARN(get_logger(), "snapshots.recapture_cooldown_sec should be >= 0, got %.2f. Disabling.",
                snapshot_recapture_cooldown_sec_);
    snapshot_recapture_cooldown_sec_ = 0.0;
  }

  // Capture concurrency bound (issue #441): cap concurrent capture threads and
  // bound the pending queue so a fault storm cannot grow memory without limit.
  // declare_parameter<int> returns int64_t on Jazzy; the static_cast<int> keeps -Wconversion quiet.
  capture_pool_size_ = static_cast<int>(declare_parameter<int>("snapshots.capture_pool_size", 2));
  if (capture_pool_size_ < 1) {
    RCLCPP_WARN(get_logger(), "snapshots.capture_pool_size must be >= 1, got %ld. Clamping to 1",
                static_cast<long>(capture_pool_size_));
    capture_pool_size_ = 1;
  }

  capture_queue_depth_ = static_cast<int>(declare_parameter<int>("snapshots.capture_queue_depth", 16));
  if (capture_queue_depth_ < 1) {
    RCLCPP_WARN(get_logger(), "snapshots.capture_queue_depth must be >= 1, got %ld. Clamping to 1",
                static_cast<long>(capture_queue_depth_));
    capture_queue_depth_ = 1;
  }

  const std::string policy_str = declare_parameter<std::string>("snapshots.capture_queue_full_policy", "reject_newest");
  if (policy_str == "reject_newest") {
    capture_queue_full_policy_ = QueueFullPolicy::kRejectNewest;
  } else if (policy_str == "drop_oldest") {
    capture_queue_full_policy_ = QueueFullPolicy::kDropOldest;
  } else {
    RCLCPP_WARN(get_logger(),
                "snapshots.capture_queue_full_policy '%s' is invalid (expected 'reject_newest' or "
                "'drop_oldest'). Using 'reject_newest'",
                policy_str.c_str());
    capture_queue_full_policy_ = QueueFullPolicy::kRejectNewest;
  }

  RCLCPP_INFO(get_logger(), "Capture concurrency: pool_size=%d, queue_depth=%d, policy=%s", capture_pool_size_,
              capture_queue_depth_,
              capture_queue_full_policy_ == QueueFullPolicy::kRejectNewest ? "reject_newest" : "drop_oldest");

  auto max_snapshots = declare_parameter<int>("snapshots.max_per_fault", 10);
  if (max_snapshots < 0) {
    RCLCPP_WARN(get_logger(), "snapshots.max_per_fault should be >= 0, got %ld. Disabling limit.", max_snapshots);
    max_snapshots = 0;
  }

  // Create storage backend
  storage_ = create_storage();

  // Create the tamper-evident fault audit log (disabled by default)
  audit_log_ = create_audit_log();

  // Apply snapshot limit to storage
  if (max_snapshots > 0) {
    storage_->set_max_snapshots_per_fault(static_cast<size_t>(max_snapshots));
  }

  // Create event publisher for SSE streaming
  event_publisher_ = create_publisher<ros2_medkit_msgs::msg::FaultEvent>("~/events", rclcpp::QoS(100).reliable());

  // Build global debounce config
  global_config_.confirmation_threshold = confirmation_threshold_;
  global_config_.healing_enabled = healing_enabled_;
  global_config_.healing_threshold = healing_threshold_;
  global_config_.auto_confirm_after_sec = auto_confirm_after_sec_;
  if (!sanitize_debounce_config(global_config_)) {
    RCLCPP_WARN(get_logger(),
                "Invalid debounce thresholds (need confirmation_threshold < 0 <= healing_threshold); using safe "
                "defaults: confirmation=%d healing=%d",
                global_config_.confirmation_threshold, global_config_.healing_threshold);
  }
  storage_->set_debounce_config(global_config_);

  // One-time cleanup: when healing is disabled, a HEALED row left by a previous (healing-enabled) run
  // would behave inconsistently under the latch, so reclassify it as CLEARED at startup. Each flipped
  // fault is audited (the audit log is already constructed above); without this the reclassification
  // would be invisible to the audit log's verify().
  if (!global_config_.healing_enabled) {
    const auto reclassified = storage_->reclassify_healed_as_cleared();
    if (!reclassified.empty()) {
      if (audit_log_) {
        const int64_t reclassified_at_ns = get_wall_clock_time().nanoseconds();
        for (const auto & fault_code : reclassified) {
          auto fault = storage_->get_fault(fault_code);
          if (fault) {
            audit_transition(kTransitionCleared, *fault, "startup_reclassify", reclassified_at_ns);
          }
        }
      }
      RCLCPP_INFO(get_logger(), "Healing disabled: reclassified %zu stale HEALED fault(s) as CLEARED",
                  reclassified.size());
    }
  }

  // Load per-entity threshold overrides (optional)
  auto entity_thresholds_file = declare_parameter<std::string>("entity_thresholds.config_file", "");
  if (!entity_thresholds_file.empty()) {
    auto entries = EntityThresholdResolver::load_from_yaml(entity_thresholds_file);
    // Validate each override against the global config: the merge is field by field, so an override
    // can break confirmation_threshold < 0 <= healing_threshold even when the global config is valid.
    for (auto & entry : entries) {
      DebounceConfig merged = global_config_;
      if (entry.confirmation_threshold) {
        merged.confirmation_threshold = *entry.confirmation_threshold;
      }
      if (entry.healing_threshold) {
        merged.healing_threshold = *entry.healing_threshold;
      }
      if (!sanitize_debounce_config(merged)) {
        RCLCPP_WARN(get_logger(),
                    "Entity '%s' debounce thresholds invalid (need confirmation_threshold < 0 <= healing_threshold); "
                    "using safe defaults",
                    entry.prefix.c_str());
        if (entry.confirmation_threshold) {
          entry.confirmation_threshold = merged.confirmation_threshold;
        }
        if (entry.healing_threshold) {
          entry.healing_threshold = merged.healing_threshold;
        }
      }
    }
    if (!entries.empty()) {
      threshold_resolver_ = std::make_unique<EntityThresholdResolver>(std::move(entries));
      RCLCPP_INFO(get_logger(), "Loaded %zu per-entity threshold overrides from %s", threshold_resolver_->size(),
                  entity_thresholds_file.c_str());
      if (auto_confirm_after_sec_ > 0.0) {
        RCLCPP_WARN(get_logger(),
                    "Per-entity thresholds are configured but auto_confirm_after_sec=%.1f is also set. "
                    "Auto-confirmation will bypass entity-specific debounce policies for PREFAILED faults.",
                    auto_confirm_after_sec_);
      }
    }
  }

  // Create service servers
  report_fault_srv_ = create_service<ros2_medkit_msgs::srv::ReportFault>(
      "~/report_fault", [this](const std::shared_ptr<ros2_medkit_msgs::srv::ReportFault::Request> & request,
                               const std::shared_ptr<ros2_medkit_msgs::srv::ReportFault::Response> & response) {
        handle_report_fault(request, response);
      });

  list_faults_srv_ = create_service<ros2_medkit_msgs::srv::ListFaults>(
      "~/list_faults", [this](const std::shared_ptr<ros2_medkit_msgs::srv::ListFaults::Request> & request,
                              const std::shared_ptr<ros2_medkit_msgs::srv::ListFaults::Response> & response) {
        handle_list_faults(request, response);
      });

  get_fault_srv_ = create_service<ros2_medkit_msgs::srv::GetFault>(
      "~/get_fault", [this](const std::shared_ptr<ros2_medkit_msgs::srv::GetFault::Request> & request,
                            const std::shared_ptr<ros2_medkit_msgs::srv::GetFault::Response> & response) {
        handle_get_fault(request, response);
      });

  clear_fault_srv_ = create_service<ros2_medkit_msgs::srv::ClearFault>(
      "~/clear_fault", [this](const std::shared_ptr<ros2_medkit_msgs::srv::ClearFault::Request> & request,
                              const std::shared_ptr<ros2_medkit_msgs::srv::ClearFault::Response> & response) {
        handle_clear_fault(request, response);
      });

  get_snapshots_srv_ = create_service<ros2_medkit_msgs::srv::GetSnapshots>(
      "~/get_snapshots", [this](const std::shared_ptr<ros2_medkit_msgs::srv::GetSnapshots::Request> & request,
                                const std::shared_ptr<ros2_medkit_msgs::srv::GetSnapshots::Response> & response) {
        handle_get_snapshots(request, response);
      });

  get_rosbag_srv_ = create_service<ros2_medkit_msgs::srv::GetRosbag>(
      "~/get_rosbag", [this](const std::shared_ptr<ros2_medkit_msgs::srv::GetRosbag::Request> & request,
                             const std::shared_ptr<ros2_medkit_msgs::srv::GetRosbag::Response> & response) {
        handle_get_rosbag(request, response);
      });

  list_rosbags_srv_ = create_service<ros2_medkit_msgs::srv::ListRosbags>(
      "~/list_rosbags", [this](const std::shared_ptr<ros2_medkit_msgs::srv::ListRosbags::Request> & request,
                               const std::shared_ptr<ros2_medkit_msgs::srv::ListRosbags::Response> & response) {
        handle_list_rosbags(request, response);
      });

  list_faults_for_entity_srv_ = create_service<ros2_medkit_msgs::srv::ListFaultsForEntity>(
      "~/list_faults_for_entity",
      [this](const std::shared_ptr<ros2_medkit_msgs::srv::ListFaultsForEntity::Request> & request,
             const std::shared_ptr<ros2_medkit_msgs::srv::ListFaultsForEntity::Response> & response) {
        handle_list_faults_for_entity(request, response);
      });

  // Initialize snapshot capture
  auto snapshot_config = create_snapshot_config();
  if (snapshot_config.enabled) {
    snapshot_capture_ = std::make_shared<SnapshotCapture>(this, storage_.get(), snapshot_config);
  }

  // Initialize rosbag capture if enabled
  if (snapshot_config.rosbag.enabled) {
    rosbag_capture_ = std::make_shared<RosbagCapture>(this, storage_.get(), snapshot_config.rosbag, snapshot_config);
  }

  // Drive captures through a bounded pool instead of one thread per fault.
  if (snapshot_capture_ || rosbag_capture_) {
    auto snap = snapshot_capture_;
    auto bag = rosbag_capture_;
    // RosbagCapture is single-writer (one shared ring buffer + one active writer +
    // one recording state machine), so on_fault_confirmed() is not safe for the
    // pool_size concurrent calls the pool contract allows. Serialize just the rosbag
    // leg under this mutex; snapshot capture is independent and stays parallel.
    auto rosbag_mutex = std::make_shared<std::mutex>();
    capture_pool_ = std::make_unique<CaptureThreadPool>(
        static_cast<std::size_t>(capture_pool_size_), static_cast<std::size_t>(capture_queue_depth_),
        capture_queue_full_policy_, get_logger(), [snap, bag, rosbag_mutex](const std::string & fault_code) {
          if (snap) {
            snap->capture(fault_code);
          }
          if (bag) {
            std::lock_guard<std::mutex> bag_lock(*rosbag_mutex);
            bag->on_fault_confirmed(fault_code);
          }
        });
  }

  // Initialize correlation engine (nullptr if disabled or not configured)
  correlation_engine_ = create_correlation_engine();

  // Create correlation cleanup timer if correlation is enabled
  if (correlation_engine_) {
    auto cleanup_interval_sec = declare_parameter<double>("correlation.cleanup_interval_sec", 5.0);
    if (cleanup_interval_sec <= 0.0) {
      RCLCPP_WARN(get_logger(), "correlation.cleanup_interval_sec must be positive, got %.2f. Using default 5.0s",
                  cleanup_interval_sec);
      cleanup_interval_sec = 5.0;
    }
    auto cleanup_interval_ms = static_cast<int64_t>(cleanup_interval_sec * 1000);
    correlation_cleanup_timer_ = create_wall_timer(std::chrono::milliseconds(cleanup_interval_ms), [this]() {
      correlation_engine_->cleanup_expired();
    });
  }

  // Create auto-confirmation timer if enabled
  if (auto_confirm_after_sec_ > 0.0) {
    auto_confirm_timer_ = create_wall_timer(std::chrono::seconds(1), [this]() {
      const auto confirmed = storage_->check_time_based_confirmation(get_wall_clock_time());
      if (confirmed.empty()) {
        return;
      }
      // Audit every timer-driven PREFAILED->CONFIRMED transition. Without this the
      // confirmations are invisible to the audit log's verify().
      const int64_t confirmed_at_ns = get_wall_clock_time().nanoseconds();
      for (const auto & fault_code : confirmed) {
        auto fault = storage_->get_fault(fault_code);
        if (fault) {
          audit_transition(kTransitionConfirmed, *fault, "auto_confirm_timer", confirmed_at_ns);
        }
      }
      RCLCPP_INFO(get_logger(), "Auto-confirmed %zu PREFAILED fault(s) due to time threshold", confirmed.size());
    });
    RCLCPP_INFO(get_logger(),
                "FaultManager node started (storage=%s, confirmation_threshold=%d, "
                "healing=%s, auto_confirm_after=%.1fs)",
                storage_type_.c_str(), confirmation_threshold_, healing_enabled_ ? "enabled" : "disabled",
                auto_confirm_after_sec_);
  } else {
    RCLCPP_INFO(get_logger(), "FaultManager node started (storage=%s, confirmation_threshold=%d, healing=%s)",
                storage_type_.c_str(), confirmation_threshold_, healing_enabled_ ? "enabled" : "disabled");
  }
}

FaultManagerNode::~FaultManagerNode() {
  // Join capture workers FIRST so no worker is mid-capture when rosbag tears
  // down. RosbagCapture::stop() is not a barrier against an in-flight
  // on_fault_confirmed() (it clears subscriptions/buffers without serializing
  // against the capture path). See issue #441.
  if (capture_pool_) {
    capture_pool_->shutdown();
  }
  if (rosbag_capture_) {
    rosbag_capture_->stop();
  }

  // Close the chain with a "logging deactivated" marker. Best-effort and appended
  // directly (recorded even in confirmed_only mode); a failure here must not throw
  // out of the destructor.
  if (audit_log_) {
    try {
      AuditEvent marker;
      marker.fault_code = "__audit__";
      marker.transition = kTransitionLoggingDeactivated;
      marker.status = "INACTIVE";
      marker.source_id = "fault_manager";
      marker.description = "audit logging deactivated";
      marker.occurred_at_ns = get_wall_clock_time().nanoseconds();
      audit_log_->append(marker);
    } catch (const std::exception & e) {
      RCLCPP_WARN(get_logger(), "Failed to append audit 'logging_deactivated' marker: %s", e.what());
    }
  }
}

std::unique_ptr<FaultStorage> FaultManagerNode::create_storage() {
  if (storage_type_ == "memory") {
    RCLCPP_INFO(get_logger(), "Using in-memory fault storage");
    return std::make_unique<InMemoryFaultStorage>();
  }

  if (storage_type_ == "sqlite") {
    // Create parent directory if it doesn't exist (except for :memory:)
    if (database_path_ != ":memory:") {
      std::filesystem::path db_path(database_path_);
      auto parent_dir = db_path.parent_path();
      std::string parent_dir_str = parent_dir.string();
      if (!parent_dir_str.empty() && !std::filesystem::exists(parent_dir)) {
        try {
          std::filesystem::create_directories(parent_dir);
          RCLCPP_INFO(get_logger(), "Created database directory: %s", parent_dir_str.c_str());
        } catch (const std::filesystem::filesystem_error & e) {
          RCLCPP_ERROR(get_logger(), "Failed to create database directory for fault manager storage at '%s': %s",
                       parent_dir_str.c_str(), e.what());
          throw;
        }
      }
    }

    RCLCPP_INFO(get_logger(), "Using SQLite fault storage: %s", database_path_.c_str());
    return std::make_unique<SqliteFaultStorage>(database_path_);
  }

  RCLCPP_ERROR(get_logger(), "Unknown storage_type '%s', falling back to in-memory", storage_type_.c_str());
  return std::make_unique<InMemoryFaultStorage>();
}

std::unique_ptr<FaultAuditLog> FaultManagerNode::create_audit_log() {
  const bool enabled = declare_parameter<bool>("audit_log.enabled", false);

  // Which transitions to record: "all" (occurred/confirmed/cleared) or
  // "confirmed_only".
  const std::string transitions = declare_parameter<std::string>("audit_log.transitions", "all");
  audit_confirmed_only_ = (transitions == "confirmed_only");
  if (transitions != "all" && transitions != "confirmed_only") {
    RCLCPP_WARN(get_logger(), "audit_log.transitions '%s' invalid, using 'all'", transitions.c_str());
    audit_confirmed_only_ = false;
  }

  // Retention: seal + prune the oldest segment beyond this many records (0 = off).
  auto retention = declare_parameter<int64_t>("audit_log.retention_max_records", 0);
  if (retention < 0) {
    RCLCPP_WARN(get_logger(), "audit_log.retention_max_records < 0, disabling rotation");
    retention = 0;
  }

  // Fail-closed: when set, an audit append failure is re-raised as a hard error
  // (fail-FAST) so a broken audit chain cannot be silently outlived. It does NOT
  // roll back the fault-state change that triggered the transition - that change
  // has already committed to the SEPARATE fault-store DB and cross-DB atomicity
  // is not achievable here - it only signals the broken audit so an operator must
  // act. Off by default so the audit never blocks fault processing.
  audit_fail_closed_ = declare_parameter<bool>("audit_log.fail_closed", false);

  // Path: explicit override, else a sibling of the fault DB, else :memory:.
  std::string audit_path = declare_parameter<std::string>("audit_log.database_path", "");

  if (!enabled) {
    return nullptr;  // No table, no file, no write overhead when off.
  }

  if (audit_path.empty()) {
    if (database_path_ == ":memory:" || storage_type_ != "sqlite") {
      audit_path = ":memory:";
    } else {
      std::filesystem::path base(database_path_);
      audit_path = (base.parent_path() / "fault_audit.db").string();
    }
  }

  if (audit_path != ":memory:") {
    std::filesystem::path p(audit_path);
    auto parent = p.parent_path();
    if (!parent.empty() && !std::filesystem::exists(parent)) {
      try {
        std::filesystem::create_directories(parent);
      } catch (const std::filesystem::filesystem_error & e) {
        RCLCPP_ERROR(get_logger(), "Failed to create audit log directory '%s': %s", parent.string().c_str(), e.what());
        throw;
      }
    }
  }

  try {
    auto log = std::make_unique<FaultAuditLog>(audit_path, retention);
    RCLCPP_INFO(get_logger(),
                "Fault audit log enabled: %s (transitions=%s, retention=%" PRId64
                ", fail_closed=%s, resume_seq=%" PRId64 ")",
                audit_path.c_str(), audit_confirmed_only_ ? "confirmed_only" : "all", retention,
                audit_fail_closed_ ? "true" : "false", log->head().seq);

    // Record a "logging activated" marker so the chain documents its own start.
    // Appended directly, so it is recorded even in confirmed_only mode.
    // Best-effort: a failure here does not abort startup.
    AuditEvent marker;
    marker.fault_code = "__audit__";
    marker.transition = kTransitionLoggingActivated;
    marker.status = "ACTIVE";
    marker.source_id = "fault_manager";
    marker.description = "audit logging activated";
    marker.occurred_at_ns = get_wall_clock_time().nanoseconds();
    try {
      log->append(marker);
    } catch (const std::exception & e) {
      RCLCPP_WARN(get_logger(), "Failed to append audit 'logging_activated' marker: %s", e.what());
    }
    return log;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Failed to open fault audit log '%s': %s", audit_path.c_str(), e.what());
    throw;
  }
}

void FaultManagerNode::audit_transition(const char * transition, const ros2_medkit_msgs::msg::Fault & fault,
                                        const std::string & source_id, int64_t occurred_at_ns) {
  if (!audit_log_) {
    return;
  }
  if (audit_confirmed_only_ && std::string(transition) != kTransitionConfirmed) {
    return;
  }

  AuditEvent event;
  event.fault_code = fault.fault_code;
  event.transition = transition;
  event.severity = fault.severity;
  event.status = fault.status;
  event.source_id = source_id;
  event.description = fault.description;
  event.occurred_at_ns = occurred_at_ns;

  try {
    audit_log_->append(event);
  } catch (const std::exception & e) {
    // A dropped append is a hole in the chain that verify() can never see (it
    // proves nothing was deleted, not that everything was written). The fault-
    // state transition that triggered this has ALREADY committed to the fault
    // store, which is a SEPARATE SQLite database: there is no cross-DB atomicity,
    // so this failure does NOT (and cannot) roll that change back. All we can do
    // is make the broken audit loud and observable via the health signals.
    audit_dropped_writes_.fetch_add(1, std::memory_order_relaxed);
    audit_healthy_.store(false, std::memory_order_relaxed);
    const uint64_t dropped = audit_dropped_writes_.load(std::memory_order_relaxed);
    RCLCPP_ERROR(get_logger(), "Failed to append audit record for '%s' (%s): %s [audit dropped_writes=%" PRIu64 "]",
                 fault.fault_code.c_str(), transition, e.what(), dropped);
    if (audit_fail_closed_) {
      // Explicit fail-FAST: surface the broken audit to the caller so a
      // compliance-strict deployment learns the chain is now incomplete and an
      // operator must act. Rethrowing propagates that signal ONLY - the fault-
      // state change already committed (see above) and is NOT rolled back; this
      // is not atomicity.
      RCLCPP_FATAL(get_logger(),
                   "audit_log.fail_closed is set: audit append failed for '%s' (%s); the audit chain is now "
                   "incomplete and requires operator action. The already-committed fault-state change is NOT "
                   "rolled back (the fault store is a separate database).",
                   fault.fault_code.c_str(), transition);
      throw;
    }
  }
}

void FaultManagerNode::handle_report_fault(
    const std::shared_ptr<ros2_medkit_msgs::srv::ReportFault::Request> & request,
    const std::shared_ptr<ros2_medkit_msgs::srv::ReportFault::Response> & response) {
  // Validate fault_code
  std::string validation_error = validate_fault_code(request->fault_code);
  if (!validation_error.empty()) {
    response->accepted = false;
    RCLCPP_WARN(get_logger(), "ReportFault rejected: %s", validation_error.c_str());
    return;
  }

  // Validate event_type
  if (request->event_type != ros2_medkit_msgs::srv::ReportFault::Request::EVENT_FAILED &&
      request->event_type != ros2_medkit_msgs::srv::ReportFault::Request::EVENT_PASSED) {
    response->accepted = false;
    RCLCPP_WARN(get_logger(), "ReportFault rejected: invalid event_type %d", request->event_type);
    return;
  }

  // For FAILED events, validate severity
  if (request->event_type == ros2_medkit_msgs::srv::ReportFault::Request::EVENT_FAILED &&
      !is_valid_severity(request->severity)) {
    response->accepted = false;
    RCLCPP_WARN(get_logger(), "ReportFault rejected: invalid severity %d for FAILED event", request->severity);
    return;
  }

  // Validate source_id
  if (request->source_id.empty()) {
    response->accepted = false;
    RCLCPP_WARN(get_logger(), "ReportFault rejected: source_id cannot be empty");
    return;
  }

  // Get status before update (if fault exists)
  auto fault_before = storage_->get_fault(request->fault_code);
  std::string status_before = fault_before ? fault_before->status : "";

  // Resolve per-entity debounce config (longest-prefix match on source_id)
  // TODO(#276): warn when different entities resolve different configs for the same fault_code
  auto resolved_config = resolve_config(request->source_id);

  // Report the fault event (use wall clock time, not sim time, for proper timestamps)
  const rclcpp::Time event_time = get_wall_clock_time();
  bool is_new =
      storage_->report_fault_event(request->fault_code, request->event_type, request->severity, request->description,
                                   request->source_id, event_time, resolved_config, request->supersedes_source_id);

  response->accepted = true;

  // Get updated fault state to publish event
  auto fault_after = storage_->get_fault(request->fault_code);
  if (fault_after) {
    // Process through correlation engine (if enabled)
    // Only process FAILED events with correlation
    bool should_mute = false;
    if (correlation_engine_ && request->event_type == ros2_medkit_msgs::srv::ReportFault::Request::EVENT_FAILED) {
      auto correlation_result =
          correlation_engine_->process_fault(request->fault_code, correlation::severity_to_string(request->severity));

      should_mute = correlation_result.should_mute;

      if (correlation_result.is_root_cause) {
        RCLCPP_DEBUG(get_logger(), "Fault %s identified as root cause (rule=%s)", request->fault_code.c_str(),
                     correlation_result.rule_id.c_str());
      } else if (should_mute) {
        RCLCPP_DEBUG(get_logger(), "Fault %s muted as symptom of %s (rule=%s, delay=%ums)", request->fault_code.c_str(),
                     correlation_result.root_cause_code.c_str(), correlation_result.rule_id.c_str(),
                     correlation_result.delay_ms);
      } else if (!correlation_result.cluster_id.empty()) {
        RCLCPP_DEBUG(get_logger(), "Fault %s added to cluster %s", request->fault_code.c_str(),
                     correlation_result.cluster_id.c_str());
        // Log retroactively muted faults when cluster becomes active
        if (!correlation_result.retroactive_mute_codes.empty()) {
          RCLCPP_DEBUG(get_logger(), "Cluster %s activated: retroactively muting %zu faults",
                       correlation_result.cluster_id.c_str(), correlation_result.retroactive_mute_codes.size());
        }
      }
    }

    // Determine event type based on status transition
    bool just_confirmed = false;
    if (is_new && fault_after->status == ros2_medkit_msgs::msg::Fault::STATUS_CONFIRMED) {
      // New fault immediately confirmed (e.g., CRITICAL severity or threshold=-1)
      if (!should_mute) {
        publish_fault_event(ros2_medkit_msgs::msg::FaultEvent::EVENT_CONFIRMED, *fault_after);
      }
      just_confirmed = true;
    } else if (!is_new && status_before != ros2_medkit_msgs::msg::Fault::STATUS_CONFIRMED &&
               fault_after->status == ros2_medkit_msgs::msg::Fault::STATUS_CONFIRMED) {
      // Existing fault transitioned to CONFIRMED
      if (!should_mute) {
        publish_fault_event(ros2_medkit_msgs::msg::FaultEvent::EVENT_CONFIRMED, *fault_after);
      }
      just_confirmed = true;
    } else if (!is_new && fault_after->status == ros2_medkit_msgs::msg::Fault::STATUS_CONFIRMED) {
      // Fault was already CONFIRMED, data updated (occurrence_count, sources, etc.)
      if (!should_mute) {
        publish_fault_event(ros2_medkit_msgs::msg::FaultEvent::EVENT_UPDATED, *fault_after);
      }
    }
    // Note: PREFAILED/PREPASSED status changes don't emit events (debounce in progress)

    // A PASSED event can drive an existing fault past the healing threshold to
    // HEALED. That is the fault's END and must be audited (with a distinct kind
    // and source) so the timeline is complete, not just occurred+confirmed.
    const bool just_healed = !is_new && status_before != ros2_medkit_msgs::msg::Fault::STATUS_HEALED &&
                             fault_after->status == ros2_medkit_msgs::msg::Fault::STATUS_HEALED;

    // Append tamper-evident audit records for the transitions that just happened.
    // Recorded regardless of correlation muting: muting affects display, not the
    // fact that the state transition occurred.
    if (is_new) {
      audit_transition(kTransitionOccurred, *fault_after, request->source_id, event_time.nanoseconds());
    }
    if (just_confirmed) {
      audit_transition(kTransitionConfirmed, *fault_after, request->source_id, event_time.nanoseconds());
    }
    if (just_healed) {
      audit_transition(kTransitionHealed, *fault_after, "auto_heal", event_time.nanoseconds());
    }

    // Capture snapshots/rosbag when a fault confirms via the bounded pool (issue #441).
    // handle_report_fault runs on the single-threaded executor, so confirmations are
    // already serialized; last_capture_mutex_ only guards last_capture_times_ itself
    // (the cooldown check, the update below, and the expired-entry sweep). It is taken
    // solely when the cooldown is enabled, since the map is otherwise never touched.
    if (just_confirmed && capture_pool_) {
      const std::string fault_code = request->fault_code;
      const bool cooldown_enabled = snapshot_recapture_cooldown_sec_ > 0.0;
      std::unique_lock<std::mutex> cd_lock(last_capture_mutex_, std::defer_lock);
      if (cooldown_enabled) {
        cd_lock.lock();
      }

      bool on_cooldown = false;
      if (cooldown_enabled) {
        const auto cooldown = std::chrono::duration<double>(snapshot_recapture_cooldown_sec_);
        const auto sweep_now = std::chrono::steady_clock::now();
        // Bound the map (issue #441): a storm of distinct fault codes would otherwise
        // leave one permanent entry per code. Entries older than the cooldown can never
        // gate a capture again, so drop them while we hold the lock.
        for (auto it = last_capture_times_.begin(); it != last_capture_times_.end();) {
          if (sweep_now - it->second >= cooldown) {
            it = last_capture_times_.erase(it);
          } else {
            ++it;
          }
        }
        auto it = last_capture_times_.find(fault_code);
        if (it != last_capture_times_.end()) {
          on_cooldown = (sweep_now - it->second) < cooldown;
        }
      }

      if (on_cooldown) {
        RCLCPP_DEBUG(get_logger(), "Skipping capture for '%s' - cooldown active", fault_code.c_str());
      } else {
        const EnqueueOutcome outcome = capture_pool_->enqueue(fault_code);
        const auto now = std::chrono::steady_clock::now();
        // RCLCPP_WARN_THROTTLE needs a non-const Clock lvalue (Humble/Lyrical
        // compat); mirror rosbag_capture.cpp's local-copy pattern. Cast the
        // uint64_t counter to unsigned long long + %llu to avoid -Wuseless-cast
        // (uint64_t == unsigned long on LP64).
        rclcpp::Clock throttle_clock(*get_clock());
        switch (outcome.result) {
          case EnqueueResult::kAccepted:
            if (cooldown_enabled) {
              last_capture_times_[fault_code] = now;
            }
            break;
          case EnqueueResult::kEvictedOldest:
            if (cooldown_enabled) {
              last_capture_times_[fault_code] = now;
              if (outcome.evicted_code) {
                last_capture_times_.erase(*outcome.evicted_code);  // keep evicted fault retriable
              }
            }
            RCLCPP_WARN_THROTTLE(get_logger(), throttle_clock, 2000,
                                 "Capture queue full (drop_oldest): evicted pending '%s' for '%s' "
                                 "(pool=%d, queue=%d, total_dropped=%llu)",
                                 outcome.evicted_code ? outcome.evicted_code->c_str() : "?", fault_code.c_str(),
                                 capture_pool_size_, capture_queue_depth_,
                                 static_cast<unsigned long long>(capture_pool_->dropped_captures()));
            break;
          case EnqueueResult::kDroppedNewest:
            // Cooldown NOT recorded: capture still possible if the fault later
            // heals/clears and re-confirms.
            RCLCPP_WARN_THROTTLE(get_logger(), throttle_clock, 2000,
                                 "Capture queue full (reject_newest): dropped capture for '%s' "
                                 "(pool=%d, queue=%d, total_dropped=%llu)",
                                 fault_code.c_str(), capture_pool_size_, capture_queue_depth_,
                                 static_cast<unsigned long long>(capture_pool_->dropped_captures()));
            break;
          case EnqueueResult::kRejectedShuttingDown:
            RCLCPP_DEBUG(get_logger(), "Capture pool shutting down; skipped capture for '%s'", fault_code.c_str());
            break;
        }
      }
    }

    // Handle PREFAILED state for lazy_start rosbag capture
    bool just_prefailed = (is_new && fault_after->status == ros2_medkit_msgs::msg::Fault::STATUS_PREFAILED) ||
                          (!is_new && status_before != ros2_medkit_msgs::msg::Fault::STATUS_PREFAILED &&
                           fault_after->status == ros2_medkit_msgs::msg::Fault::STATUS_PREFAILED);
    if (just_prefailed && rosbag_capture_) {
      rosbag_capture_->on_fault_prefailed(request->fault_code);
    }
  }

  if (request->event_type == ros2_medkit_msgs::srv::ReportFault::Request::EVENT_FAILED) {
    if (is_new) {
      RCLCPP_INFO(get_logger(), "New fault reported: %s (severity=%d, source=%s)", request->fault_code.c_str(),
                  request->severity, request->source_id.c_str());
    } else {
      RCLCPP_DEBUG(get_logger(), "Fault updated: %s (source=%s)", request->fault_code.c_str(),
                   request->source_id.c_str());
    }
  } else {
    RCLCPP_DEBUG(get_logger(), "PASSED event for fault: %s (source=%s)", request->fault_code.c_str(),
                 request->source_id.c_str());
  }
}

void FaultManagerNode::handle_list_faults(
    const std::shared_ptr<ros2_medkit_msgs::srv::ListFaults::Request> & request,
    const std::shared_ptr<ros2_medkit_msgs::srv::ListFaults::Response> & response) {
  response->faults = storage_->list_faults(request->filter_by_severity, request->severity, request->statuses);

  // Include correlation data if engine is enabled
  if (correlation_engine_) {
    // Always include counts
    response->muted_count = correlation_engine_->get_muted_count();
    response->cluster_count = correlation_engine_->get_cluster_count();

    auto muted_faults = correlation_engine_->get_muted_faults();

    // Include muted faults details if requested
    if (request->include_muted) {
      response->muted_faults.reserve(muted_faults.size());
      for (const auto & muted : muted_faults) {
        ros2_medkit_msgs::msg::MutedFaultInfo info;
        info.fault_code = muted.fault_code;
        info.root_cause_code = muted.root_cause_code;
        info.rule_id = muted.rule_id;
        info.delay_ms = muted.delay_ms;
        response->muted_faults.push_back(info);
      }
    } else {
      // Build a set of muted codes for O(1) lookup.
      std::unordered_set<std::string> muted_codes;
      for (const auto & muted : muted_faults) {
        muted_codes.insert(muted.fault_code);
      }

      // Remove all faults whose code is muted, in one pass.
      auto & faults = response->faults;
      faults.erase(std::remove_if(faults.begin(), faults.end(),
                                  [&muted_codes](const ros2_medkit_msgs::msg::Fault & fault) {
                                    return muted_codes.count(fault.fault_code) > 0;
                                  }),
                   faults.end());
    }

    // Include cluster details if requested
    if (request->include_clusters) {
      auto clusters = correlation_engine_->get_clusters();
      response->clusters.reserve(clusters.size());
      for (const auto & cluster : clusters) {
        ros2_medkit_msgs::msg::ClusterInfo info;
        info.cluster_id = cluster.cluster_id;
        info.rule_id = cluster.rule_id;
        info.rule_name = cluster.rule_name;
        info.label = cluster.label;
        info.representative_code = cluster.representative_code;
        info.representative_severity = cluster.representative_severity;
        info.fault_codes = cluster.fault_codes;
        info.count = static_cast<uint32_t>(cluster.fault_codes.size());
        // Convert chrono time_points to ROS time
        auto first_ns =
            std::chrono::duration_cast<std::chrono::nanoseconds>(cluster.first_at.time_since_epoch()).count();
        auto last_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(cluster.last_at.time_since_epoch()).count();
        info.first_at = rclcpp::Time(first_ns, RCL_SYSTEM_TIME);
        info.last_at = rclcpp::Time(last_ns, RCL_SYSTEM_TIME);
        response->clusters.push_back(info);
      }
    }
  }

  RCLCPP_DEBUG(get_logger(), "ListFaults returned %zu faults (muted=%u, clusters=%u)", response->faults.size(),
               response->muted_count, response->cluster_count);
}

void FaultManagerNode::handle_clear_fault(
    const std::shared_ptr<ros2_medkit_msgs::srv::ClearFault::Request> & request,
    const std::shared_ptr<ros2_medkit_msgs::srv::ClearFault::Response> & response) {
  // Validate fault_code
  std::string validation_error = validate_fault_code(request->fault_code);
  if (!validation_error.empty()) {
    response->success = false;
    response->message = validation_error;
    return;
  }

  // Process through correlation engine first (to get auto-clear list).
  // `skip_correlation_auto_clear` lets the caller opt out of cascade-clearing
  // correlated symptom fault codes. Per-entity DELETE routes set it to true
  // so they cannot reach across entity boundaries via the correlation graph.
  std::vector<std::string> auto_cleared_codes;
  if (correlation_engine_ && !request->skip_correlation_auto_clear) {
    auto clear_result = correlation_engine_->process_clear(request->fault_code);
    auto_cleared_codes = clear_result.auto_cleared_codes;
  }

  bool cleared = storage_->clear_fault(request->fault_code);

  response->success = cleared;
  if (cleared) {
    // Evict cooldown tracking for cleared fault and auto-cleared symptoms
    {
      std::lock_guard<std::mutex> lock(last_capture_mutex_);
      last_capture_times_.erase(request->fault_code);
      for (const auto & symptom_code : auto_cleared_codes) {
        last_capture_times_.erase(symptom_code);
      }
    }

    // Auto-clear correlated symptoms
    for (const auto & symptom_code : auto_cleared_codes) {
      storage_->clear_fault(symptom_code);
      if (audit_log_) {
        auto symptom = storage_->get_fault(symptom_code);
        if (symptom) {
          audit_transition(kTransitionCleared, *symptom, "clear_service", get_wall_clock_time().nanoseconds());
        }
      }
      RCLCPP_DEBUG(get_logger(), "Auto-cleared symptom: %s (root cause: %s)", symptom_code.c_str(),
                   request->fault_code.c_str());
      // Also cleanup rosbag for auto-cleared faults
      if (rosbag_capture_) {
        rosbag_capture_->on_fault_cleared(symptom_code);
      }
    }

    response->auto_cleared_codes = auto_cleared_codes;
    if (auto_cleared_codes.empty()) {
      response->message = "Fault cleared: " + request->fault_code;
    } else {
      response->message = "Fault cleared: " + request->fault_code + " (auto-cleared " +
                          std::to_string(auto_cleared_codes.size()) + " symptoms)";
    }
    RCLCPP_INFO(get_logger(), "Fault cleared: %s (auto-cleared %zu symptoms)", request->fault_code.c_str(),
                auto_cleared_codes.size());

    // Cleanup rosbag for the main fault (auto_cleanup handled inside RosbagCapture)
    if (rosbag_capture_) {
      rosbag_capture_->on_fault_cleared(request->fault_code);
    }

    // Publish EVENT_CLEARED - get the cleared fault to include in event
    auto fault = storage_->get_fault(request->fault_code);
    if (fault) {
      publish_fault_event(ros2_medkit_msgs::msg::FaultEvent::EVENT_CLEARED, *fault, auto_cleared_codes);
      audit_transition(kTransitionCleared, *fault, "clear_service", get_wall_clock_time().nanoseconds());
    }
  } else {
    response->message = "Fault not found: " + request->fault_code;
    RCLCPP_WARN(get_logger(), "Attempted to clear non-existent fault: %s", request->fault_code.c_str());
  }
}

bool FaultManagerNode::is_valid_severity(uint8_t severity) {
  return severity <= ros2_medkit_msgs::msg::Fault::SEVERITY_CRITICAL;
}

std::string FaultManagerNode::extract_topic_name(const std::string & topic_path) {
  auto pos = topic_path.rfind('/');
  if (pos != std::string::npos && pos < topic_path.length() - 1) {
    return topic_path.substr(pos + 1);
  }
  return topic_path;
}

void FaultManagerNode::handle_get_fault(const std::shared_ptr<ros2_medkit_msgs::srv::GetFault::Request> & request,
                                        const std::shared_ptr<ros2_medkit_msgs::srv::GetFault::Response> & response) {
  RCLCPP_DEBUG(get_logger(), "GetFault request for: %s", request->fault_code.c_str());

  // Validate fault_code
  std::string validation_error = validate_fault_code(request->fault_code);
  if (!validation_error.empty()) {
    response->success = false;
    response->error_message = validation_error;
    return;
  }

  // Get fault from storage
  auto fault = storage_->get_fault(request->fault_code);
  if (!fault) {
    response->success = false;
    response->error_message = "Fault not found: " + request->fault_code;
    return;
  }

  response->success = true;
  response->fault = *fault;

  // Populate extended_data_records with timestamps
  ros2_medkit_msgs::msg::ExtendedDataRecords extended_records;
  extended_records.first_occurrence_ns = rclcpp::Time(fault->first_occurred).nanoseconds();
  extended_records.last_occurrence_ns = rclcpp::Time(fault->last_occurred).nanoseconds();
  response->environment_data.extended_data_records = extended_records;

  // Get freeze frame snapshots from storage
  auto stored_snapshots = storage_->get_snapshots(request->fault_code);
  for (const auto & stored_snapshot : stored_snapshots) {
    ros2_medkit_msgs::msg::Snapshot snapshot;
    snapshot.type = ros2_medkit_msgs::msg::Snapshot::TYPE_FREEZE_FRAME;
    snapshot.name = extract_topic_name(stored_snapshot.topic);
    snapshot.data = stored_snapshot.data;
    snapshot.topic = stored_snapshot.topic;
    snapshot.message_type = stored_snapshot.message_type;
    snapshot.captured_at_ns = stored_snapshot.captured_at_ns;
    // Rosbag fields left empty for freeze_frame type
    response->environment_data.snapshots.push_back(snapshot);
  }

  // Per-topic snapshots are deleted on clear_fault, but the compact freeze-frame row is
  // retained. When no snapshots remain, serve the retained frame so the confirmed-state
  // record stays observable after acknowledgement.
  if (stored_snapshots.empty()) {
    auto frame = storage_->get_freeze_frame(request->fault_code);
    if (frame) {
      ros2_medkit_msgs::msg::Snapshot snapshot;
      snapshot.type = ros2_medkit_msgs::msg::Snapshot::TYPE_FREEZE_FRAME;
      snapshot.name = "freeze_frame";
      snapshot.data = frame->data;
      // topic/message_type left empty: the frame aggregates values from multiple topics
      snapshot.captured_at_ns = frame->captured_at_ns;
      response->environment_data.snapshots.push_back(snapshot);
    }
  }

  // Get rosbag info if available
  auto rosbag_info = storage_->get_rosbag_file(request->fault_code);
  if (rosbag_info) {
    ros2_medkit_msgs::msg::Snapshot rosbag_snapshot;
    rosbag_snapshot.type = ros2_medkit_msgs::msg::Snapshot::TYPE_ROSBAG;
    rosbag_snapshot.name = "rosbag_" + request->fault_code;
    rosbag_snapshot.bulk_data_id = request->fault_code;
    rosbag_snapshot.size_bytes = rosbag_info->size_bytes;
    rosbag_snapshot.duration_sec = rosbag_info->duration_sec;
    rosbag_snapshot.format = rosbag_info->format;
    rosbag_snapshot.captured_at_ns = rosbag_info->created_at_ns;
    // Freeze frame fields left empty for rosbag type
    response->environment_data.snapshots.push_back(rosbag_snapshot);
  }

  RCLCPP_DEBUG(get_logger(), "GetFault returned fault '%s' with %zu snapshots", request->fault_code.c_str(),
               response->environment_data.snapshots.size());
}

void FaultManagerNode::publish_fault_event(const std::string & event_type, const ros2_medkit_msgs::msg::Fault & fault,
                                           const std::vector<std::string> & auto_cleared_codes) {
  ros2_medkit_msgs::msg::FaultEvent event;
  event.event_type = event_type;
  event.fault = fault;
  event.timestamp = get_wall_clock_time();
  event.auto_cleared_codes = auto_cleared_codes;

  event_publisher_->publish(event);

  RCLCPP_DEBUG(get_logger(), "Published fault event: %s for fault_code=%s", event_type.c_str(),
               fault.fault_code.c_str());
}

SnapshotConfig FaultManagerNode::create_snapshot_config() {
  SnapshotConfig config;

  // Declare snapshot parameters
  config.enabled = declare_parameter<bool>("snapshots.enabled", true);
  config.background_capture = declare_parameter<bool>("snapshots.background_capture", false);

  // Validate timeout_sec (must be positive)
  config.timeout_sec = declare_parameter<double>("snapshots.timeout_sec", 1.0);
  if (config.timeout_sec <= 0.0) {
    RCLCPP_WARN(get_logger(), "snapshots.timeout_sec must be positive, got %.2f. Using default 1.0s",
                config.timeout_sec);
    config.timeout_sec = 1.0;
  }

  // Validate max_message_size (must be positive before casting to size_t)
  auto max_message_size_param = declare_parameter<int>("snapshots.max_message_size", 65536);
  if (max_message_size_param <= 0) {
    RCLCPP_WARN(get_logger(), "snapshots.max_message_size must be positive, got %ld. Using default 65536",
                static_cast<long>(max_message_size_param));
    max_message_size_param = 65536;
  }
  config.max_message_size = static_cast<size_t>(max_message_size_param);

  // Default topics (catch-all). Empty entries are dropped: launch files cannot
  // express an empty string array, so [''] is the conventional "no defaults"
  // and must not shadow the entity-default fallback below.
  config.default_topics =
      declare_parameter<std::vector<std::string>>("snapshots.default_topics", std::vector<std::string>{});
  config.default_topics.erase(std::remove(config.default_topics.begin(), config.default_topics.end(), std::string{}),
                              config.default_topics.end());

  // Zero-config fallback: capture the reporting node's own published topics
  // when no explicit config matches the fault code (opt-out).
  config.entity_default = declare_parameter<bool>("snapshots.entity_default", true);

  // Load fault_specific and patterns from YAML config file if provided
  auto config_file = declare_parameter<std::string>("snapshots.config_file", "");
  if (!config_file.empty()) {
    load_snapshot_config_from_yaml(config_file, config);
  }

  // Rosbag configuration (opt-in)
  config.rosbag.enabled = declare_parameter<bool>("snapshots.rosbag.enabled", false);
  if (config.rosbag.enabled) {
    config.rosbag.duration_sec = declare_parameter<double>("snapshots.rosbag.duration_sec", 5.0);
    if (config.rosbag.duration_sec <= 0.0) {
      RCLCPP_WARN(get_logger(), "snapshots.rosbag.duration_sec must be positive, got %.2f. Using default 5.0s",
                  config.rosbag.duration_sec);
      config.rosbag.duration_sec = 5.0;
    }

    config.rosbag.duration_after_sec = declare_parameter<double>("snapshots.rosbag.duration_after_sec", 1.0);
    if (config.rosbag.duration_after_sec < 0.0) {
      RCLCPP_WARN(get_logger(), "snapshots.rosbag.duration_after_sec must be non-negative, got %.2f. Using 0.0s",
                  config.rosbag.duration_after_sec);
      config.rosbag.duration_after_sec = 0.0;
    }

    config.rosbag.topics = declare_parameter<std::string>("snapshots.rosbag.topics", "entity");
    config.rosbag.include_topics =
        declare_parameter<std::vector<std::string>>("snapshots.rosbag.include_topics", std::vector<std::string>{});
    config.rosbag.exclude_topics =
        declare_parameter<std::vector<std::string>>("snapshots.rosbag.exclude_topics", std::vector<std::string>{});
    config.rosbag.exclude_sensor_topics = declare_parameter<bool>("snapshots.rosbag.exclude_sensor_topics", true);
    config.rosbag.qos_match = declare_parameter<bool>("snapshots.rosbag.qos_match", true);

    config.rosbag.lazy_start = declare_parameter<bool>("snapshots.rosbag.lazy_start", false);
    config.rosbag.format = declare_parameter<std::string>("snapshots.rosbag.format", "sqlite3");
    config.rosbag.storage_path = declare_parameter<std::string>("snapshots.rosbag.storage_path", "");

    int64_t max_bag_size = declare_parameter<int64_t>("snapshots.rosbag.max_bag_size_mb", 50);
    if (max_bag_size <= 0) {
      RCLCPP_WARN(get_logger(), "snapshots.rosbag.max_bag_size_mb must be positive. Using 50MB");
      max_bag_size = 50;
    }
    config.rosbag.max_bag_size_mb = static_cast<size_t>(max_bag_size);

    int64_t max_total_storage = declare_parameter<int64_t>("snapshots.rosbag.max_total_storage_mb", 500);
    if (max_total_storage <= 0) {
      RCLCPP_WARN(get_logger(), "snapshots.rosbag.max_total_storage_mb must be positive. Using 500MB");
      max_total_storage = 500;
    }
    config.rosbag.max_total_storage_mb = static_cast<size_t>(max_total_storage);

    int64_t max_buffer = declare_parameter<int64_t>("snapshots.rosbag.max_buffer_mb", 256);
    if (max_buffer <= 0) {
      RCLCPP_WARN(get_logger(), "snapshots.rosbag.max_buffer_mb must be positive. Using 256MB");
      max_buffer = 256;
    }
    config.rosbag.max_buffer_mb = static_cast<size_t>(max_buffer);

    config.rosbag.auto_cleanup = declare_parameter<bool>("snapshots.rosbag.auto_cleanup", true);

    RCLCPP_INFO(get_logger(),
                "Rosbag capture enabled (duration=%.1fs+%.1fs, topics=%s, qos_match=%s, lazy=%s, format=%s, "
                "max_buffer=%zuMB, max_bag=%zuMB, max_total=%zuMB)",
                config.rosbag.duration_sec, config.rosbag.duration_after_sec, config.rosbag.topics.c_str(),
                config.rosbag.qos_match ? "true" : "false", config.rosbag.lazy_start ? "true" : "false",
                config.rosbag.format.c_str(), config.rosbag.max_buffer_mb, config.rosbag.max_bag_size_mb,
                config.rosbag.max_total_storage_mb);
  }

  if (config.enabled) {
    RCLCPP_INFO(get_logger(),
                "Snapshot capture enabled (background=%s, timeout=%.1fs, max_size=%zu, "
                "fault_specific=%zu, patterns=%zu, default_topics=%zu)",
                config.background_capture ? "true" : "false", config.timeout_sec, config.max_message_size,
                config.fault_specific.size(), config.patterns.size(), config.default_topics.size());
  } else {
    RCLCPP_INFO(get_logger(), "Snapshot capture disabled");
  }

  return config;
}

void FaultManagerNode::load_snapshot_config_from_yaml(const std::string & config_file, SnapshotConfig & config) {
  if (!std::filesystem::exists(config_file)) {
    RCLCPP_ERROR(get_logger(), "Snapshot config file not found: %s", config_file.c_str());
    return;
  }

  try {
    YAML::Node yaml_config = YAML::LoadFile(config_file);

    // Load fault_specific: map<fault_code, vector<topics>>
    if (yaml_config["fault_specific"]) {
      for (const auto & item : yaml_config["fault_specific"]) {
        std::string fault_code = item.first.as<std::string>();
        std::vector<std::string> topics;
        for (const auto & topic : item.second) {
          topics.push_back(topic.as<std::string>());
        }
        config.fault_specific[fault_code] = topics;
        RCLCPP_DEBUG(get_logger(), "Loaded fault_specific[%s] with %zu topics", fault_code.c_str(), topics.size());
      }
    }

    // Load patterns: map<regex_pattern, vector<topics>>
    if (yaml_config["patterns"]) {
      for (const auto & item : yaml_config["patterns"]) {
        std::string pattern = item.first.as<std::string>();
        std::vector<std::string> topics;
        for (const auto & topic : item.second) {
          topics.push_back(topic.as<std::string>());
        }
        config.patterns[pattern] = topics;
        RCLCPP_DEBUG(get_logger(), "Loaded pattern[%s] with %zu topics", pattern.c_str(), topics.size());
      }
    }

    // Optionally override default_topics from YAML (if specified and not already set via parameter)
    if (yaml_config["default_topics"] && config.default_topics.empty()) {
      for (const auto & topic : yaml_config["default_topics"]) {
        config.default_topics.push_back(topic.as<std::string>());
      }
      RCLCPP_DEBUG(get_logger(), "Loaded %zu default_topics from config file", config.default_topics.size());
    }

    RCLCPP_INFO(get_logger(), "Loaded snapshot config from %s (fault_specific=%zu, patterns=%zu)", config_file.c_str(),
                config.fault_specific.size(), config.patterns.size());

  } catch (const YAML::Exception & e) {
    RCLCPP_ERROR(get_logger(), "Failed to parse snapshot config file %s: %s", config_file.c_str(), e.what());
  }
}

std::unique_ptr<correlation::CorrelationEngine> FaultManagerNode::create_correlation_engine() {
  // Get correlation config file path from parameter
  auto config_file = declare_parameter<std::string>("correlation.config_file", "");

  if (config_file.empty()) {
    RCLCPP_DEBUG(get_logger(), "Correlation disabled: no config_file specified");
    return nullptr;
  }

  if (!std::filesystem::exists(config_file)) {
    RCLCPP_ERROR(get_logger(), "Correlation config file not found: %s", config_file.c_str());
    return nullptr;
  }

  try {
    auto config = correlation::parse_config_file(config_file);

    if (!config.enabled) {
      RCLCPP_INFO(get_logger(), "Correlation explicitly disabled in config");
      return nullptr;
    }

    // Validate the config
    auto validation = correlation::validate_config(config);
    for (const auto & warning : validation.warnings) {
      RCLCPP_WARN(get_logger(), "Correlation config: %s", warning.c_str());
    }
    if (!validation.valid) {
      for (const auto & error : validation.errors) {
        RCLCPP_ERROR(get_logger(), "Correlation config error: %s", error.c_str());
      }
      return nullptr;
    }

    RCLCPP_INFO(get_logger(), "Correlation engine enabled (default_window=%ums, patterns=%zu, rules=%zu)",
                config.default_window_ms, config.patterns.size(), config.rules.size());

    return std::make_unique<correlation::CorrelationEngine>(config);

  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Failed to load correlation config: %s", e.what());
    return nullptr;
  }
}

void FaultManagerNode::handle_get_snapshots(
    const std::shared_ptr<ros2_medkit_msgs::srv::GetSnapshots::Request> & request,
    const std::shared_ptr<ros2_medkit_msgs::srv::GetSnapshots::Response> & response) {
  // Validate fault_code
  std::string validation_error = validate_fault_code(request->fault_code);
  if (!validation_error.empty()) {
    response->success = false;
    response->error_message = validation_error;
    return;
  }

  // Check if fault exists
  auto fault = storage_->get_fault(request->fault_code);
  if (!fault) {
    response->success = false;
    response->error_message = "Fault not found: " + request->fault_code;
    return;
  }

  // Get snapshots from storage
  auto snapshots = storage_->get_snapshots(request->fault_code, request->topic);

  // Build JSON response
  nlohmann::json result;
  result["fault_code"] = request->fault_code;

  // Find the latest capture time (only include if snapshots exist)
  if (!snapshots.empty()) {
    int64_t latest_captured_at = 0;
    for (const auto & snapshot : snapshots) {
      if (snapshot.captured_at_ns > latest_captured_at) {
        latest_captured_at = snapshot.captured_at_ns;
      }
    }
    result["captured_at"] = static_cast<double>(latest_captured_at) / 1e9;
  }

  // Build topics object
  nlohmann::json topics_json = nlohmann::json::object();
  for (const auto & snapshot : snapshots) {
    nlohmann::json topic_entry;
    topic_entry["message_type"] = snapshot.message_type;

    // Parse the stored JSON data
    try {
      topic_entry["data"] = nlohmann::json::parse(snapshot.data);
    } catch (const nlohmann::json::exception & e) {
      RCLCPP_WARN(get_logger(), "Failed to parse snapshot data for topic '%s': %s", snapshot.topic.c_str(), e.what());
      topic_entry["data"] = snapshot.data;  // Store as raw string if parsing fails
    }

    topics_json[snapshot.topic] = topic_entry;
  }
  result["topics"] = topics_json;

  // Include rosbag info if available
  auto rosbag_info = storage_->get_rosbag_file(request->fault_code);
  if (rosbag_info) {
    nlohmann::json rosbag_json;
    rosbag_json["available"] = true;
    rosbag_json["duration_sec"] = rosbag_info->duration_sec;
    rosbag_json["size_bytes"] = rosbag_info->size_bytes;
    rosbag_json["format"] = rosbag_info->format;
    rosbag_json["download_url"] = "/api/v1/faults/" + request->fault_code + "/snapshots/bag";
    result["rosbag"] = rosbag_json;
  } else {
    result["rosbag"] = {{"available", false}};
  }

  response->success = true;
  response->data = result.dump();

  RCLCPP_DEBUG(get_logger(), "GetSnapshots returned %zu topics for fault '%s' (rosbag=%s)", snapshots.size(),
               request->fault_code.c_str(), rosbag_info ? "available" : "not available");
}

void FaultManagerNode::handle_get_rosbag(const std::shared_ptr<ros2_medkit_msgs::srv::GetRosbag::Request> & request,
                                         const std::shared_ptr<ros2_medkit_msgs::srv::GetRosbag::Response> & response) {
  // Validate fault_code
  std::string validation_error = validate_fault_code(request->fault_code);
  if (!validation_error.empty()) {
    response->success = false;
    response->error_message = validation_error;
    return;
  }

  // Check if fault exists
  auto fault = storage_->get_fault(request->fault_code);
  if (!fault) {
    response->success = false;
    response->error_message = "Fault not found: " + request->fault_code;
    return;
  }

  // Get rosbag info from storage
  auto rosbag_info = storage_->get_rosbag_file(request->fault_code);
  if (!rosbag_info) {
    response->success = false;
    response->error_message = "No rosbag file available for fault: " + request->fault_code;
    return;
  }

  // Check if file exists
  if (!std::filesystem::exists(rosbag_info->file_path)) {
    response->success = false;
    response->error_message = "Rosbag file not found on disk: " + rosbag_info->file_path;
    // Clean up the stale record
    storage_->delete_rosbag_file(request->fault_code);
    return;
  }

  response->success = true;
  response->file_path = rosbag_info->file_path;
  response->format = rosbag_info->format;
  response->duration_sec = rosbag_info->duration_sec;
  response->size_bytes = rosbag_info->size_bytes;

  RCLCPP_DEBUG(get_logger(), "GetRosbag returned file '%s' for fault '%s'", rosbag_info->file_path.c_str(),
               request->fault_code.c_str());
}

void FaultManagerNode::handle_list_rosbags(
    const std::shared_ptr<ros2_medkit_msgs::srv::ListRosbags::Request> & request,
    const std::shared_ptr<ros2_medkit_msgs::srv::ListRosbags::Response> & response) {
  RCLCPP_DEBUG(get_logger(), "ListRosbags request for entity: %s", request->entity_fqn.c_str());

  if (request->entity_fqn.empty()) {
    response->success = false;
    response->error_message = "entity_fqn cannot be empty";
    return;
  }

  // Use batch storage API to get all rosbags for this entity
  auto rosbags = storage_->list_rosbags_for_entity(request->entity_fqn);

  for (const auto & info : rosbags) {
    // Skip rosbags whose files no longer exist on disk
    if (!std::filesystem::exists(info.file_path)) {
      storage_->delete_rosbag_file(info.fault_code);
      continue;
    }

    response->fault_codes.push_back(info.fault_code);
    response->file_paths.push_back(info.file_path);
    response->formats.push_back(info.format);
    response->durations_sec.push_back(info.duration_sec);
    response->sizes_bytes.push_back(info.size_bytes);
  }

  response->success = true;
  RCLCPP_DEBUG(get_logger(), "ListRosbags returned %zu rosbags for entity '%s'", response->fault_codes.size(),
               request->entity_fqn.c_str());
}

void FaultManagerNode::handle_list_faults_for_entity(
    const std::shared_ptr<ros2_medkit_msgs::srv::ListFaultsForEntity::Request> & request,
    const std::shared_ptr<ros2_medkit_msgs::srv::ListFaultsForEntity::Response> & response) {
  RCLCPP_DEBUG(get_logger(), "ListFaultsForEntity request for entity: %s", request->entity_id.c_str());

  // Validate entity_id is not empty
  if (request->entity_id.empty()) {
    response->success = false;
    response->error_message = "entity_id cannot be empty";
    return;
  }

  // Get all faults from storage
  auto all_faults = storage_->get_all_faults();

  // Filter faults that have this entity in their reporting_sources
  for (const auto & fault : all_faults) {
    if (matches_entity(fault.reporting_sources, request->entity_id)) {
      response->faults.push_back(fault);
    }
  }

  response->success = true;
  RCLCPP_DEBUG(get_logger(), "ListFaultsForEntity returned %zu faults for entity '%s'", response->faults.size(),
               request->entity_id.c_str());
}

bool FaultManagerNode::matches_entity(const std::vector<std::string> & reporting_sources,
                                      const std::string & entity_id) {
  for (const auto & source : reporting_sources) {
    // Exact match
    if (source == entity_id) {
      return true;
    }

    // FQN suffix match: source ends with "/" + entity_id
    // e.g., source="/powertrain/motor_controller" matches entity_id="motor_controller"
    std::string suffix = "/" + entity_id;
    if (source.size() > suffix.size() && source.substr(source.size() - suffix.size()) == suffix) {
      return true;
    }
  }
  return false;
}

DebounceConfig FaultManagerNode::resolve_config(const std::string & source_id) const {
  DebounceConfig config = global_config_;
  if (threshold_resolver_) {
    config = threshold_resolver_->resolve(source_id, global_config_);
  }
  // Defensive: the merged config is validated at load time, but never hand the storage backend a
  // config that violates confirmation_threshold < 0 <= healing_threshold (the counter would stick).
  sanitize_debounce_config(config);
  return config;
}

}  // namespace ros2_medkit_fault_manager
