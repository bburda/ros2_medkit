// Copyright 2026 mfaferek93
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

#include <chrono>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <set>
#include <string>
#include <vector>

#include "ros2_medkit_fault_manager/correlation/pattern_matcher.hpp"
#include "ros2_medkit_fault_manager/correlation/types.hpp"

namespace ros2_medkit_fault_manager {
namespace correlation {

/// Result of processing a fault through the correlation engine
struct ProcessFaultResult {
  /// Whether this fault should be muted (not published as event)
  bool should_mute{false};

  /// Whether this fault was identified as a root cause
  bool is_root_cause{false};

  /// For symptoms: the root cause fault code
  std::string root_cause_code;

  /// The rule ID that matched (if any)
  std::string rule_id;

  /// Delay from root cause in milliseconds (for symptoms)
  uint32_t delay_ms{0};

  /// For auto-cluster: cluster ID if this fault triggered or joined a cluster
  std::string cluster_id;

  /// For auto-cluster: fault codes that should be retroactively muted
  /// When cluster reaches min_count threshold, previous non-representative faults
  /// that were added before threshold was reached need to be muted retroactively
  std::vector<std::string> retroactive_mute_codes;
};

/// Result of clearing a fault
struct ProcessClearResult {
  /// List of symptom fault codes that should be auto-cleared
  std::vector<std::string> auto_cleared_codes;
};

/// Information about a muted fault (for ListFaults response)
struct MutedFaultData {
  std::string fault_code;
  std::string root_cause_code;
  std::string rule_id;
  uint32_t delay_ms{0};
};

/// Information about an active cluster (for ListFaults response)
struct ClusterData {
  std::string cluster_id;
  std::string rule_id;
  std::string rule_name;
  std::string label;
  std::string representative_code;
  std::string representative_severity;
  std::vector<std::string> fault_codes;
  std::chrono::system_clock::time_point first_at;
  std::chrono::system_clock::time_point last_at;
};

/// Main correlation engine
///
/// Processes incoming faults and determines:
/// - Whether they should be muted (symptoms of a root cause)
/// - Whether they are root causes that should collect symptoms
/// - Whether they form part of an auto-detected cluster
/// - Whether a planned stop is in force, which mutes every fault whose cycle
///   starts while it is on
///
/// Thread-safe: all public methods can be called from multiple threads.
class CorrelationEngine {
 public:
  /// Pseudo root cause recorded against a fault the planned stop muted. It is
  /// not a fault code any reporter can raise; it names the declaration as the
  /// reason the fault is not being announced.
  static constexpr const char * kPlannedStopRootCause = "PLANNED_STOP";

  /// Rule id recorded against a fault the planned stop muted. The switch is not
  /// a configured rule, so it carries a fixed id of its own and a consumer of
  /// `muted_faults` can tell an operator's declaration from a correlation rule.
  static constexpr const char * kPlannedStopRuleId = "planned_stop";

  /// Create correlation engine from configuration
  /// @param config Correlation configuration (must be enabled and valid)
  explicit CorrelationEngine(const CorrelationConfig & config);

  /// Process an incoming fault
  /// @param fault_code The fault code
  /// @param severity The fault severity (for representative selection)
  /// @param timestamp When the fault occurred
  /// @param cycle_started Whether this report STARTED a fault cycle (a new fault,
  ///        or one raised again after being cleared) as opposed to a repeat of a
  ///        condition that is already up. Only a cycle that starts while a planned
  ///        stop is declared is marked by it: reporters are level-triggered and
  ///        re-send FAILED for as long as the condition holds, so muting on any
  ///        report would swallow a fault that was announced before the stop began.
  ///        Correlation rules are unaffected and match on every report.
  /// @return Processing result indicating whether to mute, correlations, etc.
  ProcessFaultResult process_fault(const std::string & fault_code, const std::string & severity,
                                   std::chrono::steady_clock::time_point timestamp = std::chrono::steady_clock::now(),
                                   bool cycle_started = true);

  /// Process a fault being cleared
  /// @param fault_code The fault code being cleared
  /// @return Result with list of symptoms to auto-clear
  ProcessClearResult process_clear(const std::string & fault_code);

  /// Get all currently muted faults
  /// @return List of muted fault data
  std::vector<MutedFaultData> get_muted_faults() const;

  /// Get count of muted faults
  uint32_t get_muted_count() const;

  /// Whether a fault code is currently muted as a symptom.
  /// @param fault_code Code to test
  /// @return True while the code is suppressed by a root cause
  bool is_muted(const std::string & fault_code) const;

  /// Get all active clusters
  /// @return List of cluster data
  std::vector<ClusterData> get_clusters() const;

  /// Get count of active clusters
  uint32_t get_cluster_count() const;

  /// Clean up expired pending root causes and clusters
  /// Called periodically to remove old state
  void cleanup_expired();

  /// Declare a planned stop. From here every fault reported through
  /// process_fault is muted unless a rule already mutes it, and stays muted
  /// until the stop ends or the fault is cleared. Idempotent.
  void begin_planned_stop();

  /// Withdraw the planned stop and release the faults it alone was muting.
  /// A fault a rule has since claimed keeps that rule's mute and is NOT
  /// returned; nor is one that was cleared while the stop was on.
  /// @return the fault codes this call unmuted, so the caller can announce the
  ///         confirmations that were never published
  std::vector<std::string> end_planned_stop();

  /// Whether a planned stop is currently declared.
  bool planned_stop_active() const;

  /// Release one fault from the planned stop's mute, if the stop is what is
  /// muting it. A rule's mute is left alone. Called when a fault is acknowledged
  /// on a path that does not run the correlation clear (a scoped per-entity
  /// DELETE), so an acknowledged fault never stays counted as muted.
  void drop_planned_stop_mute(const std::string & fault_code);

  /// Register a fault as muted by the planned stop without a report driving it.
  /// This is how a restart re-marks the faults whose cycles started inside a stop
  /// that is still declared: their mute lived in the process that is gone, and
  /// without it the switch-off would neither release nor announce them. A fault a
  /// rule is already muting is left to that rule.
  void restore_planned_stop_mute(const std::string & fault_code);

 private:
  /// The correlation half of process_fault: rules, clusters and their muting,
  /// with no planned-stop involvement. Caller holds mutex_.
  ProcessFaultResult correlate(const std::string & fault_code, const std::string & severity,
                               std::chrono::steady_clock::time_point timestamp);

  /// Check if fault matches a root cause pattern in any hierarchical rule
  /// @return Rule ID if matched, empty optional otherwise
  std::optional<std::string> try_as_root_cause(const std::string & fault_code);

  /// Check if fault is a symptom of any pending root cause
  /// @return ProcessFaultResult with correlation info if matched
  std::optional<ProcessFaultResult> try_as_symptom(const std::string & fault_code,
                                                   std::chrono::steady_clock::time_point timestamp);

  /// Check if fault matches an auto-cluster rule
  /// @return ProcessFaultResult with cluster info if matched
  std::optional<ProcessFaultResult> try_auto_cluster(const std::string & fault_code, const std::string & severity,
                                                     std::chrono::steady_clock::time_point timestamp);

  /// Generate unique cluster ID
  std::string generate_cluster_id(const std::string & rule_id);

  CorrelationConfig config_;
  std::unique_ptr<PatternMatcher> matcher_;

  /// Active root causes waiting for symptoms
  struct PendingRootCause {
    std::string fault_code;
    std::string rule_id;
    std::chrono::steady_clock::time_point timestamp;
    uint32_t window_ms;
  };
  std::vector<PendingRootCause> pending_root_causes_;

  /// Mapping from root cause to its symptoms
  std::map<std::string, std::vector<std::string>> root_to_symptoms_;

  /// Muted faults (fault_code -> data)
  std::map<std::string, MutedFaultData> muted_faults_;

  /// Active clusters (cluster_id -> data)
  std::map<std::string, ClusterData> active_clusters_;

  /// Mapping from fault code to cluster ID (for faults in clusters)
  std::map<std::string, std::string> fault_to_cluster_;

  /// Pending cluster with steady_clock timestamp for window tracking
  struct PendingCluster {
    ClusterData data;
    std::chrono::steady_clock::time_point steady_first_at;
    std::map<std::string, std::string> fault_severities;  ///< fault_code -> severity
  };

  /// Pending clusters being formed (rule_id -> cluster data)
  /// Once min_count is reached, moved to active_clusters_
  std::map<std::string, PendingCluster> pending_clusters_;

  /// Counter for cluster ID generation
  uint64_t cluster_counter_{0};

  /// Whether an operator has declared a planned stop.
  bool planned_stop_active_{false};

  /// Fault codes muted BY the planned stop and by nothing else. A code leaves
  /// this set the moment a rule claims it or it is cleared, which is what makes
  /// the switch-off release exactly the faults it is still holding down.
  std::set<std::string> planned_stop_muted_;

  mutable std::mutex mutex_;
};

}  // namespace correlation
}  // namespace ros2_medkit_fault_manager
