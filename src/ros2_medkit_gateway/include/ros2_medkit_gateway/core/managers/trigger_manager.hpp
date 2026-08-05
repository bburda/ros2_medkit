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

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <nlohmann/json.hpp>
#include <tl/expected.hpp>

#include "ros2_medkit_gateway/core/condition_evaluator.hpp"
#include "ros2_medkit_gateway/core/resource_change_notifier.hpp"
#include "ros2_medkit_gateway/core/transports/topic_subscription_transport.hpp"
#include "ros2_medkit_gateway/core/trigger_store.hpp"

namespace ros2_medkit_gateway {

// Forward declarations
class LogManager;

/// Error categories for trigger create/update operations.
enum class TriggerError { ValidationError, CapacityExceeded, PersistenceError, NotFound, SubscribeFailed };

/// Typed error returned by TriggerManager::create() and update().
struct TriggerCreateError {
  TriggerError code;
  std::string message;
};

/// Request to create a new trigger.
struct TriggerCreateRequest {
  std::string entity_id;
  std::string entity_type;          ///< "apps", "components", "areas", "functions"
  std::string resource_uri;         ///< "/api/v1/apps/sensor/data/temperature"
  std::string collection;           ///< Parsed from resource_uri: "data"
  std::string resource_path;        ///< Parsed: "temperature"
  std::string resolved_topic_name;  ///< Full ROS 2 topic (e.g. "/sensor/temperature"), empty if unresolved
  std::string path;                 ///< JSON Pointer: "/data"
  std::string condition_type;       ///< "OnChange", "LeaveRange", etc.
  nlohmann::json condition_params;  ///< e.g. {"lower_bound":20, "upper_bound":30}
  std::string protocol = "sse";
  bool multishot = false;
  bool persistent = false;
  std::optional<int> lifetime_sec;
  std::optional<nlohmann::json> log_settings;
};

/// Configuration for TriggerManager behavior.
struct TriggerConfig {
  int max_triggers = 1000;
  std::string on_restart_behavior = "reset";  ///< "reset" or "restore"
};

/// Central coordinator for triggers.
///
/// Owns trigger lifecycle (CRUD), subscribes to ResourceChangeNotifier for
/// resource change events, evaluates conditions using ConditionRegistry,
/// stores pending events for SSE stream pickup, and manages entity hierarchy
/// matching and persistence via TriggerStore.
class TriggerManager {
 public:
  /// Construct the trigger manager.
  ///
  /// @param notifier      Resource-change notifier the manager subscribes to.
  /// @param conditions    Registry of condition evaluators (OnChange, etc.).
  /// @param store         Persistence store for trigger metadata + state.
  /// @param config        Capacity + on-restart behaviour.
  /// @param topic_transport Optional transport used to register data-trigger
  ///                      topic subscriptions. When null the manager retains
  ///                      its CRUD/dispatch behaviour but never subscribes
  ///                      (useful in tests that don't need topic routing).
  TriggerManager(ResourceChangeNotifier & notifier, ConditionRegistry & conditions, TriggerStore & store,
                 const TriggerConfig & config, std::shared_ptr<TopicSubscriptionTransport> topic_transport = nullptr);
  ~TriggerManager();

  // Non-copyable, non-movable (owns trigger state and notifier subscription)
  TriggerManager(const TriggerManager &) = delete;
  TriggerManager & operator=(const TriggerManager &) = delete;
  TriggerManager(TriggerManager &&) = delete;
  TriggerManager & operator=(TriggerManager &&) = delete;

  // --- CRUD -----------------------------------------------------------------

  /// Create a new trigger. Validates condition type and params.
  tl::expected<TriggerInfo, TriggerCreateError> create(const TriggerCreateRequest & req);

  /// Get a trigger by ID.
  std::optional<TriggerInfo> get(const std::string & trigger_id);

  /// List all triggers for a given entity.
  std::vector<TriggerInfo> list(const std::string & entity_id);

  /// Update a trigger's lifetime.
  tl::expected<TriggerInfo, TriggerCreateError> update(const std::string & trigger_id, int new_lifetime);

  /// Remove a trigger. Returns true if found and removed.
  bool remove(const std::string & trigger_id);

  // --- SSE synchronization --------------------------------------------------

  /// Block until an event is available for the given trigger, or timeout.
  /// Returns true if woken by event/shutdown, false on timeout.
  bool wait_for_event(const std::string & trigger_id, std::chrono::milliseconds timeout);

  /// Check if a trigger is active (exists, not terminated, not expired).
  bool is_active(const std::string & trigger_id);

  /// Consume the pending event for SSE delivery. Returns nullopt if no event.
  std::optional<nlohmann::json> consume_pending_event(const std::string & trigger_id);

  // --- Callbacks ------------------------------------------------------------

  /// Callback invoked when a trigger is removed.
  using OnRemovedCallback = std::function<void(const std::string & trigger_id)>;
  void set_on_removed(OnRemovedCallback callback);

  // --- Lifecycle ------------------------------------------------------------

  /// Shut down - wakes all waiting threads and marks all triggers inactive.
  void shutdown();

  /// Load persistent triggers from the store (on gateway restart).
  void load_persistent_triggers();

  // --- Hierarchy matching ---------------------------------------------------

  /// Function that returns descendant entity IDs for a given parent.
  using EntityChildrenFn =
      std::function<std::vector<std::string>(const std::string & entity_id, const std::string & entity_type)>;

  /// Set the entity hierarchy resolver. Called by GatewayNode after cache is available.
  void set_entity_children_fn(EntityChildrenFn fn);

  /// Set the LogManager for trigger log_settings integration.
  /// Called by GatewayNode after both TriggerManager and LogManager are available.
  void set_log_manager(LogManager * log_manager);

  /// Resolves a resource_path to a full ROS 2 topic name for a given entity.
  /// Returns the topic name, or empty string if unresolvable.
  using ResolveTopicFn = std::function<std::string(const std::string & entity_id, const std::string & resource_path)>;

  /// Set the topic name resolver. Called by GatewayNode after cache is available.
  void set_resolve_topic_fn(ResolveTopicFn fn);

  /// Operator-facing warning sink (transport-agnostic; GatewayNode wires it to
  /// the ROS logger). Used when deferred resolution gives up on a trigger.
  using WarnLogFn = std::function<void(const std::string & message)>;
  void set_warn_log_fn(WarnLogFn fn);

  /// How long deferred resolution keeps retrying before giving up.
  /// Default 60 s; tests shrink it to exercise the expiry path.
  void set_unresolved_timeout(std::chrono::seconds timeout);

  /// Retry resolving data triggers whose topic names were unknown at creation.
  /// Called periodically (today: from the rclcpp adapter's retry tick) so
  /// that triggers stuck without a topic name get a chance to subscribe.
  void retry_unresolved_triggers();

  // --- Entity existence check (for orphan sweep) ----------------------------

  /// Function that checks whether an entity still exists in the discovery cache.
  using EntityExistsFn = std::function<bool(const std::string & entity_id, const std::string & entity_type)>;

  /// Set the entity existence checker. Called by GatewayNode after cache is available.
  void set_entity_exists_fn(EntityExistsFn fn);

  /// Remove triggers whose entities no longer exist in discovery.
  ///
  /// Two-phase approach to avoid deadlock: collect orphaned IDs under lock,
  /// then call remove() (which re-acquires triggers_mutex_) outside the lock.
  void sweep_orphaned_triggers();

 private:
  /// Per-trigger runtime state.
  ///
  /// Threading: most fields require `mtx` for read/write access. Exceptions:
  ///   - `info.entity_id` and `info.entity_type` are immutable after creation
  ///     and safe to read without `mtx` (e.g. in matches_entity()).
  ///   - `active` is atomic and can be read/written without `mtx`.
  struct TriggerState {
    TriggerInfo info;
    nlohmann::json previous_value;
    bool has_previous_value{false};
    std::mutex mtx;
    std::condition_variable cv;
    std::atomic<bool> active{true};
    std::deque<nlohmann::json> pending_events;
    static constexpr size_t kMaxPendingEvents = 100;
    std::atomic<uint64_t> event_counter{0};
  };

  /// Generate the next trigger ID.
  std::string generate_id();

  /// Build a dispatch key from collection and entity_id.
  static std::string dispatch_key(const std::string & collection, const std::string & entity_id);

  /// Add a trigger to the dispatch index.
  void add_to_dispatch_index(const std::string & trigger_id, const std::string & collection,
                             const std::string & entity_id);

  /// Remove a trigger from the dispatch index.
  void remove_from_dispatch_index(const std::string & trigger_id, const std::string & collection,
                                  const std::string & entity_id);

  /// Check if the notification entity matches the trigger's entity (direct or hierarchy).
  bool matches_entity(const TriggerState & state, const std::string & notification_entity_id) const;

  /// Check if a trigger has expired.
  bool is_expired(const TriggerState & state) const;

  /// Format a system_clock time_point as ISO 8601 string.
  static std::string to_iso8601(const std::chrono::system_clock::time_point & tp);

  /// Clean up an expired trigger: remove from dispatch index and triggers_ map.
  /// Caller must NOT hold triggers_mutex_ or state->mtx.
  void cleanup_expired_trigger(const std::string & trigger_id, const std::shared_ptr<TriggerState> & state);

  /// Callback for ResourceChangeNotifier.
  void on_resource_change(const ResourceChange & change);

  // Dependencies (non-owning)
  ResourceChangeNotifier & notifier_;
  ConditionRegistry & conditions_;
  TriggerStore & store_;
  TriggerConfig config_;

  // Our notifier subscription
  NotifierSubscriptionId notifier_sub_id_{0};

  // Main storage
  mutable std::mutex triggers_mutex_;
  std::unordered_map<std::string, std::shared_ptr<TriggerState>> triggers_;

  // Secondary index for O(1) dispatch: "collection:entity_id" -> set of trigger IDs
  std::unordered_map<std::string, std::unordered_set<std::string>> dispatch_index_;

  // ID generation
  std::atomic<uint64_t> next_id_{1};

  // Shutdown flag
  std::atomic<bool> shutdown_flag_{false};

  // Removal callback
  OnRemovedCallback on_removed_;

  // Hierarchy resolver
  mutable std::mutex hierarchy_mutex_;
  EntityChildrenFn entity_children_fn_;

  // Data trigger topic transport (shared, optional). The adapter side owns
  // the rclcpp resources; the manager only stores per-trigger handles.
  std::shared_ptr<TopicSubscriptionTransport> topic_transport_;

  // Per-trigger live subscription handles. Destruction of a handle
  // unsubscribes the underlying topic; the manager removes entries from
  // this map on remove()/cleanup_expired_trigger() so the dtor side-effect
  // ties subscription lifetime to trigger lifetime. Guarded by triggers_mutex_.
  std::unordered_map<std::string, std::unique_ptr<TopicSubscriptionHandle>> topic_handles_;

  // LogManager for log_settings integration (non-owning, optional)
  LogManager * log_manager_{nullptr};

  // Entity existence checker (for orphan sweep)
  mutable std::mutex entity_exists_mutex_;
  EntityExistsFn entity_exists_fn_;

  // Recursive loop prevention: set when evaluating a trigger fires a log entry
  // that would re-enter on_resource_change(). Only accessed from the notifier
  // worker thread (single-threaded dispatch), so no synchronization needed.
  bool evaluating_trigger_{false};

  // Deferred resolution for data triggers created before topic was discoverable.
  // Periodically retried via retry_unresolved_triggers().
  struct UnresolvedTrigger {
    std::string trigger_id;
    std::string entity_id;
    std::string resource_path;
    std::chrono::steady_clock::time_point created_at;
  };
  std::vector<UnresolvedTrigger> unresolved_data_triggers_;  // guarded by triggers_mutex_
  ResolveTopicFn resolve_topic_fn_;                          // guarded by triggers_mutex_
  WarnLogFn warn_log_fn_;                                    // guarded by triggers_mutex_
  std::chrono::seconds unresolved_timeout_{60};              // guarded by triggers_mutex_
};

}  // namespace ros2_medkit_gateway
