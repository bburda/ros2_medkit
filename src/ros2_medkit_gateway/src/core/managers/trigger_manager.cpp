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

#include "ros2_medkit_gateway/core/managers/trigger_manager.hpp"

#include <algorithm>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <utility>

#include "ros2_medkit_gateway/core/managers/log_manager.hpp"

namespace ros2_medkit_gateway {

// ---------------------------------------------------------------------------
// Lifecycle
// ---------------------------------------------------------------------------

TriggerManager::TriggerManager(ResourceChangeNotifier & notifier, ConditionRegistry & conditions, TriggerStore & store,
                               const TriggerConfig & config,
                               std::shared_ptr<TopicSubscriptionTransport> topic_transport)
  : notifier_(notifier)
  , conditions_(conditions)
  , store_(store)
  , config_(config)
  , topic_transport_(std::move(topic_transport)) {
  // Subscribe to all resource changes (empty filter = catch-all)
  NotifierFilter filter;
  notifier_sub_id_ = notifier_.subscribe(filter, [this](const ResourceChange & change) {
    on_resource_change(change);
  });
}

TriggerManager::~TriggerManager() {
  shutdown();
}

void TriggerManager::shutdown() {
  if (shutdown_flag_.exchange(true)) {
    return;  // Already shut down
  }

  notifier_.unsubscribe(notifier_sub_id_);

  std::lock_guard<std::mutex> lock(triggers_mutex_);
  for (auto & [id, state] : triggers_) {
    state->active.store(false);
    state->cv.notify_all();
  }
  // Drop every per-trigger subscription handle so the underlying transport
  // tears down its rclcpp resources before the manager goes away.
  topic_handles_.clear();
}

// ---------------------------------------------------------------------------
// ID generation
// ---------------------------------------------------------------------------

std::string TriggerManager::generate_id() {
  uint64_t id = next_id_.fetch_add(1);
  return "trig_" + std::to_string(id);
}

// ---------------------------------------------------------------------------
// Dispatch index helpers
// ---------------------------------------------------------------------------

std::string TriggerManager::dispatch_key(const std::string & collection, const std::string & entity_id) {
  return collection + ":" + entity_id;
}

void TriggerManager::add_to_dispatch_index(const std::string & trigger_id, const std::string & collection,
                                           const std::string & entity_id) {
  // Exact match key
  dispatch_index_[dispatch_key(collection, entity_id)].insert(trigger_id);
  // Catch-all key (for hierarchy triggers)
  dispatch_index_[dispatch_key(collection, "")].insert(trigger_id);
}

void TriggerManager::remove_from_dispatch_index(const std::string & trigger_id, const std::string & collection,
                                                const std::string & entity_id) {
  auto remove_from = [&](const std::string & key) {
    auto it = dispatch_index_.find(key);
    if (it != dispatch_index_.end()) {
      it->second.erase(trigger_id);
      if (it->second.empty()) {
        dispatch_index_.erase(it);
      }
    }
  };
  remove_from(dispatch_key(collection, entity_id));
  remove_from(dispatch_key(collection, ""));
}

// ---------------------------------------------------------------------------
// Hierarchy matching
// ---------------------------------------------------------------------------

void TriggerManager::set_entity_children_fn(EntityChildrenFn fn) {
  std::lock_guard<std::mutex> lock(hierarchy_mutex_);
  entity_children_fn_ = std::move(fn);
}

void TriggerManager::set_log_manager(LogManager * log_manager) {
  log_manager_ = log_manager;
}

void TriggerManager::set_resolve_topic_fn(ResolveTopicFn fn) {
  std::lock_guard<std::mutex> lock(triggers_mutex_);
  resolve_topic_fn_ = std::move(fn);
}

void TriggerManager::set_warn_log_fn(WarnLogFn fn) {
  std::lock_guard<std::mutex> lock(triggers_mutex_);
  warn_log_fn_ = std::move(fn);
}

void TriggerManager::set_unresolved_timeout(std::chrono::seconds timeout) {
  std::lock_guard<std::mutex> lock(triggers_mutex_);
  unresolved_timeout_ = timeout;
}

void TriggerManager::retry_unresolved_triggers() {
  if (shutdown_flag_.load()) {
    return;
  }

  // Collected under the lock, emitted after release: warn_log_fn_ is an
  // arbitrary external callback and must not run under triggers_mutex_.
  std::vector<std::string> warnings;
  WarnLogFn warn_fn;
  {
    std::lock_guard<std::mutex> lock(triggers_mutex_);

    if (unresolved_data_triggers_.empty() || !resolve_topic_fn_ || !topic_transport_) {
      return;
    }

    auto now = std::chrono::steady_clock::now();
    std::vector<size_t> resolved_indices;
    std::vector<size_t> expired_indices;
    std::vector<size_t> stale_indices;

    for (size_t i = 0; i < unresolved_data_triggers_.size(); ++i) {
      auto & entry = unresolved_data_triggers_[i];

      auto trigger_it = triggers_.find(entry.trigger_id);
      if (trigger_it == triggers_.end()) {
        // The trigger was deleted or expired while its topic was still
        // unresolved. Subscribing for it now would install a handle under a
        // trigger id nothing owns, and nothing would ever release it.
        stale_indices.push_back(i);
        continue;
      }

      // The give-up budget starts when discovery first reports the entity. An
      // entity this gateway has never seen is not a resolution failure, and the
      // orphan sweep leaves the record alone for exactly the same reason - the
      // attempt and the record therefore end together, never one before the
      // other.
      if (!entry.entity_seen_at.has_value() && trigger_it->second->entity_seen.load()) {
        entry.entity_seen_at = now;
      }

      if (entry.entity_seen_at.has_value()) {
        if (now - *entry.entity_seen_at > unresolved_timeout_) {
          // Give up loudly: the trigger stays registered and ACTIVE, but with no
          // subscription it will never fire. Silence here is a dead alarm the
          // operator believes in.
          expired_indices.push_back(i);
          warnings.push_back("trigger '" + entry.trigger_id + "': gave up resolving resource_path '" +
                             entry.resource_path + "' to a topic for entity '" + entry.entity_id + "' after " +
                             std::to_string(unresolved_timeout_.count()) +
                             " s of the entity being discovered. The trigger stays active but cannot fire; delete it "
                             "and recreate it once the topic exists.");
          continue;
        }
      } else if (!entry.waiting_reported && now - entry.created_at > unresolved_timeout_) {
        // Waiting on an undiscovered entity is unbounded on purpose, so it is
        // announced once. Without this the trigger is indistinguishable from
        // one that resolved.
        entry.waiting_reported = true;
        warnings.push_back("trigger '" + entry.trigger_id + "': entity '" + entry.entity_id +
                           "' has not appeared in discovery after " + std::to_string(unresolved_timeout_.count()) +
                           " s, so resource_path '" + entry.resource_path +
                           "' cannot be resolved to a topic. The trigger stays active and keeps retrying, and it "
                           "cannot fire until that entity is discovered.");
      }

      std::string topic_name = resolve_topic_fn_(entry.entity_id, entry.resource_path);
      if (!topic_name.empty()) {
        // Update the trigger's resolved_topic_name + register a transport
        // handle whose lifetime is tied to the trigger entry (cleared on
        // remove()/cleanup_expired_trigger()).
        {
          std::lock_guard<std::mutex> state_lock(trigger_it->second->mtx);
          trigger_it->second->info.resolved_topic_name = topic_name;
        }
        const std::string trigger_id = entry.trigger_id;
        const std::string entity_id = entry.entity_id;
        const std::string resource_path = entry.resource_path;
        auto handle = topic_transport_->subscribe(
            topic_name, /*msg_type=*/"", [this, entity_id, resource_path](const nlohmann::json & sample) {
              notifier_.notify("data", entity_id, resource_path, sample, ChangeType::UPDATED);
            });
        if (handle) {
          topic_handles_[trigger_id] = std::move(handle);
          resolved_indices.push_back(i);
        } else {
          // Subscribe failed (rclcpp threw inside TriggerTopicSubscriber).
          // Keep the trigger on the pending list so the next retry tick can
          // try again; the subscriber has already logged the root cause.
          // Without this guard the trigger would be marked resolved but
          // never deliver samples.
          // Index intentionally NOT added to resolved_indices.
        }
      }
    }

    // Remove resolved, expired and stale entries (reverse order to maintain indices)
    std::vector<size_t> to_remove;
    to_remove.insert(to_remove.end(), resolved_indices.begin(), resolved_indices.end());
    to_remove.insert(to_remove.end(), expired_indices.begin(), expired_indices.end());
    to_remove.insert(to_remove.end(), stale_indices.begin(), stale_indices.end());
    std::sort(to_remove.rbegin(), to_remove.rend());
    for (size_t idx : to_remove) {
      unresolved_data_triggers_.erase(unresolved_data_triggers_.begin() + static_cast<std::ptrdiff_t>(idx));
    }
    warn_fn = warn_log_fn_;
  }

  if (warn_fn) {
    for (const auto & w : warnings) {
      warn_fn(w);
    }
  }
}

void TriggerManager::set_entity_exists_fn(EntityExistsFn fn) {
  std::lock_guard<std::mutex> lock(entity_exists_mutex_);
  entity_exists_fn_ = std::move(fn);
}

void TriggerManager::sweep_orphaned_triggers() {
  // Phase 1: collect orphaned triggers under lock
  std::vector<std::pair<std::string, std::string>> orphaned;  // trigger id, entity id
  WarnLogFn warn_fn;
  {
    std::lock_guard<std::mutex> lock(triggers_mutex_);
    std::lock_guard<std::mutex> elock(entity_exists_mutex_);
    if (!entity_exists_fn_) {
      return;
    }
    warn_fn = warn_log_fn_;
    for (const auto & [id, state] : triggers_) {
      // entity_id and entity_type are immutable after creation - safe to read without state->mtx
      if (!state->active.load()) {
        continue;
      }
      if (entity_exists_fn_(state->info.entity_id, state->info.entity_type)) {
        state->entity_seen.store(true);
        continue;
      }
      // An entity this gateway has never seen has not disappeared from its
      // discovery, so a trigger on it is not orphaned - the sweep runs from
      // the first tick after startup, long before DDS has reported nodes that
      // were already running when the gateway came up. Sweeping it there would
      // delete a restored persistent trigger from the shared store, and
      // restore runs once per process, so nothing would ever bring it back.
      if (!state->entity_seen.load()) {
        continue;
      }
      orphaned.emplace_back(id, state->info.entity_id);
    }
  }

  // Phase 2: remove without holding triggers_mutex_ (remove() re-acquires it)
  for (const auto & entry : orphaned) {
    remove(entry.first);
  }

  // A sweep deletes a persistent trigger from the store as well, so the
  // operator loses it permanently. Name what went and why.
  if (warn_fn) {
    for (const auto & entry : orphaned) {
      warn_fn("orphan sweep: removed trigger '" + entry.first + "' because entity '" + entry.second +
              "' is not in the discovery cache.");
    }
  }
}

bool TriggerManager::matches_entity(const TriggerState & state, const std::string & notification_entity_id) const {
  // Direct match
  if (state.info.entity_id == notification_entity_id) {
    return true;
  }

  // Hierarchy match: check if notification entity is a descendant
  std::lock_guard<std::mutex> lock(hierarchy_mutex_);
  if (!entity_children_fn_) {
    return false;
  }

  auto children = entity_children_fn_(state.info.entity_id, state.info.entity_type);
  return std::find(children.begin(), children.end(), notification_entity_id) != children.end();
}

// ---------------------------------------------------------------------------
// Expiry check
// ---------------------------------------------------------------------------

bool TriggerManager::is_expired(const TriggerState & state) const {
  if (!state.info.expires_at.has_value()) {
    return false;
  }
  return std::chrono::system_clock::now() >= state.info.expires_at.value();
}

// ---------------------------------------------------------------------------
// ISO 8601 formatting
// ---------------------------------------------------------------------------

std::string TriggerManager::to_iso8601(const std::chrono::system_clock::time_point & tp) {
  auto time_t_val = std::chrono::system_clock::to_time_t(tp);
  std::tm tm_val{};
  gmtime_r(&time_t_val, &tm_val);

  auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(tp.time_since_epoch()).count() % 1000;

  std::ostringstream oss;
  oss << std::put_time(&tm_val, "%Y-%m-%dT%H:%M:%S") << "." << std::setw(3) << std::setfill('0') << ms << "Z";
  return oss.str();
}

// ---------------------------------------------------------------------------
// CRUD
// ---------------------------------------------------------------------------

tl::expected<TriggerInfo, TriggerCreateError> TriggerManager::create(const TriggerCreateRequest & req) {
  // Validate condition type
  auto evaluator = conditions_.get(req.condition_type);
  if (!evaluator) {
    return tl::make_unexpected(
        TriggerCreateError{TriggerError::ValidationError, "Unknown condition type: " + req.condition_type});
  }

  // Validate condition params
  auto validation = evaluator->validate_params(req.condition_params);
  if (!validation.has_value()) {
    return tl::make_unexpected(TriggerCreateError{TriggerError::ValidationError, validation.error()});
  }

  auto now = std::chrono::system_clock::now();

  // Build TriggerInfo outside the lock - generate_id() uses atomic next_id_
  auto state = std::make_shared<TriggerState>();
  state->info.id = generate_id();
  state->info.entity_id = req.entity_id;
  state->info.entity_type = req.entity_type;
  state->info.resource_uri = req.resource_uri;
  state->info.collection = req.collection;
  state->info.resource_path = req.resource_path;
  state->info.resolved_topic_name = req.resolved_topic_name;
  state->info.path = req.path;
  state->info.condition_type = req.condition_type;
  state->info.condition_params = req.condition_params;
  state->info.protocol = req.protocol;
  state->info.multishot = req.multishot;
  state->info.persistent = req.persistent;
  state->info.lifetime_sec = req.lifetime_sec;
  state->info.log_settings = req.log_settings;
  state->info.status = TriggerStatus::ACTIVE;
  state->info.created_at = now;

  if (req.lifetime_sec.has_value()) {
    state->info.expires_at = now + std::chrono::seconds(req.lifetime_sec.value());
  }

  // Persist before inserting into memory - if persistence fails, nothing is in-memory
  if (req.persistent) {
    auto save_result = store_.save(state->info);
    if (!save_result.has_value()) {
      return tl::make_unexpected(
          TriggerCreateError{TriggerError::PersistenceError, "Failed to persist trigger: " + save_result.error()});
    }
  }

  auto info_copy = state->info;

  {
    std::lock_guard<std::mutex> lock(triggers_mutex_);

    // Check capacity under the lock
    if (triggers_.size() >= static_cast<size_t>(config_.max_triggers)) {
      // Clean up persisted record if we just saved one
      if (req.persistent) {
        (void)store_.remove(state->info.id);
      }
      return tl::make_unexpected(
          TriggerCreateError{TriggerError::CapacityExceeded,
                             "Maximum trigger capacity (" + std::to_string(config_.max_triggers) + ") reached"});
    }

    add_to_dispatch_index(state->info.id, state->info.collection, state->info.entity_id);
    triggers_[state->info.id] = std::move(state);
  }

  // Subscribe to topic for data triggers.
  // When resolved_topic_name is empty (topic not yet discoverable at creation time),
  // queue for deferred resolution - retry_unresolved_triggers() will pick it up.
  if (topic_transport_ && req.collection == "data") {
    if (!req.resolved_topic_name.empty()) {
      const std::string entity_id = req.entity_id;
      const std::string resource_path = req.resource_path;
      auto handle = topic_transport_->subscribe(
          req.resolved_topic_name, /*msg_type=*/"", [this, entity_id, resource_path](const nlohmann::json & sample) {
            notifier_.notify("data", entity_id, resource_path, sample, ChangeType::UPDATED);
          });
      if (handle) {
        std::lock_guard<std::mutex> lock(triggers_mutex_);
        topic_handles_[info_copy.id] = std::move(handle);
      } else {
        // Subscribe failed at trigger creation. Roll back the trigger we
        // just added so the caller does not get a 201 for a trigger that
        // will never deliver. The underlying TriggerTopicSubscriber has
        // already logged the rclcpp failure. Persisted records are also
        // rolled back so a restart does not resurrect a doomed trigger.
        {
          std::lock_guard<std::mutex> lock(triggers_mutex_);
          remove_from_dispatch_index(info_copy.id, info_copy.collection, info_copy.entity_id);
          triggers_.erase(info_copy.id);
        }
        if (req.persistent) {
          (void)store_.remove(info_copy.id);
        }
        return tl::make_unexpected(TriggerCreateError{
            TriggerError::SubscribeFailed, "Failed to subscribe to topic '" + req.resolved_topic_name +
                                               "' for data trigger; see gateway log for the rclcpp root cause"});
      }
    } else if (!req.resource_path.empty()) {
      std::lock_guard<std::mutex> lock(triggers_mutex_);
      // The entity was validated to exist before this request was accepted, so
      // it counts as seen and the give-up budget runs from now.
      const auto queued_at = std::chrono::steady_clock::now();
      unresolved_data_triggers_.push_back(
          {info_copy.id, req.entity_id, req.resource_path, queued_at, queued_at, /*waiting_reported=*/false});
    }
  }

  return info_copy;
}

std::optional<TriggerInfo> TriggerManager::get(const std::string & trigger_id) {
  std::shared_ptr<TriggerState> state;
  {
    std::lock_guard<std::mutex> lock(triggers_mutex_);
    auto it = triggers_.find(trigger_id);
    if (it == triggers_.end()) {
      return std::nullopt;
    }
    state = it->second;
  }
  std::lock_guard<std::mutex> sub_lock(state->mtx);
  return state->info;
}

std::vector<TriggerInfo> TriggerManager::list(const std::string & entity_id) {
  std::lock_guard<std::mutex> lock(triggers_mutex_);
  std::vector<TriggerInfo> result;
  for (const auto & [id, state] : triggers_) {
    std::lock_guard<std::mutex> sub_lock(state->mtx);
    if (state->info.entity_id == entity_id) {
      result.push_back(state->info);
    }
  }
  return result;
}

tl::expected<TriggerInfo, TriggerCreateError> TriggerManager::update(const std::string & trigger_id, int new_lifetime) {
  if (new_lifetime <= 0) {
    return tl::make_unexpected(TriggerCreateError{TriggerError::ValidationError,
                                                  "Lifetime must be positive, got: " + std::to_string(new_lifetime)});
  }

  std::shared_ptr<TriggerState> state;
  {
    std::lock_guard<std::mutex> lock(triggers_mutex_);
    auto it = triggers_.find(trigger_id);
    if (it == triggers_.end()) {
      return tl::make_unexpected(TriggerCreateError{TriggerError::NotFound, "Trigger not found: " + trigger_id});
    }
    state = it->second;
  }

  std::lock_guard<std::mutex> sub_lock(state->mtx);

  state->info.lifetime_sec = new_lifetime;
  state->info.expires_at = std::chrono::system_clock::now() + std::chrono::seconds(new_lifetime);

  // Persist if needed
  if (state->info.persistent) {
    nlohmann::json fields;
    fields["lifetime_sec"] = new_lifetime;
    fields["expires_at"] = to_iso8601(state->info.expires_at.value());
    (void)store_.update(trigger_id, fields);
  }

  return state->info;
}

bool TriggerManager::remove(const std::string & trigger_id) {
  std::shared_ptr<TriggerState> state;
  std::string collection;
  std::string entity_id;
  bool persistent = false;
  OnRemovedCallback on_removed_copy;
  // Capture the per-trigger transport handle here so its destructor
  // (which unsubscribes from the underlying topic) runs after we drop
  // triggers_mutex_, keeping the lock window short.
  std::unique_ptr<TopicSubscriptionHandle> released_handle;
  {
    std::lock_guard<std::mutex> lock(triggers_mutex_);
    auto it = triggers_.find(trigger_id);
    if (it == triggers_.end()) {
      return false;
    }
    state = it->second;
    {
      std::lock_guard<std::mutex> sub_lock(state->mtx);
      collection = state->info.collection;
      entity_id = state->info.entity_id;
      persistent = state->info.persistent;
    }
    remove_from_dispatch_index(trigger_id, collection, entity_id);
    triggers_.erase(it);
    auto h_it = topic_handles_.find(trigger_id);
    if (h_it != topic_handles_.end()) {
      released_handle = std::move(h_it->second);
      topic_handles_.erase(h_it);
    }
    on_removed_copy = on_removed_;  // Copy under lock to avoid data race
  }

  // Mark inactive and wake any waiting SSE stream
  state->active.store(false);
  state->cv.notify_all();

  // released_handle goes out of scope here, unsubscribing the topic for
  // data triggers (no-op for non-data triggers, which never registered one).

  // Remove from persistent store if applicable (outside triggers_mutex_)
  if (persistent) {
    (void)store_.remove(trigger_id);
  }

  if (on_removed_copy) {
    on_removed_copy(trigger_id);
  }

  return true;
}

// ---------------------------------------------------------------------------
// SSE synchronization
// ---------------------------------------------------------------------------

bool TriggerManager::wait_for_event(const std::string & trigger_id, std::chrono::milliseconds timeout) {
  std::shared_ptr<TriggerState> state;
  {
    std::lock_guard<std::mutex> lock(triggers_mutex_);
    auto it = triggers_.find(trigger_id);
    if (it == triggers_.end()) {
      return true;  // Already removed - signal caller to stop
    }
    state = it->second;
  }

  std::unique_lock<std::mutex> sub_lock(state->mtx);
  bool woken = state->cv.wait_for(sub_lock, timeout, [&]() {
    return !state->pending_events.empty() || !state->active.load() || shutdown_flag_.load();
  });
  return woken;
}

void TriggerManager::cleanup_expired_trigger(const std::string & trigger_id,
                                             const std::shared_ptr<TriggerState> & state) {
  // Guard: only one thread may clean up a given trigger (I7 fix).
  // Both is_active() (SSE handler) and on_resource_change() (notifier worker) can
  // call this concurrently. The atomic exchange ensures exactly one proceeds.
  if (!state->active.exchange(false)) {
    return;  // Another thread already cleaned up
  }

  // Caller must NOT hold triggers_mutex_ or state->mtx.
  // Mark terminated under state lock, then clean up under triggers lock.
  bool persistent = false;
  std::string collection;
  std::string entity_id;
  {
    std::lock_guard<std::mutex> sub_lock(state->mtx);
    state->info.status = TriggerStatus::TERMINATED;
    persistent = state->info.persistent;
    collection = state->info.collection;
    entity_id = state->info.entity_id;
  }
  state->cv.notify_all();

  // Persist status change (outside all locks)
  if (persistent) {
    nlohmann::json fields;
    fields["status"] = "TERMINATED";
    (void)store_.update(trigger_id, fields);
  }

  // Remove from dispatch index, triggers map, and topic-handle map; the
  // released handle's dtor unsubscribes the underlying topic. The unique_ptr
  // is moved out from under the lock so the unsubscribe runs without holding
  // triggers_mutex_, mirroring the remove() ordering.
  OnRemovedCallback on_removed_copy;
  std::unique_ptr<TopicSubscriptionHandle> released_handle;
  {
    std::lock_guard<std::mutex> lock(triggers_mutex_);
    remove_from_dispatch_index(trigger_id, collection, entity_id);
    triggers_.erase(trigger_id);
    auto h_it = topic_handles_.find(trigger_id);
    if (h_it != topic_handles_.end()) {
      released_handle = std::move(h_it->second);
      topic_handles_.erase(h_it);
    }
    on_removed_copy = on_removed_;
  }

  // released_handle drops here -> transport unsubscribe.

  // Fire on_removed callback (I8 fix) - consistent with remove()
  if (on_removed_copy) {
    on_removed_copy(trigger_id);
  }
}

bool TriggerManager::is_active(const std::string & trigger_id) {
  std::shared_ptr<TriggerState> state;
  {
    std::lock_guard<std::mutex> lock(triggers_mutex_);
    auto it = triggers_.find(trigger_id);
    if (it == triggers_.end()) {
      return false;
    }
    state = it->second;
  }

  {
    std::lock_guard<std::mutex> sub_lock(state->mtx);

    if (!state->active.load()) {
      return false;
    }

    if (state->info.status == TriggerStatus::TERMINATED) {
      return false;
    }

    if (!is_expired(*state)) {
      return true;
    }
  }

  // Expired: clean up fully (releases state->mtx first, then re-acquires locks as needed)
  cleanup_expired_trigger(trigger_id, state);
  return false;
}

std::optional<nlohmann::json> TriggerManager::consume_pending_event(const std::string & trigger_id) {
  std::shared_ptr<TriggerState> state;
  {
    std::lock_guard<std::mutex> lock(triggers_mutex_);
    auto it = triggers_.find(trigger_id);
    if (it == triggers_.end()) {
      return std::nullopt;
    }
    state = it->second;
  }

  std::lock_guard<std::mutex> sub_lock(state->mtx);
  if (state->pending_events.empty()) {
    return std::nullopt;
  }

  auto event = std::move(state->pending_events.front());
  state->pending_events.pop_front();
  return event;
}

// ---------------------------------------------------------------------------
// Callbacks
// ---------------------------------------------------------------------------

void TriggerManager::set_on_removed(OnRemovedCallback callback) {
  std::lock_guard<std::mutex> lock(triggers_mutex_);
  on_removed_ = std::move(callback);
}

// ---------------------------------------------------------------------------
// Persistent trigger loading
// ---------------------------------------------------------------------------

size_t TriggerManager::load_persistent_triggers() {
  if (config_.on_restart_behavior != "restore") {
    return 0;
  }

  // Collected under the lock, emitted after release: warn_log_fn_ is an
  // arbitrary external callback and must not run under triggers_mutex_.
  std::vector<std::string> warnings;
  WarnLogFn warn_fn;
  {
    std::lock_guard<std::mutex> lock(triggers_mutex_);
    warn_fn = warn_log_fn_;
  }

  auto load_result = store_.load_all();
  if (!load_result.has_value()) {
    // Restore runs once, during construction. A store that cannot be read
    // therefore does not delay the operator's persistent triggers, it ends
    // them: the REST API answers 404 for the life of the process, exactly as
    // if they had never been created.
    if (warn_fn) {
      warn_fn("persistent trigger restore: reading the trigger store failed (" + load_result.error() +
              "). No persistent trigger was restored, and restore is not retried, so they stay absent until this "
              "gateway is restarted.");
    }
    return 0;
  }

  size_t restored = 0;
  {
    std::lock_guard<std::mutex> lock(triggers_mutex_);
    for (auto & info : load_result.value()) {
      // A TERMINATED row is a trigger that already ran its course; the store
      // keeps it as a record, and not restoring it is what TERMINATED means.
      if (info.status != TriggerStatus::ACTIVE) {
        continue;
      }

      if (info.expires_at.has_value() && std::chrono::system_clock::now() >= info.expires_at.value()) {
        // The operator created this one and it is not coming back. Say so once
        // - the write-back below makes it TERMINATED, so the next restart
        // takes the branch above instead.
        warnings.push_back("persistent trigger restore: trigger '" + info.id + "' on entity '" + info.entity_id +
                           "' expired while the gateway was down and was not restored.");
        nlohmann::json fields;
        fields["status"] = "TERMINATED";
        auto update_result = store_.update(info.id, fields);
        if (!update_result.has_value()) {
          warnings.push_back("persistent trigger restore: marking expired trigger '" + info.id +
                             "' TERMINATED in the store failed (" + update_result.error() +
                             "). It stays ACTIVE on disk and is re-examined on every restart.");
        }
        continue;
      }

      auto state = std::make_shared<TriggerState>();
      state->info = std::move(info);
      // This process has never discovered the entity, so the orphan sweep must
      // not read "missing from the cache" as "gone" until discovery has had
      // its say.
      state->entity_seen.store(false);

      // Restore previous value state if available
      auto state_result = store_.load_state(state->info.id);
      if (state_result.has_value() && state_result.value().has_value()) {
        state->previous_value = state_result.value().value();
        state->has_previous_value = true;
      }

      // Update next_id_ to avoid collisions
      // IDs are "trig_N" - extract N and ensure next_id_ is beyond it
      auto underscore_pos = state->info.id.find('_');
      if (underscore_pos != std::string::npos) {
        try {
          auto loaded_id = std::stoull(state->info.id.substr(underscore_pos + 1));
          uint64_t current = next_id_.load();
          while (current <= loaded_id && !next_id_.compare_exchange_weak(current, loaded_id + 1)) {
          }
        } catch (...) {
          // Non-numeric suffix - skip ID adjustment
        }
      }

      add_to_dispatch_index(state->info.id, state->info.collection, state->info.entity_id);

      // Restored data triggers go through deferred resolution rather than
      // straight onto the persisted topic name. The entity belongs to a process
      // this gateway has not met yet, so the subscription attempt has to stay
      // open for as long as the record does, and deferred resolution is the
      // only path whose budget is tied to entity_seen. Subscribing here could
      // not honour that in any case: restore runs inside the constructor, the
      // transport's executor is wired after it returns, so the attempt would be
      // parked on a clock that starts now and runs out long before a late
      // entity arrives.
      if (topic_transport_ && state->info.collection == "data") {
        const std::string trigger_id = state->info.id;
        const std::string entity_id = state->info.entity_id;
        const std::string resource_path = state->info.resource_path;
        if (!resource_path.empty()) {
          unresolved_data_triggers_.push_back({trigger_id, entity_id, resource_path, std::chrono::steady_clock::now(),
                                               std::nullopt,
                                               /*waiting_reported=*/false});
        } else if (!state->info.resolved_topic_name.empty()) {
          // A stored row carrying a topic but no resource_path has nothing for
          // the entity cache to re-resolve, so the persisted topic is all there
          // is to go on.
          auto handle = topic_transport_->subscribe(state->info.resolved_topic_name, /*msg_type=*/"",
                                                    [this, entity_id, resource_path](const nlohmann::json & sample) {
                                                      notifier_.notify("data", entity_id, resource_path, sample,
                                                                       ChangeType::UPDATED);
                                                    });
          if (handle) {
            topic_handles_[trigger_id] = std::move(handle);
          } else {
            warnings.push_back("persistent trigger restore: trigger '" + trigger_id + "' could not subscribe to '" +
                               state->info.resolved_topic_name +
                               "' and carries no resource_path to resolve again from. It stays active but cannot "
                               "fire; delete it and recreate it.");
          }
        }
      }

      triggers_[state->info.id] = std::move(state);
      ++restored;
    }
  }

  if (warn_fn) {
    for (const auto & w : warnings) {
      warn_fn(w);
    }
  }

  return restored;
}

// ---------------------------------------------------------------------------
// Resource change evaluation
// ---------------------------------------------------------------------------

void TriggerManager::on_resource_change(const ResourceChange & change) {
  if (shutdown_flag_.load()) {
    return;
  }

  // I5 fix: prevent recursive trigger evaluation. The notifier worker is
  // single-threaded, so if log_manager_->add_log_entry() triggers a notification
  // that re-enters on_resource_change(), we break the cycle here.
  if (evaluating_trigger_) {
    return;
  }

  // Collect candidate trigger IDs from the dispatch index
  std::vector<std::pair<std::string, std::shared_ptr<TriggerState>>> candidates;
  {
    std::lock_guard<std::mutex> lock(triggers_mutex_);

    // Gather trigger IDs from both exact match and catch-all keys
    std::unordered_set<std::string> candidate_ids;

    auto exact_key = dispatch_key(change.collection, change.entity_id);
    auto it_exact = dispatch_index_.find(exact_key);
    if (it_exact != dispatch_index_.end()) {
      candidate_ids.insert(it_exact->second.begin(), it_exact->second.end());
    }

    auto catchall_key = dispatch_key(change.collection, "");
    auto it_catchall = dispatch_index_.find(catchall_key);
    if (it_catchall != dispatch_index_.end()) {
      candidate_ids.insert(it_catchall->second.begin(), it_catchall->second.end());
    }

    // Resolve IDs to states
    for (const auto & tid : candidate_ids) {
      auto it = triggers_.find(tid);
      if (it != triggers_.end()) {
        candidates.emplace_back(tid, it->second);
      }
    }
  }

  // Collect terminated one-shot trigger IDs for post-loop cleanup
  std::vector<std::string> terminated_triggers;

  // Evaluate each candidate trigger
  for (auto & [trigger_id, state] : candidates) {
    std::unique_lock<std::mutex> sub_lock(state->mtx);

    // Skip terminated or inactive triggers
    if (state->info.status != TriggerStatus::ACTIVE || !state->active.load()) {
      continue;
    }

    // Check expiry - clean up fully (remove from dispatch index + triggers map)
    if (is_expired(*state)) {
      sub_lock.unlock();
      cleanup_expired_trigger(trigger_id, state);
      continue;
    }

    // Check resource_path match (if trigger specifies a specific resource)
    if (!state->info.resource_path.empty() && state->info.resource_path != change.resource_path) {
      continue;
    }

    // Check entity hierarchy match
    // Release sub_lock before calling matches_entity (it acquires hierarchy_mutex_)
    sub_lock.unlock();

    if (!matches_entity(*state, change.entity_id)) {
      continue;
    }

    sub_lock.lock();

    // Re-check status after re-acquiring lock
    if (state->info.status != TriggerStatus::ACTIVE || !state->active.load()) {
      continue;
    }

    // Extract value via JSON Pointer - skip evaluation if pointer doesn't match
    nlohmann::json extracted_value;
    if (state->info.path.empty()) {
      extracted_value = change.value;
    } else {
      try {
        auto ptr = nlohmann::json::json_pointer(state->info.path);
        if (change.value.contains(ptr)) {
          extracted_value = change.value[ptr];
        } else {
          continue;  // Path not found in this change - skip evaluation
        }
      } catch (...) {
        continue;  // Malformed pointer - skip evaluation
      }
    }

    // Get the evaluator
    auto evaluator = conditions_.get(state->info.condition_type);
    if (!evaluator) {
      continue;
    }

    // Build optional previous value
    std::optional<nlohmann::json> prev_opt;
    if (state->has_previous_value) {
      prev_opt = state->previous_value;
    }

    // Evaluate condition
    bool fired = evaluator->evaluate(prev_opt, extracted_value, state->info.condition_params);

    // Update previous value
    state->previous_value = extracted_value;
    state->has_previous_value = true;

    if (fired) {
      // Build EventEnvelope
      // Increment event counter (used by SSE id: field for reconnection)
      state->event_counter.fetch_add(1);
      nlohmann::json envelope;
      envelope["timestamp"] = to_iso8601(std::chrono::system_clock::now());
      envelope["payload"] = change.value;

      // C2 fix: push to bounded deque instead of overwriting single optional.
      // Drop oldest events when queue is full to prevent unbounded growth.
      state->pending_events.push_back(std::move(envelope));
      if (state->pending_events.size() > TriggerState::kMaxPendingEvents) {
        state->pending_events.pop_front();
      }

      // I6 fix: capture data under lock, then release before doing I/O.
      // This prevents store_.save_state() and log_manager_->add_log_entry()
      // from blocking SSE consumers that need state->mtx.
      bool should_persist_state = (config_.on_restart_behavior == "restore" && state->info.persistent);
      const auto & previous_value_copy = extracted_value;
      bool should_terminate = !state->info.multishot;
      auto tid_copy = state->info.id;
      auto log_settings_copy = state->info.log_settings;
      auto entity_id_copy = state->info.entity_id;
      auto condition_type_copy = state->info.condition_type;
      auto resource_uri_copy = state->info.resource_uri;
      bool is_persistent = state->info.persistent;

      if (should_terminate) {
        state->info.status = TriggerStatus::TERMINATED;
        state->active.store(false);
      }

      sub_lock.unlock();

      // Wake SSE consumers
      state->cv.notify_all();

      // Schedule one-shot cleanup after evaluation loop completes
      if (should_terminate) {
        terminated_triggers.push_back(tid_copy);
      }

      // I/O operations outside state->mtx (I6 fix)
      if (should_persist_state) {
        (void)store_.save_state(tid_copy, previous_value_copy);
      }

      if (should_terminate && is_persistent) {
        nlohmann::json fields;
        fields["status"] = "TERMINATED";
        (void)store_.update(tid_copy, fields);
      }

      // I5 fix: RAII guard prevents recursive notification from log triggers
      if (log_settings_copy.has_value() && log_manager_) {
        auto & ls = *log_settings_copy;
        evaluating_trigger_ = true;
        try {
          log_manager_->add_log_entry(
              entity_id_copy, ls.value("severity", "info"), ls.value("marker", "Trigger fired"),
              {{"trigger_id", tid_copy}, {"condition_type", condition_type_copy}, {"resource", resource_uri_copy}});
        } catch (...) {
          // Don't let log entry failure prevent trigger delivery
        }
        evaluating_trigger_ = false;
      }
    } else {
      // Condition didn't fire - persist state if needed (I6: still move I/O outside lock)
      bool should_persist_state = (config_.on_restart_behavior == "restore" && state->info.persistent);
      const auto & previous_value_copy = extracted_value;
      auto tid_copy = trigger_id;
      sub_lock.unlock();

      if (should_persist_state) {
        (void)store_.save_state(tid_copy, previous_value_copy);
      }
    }
  }

  // Clean up terminated one-shot triggers (outside all locks)
  for (const auto & tid : terminated_triggers) {
    remove(tid);
  }
}

}  // namespace ros2_medkit_gateway
