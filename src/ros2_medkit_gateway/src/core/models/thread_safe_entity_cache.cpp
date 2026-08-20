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

#include "ros2_medkit_gateway/core/models/thread_safe_entity_cache.hpp"

#include <algorithm>
#include <functional>
#include <mutex>
#include <sstream>
#include <unordered_set>
#include <utility>

namespace ros2_medkit_gateway {

// ============================================================================
// Incremental reconcile primitives (in-place; reuse the existing stores)
// ============================================================================

template <typename T>
bool ThreadSafeEntityCache::reconcile(SlotStore<T> & store, FlatHashMap<std::string, uint32_t> & index,
                                      std::vector<T> && incoming) {
  bool changed = false;

  // Mark every currently-allocated slot as not-yet-seen. seen_ is sized to the
  // slot count; it only resize()s on the grow path below (steady state reuses
  // the reserved buffer). slot_count() includes dead slots, which is fine: dead
  // slots stay unseen and are skipped by the live-collect below.
  const size_t slot_count = store.slot_count();
  if (seen_.size() < slot_count) {
    seen_.assign(slot_count, 0u);
  } else {
    std::fill(seen_.begin(), seen_.begin() + static_cast<std::ptrdiff_t>(slot_count), 0u);
  }

  for (auto & e : incoming) {
    if (uint32_t * slot = index.find(e.id)) {
      const uint32_t s = *slot;
      seen_[s] = 1u;
      // Compare against current payload; only assign (and flag) on a real diff.
      if (!(store[s] == e)) {
        store.assign(s, std::move(e));
        changed = true;
      }
    } else {
      const uint32_t s = store.alloc_slot();
      // alloc_slot() may have appended a brand-new slot past seen_'s size.
      if (s >= seen_.size()) {
        seen_.resize(store.slot_count(), 0u);
      }
      seen_[s] = 1u;
      store.assign(s, std::move(e));
      // Read the id from the freshly-assigned slot, never from the moved-out e.
      index.insert_or_assign(store[s].id, s);
      changed = true;
    }
  }

  // Collect ids of live slots that were not re-seen; free them after iterating
  // so we never mutate the store while for_each_live walks it.
  dead_ids_.clear();
  store.for_each_live([&](uint32_t slot, const T & cur) {
    if (!seen_[slot]) {
      dead_ids_.push_back(cur.id);
    }
  });
  for (const auto & id : dead_ids_) {
    if (uint32_t * slot = index.find(id)) {
      const uint32_t s = *slot;
      index.erase(id);
      store.free(s);
      changed = true;
    }
  }
  dead_ids_.clear();

  return changed;
}

bool ThreadSafeEntityCache::patch_map(FlatHashMap<std::string, std::string> & map,
                                      std::unordered_map<std::string, std::string> && incoming) {
  bool changed = false;

  patch_dead_.clear();
  map.for_each([&](const std::string & k, const std::string &) {
    if (incoming.count(k) == 0) {
      patch_dead_.push_back(k);
    }
  });
  for (const auto & k : patch_dead_) {
    map.erase(k);
    changed = true;
  }
  patch_dead_.clear();

  for (auto & kv : incoming) {
    std::string * cur = map.find(kv.first);
    if (!cur) {
      map.insert_or_assign(kv.first, std::move(kv.second));
      changed = true;
    } else if (*cur != kv.second) {
      *cur = std::move(kv.second);
      changed = true;
    }
  }

  return changed;
}

void ThreadSafeEntityCache::refresh_grew() {
  const bool any_grew = areas_.grew() || components_.grew() || apps_.grew() || functions_.grew() ||
                        area_index_.grew() || component_index_.grew() || app_index_.grew() || function_index_.grew() ||
                        component_to_apps_.grew() || component_to_subcomponents_.grew() || area_to_components_.grew() ||
                        area_to_subareas_.grew() || function_to_apps_.grew() || function_to_components_.grew() ||
                        operation_index_.grew() || topic_type_cache_.grew() || node_to_app_.grew();
  if (any_grew) {
    grew_ = true;
    ++overflow_count_;
  }

  areas_.clear_grew();
  components_.clear_grew();
  apps_.clear_grew();
  functions_.clear_grew();
  area_index_.clear_grew();
  component_index_.clear_grew();
  app_index_.clear_grew();
  function_index_.clear_grew();
  component_to_apps_.clear_grew();
  component_to_subcomponents_.clear_grew();
  area_to_components_.clear_grew();
  area_to_subareas_.clear_grew();
  function_to_apps_.clear_grew();
  function_to_components_.clear_grew();
  operation_index_.clear_grew();
  topic_type_cache_.clear_grew();
  node_to_app_.clear_grew();
}

// ============================================================================
// Construction / capacity
// ============================================================================

ThreadSafeEntityCache::ThreadSafeEntityCache(size_t capacity) {
  reserve(capacity);
}

void ThreadSafeEntityCache::reserve(size_t capacity) {
  std::unique_lock lock(mutex_);

  // Capacity only ever grows. A smaller request would shrink seen_ (assign) and
  // capacity_ while the index containers (reserve never shrinks) would not,
  // breaking the documented "only grows" invariant; treat it as a no-op.
  if (capacity <= capacity_) {
    return;
  }

  areas_.reserve(capacity);
  components_.reserve(capacity);
  apps_.reserve(capacity);
  functions_.reserve(capacity);

  area_index_.reserve(capacity);
  component_index_.reserve(capacity);
  app_index_.reserve(capacity);
  function_index_.reserve(capacity);

  component_to_apps_.reserve(capacity);
  component_to_subcomponents_.reserve(capacity);
  area_to_components_.reserve(capacity);
  area_to_subareas_.reserve(capacity);
  function_to_apps_.reserve(capacity);
  function_to_components_.reserve(capacity);

  operation_index_.reserve(capacity);
  topic_type_cache_.reserve(capacity);
  node_to_app_.reserve(capacity);

  seen_.assign(capacity, 0u);
  dead_ids_.reserve(capacity);
  patch_dead_.reserve(capacity);

  capacity_ = capacity;
}

// ============================================================================
// Writer methods (exclusive lock)
// ============================================================================

void ThreadSafeEntityCache::update_all(std::vector<Area> areas, std::vector<Component> components,
                                       std::vector<App> apps, std::vector<Function> functions,
                                       std::unordered_map<std::string, std::string> node_to_app) {
  std::unique_lock lock(mutex_);

  bool changed = false;
  changed |= reconcile(areas_, area_index_, std::move(areas));
  changed |= reconcile(components_, component_index_, std::move(components));
  changed |= reconcile(apps_, app_index_, std::move(apps));
  changed |= reconcile(functions_, function_index_, std::move(functions));
  changed |= patch_map(node_to_app_, std::move(node_to_app));

  if (!changed) {
    refresh_grew();
    return;
  }

  rebuild_relationship_indexes();
  rebuild_operation_index();
  last_update_ = std::chrono::system_clock::now();
  refresh_grew();
  generation_.fetch_add(1, std::memory_order_release);
}

void ThreadSafeEntityCache::update_areas(std::vector<Area> areas) {
  std::unique_lock lock(mutex_);
  if (!reconcile(areas_, area_index_, std::move(areas))) {
    refresh_grew();
    return;
  }
  rebuild_relationship_indexes();
  last_update_ = std::chrono::system_clock::now();
  refresh_grew();
  generation_.fetch_add(1, std::memory_order_release);
}

void ThreadSafeEntityCache::update_components(std::vector<Component> components) {
  std::unique_lock lock(mutex_);
  if (!reconcile(components_, component_index_, std::move(components))) {
    refresh_grew();
    return;
  }
  rebuild_relationship_indexes();
  rebuild_operation_index();
  last_update_ = std::chrono::system_clock::now();
  refresh_grew();
  generation_.fetch_add(1, std::memory_order_release);
}

void ThreadSafeEntityCache::update_apps(std::vector<App> apps) {
  std::unique_lock lock(mutex_);
  if (!reconcile(apps_, app_index_, std::move(apps))) {
    refresh_grew();
    return;
  }
  rebuild_relationship_indexes();
  rebuild_operation_index();
  last_update_ = std::chrono::system_clock::now();
  refresh_grew();
  generation_.fetch_add(1, std::memory_order_release);
}

void ThreadSafeEntityCache::update_functions(std::vector<Function> functions) {
  std::unique_lock lock(mutex_);
  if (!reconcile(functions_, function_index_, std::move(functions))) {
    refresh_grew();
    return;
  }
  rebuild_relationship_indexes();
  last_update_ = std::chrono::system_clock::now();
  refresh_grew();
  generation_.fetch_add(1, std::memory_order_release);
}

void ThreadSafeEntityCache::update_topic_types(std::unordered_map<std::string, std::string> topic_types) {
  std::unique_lock lock(mutex_);
  if (patch_map(topic_type_cache_, std::move(topic_types))) {
    refresh_grew();
    generation_.fetch_add(1, std::memory_order_release);
  } else {
    refresh_grew();
  }
}

std::unordered_map<std::string, std::string> ThreadSafeEntityCache::get_node_to_app() const {
  std::shared_lock lock(mutex_);
  std::unordered_map<std::string, std::string> result;
  result.reserve(node_to_app_.size());
  node_to_app_.for_each([&](const std::string & fqn, const std::string & app_id) {
    result.emplace(fqn, app_id);
  });
  return result;
}

std::string ThreadSafeEntityCache::resolve_node_to_app(const std::string & node_fqn) const {
  std::shared_lock lock(mutex_);
  const std::string * app_id = node_to_app_.find(node_fqn);
  return app_id ? *app_id : "";
}

// ============================================================================
// Reader methods - List all
// ============================================================================

std::vector<Area> ThreadSafeEntityCache::get_areas() const {
  std::shared_lock lock(mutex_);
  return areas_.collect_live();
}

std::vector<Component> ThreadSafeEntityCache::get_components() const {
  std::shared_lock lock(mutex_);
  return components_.collect_live();
}

std::vector<App> ThreadSafeEntityCache::get_apps() const {
  std::shared_lock lock(mutex_);
  return apps_.collect_live();
}

std::vector<Function> ThreadSafeEntityCache::get_functions() const {
  std::shared_lock lock(mutex_);
  return functions_.collect_live();
}

std::string ThreadSafeEntityCache::get_topic_type(const std::string & topic_name) const {
  std::shared_lock lock(mutex_);
  const std::string * type = topic_type_cache_.find(topic_name);
  return type ? *type : "";
}

// ============================================================================
// Reader methods - Get by ID
// ============================================================================

std::optional<Area> ThreadSafeEntityCache::get_area(const std::string & id) const {
  std::shared_lock lock(mutex_);
  const uint32_t * slot = area_index_.find(id);
  if (slot && areas_.is_live(*slot)) {
    return areas_[*slot];
  }
  return std::nullopt;
}

std::optional<Component> ThreadSafeEntityCache::get_component(const std::string & id) const {
  std::shared_lock lock(mutex_);
  const uint32_t * slot = component_index_.find(id);
  if (slot && components_.is_live(*slot)) {
    return components_[*slot];
  }
  return std::nullopt;
}

std::optional<App> ThreadSafeEntityCache::get_app(const std::string & id) const {
  std::shared_lock lock(mutex_);
  const uint32_t * slot = app_index_.find(id);
  if (slot && apps_.is_live(*slot)) {
    return apps_[*slot];
  }
  return std::nullopt;
}

ThreadSafeEntityCache::AppLinksSnapshot ThreadSafeEntityCache::get_app_with_links(const std::string & id) const {
  AppLinksSnapshot snapshot;
  std::shared_lock lock(mutex_);

  const uint32_t * app_slot = app_index_.find(id);
  if (!app_slot || !apps_.is_live(*app_slot)) {
    return snapshot;
  }
  snapshot.app = apps_[*app_slot];

  const auto & component_id = snapshot.app->component_id;
  if (component_id.empty()) {
    return snapshot;
  }

  const uint32_t * comp_slot = component_index_.find(component_id);
  if (!comp_slot || !components_.is_live(*comp_slot)) {
    return snapshot;  // component referenced but missing - leave .component empty
  }
  snapshot.component = components_[*comp_slot];

  const auto & area_id = snapshot.component->area;
  if (area_id.empty()) {
    return snapshot;
  }

  const uint32_t * area_slot = area_index_.find(area_id);
  if (!area_slot || !areas_.is_live(*area_slot)) {
    return snapshot;  // area referenced but missing
  }
  snapshot.area = areas_[*area_slot];
  return snapshot;
}

ThreadSafeEntityCache::AppDependenciesSnapshot
ThreadSafeEntityCache::get_app_with_dependencies(const std::string & id) const {
  AppDependenciesSnapshot snapshot;
  std::shared_lock lock(mutex_);

  const uint32_t * app_slot = app_index_.find(id);
  if (!app_slot || !apps_.is_live(*app_slot)) {
    return snapshot;
  }
  snapshot.app = apps_[*app_slot];

  snapshot.dependencies.reserve(snapshot.app->depends_on.size());
  for (const auto & dep_id : snapshot.app->depends_on) {
    const uint32_t * dep_slot = app_index_.find(dep_id);
    if (dep_slot && apps_.is_live(*dep_slot)) {
      snapshot.dependencies.emplace_back(dep_id, apps_[*dep_slot]);
    } else {
      snapshot.dependencies.emplace_back(dep_id, std::nullopt);
    }
  }
  return snapshot;
}

std::optional<Function> ThreadSafeEntityCache::get_function(const std::string & id) const {
  std::shared_lock lock(mutex_);
  const uint32_t * slot = function_index_.find(id);
  if (slot && functions_.is_live(*slot)) {
    return functions_[*slot];
  }
  return std::nullopt;
}

// ============================================================================
// Reader methods - Check existence
// ============================================================================

bool ThreadSafeEntityCache::has_area(const std::string & id) const {
  std::shared_lock lock(mutex_);
  const uint32_t * slot = area_index_.find(id);
  return slot && areas_.is_live(*slot);
}

bool ThreadSafeEntityCache::has_component(const std::string & id) const {
  std::shared_lock lock(mutex_);
  const uint32_t * slot = component_index_.find(id);
  return slot && components_.is_live(*slot);
}

bool ThreadSafeEntityCache::has_app(const std::string & id) const {
  std::shared_lock lock(mutex_);
  const uint32_t * slot = app_index_.find(id);
  return slot && apps_.is_live(*slot);
}

bool ThreadSafeEntityCache::has_function(const std::string & id) const {
  std::shared_lock lock(mutex_);
  const uint32_t * slot = function_index_.find(id);
  return slot && functions_.is_live(*slot);
}

// ============================================================================
// Reader methods - Find any entity
// ============================================================================

std::optional<EntityRef> ThreadSafeEntityCache::find_entity(const std::string & id) const {
  std::shared_lock lock(mutex_);

  // Search order: Component, App, Area, Function
  if (const uint32_t * slot = component_index_.find(id); slot && components_.is_live(*slot)) {
    return EntityRef{SovdEntityType::COMPONENT, *slot};
  }
  if (const uint32_t * slot = app_index_.find(id); slot && apps_.is_live(*slot)) {
    return EntityRef{SovdEntityType::APP, *slot};
  }
  if (const uint32_t * slot = area_index_.find(id); slot && areas_.is_live(*slot)) {
    return EntityRef{SovdEntityType::AREA, *slot};
  }
  if (const uint32_t * slot = function_index_.find(id); slot && functions_.is_live(*slot)) {
    return EntityRef{SovdEntityType::FUNCTION, *slot};
  }

  return std::nullopt;
}

SovdEntityType ThreadSafeEntityCache::get_entity_type(const std::string & id) const {
  auto ref = find_entity(id);
  return ref ? ref->type : SovdEntityType::UNKNOWN;
}

// ============================================================================
// Relationship queries
// ============================================================================

std::vector<std::string> ThreadSafeEntityCache::get_apps_for_component(const std::string & component_id) const {
  std::shared_lock lock(mutex_);
  std::vector<std::string> result;

  const std::vector<uint32_t> * slots = component_to_apps_.find(component_id);
  if (slots) {
    result.reserve(slots->size());
    for (uint32_t slot : *slots) {
      if (apps_.is_live(slot)) {
        result.push_back(apps_[slot].id);
      }
    }
  }
  return result;
}

std::vector<std::string> ThreadSafeEntityCache::get_components_for_area(const std::string & area_id) const {
  std::shared_lock lock(mutex_);
  std::vector<std::string> result;

  const std::vector<uint32_t> * slots = area_to_components_.find(area_id);
  if (slots) {
    result.reserve(slots->size());
    for (uint32_t slot : *slots) {
      if (components_.is_live(slot)) {
        result.push_back(components_[slot].id);
      }
    }
  }
  return result;
}

std::vector<std::string> ThreadSafeEntityCache::get_apps_for_function(const std::string & function_id) const {
  std::shared_lock lock(mutex_);
  std::vector<std::string> result;

  const std::vector<uint32_t> * slots = function_to_apps_.find(function_id);
  if (slots) {
    result.reserve(slots->size());
    for (uint32_t slot : *slots) {
      if (apps_.is_live(slot)) {
        result.push_back(apps_[slot].id);
      }
    }
  }
  return result;
}

std::vector<std::string> ThreadSafeEntityCache::get_subareas(const std::string & area_id) const {
  std::shared_lock lock(mutex_);
  std::vector<std::string> result;

  const std::vector<uint32_t> * slots = area_to_subareas_.find(area_id);
  if (slots) {
    result.reserve(slots->size());
    for (uint32_t slot : *slots) {
      if (areas_.is_live(slot)) {
        result.push_back(areas_[slot].id);
      }
    }
  }
  return result;
}

// ============================================================================
// Aggregation methods
// ============================================================================

AggregatedOperations ThreadSafeEntityCache::get_app_operations(const std::string & app_id) const {
  std::shared_lock lock(mutex_);
  AggregatedOperations result;
  result.aggregation_level = "app";
  result.is_aggregated = false;

  const uint32_t * slot = app_index_.find(app_id);
  if (!slot || !apps_.is_live(*slot)) {
    return result;
  }

  const auto & app = apps_[*slot];
  result.services = app.services;
  result.actions = app.actions;
  result.source_ids.push_back(app_id);

  return result;
}

AggregatedOperations ThreadSafeEntityCache::get_component_operations(const std::string & component_id) const {
  std::shared_lock lock(mutex_);
  AggregatedOperations result;
  result.aggregation_level = "component";

  const uint32_t * comp_slot = component_index_.find(component_id);
  if (!comp_slot || !components_.is_live(*comp_slot)) {
    return result;
  }

  std::unordered_set<std::string> seen_paths;

  // Add component's own operations first
  collect_operations_from_component(*comp_slot, seen_paths, result);

  // Add operations from hosted apps
  const std::vector<uint32_t> * app_slots = component_to_apps_.find(component_id);
  if (app_slots) {
    collect_operations_from_apps(*app_slots, seen_paths, result);
    // Mark as aggregated if we collected from apps
    if (!app_slots->empty()) {
      result.is_aggregated = true;
    }
  }

  return result;
}

AggregatedOperations ThreadSafeEntityCache::get_area_operations(const std::string & area_id) const {
  std::shared_lock lock(mutex_);
  AggregatedOperations result;
  result.aggregation_level = "area";
  result.is_aggregated = true;  // Area operations are always aggregated

  const uint32_t * area_slot = area_index_.find(area_id);
  if (!area_slot || !areas_.is_live(*area_slot)) {
    return result;
  }

  std::unordered_set<std::string> seen_paths;

  // Get all components in this area
  const std::vector<uint32_t> * comp_slots = area_to_components_.find(area_id);
  if (comp_slots) {
    for (uint32_t comp_slot : *comp_slots) {
      if (!components_.is_live(comp_slot)) {
        continue;
      }

      const auto & comp = components_[comp_slot];

      // Add component's own operations
      collect_operations_from_component(comp_slot, seen_paths, result);

      // Add operations from component's apps
      const std::vector<uint32_t> * app_slots = component_to_apps_.find(comp.id);
      if (app_slots) {
        collect_operations_from_apps(*app_slots, seen_paths, result);
      }
    }
  }

  return result;
}

AggregatedOperations ThreadSafeEntityCache::get_function_operations(const std::string & function_id) const {
  std::shared_lock lock(mutex_);
  AggregatedOperations result;
  result.aggregation_level = "function";
  result.is_aggregated = true;  // Function operations are always aggregated

  const uint32_t * func_slot = function_index_.find(function_id);
  if (!func_slot || !functions_.is_live(*func_slot)) {
    return result;
  }

  std::unordered_set<std::string> seen_paths;

  // Get all apps implementing this function
  const std::vector<uint32_t> * app_slots = function_to_apps_.find(function_id);
  if (app_slots) {
    collect_operations_from_apps(*app_slots, seen_paths, result);
  }

  return result;
}

// ============================================================================
// Configuration aggregation methods
// ============================================================================

AggregatedConfigurations ThreadSafeEntityCache::get_entity_configurations(const std::string & entity_id) const {
  // Find entity by ID - O(1) lookups in each index
  auto entity = find_entity(entity_id);
  if (!entity) {
    return {};
  }

  switch (entity->type) {
    case SovdEntityType::APP:
      return get_app_configurations(entity_id);
    case SovdEntityType::COMPONENT:
      return get_component_configurations(entity_id);
    case SovdEntityType::AREA:
      return get_area_configurations(entity_id);
    case SovdEntityType::FUNCTION:
      return get_function_configurations(entity_id);
    case SovdEntityType::SERVER:
    case SovdEntityType::UNKNOWN:
    default:
      return {};
  }
}

AggregatedConfigurations ThreadSafeEntityCache::get_app_configurations(const std::string & app_id) const {
  std::shared_lock lock(mutex_);
  AggregatedConfigurations result;
  result.aggregation_level = "app";
  result.is_aggregated = false;

  const uint32_t * slot = app_index_.find(app_id);
  if (!slot || !apps_.is_live(*slot)) {
    return result;
  }

  const auto & app = apps_[*slot];

  // App must have a resolvable FQN to have parameters
  auto eff_fqn = app.effective_fqn();
  if (!eff_fqn.empty()) {
    NodeConfigInfo info;
    info.node_fqn = std::move(eff_fqn);
    info.app_id = app_id;
    info.entity_id = app_id;
    result.nodes.push_back(info);
  }

  result.source_ids.push_back(app_id);
  return result;
}

AggregatedConfigurations ThreadSafeEntityCache::get_component_configurations(const std::string & component_id) const {
  std::shared_lock lock(mutex_);
  AggregatedConfigurations result;
  result.aggregation_level = "component";

  const uint32_t * comp_slot = component_index_.find(component_id);
  if (!comp_slot || !components_.is_live(*comp_slot)) {
    return result;
  }

  result.source_ids.push_back(component_id);

  // Collect node FQNs from hosted apps
  const std::vector<uint32_t> * app_slots = component_to_apps_.find(component_id);
  if (app_slots) {
    for (uint32_t app_slot : *app_slots) {
      if (!apps_.is_live(app_slot)) {
        continue;
      }
      const auto & app = apps_[app_slot];
      auto eff_fqn = app.effective_fqn();
      if (!eff_fqn.empty()) {
        NodeConfigInfo info;
        info.node_fqn = std::move(eff_fqn);
        info.app_id = app.id;
        info.entity_id = component_id;
        result.nodes.push_back(info);
        result.source_ids.push_back(app.id);
      }
    }
    // Only mark as aggregated if there are multiple nodes (sources)
    // A component with a single app should behave like a non-aggregated entity
    result.is_aggregated = result.nodes.size() > 1;
  }

  return result;
}

AggregatedConfigurations ThreadSafeEntityCache::get_area_configurations(const std::string & area_id) const {
  std::shared_lock lock(mutex_);
  AggregatedConfigurations result;
  result.aggregation_level = "area";

  const uint32_t * area_slot = area_index_.find(area_id);
  if (!area_slot || !areas_.is_live(*area_slot)) {
    return result;
  }

  result.source_ids.push_back(area_id);

  // Get all components in this area
  const std::vector<uint32_t> * comp_slots = area_to_components_.find(area_id);
  if (comp_slots) {
    for (uint32_t comp_slot : *comp_slots) {
      if (!components_.is_live(comp_slot)) {
        continue;
      }

      const auto & comp = components_[comp_slot];
      result.source_ids.push_back(comp.id);

      // Add node FQNs from component's apps
      const std::vector<uint32_t> * app_slots = component_to_apps_.find(comp.id);
      if (app_slots) {
        for (uint32_t app_slot : *app_slots) {
          if (!apps_.is_live(app_slot)) {
            continue;
          }
          const auto & app = apps_[app_slot];
          auto eff_fqn = app.effective_fqn();
          if (!eff_fqn.empty()) {
            NodeConfigInfo info;
            info.node_fqn = std::move(eff_fqn);
            info.app_id = app.id;
            info.entity_id = area_id;
            result.nodes.push_back(info);
            result.source_ids.push_back(app.id);
          }
        }
      }
    }
  }

  // Only mark as aggregated if there are multiple nodes
  result.is_aggregated = result.nodes.size() > 1;

  return result;
}

AggregatedConfigurations ThreadSafeEntityCache::get_function_configurations(const std::string & function_id) const {
  std::shared_lock lock(mutex_);
  AggregatedConfigurations result;
  result.aggregation_level = "function";

  const uint32_t * func_slot = function_index_.find(function_id);
  if (!func_slot || !functions_.is_live(*func_slot)) {
    return result;
  }

  result.source_ids.push_back(function_id);

  // Get all apps implementing this function
  const std::vector<uint32_t> * app_slots = function_to_apps_.find(function_id);
  if (app_slots) {
    for (uint32_t app_slot : *app_slots) {
      if (!apps_.is_live(app_slot)) {
        continue;
      }
      const auto & app = apps_[app_slot];
      auto eff_fqn = app.effective_fqn();
      if (!eff_fqn.empty()) {
        NodeConfigInfo info;
        info.node_fqn = std::move(eff_fqn);
        info.app_id = app.id;
        info.entity_id = function_id;
        result.nodes.push_back(info);
        result.source_ids.push_back(app.id);
      }
    }
  }

  // Only mark as aggregated if there are multiple nodes
  result.is_aggregated = result.nodes.size() > 1;

  return result;
}

// ============================================================================
// Operation lookup
// ============================================================================

std::optional<EntityRef> ThreadSafeEntityCache::find_operation_owner(const std::string & operation_path) const {
  std::shared_lock lock(mutex_);
  const EntityRef * ref = operation_index_.find(operation_path);
  if (ref) {
    return *ref;
  }
  return std::nullopt;
}

// ============================================================================
// Diagnostics
// ============================================================================

EntityCacheStats ThreadSafeEntityCache::get_stats() const {
  std::shared_lock lock(mutex_);
  EntityCacheStats stats;
  stats.area_count = areas_.live_count();
  stats.component_count = components_.live_count();
  stats.app_count = apps_.live_count();
  stats.function_count = functions_.live_count();
  stats.total_operations = operation_index_.size();
  stats.capacity = capacity_;
  // refresh_grew() folds each container's transient grew() flag into grew_ and
  // clears the containers, so the cached member is the source of truth here.
  // Re-querying the containers would always read false post-refresh.
  stats.grew = grew_;
  stats.generation = generation_.load(std::memory_order_acquire);
  stats.overflow_count = overflow_count_;
  stats.last_update = last_update_;
  return stats;
}

std::string ThreadSafeEntityCache::validate() const {
  std::shared_lock lock(mutex_);
  std::ostringstream errors;

  auto check_index = [&errors](const char * what, const auto & index, const auto & store) {
    index.for_each([&](const std::string & id, uint32_t slot) {
      if (slot >= store.slot_count()) {
        errors << what << " index out of bounds: " << id << " -> " << slot << "\n";
      } else if (!store.is_live(slot)) {
        errors << what << " index points to dead slot: " << id << " -> " << slot << "\n";
      } else if (store[static_cast<size_t>(slot)].id != id) {
        errors << what << " index mismatch: " << id << " -> " << store[static_cast<size_t>(slot)].id << "\n";
      }
    });
  };

  check_index("Area", area_index_, areas_);
  check_index("Component", component_index_, components_);
  check_index("App", app_index_, apps_);
  check_index("Function", function_index_, functions_);

  // Live-count consistency: each live slot must be reachable via its index.
  auto check_live_consistency = [&errors](const char * what, const auto & store, const auto & index) {
    size_t reachable = 0;
    store.for_each_live([&](uint32_t slot, const auto & entity) {
      const uint32_t * mapped = index.find(entity.id);
      if (mapped && *mapped == slot) {
        ++reachable;
      } else {
        errors << what << " live slot not reachable via index: " << entity.id << " (slot " << slot << ")\n";
      }
    });
    if (reachable != store.live_count()) {
      errors << what << " live-count inconsistency: reachable=" << reachable << " live_count=" << store.live_count()
             << "\n";
    }
  };

  check_live_consistency("Area", areas_, area_index_);
  check_live_consistency("Component", components_, component_index_);
  check_live_consistency("App", apps_, app_index_);
  check_live_consistency("Function", functions_, function_index_);

  // Check for duplicate IDs across all live entities
  std::unordered_set<std::string> seen_ids;
  areas_.for_each_live([&](uint32_t, const Area & area) {
    if (!seen_ids.insert(area.id).second) {
      errors << "Duplicate area ID: " << area.id << "\n";
    }
  });
  components_.for_each_live([&](uint32_t, const Component & comp) {
    if (!seen_ids.insert(comp.id).second) {
      errors << "Duplicate component ID (or conflicts with area): " << comp.id << "\n";
    }
  });
  apps_.for_each_live([&](uint32_t, const App & app) {
    if (!seen_ids.insert(app.id).second) {
      errors << "Duplicate app ID (or conflicts with other entity): " << app.id << "\n";
    }
  });
  functions_.for_each_live([&](uint32_t, const Function & func) {
    if (!seen_ids.insert(func.id).second) {
      errors << "Duplicate function ID (or conflicts with other entity): " << func.id << "\n";
    }
  });

  return errors.str();
}

std::chrono::system_clock::time_point ThreadSafeEntityCache::get_last_update() const {
  std::shared_lock lock(mutex_);
  return last_update_;
}

uint64_t ThreadSafeEntityCache::generation() const {
  return generation_.load(std::memory_order_acquire);
}

// ============================================================================
// Index rebuild helpers
// ============================================================================
//
// Primary id->slot indexes are maintained incrementally inside reconcile().
// Only the derived relationship and operation indexes are rebuilt from scratch
// after a change, since they are pure functions of the (now-updated) stores.

void ThreadSafeEntityCache::rebuild_relationship_indexes() {
  component_to_apps_.reset();
  component_to_subcomponents_.reset();
  area_to_components_.reset();
  area_to_subareas_.reset();
  function_to_apps_.reset();
  function_to_components_.reset();

  // Build component_to_apps from apps' component_id
  apps_.for_each_live([&](uint32_t slot, const App & app) {
    if (!app.component_id.empty()) {
      component_to_apps_.get_or_create(app.component_id).push_back(slot);
    }
  });

  // Build area_to_components from components' area
  components_.for_each_live([&](uint32_t slot, const Component & comp) {
    if (!comp.area.empty()) {
      area_to_components_.get_or_create(comp.area).push_back(slot);
    }
  });

  // Build component_to_subcomponents from components' parent_component_id.
  //
  // Nothing indexed this direction before, so a hierarchical parent Component
  // could not have its children enumerated from the cache at all: the only
  // child lookups were linear scans in the discovery layer. Member resolution
  // needs it, because a parent Component is a grouping whose resources come
  // from its children.
  components_.for_each_live([&](uint32_t slot, const Component & comp) {
    if (!comp.parent_component_id.empty()) {
      component_to_subcomponents_.get_or_create(comp.parent_component_id).push_back(slot);
    }
  });

  // Build area_to_subareas from areas' parent_area_id
  areas_.for_each_live([&](uint32_t slot, const Area & area) {
    if (!area.parent_area_id.empty()) {
      area_to_subareas_.get_or_create(area.parent_area_id).push_back(slot);
    }
  });

  // Build function_to_apps and function_to_components from Function::hosts.
  //
  // `hosts` may name either an App or a Component. Only the App half was
  // indexed, and a host id that resolved to a Component was dropped without a
  // word, so a Function hosting a Component reported none of that Component's
  // resources. Both halves are indexed now; a host id that resolves to neither
  // is still skipped, because there is nothing to point at.
  functions_.for_each_live([&](uint32_t, const Function & func) {
    for (const auto & host_id : func.hosts) {
      const uint32_t * app_slot = app_index_.find(host_id);
      if (app_slot && apps_.is_live(*app_slot)) {
        function_to_apps_.get_or_create(func.id).push_back(*app_slot);
        continue;
      }
      const uint32_t * comp_slot = component_index_.find(host_id);
      if (comp_slot && components_.is_live(*comp_slot)) {
        function_to_components_.get_or_create(func.id).push_back(*comp_slot);
      }
    }
  });
}

std::vector<std::string> ThreadSafeEntityCache::get_members(SovdEntityType type, const std::string & entity_id) const {
  std::shared_lock lock(mutex_);
  std::vector<std::string> members;
  std::unordered_set<std::string> seen;

  // One traversal for every grouping kind. Before this there were several, and
  // they disagreed: Area operations did not descend into subareas, Function
  // walks dropped Component hosts, and a parent Component had no walk at all.
  // A caller that resolves an item to a member and a caller that lists the
  // grouping's items have to agree, or the aggregator advertises what it will
  // not serve.
  //
  // `seen` is not defensive bookkeeping. Component-parent cycles are known to
  // occur - the aggregation classifier handles them explicitly - and the
  // manifest validator only checks that a parent exists, not that the graph is
  // acyclic.
  const auto add = [&](const std::string & id) {
    if (!id.empty() && seen.insert(id).second) {
      members.push_back(id);
    }
  };

  // Declared before the recursive lambdas that call each other.
  std::function<void(const std::string &)> collect_component;
  std::function<void(const std::string &)> collect_area;

  collect_component = [&](const std::string & component_id) {
    add(component_id);
    if (const std::vector<uint32_t> * app_slots = component_to_apps_.find(component_id)) {
      for (uint32_t slot : *app_slots) {
        if (apps_.is_live(slot)) {
          add(apps_[slot].id);
        }
      }
    }
    if (const std::vector<uint32_t> * child_slots = component_to_subcomponents_.find(component_id)) {
      for (uint32_t slot : *child_slots) {
        if (components_.is_live(slot) && !seen.count(components_[slot].id)) {
          collect_component(components_[slot].id);
        }
      }
    }
  };

  collect_area = [&](const std::string & area_id) {
    if (const std::vector<uint32_t> * comp_slots = area_to_components_.find(area_id)) {
      for (uint32_t slot : *comp_slots) {
        if (components_.is_live(slot)) {
          collect_component(components_[slot].id);
        }
      }
    }
    if (const std::vector<uint32_t> * sub_slots = area_to_subareas_.find(area_id)) {
      for (uint32_t slot : *sub_slots) {
        if (areas_.is_live(slot) && seen.insert("area:" + areas_[slot].id).second) {
          collect_area(areas_[slot].id);
        }
      }
    }
  };

  switch (type) {
    case SovdEntityType::AREA:
      seen.insert("area:" + entity_id);
      collect_area(entity_id);
      break;
    case SovdEntityType::COMPONENT:
      // The Component itself is a member: unlike an Area or a Function it can
      // own services and actions of its own, and the operations aggregation
      // already counts those before its Apps.
      collect_component(entity_id);
      break;
    case SovdEntityType::FUNCTION:
      if (const std::vector<uint32_t> * app_slots = function_to_apps_.find(entity_id)) {
        for (uint32_t slot : *app_slots) {
          if (apps_.is_live(slot)) {
            add(apps_[slot].id);
          }
        }
      }
      if (const std::vector<uint32_t> * comp_slots = function_to_components_.find(entity_id)) {
        for (uint32_t slot : *comp_slots) {
          if (components_.is_live(slot)) {
            collect_component(components_[slot].id);
          }
        }
      }
      break;
    case SovdEntityType::APP:
    case SovdEntityType::SERVER:
    case SovdEntityType::UNKNOWN:
      // A leaf is its own only member, so a caller that resolves an item
      // against one needs no traversal and no special case of its own. Listed
      // rather than defaulted because the build treats an unhandled enumerator
      // as an error, which is what makes a new entity type surface here.
      add(entity_id);
      break;
  }

  return members;
}

void ThreadSafeEntityCache::rebuild_operation_index() {
  operation_index_.reset();

  // Index operations from components
  components_.for_each_live([&](uint32_t slot, const Component & comp) {
    for (const auto & svc : comp.services) {
      operation_index_.insert_or_assign(svc.full_path, EntityRef{SovdEntityType::COMPONENT, slot});
    }
    for (const auto & act : comp.actions) {
      operation_index_.insert_or_assign(act.full_path, EntityRef{SovdEntityType::COMPONENT, slot});
    }
  });

  // Index operations from apps (apps take priority over components for same path)
  apps_.for_each_live([&](uint32_t slot, const App & app) {
    for (const auto & svc : app.services) {
      operation_index_.insert_or_assign(svc.full_path, EntityRef{SovdEntityType::APP, slot});
    }
    for (const auto & act : app.actions) {
      operation_index_.insert_or_assign(act.full_path, EntityRef{SovdEntityType::APP, slot});
    }
  });
}

// ============================================================================
// Aggregation helpers
// ============================================================================

void ThreadSafeEntityCache::collect_operations_from_apps(const std::vector<uint32_t> & app_slots,
                                                         std::unordered_set<std::string> & seen_paths,
                                                         AggregatedOperations & result) const {
  for (uint32_t slot : app_slots) {
    if (!apps_.is_live(slot)) {
      continue;
    }
    const auto & app = apps_[slot];
    result.source_ids.push_back(app.id);

    for (const auto & svc : app.services) {
      if (seen_paths.insert(svc.full_path).second) {
        result.services.push_back(svc);
        result.owner_by_path.emplace(svc.full_path, app.id);
      }
    }
    for (const auto & act : app.actions) {
      if (seen_paths.insert(act.full_path).second) {
        result.actions.push_back(act);
        result.owner_by_path.emplace(act.full_path, app.id);
      }
    }
  }
}

void ThreadSafeEntityCache::collect_operations_from_component(uint32_t comp_slot,
                                                              std::unordered_set<std::string> & seen_paths,
                                                              AggregatedOperations & result) const {
  if (!components_.is_live(comp_slot)) {
    return;
  }

  const auto & comp = components_[comp_slot];
  result.source_ids.push_back(comp.id);

  for (const auto & svc : comp.services) {
    if (seen_paths.insert(svc.full_path).second) {
      result.services.push_back(svc);
      result.owner_by_path.emplace(svc.full_path, comp.id);
    }
  }
  for (const auto & act : comp.actions) {
    if (seen_paths.insert(act.full_path).second) {
      result.actions.push_back(act);
      result.owner_by_path.emplace(act.full_path, comp.id);
    }
  }
}

// ============================================================================
// Data aggregation methods
// ============================================================================

AggregatedData ThreadSafeEntityCache::get_entity_data(const std::string & entity_id) const {
  // find_entity takes its own lock, so we don't need one here
  auto entity_ref = find_entity(entity_id);
  if (!entity_ref) {
    return {};
  }

  // Each get_*_data method takes its own lock
  switch (entity_ref->type) {
    case SovdEntityType::APP:
      return get_app_data(entity_id);
    case SovdEntityType::COMPONENT:
      return get_component_data(entity_id);
    case SovdEntityType::AREA:
      return get_area_data(entity_id);
    case SovdEntityType::FUNCTION:
      return get_function_data(entity_id);
    case SovdEntityType::SERVER:
    case SovdEntityType::UNKNOWN:
    default:
      return {};
  }
}

AggregatedData ThreadSafeEntityCache::get_app_data(const std::string & app_id) const {
  std::shared_lock lock(mutex_);

  const uint32_t * slot = app_index_.find(app_id);
  if (!slot || !apps_.is_live(*slot)) {
    return {};
  }

  AggregatedData result;
  result.aggregation_level = "app";
  result.is_aggregated = false;

  std::unordered_set<std::string> seen_topics;
  collect_topics_from_app(*slot, seen_topics, result);

  return result;
}

AggregatedData ThreadSafeEntityCache::get_component_data(const std::string & component_id) const {
  std::shared_lock lock(mutex_);

  const uint32_t * comp_slot = component_index_.find(component_id);
  if (!comp_slot || !components_.is_live(*comp_slot)) {
    return {};
  }

  AggregatedData result;
  result.aggregation_level = "component";

  std::unordered_set<std::string> seen_topics;

  // Collect from component itself
  collect_topics_from_component(*comp_slot, seen_topics, result);

  // Collect from hosted apps
  const std::vector<uint32_t> * app_slots = component_to_apps_.find(component_id);
  if (app_slots) {
    collect_topics_from_apps(*app_slots, seen_topics, result);
    result.is_aggregated = !app_slots->empty();
  }

  return result;
}

AggregatedData ThreadSafeEntityCache::get_area_data(const std::string & area_id) const {
  std::shared_lock lock(mutex_);

  const uint32_t * area_slot = area_index_.find(area_id);
  if (!area_slot || !areas_.is_live(*area_slot)) {
    return {};
  }

  AggregatedData result;
  result.aggregation_level = "area";
  result.is_aggregated = true;
  result.source_ids.push_back(area_id);

  std::unordered_set<std::string> seen_topics;

  // Collect from all components in this area **and its subareas** (recursive)
  std::vector<uint32_t> component_slots;
  std::unordered_set<std::string> visited_areas;
  std::vector<std::string> pending_areas;
  pending_areas.push_back(area_id);

  while (!pending_areas.empty()) {
    const std::string current_area = pending_areas.back();
    pending_areas.pop_back();

    // Skip already-visited areas to prevent infinite loops in case of cycles
    if (!visited_areas.insert(current_area).second) {
      continue;
    }

    // Collect components directly associated with this area
    const std::vector<uint32_t> * comp_slots = area_to_components_.find(current_area);
    if (comp_slots) {
      component_slots.insert(component_slots.end(), comp_slots->begin(), comp_slots->end());
    }

    // Traverse into subareas
    const std::vector<uint32_t> * subarea_slots = area_to_subareas_.find(current_area);
    if (subarea_slots) {
      for (uint32_t subarea_slot : *subarea_slots) {
        if (areas_.is_live(subarea_slot)) {
          pending_areas.push_back(areas_[subarea_slot].id);
        }
      }
    }
  }

  for (uint32_t comp_slot : component_slots) {
    collect_topics_from_component(comp_slot, seen_topics, result);

    // Also collect from apps hosted on each component
    if (components_.is_live(comp_slot)) {
      const std::vector<uint32_t> * app_slots = component_to_apps_.find(components_[comp_slot].id);
      if (app_slots) {
        collect_topics_from_apps(*app_slots, seen_topics, result);
      }
    }
  }

  return result;
}

AggregatedData ThreadSafeEntityCache::get_function_data(const std::string & function_id) const {
  std::shared_lock lock(mutex_);

  const uint32_t * func_slot = function_index_.find(function_id);
  if (!func_slot || !functions_.is_live(*func_slot)) {
    return {};
  }

  AggregatedData result;
  result.aggregation_level = "function";
  result.is_aggregated = true;
  result.source_ids.push_back(function_id);

  std::unordered_set<std::string> seen_topics;

  // Collect from all apps implementing this function
  const std::vector<uint32_t> * app_slots = function_to_apps_.find(function_id);
  if (app_slots) {
    collect_topics_from_apps(*app_slots, seen_topics, result);
  }

  return result;
}

// ============================================================================
// Data aggregation helpers
// ============================================================================

void ThreadSafeEntityCache::collect_topics_from_app(uint32_t app_slot, std::unordered_set<std::string> & seen_topics,
                                                    AggregatedData & result) const {
  if (!apps_.is_live(app_slot)) {
    return;
  }

  const auto & app = apps_[app_slot];
  result.source_ids.push_back(app.id);

  // Record this app against every topic it touches, whichever side it is on.
  // A topic seen from both sides stays one item, so its owner list is what
  // tells a caller that addressing it names more than one member.
  const auto attribute = [&result, &app](const std::string & topic) {
    auto & owners = result.owners_by_topic[topic];
    if (std::find(owners.begin(), owners.end(), app.id) == owners.end()) {
      owners.push_back(app.id);
    }
  };

  // Publishers
  for (const auto & topic : app.topics.publishes) {
    attribute(topic);
    auto [_, inserted] = seen_topics.insert(topic);
    if (inserted) {
      result.topics.push_back({topic, "", "publish"});
    } else {
      // Topic already seen as subscriber - update direction to "both"
      for (auto & t : result.topics) {
        if (t.name == topic && t.direction == "subscribe") {
          t.direction = "both";
          break;
        }
      }
    }
  }

  // Subscribers
  for (const auto & topic : app.topics.subscribes) {
    attribute(topic);
    auto [_, inserted] = seen_topics.insert(topic);
    if (inserted) {
      result.topics.push_back({topic, "", "subscribe"});
    } else {
      // Topic already seen - might be both pub and sub, update direction
      for (auto & t : result.topics) {
        if (t.name == topic && t.direction == "publish") {
          t.direction = "both";
          break;
        }
      }
    }
  }
}

void ThreadSafeEntityCache::collect_topics_from_apps(const std::vector<uint32_t> & app_slots,
                                                     std::unordered_set<std::string> & seen_topics,
                                                     AggregatedData & result) const {
  for (uint32_t slot : app_slots) {
    collect_topics_from_app(slot, seen_topics, result);
  }
}

void ThreadSafeEntityCache::collect_topics_from_component(uint32_t comp_slot,
                                                          std::unordered_set<std::string> & seen_topics,
                                                          AggregatedData & result) const {
  if (!components_.is_live(comp_slot)) {
    return;
  }

  const auto & comp = components_[comp_slot];
  result.source_ids.push_back(comp.id);

  // Publishers
  for (const auto & topic : comp.topics.publishes) {
    auto [_, inserted] = seen_topics.insert(topic);
    if (inserted) {
      result.topics.push_back({topic, "", "publish"});
    } else {
      // Topic already seen as subscriber - update direction to "both"
      for (auto & t : result.topics) {
        if (t.name == topic && t.direction == "subscribe") {
          t.direction = "both";
          break;
        }
      }
    }
  }

  // Subscribers
  for (const auto & topic : comp.topics.subscribes) {
    auto [_, inserted] = seen_topics.insert(topic);
    if (inserted) {
      result.topics.push_back({topic, "", "subscribe"});
    } else {
      // Topic already seen - might be both pub and sub, update direction
      for (auto & t : result.topics) {
        if (t.name == topic && t.direction == "publish") {
          t.direction = "both";
          break;
        }
      }
    }
  }
}

}  // namespace ros2_medkit_gateway
