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

#include "ros2_medkit_gateway/core/discovery/models/app.hpp"
#include "ros2_medkit_gateway/core/discovery/models/area.hpp"
#include "ros2_medkit_gateway/core/discovery/models/component.hpp"
#include "ros2_medkit_gateway/core/discovery/models/function.hpp"
#include "ros2_medkit_gateway/core/models/entity_types.hpp"
#include "ros2_medkit_gateway/core/util/flat_hash_map.hpp"
#include "ros2_medkit_gateway/core/util/slot_store.hpp"

#include <atomic>
#include <chrono>
#include <cstdint>
#include <optional>
#include <shared_mutex>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace ros2_medkit_gateway {

/**
 * @brief Reference to an entity in the cache
 *
 * `index` is a stable SlotStore slot id (not a compacted vector position).
 */
struct EntityRef {
  SovdEntityType type{SovdEntityType::UNKNOWN};
  size_t index{0};
};

/**
 * @brief Topic information with direction
 */
struct TopicData {
  std::string name;       ///< Full topic path (e.g., "/sensor/temperature")
  std::string type;       ///< Message type (e.g., "std_msgs/msg/Float32")
  std::string direction;  ///< "publish" | "subscribe" | "both"
};

/**
 * @brief Aggregated data (topics) result from entity hierarchy
 */
struct AggregatedData {
  std::vector<TopicData> topics;
  std::vector<std::string> source_ids;  ///< Entity IDs that contributed
  /// Contributing members per topic name, for the items in `topics`.
  ///
  /// `source_ids` says which members contributed something; it does not say
  /// which member contributed WHICH item, and a caller holding one item id
  /// cannot recover the owner from it. A grouping that lists an item it cannot
  /// then address is the defect this exists to close.
  ///
  /// A list rather than one id, because a topic legitimately has more than one
  /// contributor: a topic published by one member and subscribed by another is
  /// merged into a single item with direction "both", so recording only the
  /// first would name a publisher for an item a subscriber also owns.
  std::unordered_map<std::string, std::vector<std::string>> owners_by_topic;
  std::string aggregation_level;  ///< "app" | "component" | "area" | "function"
  bool is_aggregated{false};      ///< true if collected from sub-entities

  bool empty() const {
    return topics.empty();
  }
  size_t total_count() const {
    return topics.size();
  }
};

/**
 * @brief Aggregated operations result from entity hierarchy
 */
struct AggregatedOperations {
  std::vector<ServiceInfo> services;
  std::vector<ActionInfo> actions;
  std::vector<std::string> source_ids;  ///< Entity IDs that contributed
  /// Owning member per full ROS path, for the items in `services`/`actions`.
  ///
  /// See AggregatedData::owner_by_topic for why the collection-level
  /// `source_ids` is not enough. Keyed by full path, matching the existing
  /// deduplication: two members exposing the same full path are one item with
  /// one owner, while two members exposing the same SHORT name at different
  /// paths stay two items with an owner each - which is what lets a caller see
  /// that the short name, the id the HTTP API uses, is ambiguous.
  std::unordered_map<std::string, std::string> owner_by_path;
  std::string aggregation_level;  ///< "app" | "component" | "area" | "function"
  bool is_aggregated{false};      ///< true if collected from sub-entities

  bool empty() const {
    return services.empty() && actions.empty();
  }
  size_t total_count() const {
    return services.size() + actions.size();
  }
};

/**
 * @brief Node info for configuration aggregation
 */
struct NodeConfigInfo {
  std::string node_fqn;   ///< Fully qualified node name
  std::string app_id;     ///< Source app ID
  std::string entity_id;  ///< Owning entity ID (app/component/area)
};

/**
 * @brief Aggregated configurations (node FQNs) result from entity hierarchy
 *
 * Unlike data/operations which store actual values, configurations stores
 * the list of ROS 2 nodes whose parameters should be queried. This is because
 * parameters are owned by nodes, and aggregated entities need to iterate
 * over all child nodes.
 */
struct AggregatedConfigurations {
  std::vector<NodeConfigInfo> nodes;    ///< Nodes to query for parameters
  std::vector<std::string> source_ids;  ///< Entity IDs that contributed
  std::string aggregation_level;        ///< "app" | "component" | "area" | "function"
  bool is_aggregated{false};            ///< true if collected from sub-entities

  bool empty() const {
    return nodes.empty();
  }
  size_t node_count() const {
    return nodes.size();
  }
};

/**
 * @brief Cache statistics
 */
struct EntityCacheStats {
  size_t area_count{0};       ///< Live areas
  size_t component_count{0};  ///< Live components
  size_t app_count{0};        ///< Live apps
  size_t function_count{0};   ///< Live functions
  size_t total_operations{0};
  size_t capacity{0};        ///< Configured reserve capacity (0 = lazy growth)
  bool grew{false};          ///< true if any backing store/index reallocated since reserve()
  uint64_t generation{0};    ///< Current cache generation counter
  size_t overflow_count{0};  ///< Number of reconcile cycles that triggered a structural grow
  std::chrono::system_clock::time_point last_update;
};

/**
 * @brief Thread-safe, index-optimized cache for SOVD entities
 *
 * Design principles:
 * 1. Primary storage in vectors (cache-friendly, predictable memory)
 * 2. Hash indexes for O(1) lookup by ID
 * 3. Relationship indexes for O(1) aggregation queries
 * 4. Reader-writer lock for concurrent access
 * 5. Batch updates to minimize lock contention
 *
 * Thread Safety:
 * - Multiple readers can access concurrently (shared lock)
 * - Writers get exclusive access (unique lock)
 * - Readers never block each other
 * - Writers block all readers and other writers
 *
 * Usage:
 * - Discovery thread calls update_*() methods (writer)
 * - HTTP handlers call get_*() methods (reader)
 */
class ThreadSafeEntityCache {
 public:
  ThreadSafeEntityCache() = default;

  /// Construct with all stores, indexes and scratch buffers pre-reserved for
  /// `capacity` entities. Steady-state updates below this size cause no
  /// structural reallocation. `capacity == 0` is equivalent to the default
  /// constructor (lazy growth from empty).
  explicit ThreadSafeEntityCache(size_t capacity);

  /// Reserve every backing store, every index/relationship map, and the
  /// reconcile scratch buffers for `capacity` entities. Idempotent; safe to
  /// call on a populated cache (only grows the backing arrays).
  void reserve(size_t capacity);

  // =========================================================================
  // Writer methods (exclusive lock) - called by discovery thread
  // =========================================================================

  /**
   * @brief Atomic batch update of all entities
   *
   * This is the preferred update method as it:
   * 1. Minimizes lock hold time
   * 2. Ensures readers never see partial state
   * 3. Rebuilds all indexes in one pass
   *
   * @param areas All discovered areas
   * @param components All discovered components
   * @param apps All discovered apps
   * @param functions All discovered functions
   * @param node_to_app Node FQN to app entity ID mapping from linking result (optional)
   */
  void update_all(std::vector<Area> areas, std::vector<Component> components, std::vector<App> apps,
                  std::vector<Function> functions, std::unordered_map<std::string, std::string> node_to_app = {});

  /**
   * @brief Incremental update for single entity type
   *
   * Use sparingly - prefer update_all() for full refresh.
   * Each call acquires exclusive lock and rebuilds relevant indexes.
   */
  void update_areas(std::vector<Area> areas);
  void update_components(std::vector<Component> components);
  void update_apps(std::vector<App> apps);
  void update_functions(std::vector<Function> functions);

  /**
   * @brief Update cached topic type mapping
   *
   * Called during cache refresh to cache topic name -> message type mapping.
   * This avoids expensive ROS graph queries on every /data request.
   *
   * @param topic_types Map of topic name to message type
   */
  void update_topic_types(std::unordered_map<std::string, std::string> topic_types);

  // =========================================================================
  // Reader methods (shared lock) - called by HTTP handlers
  // =========================================================================

  // --- List all entities (returns copy) ---
  std::vector<Area> get_areas() const;
  std::vector<Component> get_components() const;
  std::vector<App> get_apps() const;
  std::vector<Function> get_functions() const;

  // --- Get single entity by ID (O(1) lookup) ---
  std::optional<Area> get_area(const std::string & id) const;
  std::optional<Component> get_component(const std::string & id) const;
  std::optional<App> get_app(const std::string & id) const;
  std::optional<Function> get_function(const std::string & id) const;

  /// Atomic snapshot of an App together with its parent Component and Area.
  ///
  /// Three sequential get_app/get_component/get_area calls each acquire a
  /// fresh shared_lock and a writer refresh can advance the generation
  /// between them, so handlers that traverse App -> Component -> Area can
  /// observe a mixed-generation view (e.g. app from gen N, component from
  /// N+1, area from N). This helper resolves the chain under a single
  /// shared_lock so the result is internally consistent.
  ///
  /// `app` is empty if `app_id` is not in the cache. `component` is empty if
  /// the app has no `component_id` or the referenced component is missing.
  /// `area` is empty if the component has no `area` or the referenced area
  /// is missing. The two latter cases are distinguishable from "no parent"
  /// because `app.component_id` / `component.area` remain set on the
  /// returned models.
  struct AppLinksSnapshot {
    std::optional<App> app;
    std::optional<Component> component;
    std::optional<Area> area;
  };
  AppLinksSnapshot get_app_with_links(const std::string & id) const;

  /// Atomic snapshot of an App together with its declared `depends_on` apps.
  ///
  /// `handle_app_depends_on` iterates `app.depends_on` and resolves each id
  /// via `get_app()` - one shared_lock per dependency. A writer refresh can
  /// land between the app fetch and any of the per-dependency fetches, so a
  /// 5-dependency app can return 5 apps from 5 different cache generations.
  /// This helper takes a single shared_lock and returns the app together
  /// with every dependency resolved in the same generation.
  ///
  /// `app` is empty if `app_id` is not in the cache. `dependencies` lists
  /// every entry in `app->depends_on` in declaration order; the optional is
  /// empty when the referenced dependency cannot be resolved (broken ref).
  struct AppDependenciesSnapshot {
    std::optional<App> app;
    std::vector<std::pair<std::string, std::optional<App>>> dependencies;
  };
  AppDependenciesSnapshot get_app_with_dependencies(const std::string & id) const;

  // --- Check existence (O(1)) ---
  bool has_area(const std::string & id) const;
  bool has_component(const std::string & id) const;
  bool has_app(const std::string & id) const;
  bool has_function(const std::string & id) const;

  // --- Topic type lookup (O(1)) ---
  /**
   * @brief Get cached message type for a topic
   * @param topic_name Full topic path (e.g., "/sensor/temperature")
   * @return Message type string, or empty if not cached
   */
  std::string get_topic_type(const std::string & topic_name) const;

  // --- Node-to-app mapping (for trigger entity resolution) ---
  /**
   * @brief Get the node FQN to app entity ID mapping
   *
   * Used by trigger subscribers to resolve ROS 2 node FQNs to manifest
   * entity IDs. The mapping is populated from the linking result during
   * cache refresh.
   *
   * @return Map of node FQN -> app entity ID (empty if no linking available)
   */
  std::unordered_map<std::string, std::string> get_node_to_app() const;

  /// Look up a single node FQN to app entity ID mapping (shared lock, no map copy)
  std::string resolve_node_to_app(const std::string & node_fqn) const;

  // --- Resolve any entity by ID ---
  std::optional<EntityRef> find_entity(const std::string & id) const;
  SovdEntityType get_entity_type(const std::string & id) const;

  // =========================================================================
  // Relationship queries (O(1) via indexes)
  // =========================================================================

  /**
   * @brief Get Apps hosted on a Component
   * @return Vector of App IDs (empty if component not found)
   */
  std::vector<std::string> get_apps_for_component(const std::string & component_id) const;

  /**
   * @brief Get Components in an Area
   * @return Vector of Component IDs (empty if area not found)
   */
  std::vector<std::string> get_components_for_area(const std::string & area_id) const;

  /**
   * @brief Get Apps implementing a Function
   * @return Vector of App IDs (empty if function not found)
   */
  std::vector<std::string> get_apps_for_function(const std::string & function_id) const;

  /**
   * @brief Get subareas of an Area
   * @return Vector of Area IDs (empty if area not found or no subareas)
   */
  std::vector<std::string> get_subareas(const std::string & area_id) const;

  /// Entity ids that contribute the resources of `entity_id`.
  ///
  /// A grouping entity - an Area, a Function, or a Component with children -
  /// has no resources of its own beyond what its members provide, so resolving
  /// an item to its owner and listing the grouping's items must walk the same
  /// graph. This is that walk, and it is the only one: the per-collection walks
  /// that predate it disagree about subareas, about Function hosts that are
  /// Components, and about whether a parent Component has children at all.
  ///
  /// A Component is included in its own member list because, unlike an Area or
  /// a Function, it can expose services and actions directly.
  ///
  /// An App is a leaf and returns just itself, so a caller does not have to
  /// special-case the non-grouping kinds.
  ///
  /// Cycles in the parent graph terminate: parents are only checked for
  /// existence when a manifest is validated, not for acyclicity.
  std::vector<std::string> get_members(SovdEntityType type, const std::string & entity_id) const;

  // =========================================================================
  // Aggregation methods (uses relationship indexes)
  // =========================================================================

  /**
   * @brief Aggregate operations for an App (no aggregation, returns own ops)
   */
  AggregatedOperations get_app_operations(const std::string & app_id) const;

  /**
   * @brief Aggregate operations for a Component
   *
   * Returns: Component's own operations + all operations from hosted Apps.
   * Deduplicates by operation full_path.
   */
  AggregatedOperations get_component_operations(const std::string & component_id) const;

  /**
   * @brief Aggregate operations for an Area (x-medkit extension)
   *
   * Returns: All operations from all Components in the Area.
   * Recursive through Component→App hierarchy.
   *
   * Note: This is a ros2_medkit extension. SOVD spec doesn't allow
   * Area to have resource collections.
   */
  AggregatedOperations get_area_operations(const std::string & area_id) const;

  /**
   * @brief Aggregate operations for a Function
   *
   * Returns: All operations from all Apps implementing this Function.
   */
  AggregatedOperations get_function_operations(const std::string & function_id) const;

  // =========================================================================
  // Data aggregation methods (uses relationship indexes)
  // =========================================================================

  /**
   * @brief Aggregate data (topics) for any entity by ID
   *
   * Unified method that works for all entity types.
   * Aggregates topics from the entity and its children.
   *
   * @param entity_id Entity ID to get data for
   * @return AggregatedData with topics, or empty if entity not found
   */
  AggregatedData get_entity_data(const std::string & entity_id) const;

  /**
   * @brief Aggregate data (topics) for an App
   */
  AggregatedData get_app_data(const std::string & app_id) const;

  /**
   * @brief Aggregate data (topics) for a Component
   *
   * Returns: Component's own topics + all topics from hosted Apps.
   */
  AggregatedData get_component_data(const std::string & component_id) const;

  /**
   * @brief Aggregate data (topics) for an Area
   *
   * Returns: All topics from all Components in the Area.
   */
  AggregatedData get_area_data(const std::string & area_id) const;

  /**
   * @brief Aggregate data (topics) for a Function
   *
   * Returns: All topics from all Apps implementing this Function.
   */
  AggregatedData get_function_data(const std::string & function_id) const;

  // =========================================================================
  // Configuration aggregation methods (collects node FQNs for parameter access)
  // =========================================================================

  /**
   * @brief Aggregate configurations (node FQNs) for any entity by ID
   *
   * Unified method that works for all entity types.
   * For Apps, returns the single bound node.
   * For Components/Areas/Functions, aggregates all child app nodes.
   *
   * @param entity_id Entity ID to get configurations for
   * @return AggregatedConfigurations with node FQNs, or empty if entity not found
   */
  AggregatedConfigurations get_entity_configurations(const std::string & entity_id) const;

  /**
   * @brief Get configurations for an App (returns its single bound node)
   */
  AggregatedConfigurations get_app_configurations(const std::string & app_id) const;

  /**
   * @brief Aggregate configurations for a Component
   *
   * Returns: All node FQNs from hosted Apps.
   */
  AggregatedConfigurations get_component_configurations(const std::string & component_id) const;

  /**
   * @brief Aggregate configurations for an Area
   *
   * Returns: All node FQNs from all Components in the Area.
   */
  AggregatedConfigurations get_area_configurations(const std::string & area_id) const;

  /**
   * @brief Aggregate configurations for a Function
   *
   * Returns: All node FQNs from all Apps implementing this Function.
   */
  AggregatedConfigurations get_function_configurations(const std::string & function_id) const;

  // =========================================================================
  // Operation lookup (O(1) via operation index)
  // =========================================================================

  /**
   * @brief Find which entity owns an operation by full path
   * @param operation_path e.g., "/nav2/navigate_to_pose"
   * @return EntityRef if found
   */
  std::optional<EntityRef> find_operation_owner(const std::string & operation_path) const;

  // =========================================================================
  // Diagnostics
  // =========================================================================

  /**
   * @brief Get cache statistics
   */
  EntityCacheStats get_stats() const;

  /**
   * @brief Validate internal consistency
   *
   * Checks:
   * 1. All indexes point to valid vector entries
   * 2. Relationship indexes are consistent
   * 3. No duplicate IDs within entity type
   *
   * @return Empty string if valid, error message otherwise
   */
  std::string validate() const;

  /**
   * @brief Get last update timestamp
   */
  std::chrono::system_clock::time_point get_last_update() const;

  /**
   * @brief Get the current generation counter
   *
   * The generation counter advances ONLY when a mutation actually changes the
   * cache contents. This gate applies to every mutator (update_all, the
   * per-type update_* methods, and update_topic_types): a no-op call (e.g.,
   * update_all with the same entity set, or update_topic_types with an
   * unchanged topic map) does NOT bump the generation.
   *
   * Consumers use the generation to detect when cached derived data
   * (e.g., OpenAPI specs) is stale and needs regeneration. Because the counter
   * is gated on actual changes, polling it is cheap on a stable graph.
   */
  uint64_t generation() const;

 private:
  mutable std::shared_mutex mutex_;

  // Generation counter - advances only when a mutation changes the cache.
  std::atomic<uint64_t> generation_{0};

  // Primary storage: stable-slot object pools (slots never shift on free).
  SlotStore<Area> areas_;
  SlotStore<Component> components_;
  SlotStore<App> apps_;
  SlotStore<Function> functions_;

  // Timestamp
  std::chrono::system_clock::time_point last_update_;

  // Primary indexes (ID -> stable slot id)
  FlatHashMap<std::string, uint32_t> area_index_;
  FlatHashMap<std::string, uint32_t> component_index_;
  FlatHashMap<std::string, uint32_t> app_index_;
  FlatHashMap<std::string, uint32_t> function_index_;

  // Relationship indexes (parent ID -> child slot ids)
  FlatHashMap<std::string, std::vector<uint32_t>> component_to_apps_;
  FlatHashMap<std::string, std::vector<uint32_t>> component_to_subcomponents_;
  FlatHashMap<std::string, std::vector<uint32_t>> area_to_components_;
  FlatHashMap<std::string, std::vector<uint32_t>> area_to_subareas_;
  FlatHashMap<std::string, std::vector<uint32_t>> function_to_apps_;
  FlatHashMap<std::string, std::vector<uint32_t>> function_to_components_;

  // Operation index (operation full_path -> owning entity)
  FlatHashMap<std::string, EntityRef> operation_index_;

  // Topic type cache (topic name -> message type) - refreshed periodically
  FlatHashMap<std::string, std::string> topic_type_cache_;

  // Node FQN -> app entity ID mapping from linking result (for trigger entity resolution)
  FlatHashMap<std::string, std::string> node_to_app_;

  // Reconcile scratch buffers (reserved alongside the stores). seen_ marks which
  // live slots were re-seen during a reconcile pass; dead_ids_ collects ids of
  // slots to free (collected before mutating to avoid invalidating iteration);
  // patch_dead_ collects map keys to erase. Pre-reserved so steady-state
  // reconcile does not allocate; seen_ only ever resize()s on the grow path.
  std::vector<uint8_t> seen_;
  std::vector<std::string> dead_ids_;
  std::vector<std::string> patch_dead_;

  // Configured reserve capacity (0 = lazy growth from empty).
  size_t capacity_{0};
  // Sticky "any backing store/index reallocated since reserve()" flag, folded
  // from the container grew() flags by refresh_grew() (which clears them).
  bool grew_{false};
  // Count of reconcile cycles that triggered a fresh structural grow.
  size_t overflow_count_{0};

  // Internal helpers (called under lock). Primary id->slot indexes are kept up
  // to date incrementally by reconcile(); only the derived relationship and
  // operation indexes are rebuilt wholesale (cheap, slot-based) after a change.
  void rebuild_relationship_indexes();
  void rebuild_operation_index();

  /// In-place incremental reconcile of one store against `incoming` (move-from).
  /// Reuses `store`/`index`: only allocates/frees slots that actually change.
  /// Returns true if any slot was assigned, allocated, or freed (i.e. the live
  /// set or any payload changed). Duplicate ids within `incoming` collapse to
  /// last-wins in a single slot. Uses the member scratch buffers `seen_` /
  /// `dead_ids_` so there is no per-call heap allocation in steady state.
  template <typename T>
  bool reconcile(SlotStore<T> & store, FlatHashMap<std::string, uint32_t> & index, std::vector<T> && incoming);

  /// In-place incremental patch of a string-valued map against `incoming`
  /// (move-from). Erases keys absent from `incoming`, updates changed values,
  /// inserts new keys, leaves unchanged keys untouched. Returns true iff the
  /// map content changed. Uses the member scratch buffer `patch_dead_`.
  bool patch_map(FlatHashMap<std::string, std::string> & map, std::unordered_map<std::string, std::string> && incoming);

  /// Fold each backing container's transient `grew()` flag into the cached
  /// `grew_` member and clear the container flags. Increments `overflow_count_`
  /// once whenever a new grow happened since the last refresh. Because this
  /// clears the container flags, `get_stats()` must report the cached members.
  void refresh_grew();

  // Aggregation helpers (called under shared lock); slot ids guarded with is_live.
  void collect_operations_from_apps(const std::vector<uint32_t> & app_slots,
                                    std::unordered_set<std::string> & seen_paths, AggregatedOperations & result) const;
  void collect_operations_from_component(uint32_t comp_slot, std::unordered_set<std::string> & seen_paths,
                                         AggregatedOperations & result) const;

  // Data aggregation helpers (called under shared lock); slot ids guarded with is_live.
  void collect_topics_from_app(uint32_t app_slot, std::unordered_set<std::string> & seen_topics,
                               AggregatedData & result) const;
  void collect_topics_from_apps(const std::vector<uint32_t> & app_slots, std::unordered_set<std::string> & seen_topics,
                                AggregatedData & result) const;
  void collect_topics_from_component(uint32_t comp_slot, std::unordered_set<std::string> & seen_topics,
                                     AggregatedData & result) const;
};

}  // namespace ros2_medkit_gateway
