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

#include <nlohmann/json.hpp>
#include <optional>
#include <shared_mutex>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "path_resolver.hpp"
#include "route_registry.hpp"
#include "schema_builder.hpp"

#include "ros2_medkit_gateway/core/models/entity_types.hpp"

namespace ros2_medkit_gateway {

class GatewayNode;
class PluginManager;

namespace handlers {
class HandlerContext;
}  // namespace handlers

namespace openapi {

/// Entry-count bound on `CapabilityGenerator`'s document cache.
///
/// Kept alongside the byte bound rather than replaced by it: the byte budget
/// accounts for the key and the document text, which is what the cache spends
/// nearly all of its memory on, but not for the per-entry hash node the map
/// allocates. This bounds that.
inline constexpr size_t kDocsCacheMaxEntries = 256;

/// Byte bound on `CapabilityGenerator`'s document cache, counting each entry's
/// key plus its serialized document.
///
/// The bound that matters: an entry's size is a function of how large the
/// ROS 2 graph is, so a count alone leaves the cache unbounded in bytes.
inline constexpr size_t kDocsCacheMaxBytes = 16UL * 1024 * 1024;

/// The two bounds above, overridable at construction.
///
/// Overridable so that eviction is reachable from a test without generating
/// megabytes of documents to reach the shipped budget. Production
/// construction takes the defaults.
struct DocsCacheBounds {
  size_t max_entries{kDocsCacheMaxEntries};
  size_t max_bytes{kDocsCacheMaxBytes};
};

/// Main engine that generates context-aware OpenAPI specs for any valid
/// gateway path. Uses PathResolver to classify the path, then dispatches
/// to the appropriate combination of SchemaBuilder + PathBuilder +
/// OpenApiSpecBuilder to produce the full document.
class CapabilityGenerator {
 public:
  CapabilityGenerator(handlers::HandlerContext & ctx, GatewayNode & node, PluginManager * plugin_mgr,
                      const RouteRegistry * route_registry = nullptr, DocsCacheBounds bounds = DocsCacheBounds{});

  /// Number of documents currently cached.
  size_t cache_entry_count() const;

  /// Bytes the cache currently accounts for: every entry's key plus its
  /// serialized document.
  size_t cache_byte_size() const;

  /// The OpenAPI document for the given base path (without the /docs suffix),
  /// serialized exactly as `http::detail::write_json_body` would serialize it
  /// (`dump(2)`). Returns nullopt if the path is not valid or resolvable.
  ///
  /// This is the form the cache holds and the form the `/docs` routes serve,
  /// so a served document is never a copy of a parsed DOM.
  std::optional<std::string> generate_serialized(const std::string & base_path) const;

  /// The same document parsed.
  ///
  /// A convenience over `generate_serialized` for callers that want to inspect
  /// the structure - today the unit tests. Serving code should call
  /// `generate_serialized`: this parses on every call, because the parsed form
  /// is not what is stored. Returns nullopt if the path is not valid or
  /// resolvable.
  std::optional<nlohmann::json> generate(const std::string & base_path) const;

 private:
  /// One `{param}` a path template carries and the literal id the requested
  /// path bound it to.
  using PathBinding = std::pair<std::string, std::string>;

  nlohmann::json generate_root() const;
  nlohmann::json generate_entity_collection(const ResolvedPath & resolved) const;
  nlohmann::json generate_specific_entity(const ResolvedPath & resolved) const;
  nlohmann::json generate_resource_collection(const ResolvedPath & resolved) const;
  nlohmann::json generate_specific_resource(const ResolvedPath & resolved) const;

  /// Validate entity exists and parent-child relationships hold.
  bool validate_entity_hierarchy(const ResolvedPath & resolved) const;

  /// Build the server URL from node parameters.
  std::string build_server_url() const;

  /// Map path resolver entity type to SovdEntityType for cache lookups.
  static ros2_medkit_gateway::SovdEntityType entity_type_from_keyword(const std::string & keyword);

  /// Every path the running gateway serves, registry routes and plugin-mounted
  /// routes alike, in the shape the root document publishes them.
  ///
  /// Where a plugin describes a path the registry already holds, the
  /// registry's description wins - the same precedence `generate_root` applies,
  /// which is also where the shadowing is warned about, so this stays quiet.
  nlohmann::json served_paths() const;

  /// The slice of `served` at or beneath `template_prefix`, with each binding's
  /// `{param}` replaced by the literal id in the path key and the parameter
  /// that described it removed from every operation.
  static nlohmann::json project(const nlohmann::json & served, const std::string & template_prefix,
                                const std::vector<PathBinding> & bindings);

  /// Wrap a projected `paths` object in the sub-document envelope: info block,
  /// server, the bearer scheme its operations may name, and the component
  /// schemas its `$ref`s reach.
  nlohmann::json build_subtree_document(const std::string & title, const nlohmann::json & paths) const;

  /// The entity path template for a resolved path's parent chain, plus the
  /// bindings that turn it back into the concrete path the caller asked about.
  /// The resolved path's own entity is included only when `include_self` -
  /// an entity *collection* names a type with no id of its own.
  static std::pair<std::string, std::vector<PathBinding>> entity_template(const ResolvedPath & resolved,
                                                                          bool include_self);

  /// `template_prefix` with every binding substituted - i.e. the concrete path
  /// the caller asked about, rebuilt from the same two pieces the projection
  /// uses so the two cannot disagree about where an entity lives.
  static std::string concrete_path(const std::string & template_prefix, const std::vector<PathBinding> & bindings);

  /// The parameter name of the single-segment `{param}` route directly under
  /// `prefix`, if `served` holds one.
  ///
  /// Read from the served paths rather than tabulated per collection: the item
  /// route is the one key that extends the collection by exactly one segment
  /// and whose segment is a whole `{param}`. A hand-written map from
  /// collection to parameter name is the kind of second source this projection
  /// exists to delete. Returns nullopt when the collection has no item route
  /// (`/logs` has only the literal `/logs/configuration`) - and also when more
  /// than one candidate exists, because that is a routing shape this reading
  /// does not model and guessing would bind an id to the wrong parameter.
  static std::optional<std::string> single_parameter_segment_under(const nlohmann::json & served,
                                                                   const std::string & prefix);

  /// Add the concrete data / operation item paths under `entity_path`.
  ///
  /// The one part of a sub-document the route registry cannot supply: the
  /// registry holds `/apps/{app_id}/data/{data_id}` with a payload schema that
  /// has to cover every topic, while the schema of a *particular* topic comes
  /// from the ROS type in the entity cache.
  ///
  /// `resolved.resource_id` empty means "every item in the collection", each
  /// at its own concrete key alongside the projected template. Non-empty
  /// narrows it to that one item, whose key the projection also produced -
  /// there the cache-derived item replaces it, because the ROS schema is the
  /// reason a caller asked about a single data point.
  ///
  /// Adds nothing for a resource the cache does not hold, deliberately: the
  /// projected item route already describes what the gateway will do with the
  /// request, and a hand-built item with an empty schema would say less.
  void add_cache_derived_items(nlohmann::json & paths, const ResolvedPath & resolved,
                               const std::string & entity_path) const;

  /// Every path item the loaded plugins describe (via the optional
  /// `describe_plugin_routes` dlsym export), normalised into the shape the
  /// rest of the document uses. Empty when no plugin exports the symbol.
  nlohmann::json plugin_paths() const;

  /// Generate OpenAPI docs for plugin-registered routes (via dlsym).
  nlohmann::json generate_plugin_docs(const std::string & path) const;

  /// Core generation logic (called on cache miss). Dispatches to the producer
  /// for the resolved path category via `build_document`, then applies the
  /// document-wide rules that hold whatever produced an operation.
  std::optional<nlohmann::json> generate_impl(const std::string & base_path) const;

  /// Dispatch to the producer for the resolved path category. Everything it
  /// returns still goes through `generate_impl`'s document-wide pass.
  std::optional<nlohmann::json> build_document(const std::string & base_path) const;

  /// Rewrite every per-operation `security` requirement so it states what the
  /// running gateway's auth middleware does to that operation, whichever
  /// producer wrote it. See the implementation for the two configurations.
  void project_security_onto_enforcement(nlohmann::json & document) const;

  /// Give a cache-derived item everything the projected route it sits beside
  /// already says - the declared role, every non-2xx status, the lock marker
  /// and the non-path parameters - leaving only the ROS payload locally built.
  ///
  /// `sibling_key` is that route's key in `paths`, which differs by scope:
  /// templated at collection scope, already concrete at specific-resource
  /// scope, because `project()` substitutes bindings into path keys.
  ///
  /// @return false when no such operation is there, in which case `item` is
  ///   unusable and the caller must not write it over the projection.
  [[nodiscard]] static bool adopt_projected_framework(nlohmann::json & item, const nlohmann::json & paths,
                                                      const std::string & sibling_key);

  /// Build a cache key for the given path, invalidating the cache if the
  /// entity cache generation has changed.
  std::string get_cache_key(const std::string & path) const;

  /// Look up a previously cached document by key.
  std::optional<std::string> lookup_cache(const std::string & key) const;

  /// Store a serialized document in the cache. A document that would not fit
  /// the byte budget on its own is not stored.
  void store_cache(const std::string & key, const std::string & document) const;

  /// Drop every entry and reset the byte total. Caller holds `cache_mutex_`
  /// exclusively.
  void clear_cache_locked() const;

  handlers::HandlerContext & ctx_;
  GatewayNode & node_;
  PluginManager * plugin_mgr_;
  const RouteRegistry * route_registry_;
  SchemaBuilder schema_builder_;
  DocsCacheBounds bounds_;

  // Generation-based document cache - invalidated when entity cache changes.
  //
  // Values are serialized documents, not parsed DOMs. A DOM of one of these
  // documents costs several times its serialized size in resident memory (a
  // node per value, each separately allocated), and every cache hit would
  // have to deep-copy it. Text costs its own length and a hit hands the
  // handler bytes it can write straight out.
  mutable std::shared_mutex cache_mutex_;
  mutable std::unordered_map<std::string, std::string> spec_cache_;
  mutable size_t cache_bytes_{0};
  mutable uint64_t cached_generation_{0};
};

}  // namespace openapi
}  // namespace ros2_medkit_gateway
