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
#include <functional>
#include <memory>
#include <optional>
#include <shared_mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include <nlohmann/json.hpp>
#include <tl/expected.hpp>

namespace ros2_medkit_gateway {

/// Callback: (entity_id, resource_path) -> JSON payload or error string.
using ResourceSamplerFn = std::function<tl::expected<nlohmann::json, std::string>(const std::string & entity_id,
                                                                                  const std::string & resource_path)>;

/// Thread-safe registry mapping collection names to sampling functions.
///
/// `get_sampler()` hands out the stored callable *by value* - callers (e.g.
/// `SubscriptionTransportProvider::start()`) keep their own copy for the
/// lifetime of a cyclic subscription, entirely independent of this registry.
/// A plugin's callable captures the plugin instance, so once the plugin is
/// torn down and `remove_sampler()` erases the registry entry, any copy
/// handed out *before* removal would otherwise still point at freed memory.
///
/// A liveness flag alone is not enough to close that gap: a caller could load
/// the flag as `true`, get preempted, and only then invoke the underlying
/// plugin callable - by which point `remove_sampler()` has already returned
/// and the caller that erased the entry (`PluginManager::disable_plugin`) may
/// have already freed the plugin object. Checking the flag and calling
/// through are two separate steps, and nothing serialized them against
/// removal.
///
/// To close that check-then-call race, every callable this registry hands
/// out (via `register_sampler`, and therefore via every `get_sampler` copy)
/// shares a `ControlBlock` containing both the liveness flag and a
/// `shared_mutex`. Invocation takes a `shared_lock` on that mutex for the
/// *entire* duration of the flag check plus the call into the underlying
/// callable - not just the flag check. `remove_sampler()` takes a
/// `unique_lock` on the same mutex before flipping the flag, which blocks
/// until every in-flight invocation (holding the `shared_lock`) has finished;
/// only then does it flip the flag and return. This means `remove_sampler()`
/// does not return until no call into the underlying (about-to-be-freed)
/// callable is in progress, closing the race a plain atomic flag could not.
///
/// The `ControlBlock` is shared - not owned - by the registry entry and by
/// every outstanding copy of the callable, via `shared_ptr`, so the drain
/// works even for copies that already escaped the registry (e.g. a running
/// cyclic subscription's transport) before removal. The registry's own
/// `mutex_` only ever protects `samplers_` itself (the map); it is
/// deliberately released before draining a specific entry's `ControlBlock`,
/// so a slow in-flight sampler call for one collection cannot block
/// unrelated registry operations (register/get/has_sampler for other
/// collections) for its duration.
class ResourceSamplerRegistry {
 public:
  /// Register a sampler for `collection`.
  ///
  /// `honours_resource_path` states that `fn` narrows its payload to the one
  /// resource named by its `resource_path` argument. A sampler that ignores
  /// that argument answers with its whole collection on every tick and must
  /// leave the flag false: the subscription layer then refuses a resource URI
  /// naming a single item of that collection, rather than accepting the
  /// request and streaming something wider than was asked for.
  void register_sampler(const std::string & collection, ResourceSamplerFn fn, bool is_builtin = false,
                        bool honours_resource_path = false);
  std::optional<ResourceSamplerFn> get_sampler(const std::string & collection) const;
  bool has_sampler(const std::string & collection) const;

  /// Whether the sampler registered for `collection` narrows its payload to
  /// the resource named by a resource path. False for a collection with no
  /// sampler, and for any sampler registered without declaring it.
  bool honours_resource_path(const std::string & collection) const;

  /// Remove a previously registered sampler. A no-op if the collection was
  /// never registered, or if it was registered with `is_builtin = true` -
  /// built-ins are owned by the gateway node, not by any plugin, and removing
  /// one would silently break a core feature. For non-builtin collections,
  /// blocks until any invocation of the callable already in progress (on any
  /// thread, via any outstanding copy obtained through `get_sampler()`)
  /// finishes, then flips the shared liveness flag so any *future*
  /// invocation of an outstanding copy returns an error instead of calling
  /// into the (now unsafe to touch) plugin. See the class comment for why
  /// the drain, not just the flag, is required.
  ///
  /// Caller-blocking contract: this call blocks until any in-flight
  /// invocation of that sampler callable returns. Callers (e.g.
  /// `PluginManager::disable_plugin`, which holds `plugins_mutex_` for the
  /// duration of this call) must not hold a lock that the sampler callable
  /// could itself need to acquire, or the two will deadlock - this call
  /// cannot return until that invocation finishes, and the invocation cannot
  /// finish if it blocks on a lock its own caller is still holding. Today the
  /// only in-tree sampler callable (graph_provider's `x-medkit-graph`
  /// sampler) takes no such lock, so this is currently safe; this note is for
  /// the next plugin author who adds one that does.
  void remove_sampler(const std::string & collection);

  /// Snapshot of all currently registered collection names (built-in and
  /// plugin-owned). Used by PluginManager to determine, by diffing two
  /// snapshots taken around a single plugin's set_context() call, which
  /// collections that specific call added.
  std::vector<std::string> collection_names() const;

 private:
  /// Shared between the registry entry and every outstanding copy of the
  /// wrapped callable (see class comment). `call_mutex` serializes
  /// invocation against removal (`shared_lock` while calling through,
  /// `unique_lock` while draining and flipping `alive`); `alive` is only ever
  /// read/written while holding `call_mutex`, so the mutex's own
  /// acquire/release semantics are what make the flag's value visible across
  /// threads - `atomic<bool>` is used for clarity and defense in depth, not
  /// because the mutex alone would be insufficient.
  struct ControlBlock {
    std::shared_mutex call_mutex;
    std::atomic<bool> alive{true};
  };

  struct Entry {
    /// Already wrapped with the liveness check - see class comment. This is
    /// exactly what `get_sampler()` copies out.
    ResourceSamplerFn fn;
    bool is_builtin;
    /// See `register_sampler`. The subscription layer reads this to decide
    /// whether a resource URI may name a single item of this collection.
    bool honours_resource_path;
    /// Shared with every copy of `fn` handed out via `get_sampler()`. See
    /// `ControlBlock` and the class comment.
    std::shared_ptr<ControlBlock> control;
  };

  mutable std::shared_mutex mutex_;
  std::unordered_map<std::string, Entry> samplers_;
};

}  // namespace ros2_medkit_gateway
