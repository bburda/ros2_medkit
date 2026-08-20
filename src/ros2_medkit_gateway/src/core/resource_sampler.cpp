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

#include "ros2_medkit_gateway/core/resource_sampler.hpp"

#include <mutex>
#include <stdexcept>

namespace ros2_medkit_gateway {

void ResourceSamplerRegistry::register_sampler(const std::string & collection, ResourceSamplerFn fn, bool is_builtin,
                                               bool honours_resource_path) {
  std::unique_lock lock(mutex_);

  if (!is_builtin) {
    if (collection.size() < 2 || collection.substr(0, 2) != "x-") {
      throw std::runtime_error("Plugin sampler collection must use 'x-' prefix: " + collection);
    }
    if (samplers_.count(collection) > 0) {
      throw std::runtime_error("Sampler already registered for collection: " + collection);
    }
  }

  auto control = std::make_shared<ControlBlock>();
  ResourceSamplerFn wrapped = [control, fn = std::move(fn), collection](
                                  const std::string & entity_id,
                                  const std::string & resource_path) -> tl::expected<nlohmann::json, std::string> {
    // Hold the shared lock for the entire check-then-call sequence, not just
    // the flag read: remove_sampler() takes a unique_lock on this same mutex
    // before flipping the flag, so as long as this lock is held for the full
    // duration of the call into `fn`, remove_sampler() cannot return (and the
    // caller cannot free the plugin) while this call is still in progress.
    std::shared_lock call_lock(control->call_mutex);
    if (!control->alive.load(std::memory_order_acquire)) {
      return tl::make_unexpected("Sampler for collection '" + collection +
                                 "' is no longer available: its plugin has been removed");
    }
    return fn(entity_id, resource_path);
  };

  samplers_[collection] = Entry{std::move(wrapped), is_builtin, honours_resource_path, std::move(control)};
}

std::optional<ResourceSamplerFn> ResourceSamplerRegistry::get_sampler(const std::string & collection) const {
  std::shared_lock lock(mutex_);
  auto it = samplers_.find(collection);
  if (it == samplers_.end()) {
    return std::nullopt;
  }
  return it->second.fn;
}

bool ResourceSamplerRegistry::has_sampler(const std::string & collection) const {
  std::shared_lock lock(mutex_);
  return samplers_.count(collection) > 0;
}

bool ResourceSamplerRegistry::honours_resource_path(const std::string & collection) const {
  std::shared_lock lock(mutex_);
  auto it = samplers_.find(collection);
  return it != samplers_.end() && it->second.honours_resource_path;
}

void ResourceSamplerRegistry::remove_sampler(const std::string & collection) {
  std::shared_ptr<ControlBlock> control;
  {
    std::unique_lock lock(mutex_);
    auto it = samplers_.find(collection);
    if (it == samplers_.end() || it->second.is_builtin) {
      return;
    }
    // Grab our own reference to the control block before erasing - the map
    // entry is about to go away, but every outstanding copy of the wrapped
    // callable (e.g. a running cyclic subscription's transport) keeps this
    // same block alive independently via its own shared_ptr.
    control = it->second.control;
    samplers_.erase(it);
  }
  // Release the registry's own mutex_ before draining: a slow in-flight
  // sampler call for THIS collection must not block unrelated
  // register/get/has_sampler calls for other collections.
  //
  // Taking the unique_lock here blocks until every invocation currently
  // holding the shared_lock (see register_sampler's wrapped callable) has
  // finished calling into the underlying plugin callable. Only once that
  // drain completes do we flip `alive` to false and return - this is what
  // closes the check-then-call race: by the time this function returns, no
  // call into the plugin's callable can still be in progress, so the caller
  // (PluginManager::disable_plugin) freeing the plugin object right after is
  // safe.
  std::unique_lock call_lock(control->call_mutex);
  control->alive.store(false, std::memory_order_release);
}

std::vector<std::string> ResourceSamplerRegistry::collection_names() const {
  std::shared_lock lock(mutex_);
  std::vector<std::string> names;
  names.reserve(samplers_.size());
  for (const auto & [name, entry] : samplers_) {
    names.push_back(name);
  }
  return names;
}

}  // namespace ros2_medkit_gateway
