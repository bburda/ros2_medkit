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

#include "ros2_medkit_gateway/core/plugins/plugin_manager.hpp"

#include <dlfcn.h>
#include <httplib.h>

#include <rclcpp/rclcpp.hpp>
#include <regex>
#include <unordered_set>

#include "ros2_medkit_gateway/core/http/error_codes.hpp"
#include "ros2_medkit_gateway/core/http/http_utils.hpp"
#include "ros2_medkit_gateway/core/plugins/plugin_http_types.hpp"

namespace ros2_medkit_gateway {

namespace {
auto logger() {
  return rclcpp::get_logger("plugin_manager");
}
}  // namespace

void PluginManager::setup_plugin_logging(GatewayPlugin & plugin) {
  std::string plugin_name = plugin.name();
  plugin.set_logger([plugin_name](PluginLogLevel level, const std::string & msg) {
    auto log = rclcpp::get_logger("plugin." + plugin_name);
    switch (level) {
      case PluginLogLevel::kInfo:
        RCLCPP_INFO(log, "%s", msg.c_str());
        break;
      case PluginLogLevel::kWarn:
        RCLCPP_WARN(log, "%s", msg.c_str());
        break;
      case PluginLogLevel::kError:
        RCLCPP_ERROR(log, "%s", msg.c_str());
        break;
    }
  });
}

PluginManager::~PluginManager() {
  shutdown_all();
  // GatewayPluginLoadResult RAII handles destruction order
  plugins_.clear();
}

// The plugin list is published to readers as soon as it is written to, so both
// registration paths take the writer lock. "Registration happens before the
// HTTP server starts" was true and still is, and it is not enough: the ROS log
// subscription is already delivering while the gateway constructor runs, and
// LogManager::on_log_entry reads this list through log_observers(). Those
// readers hold a shared lock, so an unlocked push_back here is a write racing
// a read - reported by ThreadSanitizer inside vector's reallocation, where the
// reader walks a buffer the writer has just freed.
void PluginManager::add_plugin(std::unique_ptr<GatewayPlugin> plugin) {
  LoadedPlugin lp;
  lp.config = nlohmann::json::object();

  // For in-process plugins, use dynamic_cast (safe within same binary)
  lp.update_provider = dynamic_cast<UpdateProvider *>(plugin.get());
  lp.introspection_provider = dynamic_cast<IntrospectionProvider *>(plugin.get());
  lp.log_provider = dynamic_cast<LogProvider *>(plugin.get());
  lp.script_provider = dynamic_cast<ScriptProvider *>(plugin.get());
  lp.data_provider = dynamic_cast<DataProvider *>(plugin.get());
  lp.operation_provider = dynamic_cast<OperationProvider *>(plugin.get());
  lp.lifecycle_provider = dynamic_cast<LifecycleProvider *>(plugin.get());
  lp.fault_provider = dynamic_cast<FaultProvider *>(plugin.get());

  // Held from the first cached provider through the push_back: the caches and
  // the list are read together, so publishing one without the other lets a
  // reader see a provider the list does not yet carry.
  std::unique_lock<std::shared_mutex> lock(plugins_mutex_);

  // Cache first UpdateProvider, warn on duplicates
  if (lp.update_provider) {
    if (!first_update_provider_) {
      first_update_provider_ = lp.update_provider;
    } else {
      RCLCPP_WARN(logger(), "Multiple UpdateProvider plugins loaded - ignoring '%s'", plugin->name().c_str());
    }
  }

  // Cache first LogProvider; additional LogProvider plugins are observers only
  if (lp.log_provider) {
    if (!first_log_provider_) {
      first_log_provider_ = lp.log_provider;
    } else {
      RCLCPP_DEBUG(logger(), "LogProvider plugin '%s' registered as observer only", plugin->name().c_str());
    }
  }

  // Cache first ScriptProvider, warn on duplicates
  if (lp.script_provider) {
    if (!first_script_provider_) {
      first_script_provider_ = lp.script_provider;
    } else {
      RCLCPP_WARN(logger(), "Multiple ScriptProvider plugins loaded - ignoring '%s'", plugin->name().c_str());
    }
  }

  setup_plugin_logging(*plugin);
  lp.load_result.plugin = std::move(plugin);
  plugins_.push_back(std::move(lp));
}

// Same reasoning as add_plugin: registration is not the only thing running.
size_t PluginManager::load_plugins(const std::vector<PluginConfig> & configs) {
  size_t loaded = 0;
  for (const auto & cfg : configs) {
    auto result = PluginLoader::load(cfg.path);
    if (result) {
      RCLCPP_INFO(logger(), "Loaded plugin '%s' from %s", result->plugin->name().c_str(), cfg.path.c_str());

      LoadedPlugin lp;
      lp.config = cfg.config;

      // Provider pointers from extern "C" query functions (safe across dlopen boundary)
      lp.update_provider = result->update_provider;
      lp.introspection_provider = result->introspection_provider;
      // LogProvider: discovered via extern "C" query function (mirrors UpdateProvider /
      // IntrospectionProvider mechanism - safe across the dlopen boundary).
      lp.log_provider = result->log_provider;
      lp.script_provider = result->script_provider;
      lp.data_provider = result->data_provider;
      lp.operation_provider = result->operation_provider;
      // LifecycleProvider: discovered via extern "C" query function - dynamic_cast
      // is unreliable across the dlopen boundary (typeinfo can differ between the
      // plugin and the host, casting a real provider to null).
      lp.lifecycle_provider = result->lifecycle_provider;
      lp.fault_provider = result->fault_provider;

      // Taken per plugin rather than around the whole loop, so a slow dlopen
      // for the next one does not hold readers off the ones already loaded.
      std::unique_lock<std::shared_mutex> lock(plugins_mutex_);

      // Cache first UpdateProvider, warn on duplicates
      if (lp.update_provider) {
        if (!first_update_provider_) {
          first_update_provider_ = lp.update_provider;
        } else {
          RCLCPP_WARN(logger(), "Multiple UpdateProvider plugins loaded - ignoring '%s'",
                      result->plugin->name().c_str());
        }
      }

      // Cache first LogProvider; additional LogProvider plugins are observers only
      if (lp.log_provider) {
        if (!first_log_provider_) {
          first_log_provider_ = lp.log_provider;
        } else {
          RCLCPP_DEBUG(logger(), "LogProvider plugin '%s' registered as observer only", result->plugin->name().c_str());
        }
      }

      // Cache first ScriptProvider, warn on duplicates
      if (lp.script_provider) {
        if (!first_script_provider_) {
          first_script_provider_ = lp.script_provider;
        } else {
          RCLCPP_WARN(logger(), "Multiple ScriptProvider plugins loaded - ignoring '%s'",
                      result->plugin->name().c_str());
        }
      }

      setup_plugin_logging(*result->plugin);
      lp.load_result = std::move(*result);
      plugins_.push_back(std::move(lp));
      ++loaded;
    } else {
      RCLCPP_ERROR(logger(), "Failed to load plugin from %s: %s", cfg.path.c_str(), result.error().c_str());
    }
  }
  return loaded;
}

void PluginManager::disable_plugin(LoadedPlugin & lp) {
  if (lp.load_result.plugin) {
    try {
      lp.load_result.plugin->shutdown();
    } catch (const std::exception & e) {
      RCLCPP_WARN(logger(), "Plugin '%s' threw during shutdown(): %s", lp.load_result.plugin->name().c_str(), e.what());
    } catch (...) {
      RCLCPP_WARN(logger(), "Plugin '%s' threw unknown exception during shutdown()",
                  lp.load_result.plugin->name().c_str());
    }
  }
  // Invalidate cached provider if this plugin was the source, re-scan for next
  if (first_update_provider_ && lp.update_provider == first_update_provider_) {
    first_update_provider_ = nullptr;
    for (const auto & other : plugins_) {
      if (&other != &lp && other.load_result.plugin && other.update_provider) {
        first_update_provider_ = other.update_provider;
        break;
      }
    }
  }
  if (first_log_provider_ && lp.log_provider == first_log_provider_) {
    first_log_provider_ = nullptr;
    for (const auto & other : plugins_) {
      if (&other != &lp && other.load_result.plugin && other.log_provider) {
        first_log_provider_ = other.log_provider;
        break;
      }
    }
  }
  if (first_script_provider_ && lp.script_provider == first_script_provider_) {
    first_script_provider_ = nullptr;
    for (const auto & other : plugins_) {
      if (&other != &lp && other.load_result.plugin && other.script_provider) {
        first_script_provider_ = other.script_provider;
        break;
      }
    }
  }
  lp.update_provider = nullptr;
  lp.introspection_provider = nullptr;
  lp.log_provider = nullptr;
  lp.script_provider = nullptr;
  lp.data_provider = nullptr;
  lp.operation_provider = nullptr;
  lp.lifecycle_provider = nullptr;
  lp.fault_provider = nullptr;
  lp.load_result.update_provider = nullptr;
  lp.load_result.introspection_provider = nullptr;
  lp.load_result.log_provider = nullptr;
  lp.load_result.script_provider = nullptr;
  lp.load_result.data_provider = nullptr;
  lp.load_result.operation_provider = nullptr;
  lp.load_result.fault_provider = nullptr;
  // Cached route handlers capture the plugin instance - drop them with it.
  lp.routes.clear();
  lp.routes_cached = false;
  // Cyclic-subscription samplers capture the plugin instance too - same hazard.
  remove_plugin_samplers(lp);
  lp.load_result.plugin.reset();
}

void PluginManager::capture_new_sampler_collections(LoadedPlugin & lp, const std::vector<std::string> & before) {
  if (!sampler_registry_) {
    return;
  }
  std::unordered_set<std::string> before_set(before.begin(), before.end());
  for (auto & name : sampler_registry_->collection_names()) {
    if (before_set.count(name) == 0) {
      lp.sampler_collections.push_back(std::move(name));
    }
  }
}

void PluginManager::remove_plugin_samplers(LoadedPlugin & lp) {
  if (!sampler_registry_) {
    return;
  }
  for (const auto & collection : lp.sampler_collections) {
    sampler_registry_->remove_sampler(collection);
  }
  lp.sampler_collections.clear();
}

void PluginManager::configure_plugins() {
  std::unique_lock<std::shared_mutex> lock(plugins_mutex_);
  for (auto & lp : plugins_) {
    if (!lp.load_result.plugin) {
      continue;
    }
    try {
      lp.load_result.plugin->configure(lp.config);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(logger(), "Plugin '%s' threw during configure(): %s", lp.load_result.plugin->name().c_str(),
                   e.what());
      disable_plugin(lp);
    } catch (...) {
      RCLCPP_ERROR(logger(), "Plugin '%s' threw unknown exception during configure()",
                   lp.load_result.plugin->name().c_str());
      disable_plugin(lp);
    }
  }
}

void PluginManager::set_context(PluginContext & context) {
  context_ = &context;
  std::unique_lock<std::shared_mutex> lock(plugins_mutex_);
  for (auto & lp : plugins_) {
    if (!lp.load_result.plugin) {
      continue;
    }
    // Plugins register cyclic-subscription samplers directly against the
    // shared ResourceSamplerRegistry from within set_context() (see
    // GraphProviderPlugin::set_context()), bypassing this manager. Snapshot
    // the registry's collections before and after the call so any newly
    // added ones can be attributed to this plugin and removed on teardown.
    std::vector<std::string> before_collections;
    if (sampler_registry_) {
      before_collections = sampler_registry_->collection_names();
    }
    try {
      lp.load_result.plugin->set_context(context);
      capture_new_sampler_collections(lp, before_collections);
    } catch (const std::exception & e) {
      capture_new_sampler_collections(lp, before_collections);
      RCLCPP_ERROR(logger(), "Plugin '%s' threw during set_context(): %s - disabling",
                   lp.load_result.plugin->name().c_str(), e.what());
      disable_plugin(lp);
    } catch (...) {
      capture_new_sampler_collections(lp, before_collections);
      RCLCPP_ERROR(logger(), "Plugin '%s' threw unknown exception during set_context() - disabling",
                   lp.load_result.plugin->name().c_str());
      disable_plugin(lp);
    }
  }
}

void PluginManager::set_registries(ResourceSamplerRegistry & samplers, TransportRegistry & transports) {
  sampler_registry_ = &samplers;
  transport_registry_ = &transports;
}

void PluginManager::register_transport(std::unique_ptr<SubscriptionTransportProvider> provider) {
  if (!transport_registry_) {
    throw std::runtime_error("Transport registry not initialized");
  }
  transport_registry_->register_transport(std::move(provider));
}

bool PluginManager::cache_routes_locked(LoadedPlugin & lp) {
  if (lp.routes_cached) {
    return true;
  }
  try {
    lp.routes = lp.load_result.plugin->get_routes();
    lp.routes_cached = true;
    return true;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger(), "Plugin '%s' threw during get_routes(): %s - disabling",
                 lp.load_result.plugin->name().c_str(), e.what());
    disable_plugin(lp);
  } catch (...) {
    RCLCPP_ERROR(logger(), "Plugin '%s' threw unknown exception during get_routes() - disabling",
                 lp.load_result.plugin->name().c_str());
    disable_plugin(lp);
  }
  return false;
}

void PluginManager::register_routes(httplib::Server & server, const std::string & api_prefix) {
  std::unique_lock<std::shared_mutex> lock(plugins_mutex_);
  for (auto & lp : plugins_) {
    if (!lp.load_result.plugin || !cache_routes_locked(lp)) {
      continue;
    }
    for (auto & route : lp.routes) {
      std::string full_pattern = api_prefix + "/" + route.pattern;
      auto handler_fn = route.handler;  // capture by value for lambda
      auto plugin_name = lp.load_result.plugin->name();
      auto httplib_handler = [handler_fn, plugin_name, full_pattern](const httplib::Request & req,
                                                                     httplib::Response & res) {
        try {
          PluginRequest plugin_req(&req);
          PluginResponse plugin_res(&res);
          handler_fn(plugin_req, plugin_res);
        } catch (const std::exception & e) {
          RCLCPP_ERROR(rclcpp::get_logger("plugin_manager"), "Plugin '%s' handler threw on %s: %s", plugin_name.c_str(),
                       full_pattern.c_str(), e.what());
          PluginResponse plugin_res(&res);
          plugin_res.send_error(500, ERR_PLUGIN_ERROR, "Internal plugin error");
        } catch (...) {
          RCLCPP_ERROR(rclcpp::get_logger("plugin_manager"), "Plugin '%s' handler threw unknown exception on %s",
                       plugin_name.c_str(), full_pattern.c_str());
          PluginResponse plugin_res(&res);
          plugin_res.send_error(500, ERR_PLUGIN_ERROR, "Internal plugin error");
        }
      };

      if (route.method == "GET") {
        server.Get(full_pattern, httplib_handler);
      } else if (route.method == "POST") {
        server.Post(full_pattern, httplib_handler);
      } else if (route.method == "PUT") {
        server.Put(full_pattern, httplib_handler);
      } else if (route.method == "DELETE") {
        server.Delete(full_pattern, httplib_handler);
      } else {
        RCLCPP_WARN(logger(), "Plugin '%s' registered route with unknown method '%s' - skipping",
                    lp.load_result.plugin->name().c_str(), route.method.c_str());
      }
    }
  }
}

bool PluginManager::has_entity_data_route(const std::string & entity_id) {
  const std::string full_path = std::string(API_BASE_PATH) + "/apps/" + entity_id + "/x-plc-data";
  std::unique_lock<std::shared_mutex> lock(plugins_mutex_);
  auto own_it = entity_ownership_.find(entity_id);
  if (own_it == entity_ownership_.end()) {
    return false;
  }
  for (auto & lp : plugins_) {
    if (!lp.load_result.plugin || lp.load_result.plugin->name() != own_it->second) {
      continue;
    }
    if (!cache_routes_locked(lp)) {
      return false;
    }
    for (const auto & route : lp.routes) {
      if (route.method != "GET") {
        continue;
      }
      try {
        if (std::regex_match(full_path, std::regex(API_BASE_PATH + ("/" + route.pattern)))) {
          return true;
        }
      } catch (const std::regex_error &) {
        continue;
      }
    }
    break;
  }
  return false;
}

std::optional<nlohmann::json> PluginManager::fetch_entity_data_via_route(const std::string & entity_id,
                                                                         const std::string & item) {
  const std::string full_path =
      std::string(API_BASE_PATH) + "/apps/" + entity_id + "/x-plc-data" + (item.empty() ? "" : "/" + item);

  // Copy the owning plugin's matching GET handler out under the lock and
  // invoke it after release: handlers run arbitrary plugin code that may call
  // back into this manager (shared_mutex is not reentrant).
  std::function<void(const PluginRequest &, PluginResponse &)> handler;
  std::string matched_pattern;
  {
    // Unique lock: first call may populate the route cache.
    std::unique_lock<std::shared_mutex> lock(plugins_mutex_);
    auto own_it = entity_ownership_.find(entity_id);
    if (own_it == entity_ownership_.end()) {
      return std::nullopt;
    }
    for (auto & lp : plugins_) {
      if (!lp.load_result.plugin || lp.load_result.plugin->name() != own_it->second) {
        continue;
      }
      if (!cache_routes_locked(lp)) {
        return std::nullopt;
      }
      for (const auto & route : lp.routes) {
        if (route.method != "GET") {
          continue;
        }
        try {
          if (std::regex_match(full_path, std::regex(API_BASE_PATH + ("/" + route.pattern)))) {
            handler = route.handler;
            matched_pattern = route.pattern;
            break;
          }
        } catch (const std::regex_error &) {
          continue;  // malformed plugin pattern - the HTTP server would reject it too
        }
      }
      break;
    }
  }
  if (!handler) {
    return std::nullopt;
  }

  // Synthesize the request exactly as the HTTP server presents it: full API
  // path plus regex captures, so path_param() and path()-based entity-type
  // checks in the handler behave identically to a real request.
  httplib::Request req;
  req.method = "GET";
  req.path = full_path;
  std::regex_match(req.path, req.matches, std::regex(API_BASE_PATH + ("/" + matched_pattern)));
  httplib::Response res;
  res.status = 200;  // the server defaults handled routes to 200; send_error overrides

  try {
    PluginRequest plugin_req(&req);
    PluginResponse plugin_res(&res);
    handler(plugin_req, plugin_res);
  } catch (const std::exception & e) {
    RCLCPP_WARN(logger(), "In-process x-plc-data dispatch for entity '%s' threw: %s", entity_id.c_str(), e.what());
    return std::nullopt;
  } catch (...) {
    RCLCPP_WARN(logger(), "In-process x-plc-data dispatch for entity '%s' threw unknown exception", entity_id.c_str());
    return std::nullopt;
  }

  if (res.status != 200) {
    return std::nullopt;
  }
  auto body = nlohmann::json::parse(res.body, nullptr, /*allow_exceptions=*/false);
  if (body.is_discarded()) {
    return std::nullopt;
  }
  return body;
}

void PluginManager::shutdown_all() {
  if (shutdown_called_) {
    return;
  }
  shutdown_called_ = true;
  std::unique_lock<std::shared_mutex> lock(plugins_mutex_);
  for (auto & lp : plugins_) {
    if (!lp.load_result.plugin) {
      continue;
    }
    try {
      lp.load_result.plugin->shutdown();
    } catch (const std::exception & e) {
      RCLCPP_WARN(logger(), "Plugin '%s' threw during shutdown(): %s", lp.load_result.plugin->name().c_str(), e.what());
    } catch (...) {
      RCLCPP_WARN(logger(), "Plugin '%s' threw unknown exception during shutdown()",
                  lp.load_result.plugin->name().c_str());
    }
    // The plugin instance itself is freed later, in ~PluginManager()'s
    // plugins_.clear() - but ResourceSamplerRegistry is owned by GatewayNode
    // and outlives PluginManager, so its dangling entries must be dropped now.
    remove_plugin_samplers(lp);
  }
}

UpdateProvider * PluginManager::get_update_provider() const {
  std::shared_lock<std::shared_mutex> lock(plugins_mutex_);
  return first_update_provider_;
}

std::vector<IntrospectionProvider *> PluginManager::get_introspection_providers() const {
  std::shared_lock<std::shared_mutex> lock(plugins_mutex_);
  std::vector<IntrospectionProvider *> result;
  for (const auto & lp : plugins_) {
    if (!lp.load_result.plugin) {
      continue;
    }
    if (lp.introspection_provider) {
      result.push_back(lp.introspection_provider);
    }
  }
  return result;
}

LogProvider * PluginManager::get_log_provider() const {
  std::shared_lock<std::shared_mutex> lock(plugins_mutex_);
  return first_log_provider_;
}

ScriptProvider * PluginManager::get_script_provider() const {
  std::shared_lock<std::shared_mutex> lock(plugins_mutex_);
  return first_script_provider_;
}

std::vector<LogProvider *> PluginManager::get_log_observers() const {
  std::shared_lock<std::shared_mutex> lock(plugins_mutex_);
  std::vector<LogProvider *> result;
  for (const auto & lp : plugins_) {
    if (lp.log_provider) {
      result.push_back(lp.log_provider);
    }
  }
  return result;
}

std::vector<std::pair<std::string, IntrospectionProvider *>> PluginManager::get_named_introspection_providers() const {
  std::shared_lock<std::shared_mutex> lock(plugins_mutex_);
  std::vector<std::pair<std::string, IntrospectionProvider *>> result;
  for (const auto & lp : plugins_) {
    if (!lp.load_result.plugin) {
      continue;
    }
    if (lp.introspection_provider) {
      result.emplace_back(lp.load_result.plugin->name(), lp.introspection_provider);
    }
  }
  return result;
}

bool PluginManager::has_plugins() const {
  std::shared_lock<std::shared_mutex> lock(plugins_mutex_);
  for (const auto & lp : plugins_) {
    if (lp.load_result.plugin) {
      return true;
    }
  }
  return false;
}

std::vector<std::string> PluginManager::plugin_names() const {
  std::shared_lock<std::shared_mutex> lock(plugins_mutex_);
  std::vector<std::string> names;
  for (const auto & lp : plugins_) {
    if (lp.load_result.plugin) {
      names.push_back(lp.load_result.plugin->name());
    }
  }
  return names;
}

void PluginManager::register_entity_ownership(const std::string & plugin_name,
                                              const std::vector<std::string> & entity_ids) {
  std::unique_lock<std::shared_mutex> lock(plugins_mutex_);
  for (const auto & eid : entity_ids) {
    auto it = entity_ownership_.find(eid);
    if (it != entity_ownership_.end() && it->second != plugin_name) {
      RCLCPP_WARN(logger(), "Entity '%s' ownership transferred from plugin '%s' to '%s'", eid.c_str(),
                  it->second.c_str(), plugin_name.c_str());
    }
    entity_ownership_[eid] = plugin_name;
  }
}

void PluginManager::clear_entity_ownership(const std::string & plugin_name) {
  std::unique_lock<std::shared_mutex> lock(plugins_mutex_);
  for (auto it = entity_ownership_.begin(); it != entity_ownership_.end();) {
    if (it->second == plugin_name) {
      it = entity_ownership_.erase(it);
    } else {
      ++it;
    }
  }
}

DataProvider * PluginManager::get_data_provider_for_entity(const std::string & entity_id) const {
  std::shared_lock<std::shared_mutex> lock(plugins_mutex_);
  auto own_it = entity_ownership_.find(entity_id);
  if (own_it == entity_ownership_.end()) {
    return nullptr;
  }
  for (const auto & lp : plugins_) {
    if (lp.load_result.plugin && lp.load_result.plugin->name() == own_it->second) {
      // Ownership is plugin-wide, but data is not: ask the provider whether
      // this specific entity is data-bearing (see DataProvider::has_data).
      return lp.data_provider && lp.data_provider->has_data(entity_id) ? lp.data_provider : nullptr;
    }
  }
  return nullptr;
}

OperationProvider * PluginManager::get_operation_provider_for_entity(const std::string & entity_id) const {
  std::shared_lock<std::shared_mutex> lock(plugins_mutex_);
  auto own_it = entity_ownership_.find(entity_id);
  if (own_it == entity_ownership_.end()) {
    return nullptr;
  }
  for (const auto & lp : plugins_) {
    if (lp.load_result.plugin && lp.load_result.plugin->name() == own_it->second) {
      // Same reasoning as get_data_provider_for_entity: ownership does not
      // imply this entity has operations (see OperationProvider::has_operations).
      return lp.operation_provider && lp.operation_provider->has_operations(entity_id) ? lp.operation_provider
                                                                                       : nullptr;
    }
  }
  return nullptr;
}

LifecycleProvider * PluginManager::get_lifecycle_provider_for_entity(const std::string & entity_id) const {
  std::shared_lock<std::shared_mutex> lock(plugins_mutex_);
  auto own_it = entity_ownership_.find(entity_id);
  if (own_it == entity_ownership_.end()) {
    return nullptr;
  }
  for (const auto & lp : plugins_) {
    if (lp.load_result.plugin && lp.load_result.plugin->name() == own_it->second) {
      return lp.lifecycle_provider;
    }
  }
  return nullptr;
}

FaultProvider * PluginManager::get_fault_provider_for_entity(const std::string & entity_id) const {
  std::shared_lock<std::shared_mutex> lock(plugins_mutex_);
  auto own_it = entity_ownership_.find(entity_id);
  if (own_it == entity_ownership_.end()) {
    return nullptr;
  }
  for (const auto & lp : plugins_) {
    if (lp.load_result.plugin && lp.load_result.plugin->name() == own_it->second) {
      return lp.fault_provider;
    }
  }
  return nullptr;
}

std::optional<std::string> PluginManager::get_entity_owner(const std::string & entity_id) const {
  std::shared_lock<std::shared_mutex> lock(plugins_mutex_);
  auto it = entity_ownership_.find(entity_id);
  if (it != entity_ownership_.end()) {
    return it->second;
  }
  return std::nullopt;
}

std::vector<openapi::RouteDescriptions> PluginManager::collect_route_descriptions() const {
  std::vector<openapi::RouteDescriptions> all_descriptions;
  std::shared_lock<std::shared_mutex> lock(plugins_mutex_);

  for (const auto & lp : plugins_) {
    if (!lp.load_result.plugin) {
      continue;
    }

    void * handle = lp.load_result.dl_handle();
    if (!handle) {
      continue;
    }

    // Check for optional describe_plugin_routes symbol
    using DescribeFn = openapi::RouteDescriptions (*)();
    auto fn = reinterpret_cast<DescribeFn>(dlsym(handle, "describe_plugin_routes"));
    if (!fn) {
      continue;  // Plugin doesn't export route descriptions - skip silently
    }

    try {
      all_descriptions.push_back(fn());
    } catch (const std::exception & e) {
      RCLCPP_WARN(logger(), "Plugin '%s' threw in describe_plugin_routes(): %s", lp.load_result.plugin->name().c_str(),
                  e.what());
    } catch (...) {
      RCLCPP_WARN(logger(), "Plugin '%s' threw unknown exception in describe_plugin_routes()",
                  lp.load_result.plugin->name().c_str());
    }
  }

  return all_descriptions;
}

}  // namespace ros2_medkit_gateway
