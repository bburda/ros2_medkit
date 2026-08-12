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

#include "ros2_medkit_gateway/discovery/discovery_manager.hpp"

#include "ros2_medkit_gateway/core/discovery/layers/manifest_layer.hpp"
#include "ros2_medkit_gateway/core/discovery/layers/runtime_layer.hpp"
#include "ros2_medkit_gateway/discovery/layers/plugin_layer.hpp"
#include "ros2_medkit_gateway/discovery/manifest/runtime_linker.hpp"

#include <utility>

namespace ros2_medkit_gateway {

DiscoveryManager::DiscoveryManager(rclcpp::Node * node)
  : node_(node), runtime_introspection_(std::make_unique<ros2::Ros2RuntimeIntrospection>(node)) {
}

bool DiscoveryManager::initialize(const DiscoveryConfig & config) {
  config_ = config;

  RCLCPP_INFO(node_->get_logger(), "Initializing discovery with mode: %s",
              discovery_mode_to_string(config.mode).c_str());

  // Create HostInfoProvider when default component is enabled (runtime_only mode)
  if (config.runtime.default_component_enabled) {
    host_info_provider_ = std::make_unique<HostInfoProvider>();
    RCLCPP_INFO(node_->get_logger(), "Default component enabled: id='%s' (%s)",
                host_info_provider_->get_default_component().id.c_str(),
                host_info_provider_->get_default_component().description.c_str());
  } else {
    host_info_provider_.reset();
  }

  // Create manifest manager if needed
  if (config.mode == DiscoveryMode::MANIFEST_ONLY || config.mode == DiscoveryMode::HYBRID) {
    manifest_manager_ = std::make_unique<discovery::ManifestManager>(node_);
    if (!config.manifest_fragments_dir.empty()) {
      manifest_manager_->set_fragments_dir(config.manifest_fragments_dir);
      RCLCPP_INFO(node_->get_logger(), "Manifest fragments_dir: %s", config.manifest_fragments_dir.c_str());
    }
    if (!config.inventory_csv_path.empty()) {
      manifest_manager_->set_inventory_csv_path(config.inventory_csv_path);
      RCLCPP_INFO(node_->get_logger(), "Inventory CSV: %s", config.inventory_csv_path.c_str());
    }

    if (config.manifest_path.empty()) {
      if (config.mode == DiscoveryMode::HYBRID) {
        RCLCPP_WARN(node_->get_logger(),
                    "No manifest_path set for hybrid mode. Falling back to runtime_only. "
                    "This fallback is deprecated and will be removed in a future release.");
        config_.mode = DiscoveryMode::RUNTIME_ONLY;
        build_pipeline();
        return true;
      }
      RCLCPP_ERROR(node_->get_logger(), "Manifest path required for %s mode. Set discovery.manifest_path.",
                   discovery_mode_to_string(config.mode).c_str());
      return false;
    }

    if (!manifest_manager_->load_manifest(config.manifest_path, config.manifest_strict_validation)) {
      if (config.mode == DiscoveryMode::HYBRID) {
        RCLCPP_WARN(node_->get_logger(),
                    "Manifest load failed in hybrid mode. Falling back to runtime_only. "
                    "This fallback is deprecated and will be removed in a future release.");
        config_.mode = DiscoveryMode::RUNTIME_ONLY;
        manifest_manager_.reset();
        build_pipeline();
        return true;
      }
      RCLCPP_ERROR(node_->get_logger(), "Manifest load failed in %s mode. Cannot proceed.",
                   discovery_mode_to_string(config.mode).c_str());
      return false;
    }
  }

  build_pipeline();
  return true;
}

template <typename LayerT>
void DiscoveryManager::apply_layer_policy_overrides(const std::string & layer_name, LayerT & layer) {
  auto it = config_.merge_pipeline.layer_policies.find(layer_name);
  if (it == config_.merge_pipeline.layer_policies.end()) {
    return;
  }
  for (const auto & [fg_str, policy_str] : it->second) {
    auto fg = discovery::field_group_from_string(fg_str);
    auto policy = discovery::merge_policy_from_string(policy_str);
    if (fg && policy) {
      layer.set_policy(*fg, *policy);
      RCLCPP_INFO(node_->get_logger(), "Layer '%s': override %s = %s", layer_name.c_str(), fg_str.c_str(),
                  policy_str.c_str());
    } else {
      RCLCPP_WARN(node_->get_logger(), "Layer '%s': ignoring invalid override %s = %s", layer_name.c_str(),
                  fg_str.c_str(), policy_str.c_str());
    }
  }
}

// The manifest layer's policies come from two sources that must be applied in
// this order, at boot AND after every reload. Shared so the two paths cannot
// drift: a reload that applied them in the other order would silently invert
// R11a's precedence and let the blanket flag beat an explicit deployment
// parameter.
void DiscoveryManager::apply_manifest_layer_policies(discovery::ManifestLayer & manifest_layer) {
  // allow_manifest_override=false is sugar over the layer policies: it demotes
  // every field group of the manifest layer to FALLBACK, so the manifest only
  // fills gaps the other layers leave. Applied BEFORE the per-group parameter
  // overrides, so a deployment that explicitly pins
  // discovery.merge_pipeline.layers.manifest.<group> still wins - only
  // non-empty parameter values reach layer_policies at all, so this falls out
  // of the existing last-writer-wins ordering.
  if (!get_manifest_config().allow_manifest_override) {
    for (auto fg : {discovery::FieldGroup::IDENTITY, discovery::FieldGroup::HIERARCHY, discovery::FieldGroup::LIVE_DATA,
                    discovery::FieldGroup::STATUS, discovery::FieldGroup::METADATA}) {
      manifest_layer.set_policy(fg, discovery::MergePolicy::FALLBACK);
    }
    RCLCPP_INFO(node_->get_logger(),
                "Layer 'manifest': allow_manifest_override=false, every field group demoted to fallback");
  } else {
    // A reload can flip the flag back on, so restore the shipped defaults
    // rather than leaving the previous demotion in place.
    manifest_layer.reset_policies_to_defaults();
  }
  apply_layer_policy_overrides("manifest", manifest_layer);
}

void DiscoveryManager::refresh_manifest_config() {
  if (!pipeline_) {
    return;
  }
  // /health reads the config live from the ManifestManager while the pipeline
  // captured it once at construction. After a reload the two disagree, and the
  // documented monitor recipe then reports a policy the orphan filter is not
  // actually running. Push the new config in and re-apply the layer policies
  // it drives. NOT a pipeline rebuild: plugin layers added at runtime through
  // add_plugin_layer would be lost.
  pipeline_->set_manifest_config(get_manifest_config());
  if (auto * layer = pipeline_->find_layer("manifest")) {
    apply_manifest_layer_policies(*static_cast<discovery::ManifestLayer *>(layer));
  }
}

void DiscoveryManager::build_pipeline() {
  // Configure runtime introspection (always-on adapter, used by all modes for
  // service/action queries even when the merge pipeline is bypassed).
  ros2::Ros2RuntimeIntrospection::RuntimeConfig runtime_config;
  runtime_config.create_functions_from_namespaces = config_.runtime.create_functions_from_namespaces;
  runtime_introspection_->set_config(runtime_config);

  switch (config_.mode) {
    case DiscoveryMode::MANIFEST_ONLY:
      // Manifest-only: entity structure comes from manifest_manager_; the
      // pipeline is not used. Service/action queries still go through
      // runtime_introspection_ directly.
      RCLCPP_INFO(node_->get_logger(), "Discovery mode: manifest_only");
      pipeline_.reset();
      break;

    case DiscoveryMode::HYBRID: {
      auto pipeline = std::make_unique<discovery::MergePipeline>(node_->get_logger());

      if (config_.manifest_enabled) {
        auto manifest_layer = std::make_unique<discovery::ManifestLayer>(manifest_manager_.get());
        // allow_manifest_override=false is sugar over the layer policies: it
        // demotes every field group of the manifest layer to FALLBACK, so the
        // manifest only fills gaps the other layers leave. Applied BEFORE the
        // per-group parameter overrides, so a deployment that explicitly pins
        // discovery.merge_pipeline.layers.manifest.<group> still wins - only
        // non-empty parameter values reach layer_policies at all, so this falls
        // out of the existing last-writer-wins ordering.
        apply_manifest_layer_policies(*manifest_layer);
        // LAYER ORDER IS PART OF THE CONTRACT, not a convenience. MergePipeline
        // seeds each merged entity from the FIRST layer that declares it and
        // fills provenance first-writer-wins (see fill_provenance() in
        // merge_pipeline.cpp), so whichever layer is added first owns
        // `source` - and `source` is what the orphan filter keys deletion on.
        // The manifest must therefore be added before the runtime layer, and
        // plugin layers are appended after both (add_plugin_layer). Reordering
        // these calls would silently make manifest entities deletable.
        pipeline->add_layer(std::move(manifest_layer));
      } else {
        RCLCPP_INFO(node_->get_logger(), "Manifest layer disabled in hybrid mode");
      }

      if (config_.runtime_enabled) {
        auto runtime_layer = std::make_unique<discovery::RuntimeLayer>(runtime_introspection_.get());
        runtime_layer->set_gap_fill_config(config_.merge_pipeline.gap_fill);
        apply_layer_policy_overrides("runtime", *runtime_layer);
        pipeline->add_layer(std::move(runtime_layer));
      } else {
        RCLCPP_INFO(node_->get_logger(), "Runtime layer disabled in hybrid mode");
      }

      // Warn if no layers at all (plugins may still be loaded later)
      if (!config_.manifest_enabled && !config_.runtime_enabled) {
        RCLCPP_WARN(node_->get_logger(),
                    "Both manifest and runtime layers disabled in hybrid mode. "
                    "Entity discovery relies entirely on plugins.");
      }

      // RuntimeLinker only makes sense when runtime is enabled.
      if (config_.runtime_enabled) {
        pipeline->set_linker(std::make_unique<discovery::RuntimeLinker>(node_), get_manifest_config());
      }

      pipeline_ = std::move(pipeline);
      run_pipeline();
      RCLCPP_INFO(node_->get_logger(), "Discovery mode: hybrid (manifest=%s, runtime=%s)",
                  config_.manifest_enabled ? "on" : "off", config_.runtime_enabled ? "on" : "off");
      break;
    }

    case DiscoveryMode::RUNTIME_ONLY:
    default:
      RCLCPP_INFO(node_->get_logger(), "Discovery mode: runtime_only (default_component=%s)",
                  host_info_provider_ ? "true" : "false");
      pipeline_.reset();
      break;
  }
}

void DiscoveryManager::run_pipeline() {
  if (!pipeline_) {
    return;
  }
  // Execute pipeline WITHOUT lock - safe because the layers' add_layer() calls
  // happen during initialization (build_pipeline + plugin setup) before the
  // EntityCache timer starts, and run_pipeline() runs only on the gateway's
  // single-threaded refresh path.
  auto new_result = pipeline_->execute();

  std::lock_guard<std::mutex> lock(result_mutex_);
  cached_result_ = std::move(new_result);
  RCLCPP_INFO(node_->get_logger(), "Hybrid discovery refreshed: %zu entities", cached_result_.report.total_entities);
}

std::vector<Area> DiscoveryManager::discover_areas() {
  if (config_.mode == DiscoveryMode::MANIFEST_ONLY && manifest_manager_ && manifest_manager_->is_manifest_active()) {
    return manifest_manager_->get_areas();
  }
  if (config_.mode == DiscoveryMode::HYBRID) {
    std::lock_guard<std::mutex> lock(result_mutex_);
    return cached_result_.areas;
  }
  // RUNTIME_ONLY: runtime introspection never produces Areas.
  return {};
}

std::vector<Component> DiscoveryManager::discover_components() {
  if (config_.mode == DiscoveryMode::MANIFEST_ONLY && manifest_manager_ && manifest_manager_->is_manifest_active()) {
    return manifest_manager_->get_components();
  }

  // In RUNTIME_ONLY mode with host info provider, return only the
  // single host-derived Component instead of synthetic namespace components
  if (config_.mode == DiscoveryMode::RUNTIME_ONLY && host_info_provider_) {
    return {host_info_provider_->get_default_component()};
  }

  if (config_.mode == DiscoveryMode::HYBRID) {
    std::lock_guard<std::mutex> lock(result_mutex_);
    return cached_result_.components;
  }

  return {};
}

std::vector<App> DiscoveryManager::discover_apps() {
  if (config_.mode == DiscoveryMode::MANIFEST_ONLY && manifest_manager_ && manifest_manager_->is_manifest_active()) {
    return manifest_manager_->get_apps();
  }
  if (config_.mode == DiscoveryMode::HYBRID) {
    std::lock_guard<std::mutex> lock(result_mutex_);
    return cached_result_.apps;
  }
  return runtime_introspection_->discover_apps();
}

std::vector<Function> DiscoveryManager::discover_functions() {
  if (config_.mode == DiscoveryMode::MANIFEST_ONLY && manifest_manager_ && manifest_manager_->is_manifest_active()) {
    return manifest_manager_->get_functions();
  }
  if (config_.mode == DiscoveryMode::HYBRID) {
    std::lock_guard<std::mutex> lock(result_mutex_);
    return cached_result_.functions;
  }
  return runtime_introspection_->discover_functions();
}

std::vector<Function> DiscoveryManager::discover_functions(const std::vector<App> & apps) {
  if (config_.mode == DiscoveryMode::MANIFEST_ONLY && manifest_manager_ && manifest_manager_->is_manifest_active()) {
    return manifest_manager_->get_functions();
  }
  if (config_.mode == DiscoveryMode::HYBRID) {
    std::lock_guard<std::mutex> lock(result_mutex_);
    return cached_result_.functions;
  }
  // RUNTIME_ONLY: avoid redundant graph queries by reusing the apps vector.
  return runtime_introspection_->discover_functions(apps);
}

std::optional<Area> DiscoveryManager::get_area(const std::string & id) {
  // In MANIFEST_ONLY mode, use direct manifest lookup (O(1))
  if (config_.mode == DiscoveryMode::MANIFEST_ONLY && manifest_manager_ && manifest_manager_->is_manifest_active()) {
    return manifest_manager_->get_area(id);
  }
  // For HYBRID and RUNTIME modes, scan the strategy's output (cached for HYBRID)
  auto areas = discover_areas();
  for (const auto & a : areas) {
    if (a.id == id) {
      return a;
    }
  }
  return std::nullopt;
}

std::optional<Component> DiscoveryManager::get_component(const std::string & id) {
  if (config_.mode == DiscoveryMode::MANIFEST_ONLY && manifest_manager_ && manifest_manager_->is_manifest_active()) {
    return manifest_manager_->get_component(id);
  }
  auto components = discover_components();
  for (const auto & c : components) {
    if (c.id == id) {
      return c;
    }
  }
  return std::nullopt;
}

std::optional<App> DiscoveryManager::get_app(const std::string & id) {
  if (config_.mode == DiscoveryMode::MANIFEST_ONLY && manifest_manager_ && manifest_manager_->is_manifest_active()) {
    return manifest_manager_->get_app(id);
  }
  auto apps = discover_apps();
  for (const auto & app : apps) {
    if (app.id == id) {
      return app;
    }
  }
  return std::nullopt;
}

std::optional<Function> DiscoveryManager::get_function(const std::string & id) {
  if (config_.mode == DiscoveryMode::MANIFEST_ONLY && manifest_manager_ && manifest_manager_->is_manifest_active()) {
    return manifest_manager_->get_function(id);
  }
  auto functions = discover_functions();
  for (const auto & f : functions) {
    if (f.id == id) {
      return f;
    }
  }
  return std::nullopt;
}

std::vector<Area> DiscoveryManager::get_subareas(const std::string & area_id) {
  if (config_.mode == DiscoveryMode::MANIFEST_ONLY && manifest_manager_ && manifest_manager_->is_manifest_active()) {
    return manifest_manager_->get_subareas(area_id);
  }
  // HYBRID: filter from pipeline-merged output; RUNTIME: no subareas
  std::vector<Area> result;
  for (const auto & a : discover_areas()) {
    if (a.parent_area_id == area_id) {
      result.push_back(a);
    }
  }
  return result;
}

std::vector<Component> DiscoveryManager::get_subcomponents(const std::string & component_id) {
  if (config_.mode == DiscoveryMode::MANIFEST_ONLY && manifest_manager_ && manifest_manager_->is_manifest_active()) {
    return manifest_manager_->get_subcomponents(component_id);
  }
  // HYBRID: filter from pipeline-merged output; RUNTIME: no subcomponents
  std::vector<Component> result;
  for (const auto & c : discover_components()) {
    if (c.parent_component_id == component_id) {
      result.push_back(c);
    }
  }
  return result;
}

std::vector<Component> DiscoveryManager::get_components_for_area(const std::string & area_id) {
  if (config_.mode == DiscoveryMode::MANIFEST_ONLY && manifest_manager_ && manifest_manager_->is_manifest_active()) {
    return manifest_manager_->get_components_for_area(area_id);
  }
  // HYBRID: filter from pipeline-merged output; RUNTIME: filter by area
  std::vector<Component> result;
  for (const auto & c : discover_components()) {
    if (c.area == area_id) {
      result.push_back(c);
    }
  }
  return result;
}

std::vector<App> DiscoveryManager::get_apps_for_component(const std::string & component_id) {
  if (config_.mode == DiscoveryMode::MANIFEST_ONLY && manifest_manager_ && manifest_manager_->is_manifest_active()) {
    return manifest_manager_->get_apps_for_component(component_id);
  }
  // HYBRID: filter from pipeline-merged output; RUNTIME: filter by component
  std::vector<App> result;
  for (const auto & app : discover_apps()) {
    if (app.component_id == component_id) {
      result.push_back(app);
    }
  }
  return result;
}

std::vector<std::string> DiscoveryManager::get_hosts_for_function(const std::string & function_id) {
  if (manifest_manager_ && manifest_manager_->is_manifest_active()) {
    return manifest_manager_->get_hosts_for_function(function_id);
  }
  // Check strategy-discovered functions (e.g., runtime namespace-based functions)
  auto func = get_function(function_id);
  if (func) {
    return func->hosts;
  }
  return {};
}

std::vector<ServiceInfo> DiscoveryManager::discover_services() {
  return runtime_introspection_->discover_services();
}

std::vector<ActionInfo> DiscoveryManager::discover_actions() {
  return runtime_introspection_->discover_actions();
}

std::optional<ServiceInfo> DiscoveryManager::find_service(const std::string & component_ns,
                                                          const std::string & operation_name) const {
  return runtime_introspection_->find_service(component_ns, operation_name);
}

std::optional<ActionInfo> DiscoveryManager::find_action(const std::string & component_ns,
                                                        const std::string & operation_name) const {
  return runtime_introspection_->find_action(component_ns, operation_name);
}

void DiscoveryManager::set_topic_data_provider(TopicDataProvider * provider) {
  runtime_introspection_->set_topic_data_provider(provider);
}

void DiscoveryManager::set_type_introspection(ros2_medkit_serialization::TypeIntrospection * introspection) {
  runtime_introspection_->set_type_introspection(introspection);
}

void DiscoveryManager::refresh_topic_map() {
  runtime_introspection_->refresh_topic_map();
  if (pipeline_) {
    run_pipeline();
  }
}

bool DiscoveryManager::is_topic_map_ready() const {
  return runtime_introspection_->is_topic_map_ready();
}

void DiscoveryManager::add_plugin_layer(const std::string & plugin_name, IntrospectionProvider * provider) {
  if (!pipeline_) {
    RCLCPP_WARN(node_->get_logger(), "Cannot add plugin layer '%s': not in hybrid mode", plugin_name.c_str());
    return;
  }
  // Appended AFTER the manifest and runtime layers, and that is load-bearing:
  // provenance is first-writer-wins, so a plugin can never take `source` away
  // from a manifest entity it also describes. See the note at the manifest
  // add_layer() call above.
  pipeline_->add_layer(std::make_unique<discovery::PluginLayer>(plugin_name, provider));
  RCLCPP_INFO(node_->get_logger(), "Added plugin layer '%s' to merge pipeline", plugin_name.c_str());
}

void DiscoveryManager::register_introspection_provider(const std::string & name,
                                                       std::shared_ptr<IntrospectionProvider> provider) {
  if (!provider) {
    return;
  }
  IntrospectionProvider * raw = provider.get();
  owned_providers_.push_back(std::move(provider));
  add_plugin_layer(name, raw);
}

void DiscoveryManager::refresh_pipeline() {
  if (pipeline_) {
    run_pipeline();
  }
}

discovery::ManifestManager * DiscoveryManager::get_manifest_manager() {
  return manifest_manager_.get();
}

discovery::ManifestConfig DiscoveryManager::get_manifest_config() const {
  return manifest_manager_ ? manifest_manager_->get_config() : discovery::ManifestConfig{};
}

std::string DiscoveryManager::get_strategy_name() const {
  switch (config_.mode) {
    case DiscoveryMode::MANIFEST_ONLY:
      return "manifest";
    case DiscoveryMode::HYBRID:
      return "hybrid";
    case DiscoveryMode::RUNTIME_ONLY:
    default:
      return "runtime";
  }
}

std::optional<discovery::MergeReport> DiscoveryManager::get_merge_report() const {
  if (!pipeline_) {
    return std::nullopt;
  }
  std::lock_guard<std::mutex> lock(result_mutex_);
  return cached_result_.report;
}

std::optional<discovery::LinkingResult> DiscoveryManager::get_linking_result() const {
  if (!pipeline_) {
    return std::nullopt;
  }
  std::lock_guard<std::mutex> lock(result_mutex_);
  return cached_result_.linking_result;
}

bool DiscoveryManager::has_host_info_provider() const {
  return host_info_provider_ != nullptr;
}

std::optional<Component> DiscoveryManager::get_default_component() const {
  if (host_info_provider_) {
    return host_info_provider_->get_default_component();
  }
  return std::nullopt;
}

}  // namespace ros2_medkit_gateway
