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

#include "ros2_medkit_gateway/core/plugins/gateway_plugin.hpp"
#include "ros2_medkit_gateway/core/plugins/plugin_http_types.hpp"
#include "ros2_medkit_gateway/core/plugins/plugin_types.hpp"
#include "ros2_medkit_gateway/core/providers/introspection_provider.hpp"
#include "ros2_medkit_gateway/plugins/ros_plugin_context.hpp"
#include "ros2_medkit_linux_introspection/cgroup_reader.hpp"
#include "ros2_medkit_linux_introspection/container_utils.hpp"
#include "ros2_medkit_linux_introspection/plugin_config.hpp"
#include "ros2_medkit_linux_introspection/proc_reader.hpp"

#include <nlohmann/json.hpp>

#include <map>
#include <memory>
#include <string>

using namespace ros2_medkit_gateway;  // NOLINT(build/namespaces)

class ContainerPlugin : public GatewayPlugin, public IntrospectionProvider {
 public:
  std::string name() const override {
    return "container_introspection";
  }

  void configure(const nlohmann::json & config) override {
    auto cfg = ros2_medkit_linux_introspection::parse_introspection_config(config);
    pid_cache_ = std::move(cfg.pid_cache);
    proc_root_ = std::move(cfg.proc_root);
  }

  void set_context(PluginContext & ctx) override {
    ctx_ = &ctx;
    ctx.register_capability(SovdEntityType::APP, "x-medkit-container");
    ctx.register_capability(SovdEntityType::COMPONENT, "x-medkit-container");
  }

  std::vector<PluginRoute> get_routes() override {
    return {
        {"GET", R"(apps/([^/]+)/x-medkit-container)",
         [this](const PluginRequest & req, PluginResponse & res) {
           handle_app_request(req, res);
         }},
        {"GET", R"(components/([^/]+)/x-medkit-container)",
         [this](const PluginRequest & req, PluginResponse & res) {
           handle_component_request(req, res);
         }},
    };
  }

  IntrospectionResult introspect(const IntrospectionInput & input) override {
    IntrospectionResult result;
    pid_cache_->refresh(proc_root_);

    for (const auto & app : input.apps) {
      auto fqn = app.effective_fqn();
      if (fqn.empty()) {
        continue;
      }

      auto pid_opt = pid_cache_->lookup(fqn, proc_root_);
      if (!pid_opt) {
        continue;
      }

      auto cgroup_info = ros2_medkit_linux_introspection::read_cgroup_info(*pid_opt, proc_root_);
      if (!cgroup_info || !cgroup_info->containerized) {
        continue;
      }

      result.metadata[app.id] = ros2_medkit_linux_introspection::cgroup_info_to_json(*cgroup_info);
    }
    return result;
  }

 private:
  PluginContext * ctx_{nullptr};
  std::unique_ptr<ros2_medkit_linux_introspection::PidCache> pid_cache_ =
      std::make_unique<ros2_medkit_linux_introspection::PidCache>();
  std::string proc_root_{"/"};

  void handle_app_request(const PluginRequest & req, PluginResponse & res) {
    auto entity_id = req.path_param(1);
    auto entity = ctx_->validate_entity_for_route(req, res, entity_id);
    if (!entity) {
      return;
    }

    auto pid_opt = pid_cache_->lookup(entity->fqn, proc_root_);
    if (!pid_opt) {
      res.send_error(404, "x-medkit-pid-lookup-failed", "Process not found for entity " + entity_id);
      return;
    }

    auto cgroup_info = ros2_medkit_linux_introspection::read_cgroup_info(*pid_opt, proc_root_);
    if (!cgroup_info) {
      res.send_error(503, "x-medkit-cgroup-read-failed", "Failed to read cgroup information for entity " + entity_id);
      return;
    }

    if (!cgroup_info->containerized) {
      res.send_error(404, "x-medkit-not-containerized", "Entity " + entity_id + " is not running in a container");
      return;
    }

    res.send_json(ros2_medkit_linux_introspection::cgroup_info_to_json(*cgroup_info));
  }

  void handle_component_request(const PluginRequest & req, PluginResponse & res) {
    auto entity_id = req.path_param(1);
    auto entity = ctx_->validate_entity_for_route(req, res, entity_id);
    if (!entity) {
      return;
    }

    auto child_apps = ctx_->get_child_apps(entity_id);
    // Deduplicate by container_id. Under cgroupns=private every app reports an
    // empty id and they all collapse into one entry, which is what they are:
    // the single container whose id the namespace hides.
    std::map<std::string, nlohmann::json> containers;

    for (const auto & app : child_apps) {
      auto pid_opt = pid_cache_->lookup(app.fqn, proc_root_);
      if (!pid_opt) {
        continue;
      }

      auto cgroup_info = ros2_medkit_linux_introspection::read_cgroup_info(*pid_opt, proc_root_);
      if (!cgroup_info || !cgroup_info->containerized) {
        continue;
      }

      auto & cid = cgroup_info->container_id;
      if (containers.find(cid) == containers.end()) {
        auto j = ros2_medkit_linux_introspection::cgroup_info_to_json(*cgroup_info);
        j["node_ids"] = nlohmann::json::array();
        containers[cid] = std::move(j);
      }
      containers[cid]["node_ids"].push_back(app.id);
    }

    nlohmann::json result;
    result["containers"] = nlohmann::json::array();
    for (auto & [_, container_json] : containers) {
      result["containers"].push_back(std::move(container_json));
    }
    res.send_json(result);
  }
};

extern "C" GATEWAY_PLUGIN_EXPORT int plugin_api_version() {
  return PLUGIN_API_VERSION;
}
extern "C" GATEWAY_PLUGIN_EXPORT GatewayPlugin * create_plugin() {
  return new ContainerPlugin();
}
extern "C" GATEWAY_PLUGIN_EXPORT IntrospectionProvider * get_introspection_provider(GatewayPlugin * p) {
  return static_cast<ContainerPlugin *>(p);
}
