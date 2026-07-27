// Copyright 2026 mfaferek93
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

#include <nlohmann/json.hpp>
#include <string>
#include <vector>

#include "ros2_medkit_gateway/core/http/error_codes.hpp"
#include "ros2_medkit_gateway/core/plugins/gateway_plugin.hpp"
#include "ros2_medkit_gateway/core/plugins/plugin_context.hpp"
#include "ros2_medkit_gateway/core/plugins/plugin_types.hpp"
#include "ros2_medkit_gateway/core/providers/introspection_provider.hpp"

using namespace ros2_medkit_gateway;

/**
 * @brief Demo plugin mirroring the commercial PLC bridge shape (.so for tests).
 *
 * Deliberately exports ONLY create_plugin + get_introspection_provider - no
 * DataProvider, no FaultProvider - like the S7comm bridge. Live values are
 * served exclusively through a registered `apps/<id>/x-plc-data` route
 * (poller-snapshot style response with `connected` + `items[]`); faults are
 * reported to the fault_manager via ReportFault only. Used by
 * test_entity_freeze_frame.test.py to prove the gateway's in-process
 * route-dispatch fallback captures a zero-config freeze-frame for such
 * plugins, and that their faults are served through the fault_manager
 * fall-through despite the missing FaultProvider.
 */
class TestRouteDataPlugin : public GatewayPlugin, public IntrospectionProvider {
 public:
  static constexpr const char * kAppId = "test_route_plc_app";
  /// Loss-of-comms twin: reports connected=false while serving last known values.
  static constexpr const char * kDownAppId = "test_route_plc_down_app";

  std::string name() const override {
    return "test_route_data";
  }

  void configure(const nlohmann::json & /*config*/) override {
  }

  void set_context(PluginContext & context) override {
    ctx_ = &context;
  }

  std::vector<PluginRoute> get_routes() override {
    return {
        {"GET", R"(apps/([^/]+)/x-plc-data)",
         [this](const PluginRequest & req, PluginResponse & res) {
           handle_plc_data(req, res);
         }},
    };
  }

  // --- IntrospectionProvider ---
  IntrospectionResult introspect(const IntrospectionInput & /*input*/) override {
    IntrospectionResult result;

    Area area;
    area.id = "test_route_plc_area";
    area.name = "Test Route PLC Cell";
    area.namespace_path = "/test_route_plc_area";
    area.source = "plugin";
    result.new_entities.areas.push_back(std::move(area));

    Component comp;
    comp.id = "test_route_plc";
    comp.name = "Test Route PLC Runtime";
    comp.namespace_path = "/test_route_plc_area";
    comp.fqn = "/test_route_plc_area/test_route_plc";
    comp.area = "test_route_plc_area";
    comp.source = "plugin";
    result.new_entities.components.push_back(std::move(comp));

    App app;
    app.id = kAppId;
    app.name = "Test Route PLC Process";
    app.component_id = "test_route_plc";
    app.external = true;  // faults report under the bare app id, not a ROS FQN
    app.is_online = true;
    app.source = "plugin";
    result.new_entities.apps.push_back(std::move(app));

    App down_app;
    down_app.id = kDownAppId;
    down_app.name = "Test Route PLC Down Process";
    down_app.component_id = "test_route_plc";
    down_app.external = true;
    down_app.is_online = true;
    down_app.source = "plugin";
    result.new_entities.apps.push_back(std::move(down_app));

    return result;
  }

 private:
  void handle_plc_data(const PluginRequest & req, PluginResponse & res) {
    auto entity_id = req.path_param(1);
    if (ctx_ != nullptr) {
      // Same gate as the real PLC bridges: entity must exist and match the
      // route's entity type (exercises the synthesized request's path()).
      auto entity = ctx_->validate_entity_for_route(req, res, entity_id);
      if (!entity) {
        return;
      }
    }
    if (entity_id == kDownAppId) {
      // Loss-of-comms shape: link down, last known values still served.
      res.send_json({{"connected", false},
                     {"entity_id", entity_id},
                     {"items", nlohmann::json::array({{{"name", "level"}, {"value", 42.0}, {"unit", "mm"}},
                                                      {{"name", "alarm"}, {"value", false}}})},
                     {"timestamp", 1234567800}});
      return;
    }
    if (entity_id != kAppId) {
      res.send_error(404, ERR_RESOURCE_NOT_FOUND, "No PLC data mapped for entity: " + entity_id);
      return;
    }
    // Poller-snapshot style response, canned values (S7/ADS bridge shape).
    res.send_json({{"connected", true},
                   {"entity_id", entity_id},
                   {"items", nlohmann::json::array({{{"name", "level"}, {"value", 87.5}, {"unit", "mm"}},
                                                    {{"name", "alarm"}, {"value", true}}})},
                   {"timestamp", 1234567890}});
  }

  PluginContext * ctx_{nullptr};
};

// --- Plugin exports (intentionally ONLY the two below - no get_data_provider,
// no get_fault_provider - matching the leanest commercial bridge) ---

extern "C" GATEWAY_PLUGIN_EXPORT int plugin_api_version() {
  return PLUGIN_API_VERSION;
}

extern "C" GATEWAY_PLUGIN_EXPORT GatewayPlugin * create_plugin() {
  return new TestRouteDataPlugin();
}

extern "C" GATEWAY_PLUGIN_EXPORT IntrospectionProvider * get_introspection_provider(GatewayPlugin * plugin) {
  return static_cast<TestRouteDataPlugin *>(plugin);
}
