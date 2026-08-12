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

// Minimal topic-backed test plugin for the issue #584 integration test.
// Registers one entity whose data point is backed by a topic the plugin
// itself publishes (the PLC-bridge pattern in miniature): entity
// `plugin_dev`, data point `value`, topic `/plugin_dev/value` declared via
// App::topics and published at 5 Hz with an incrementing value. Serves the
// enumeration route (x-plc-data) the gateway uses for create-time trigger
// validation and the /data fallback.

#include <atomic>
#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <ros2_medkit_gateway/core/plugins/gateway_plugin.hpp>
#include <ros2_medkit_gateway/core/providers/introspection_provider.hpp>
#include <ros2_medkit_gateway/plugins/ros_plugin_context.hpp>
#include <std_msgs/msg/float64.hpp>

namespace ros2_medkit_integration_tests {

using namespace ros2_medkit_gateway;

class TopicsTestPlugin : public GatewayPlugin, public IntrospectionProvider {
 public:
  ~TopicsTestPlugin() override {
    stop();
  }

  std::string name() const override {
    return "topics_test";
  }

  void configure(const nlohmann::json & /*config*/) override {
  }

  void set_context(PluginContext & context) override {
    auto * ctx = as_ros_plugin_context(context);
    auto * node = ctx != nullptr ? ctx->node() : nullptr;
    if (node == nullptr) {
      return;
    }
    publisher_ = node->create_publisher<std_msgs::msg::Float64>("/plugin_dev/value", rclcpp::SensorDataQoS());
    running_.store(true);
    publish_thread_ = std::thread([this] {
      while (running_.load()) {
        std_msgs::msg::Float64 msg;
        msg.data = static_cast<double>(counter_.fetch_add(1));
        publisher_->publish(msg);
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
      }
    });
  }

  std::vector<PluginRoute> get_routes() override {
    return {
        {"GET", R"(apps/([^/]+)/x-plc-data)",
         [this](const PluginRequest & /*req*/, PluginResponse & res) {
           res.send_json({{"connected", true},
                          {"items", nlohmann::json::array(
                                        {{{"name", "value"}, {"value", static_cast<double>(counter_.load())}}})}});
         }},
    };
  }

  IntrospectionResult introspect(const IntrospectionInput & /*input*/) override {
    IntrospectionResult result;

    Area area;
    area.id = "test_plugin_area";
    area.name = "Test Plugin Area";
    result.new_entities.areas.push_back(std::move(area));

    Component comp;
    comp.id = "test_plugin_comp";
    comp.name = "Test Plugin Component";
    comp.area = "test_plugin_area";
    comp.external = true;
    result.new_entities.components.push_back(std::move(comp));

    App app;
    app.id = "plugin_dev";
    app.name = "Plugin Device";
    app.component_id = "test_plugin_comp";
    app.external = true;
    app.is_online = true;
    app.source = "plugin";
    // The point of this plugin: the data point's backing topic is declared on
    // the owning entity, so entity-scoped topic lookup (trigger resolution)
    // can see it (issue #584).
    app.topics.publishes.push_back("/plugin_dev/value");
    result.new_entities.apps.push_back(std::move(app));

    // A second app carrying a source tag the protection whitelist does NOT
    // list. Providers pick their own tag (the beacon mapper stamps "beacon"),
    // and protection must follow the owning LAYER rather than the string, so
    // this app has to survive unmanifested_nodes=ignore exactly as the one
    // above does.
    App unlisted;
    unlisted.id = "plugin_unlisted_source";
    unlisted.name = "Plugin Device With Unlisted Source";
    unlisted.component_id = "test_plugin_comp";
    unlisted.external = true;
    unlisted.is_online = true;
    unlisted.source = "beacon";
    result.new_entities.apps.push_back(std::move(unlisted));

    return result;
  }

  void shutdown() override {
    stop();
  }

 private:
  void stop() {
    if (running_.exchange(false) && publish_thread_.joinable()) {
      publish_thread_.join();
    }
  }

  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_;
  std::thread publish_thread_;
  std::atomic<bool> running_{false};
  std::atomic<uint64_t> counter_{0};
};

}  // namespace ros2_medkit_integration_tests

extern "C" GATEWAY_PLUGIN_EXPORT int plugin_api_version() {
  return ros2_medkit_gateway::PLUGIN_API_VERSION;
}

extern "C" GATEWAY_PLUGIN_EXPORT ros2_medkit_gateway::GatewayPlugin * create_plugin() {
  return new ros2_medkit_integration_tests::TopicsTestPlugin();
}

extern "C" GATEWAY_PLUGIN_EXPORT ros2_medkit_gateway::IntrospectionProvider *
get_introspection_provider(ros2_medkit_gateway::GatewayPlugin * plugin) {
  return static_cast<ros2_medkit_integration_tests::TopicsTestPlugin *>(plugin);
}
