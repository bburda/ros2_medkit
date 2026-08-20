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

#include <gtest/gtest.h>

#include <map>
#include <memory>
#include <nlohmann/json.hpp>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <unordered_map>
#include <vector>

#include "ros2_medkit_gateway/core/discovery/models/app.hpp"
#include "ros2_medkit_gateway/core/discovery/models/area.hpp"
#include "ros2_medkit_gateway/core/discovery/models/component.hpp"
#include "ros2_medkit_gateway/core/discovery/models/function.hpp"
#include "ros2_medkit_gateway/core/plugins/plugin_http_types.hpp"
#include "ros2_medkit_gateway/plugins/ros_plugin_context.hpp"
#include "sovd_service_interface.hpp"

using namespace ros2_medkit_gateway;

// Stubs for PluginRequest/PluginResponse (defined in gateway_core, not linked into plugin tests)
namespace ros2_medkit_gateway {
PluginRequest::PluginRequest(const void * impl) : impl_(impl) {
}
std::string PluginRequest::path_param(size_t /*index*/) const {
  return {};
}
std::string PluginRequest::header(const std::string & /*name*/) const {
  return {};
}
const std::string & PluginRequest::path() const {
  static const std::string empty;
  return empty;
}
const std::string & PluginRequest::body() const {
  static const std::string empty;
  return empty;
}
PluginResponse::PluginResponse(void * impl) : impl_(impl) {
}
void PluginResponse::send_json(const nlohmann::json & /*data*/) {
}
void PluginResponse::send_error(int /*status*/, const std::string & /*error_code*/, const std::string & /*message*/,
                                const nlohmann::json & /*parameters*/) {
}
}  // namespace ros2_medkit_gateway

namespace {

class FakePluginContext : public RosPluginContext {
 public:
  explicit FakePluginContext(rclcpp::Node * node) : node_(node) {
  }

  rclcpp::Node * node() const override {
    return node_;
  }

  std::optional<PluginEntityInfo> get_entity(const std::string & id) const override {
    auto it = entities_.find(id);
    if (it == entities_.end()) {
      return std::nullopt;
    }
    return it->second;
  }

  nlohmann::json list_entity_faults(const std::string & entity_id) const override {
    auto it = faults_.find(entity_id);
    if (it == faults_.end()) {
      return nlohmann::json::array();
    }
    return it->second;
  }

  std::optional<PluginEntityInfo> validate_entity_for_route(const PluginRequest & /*req*/, PluginResponse & res,
                                                            const std::string & entity_id) const override {
    auto entity = get_entity(entity_id);
    if (!entity) {
      res.send_error(404, "entity-not-found", "Entity not found");
      return std::nullopt;
    }
    return entity;
  }

  void register_capability(SovdEntityType entity_type, const std::string & capability_name) override {
    type_capabilities_[entity_type].push_back(capability_name);
  }

  void register_entity_capability(const std::string & entity_id, const std::string & capability_name) override {
    entity_capabilities_[entity_id].push_back(capability_name);
  }

  std::vector<std::string> get_type_capabilities(SovdEntityType entity_type) const override {
    auto it = type_capabilities_.find(entity_type);
    if (it == type_capabilities_.end()) {
      return {};
    }
    return it->second;
  }

  std::vector<std::string> get_entity_capabilities(const std::string & entity_id) const override {
    auto it = entity_capabilities_.find(entity_id);
    if (it == entity_capabilities_.end()) {
      return {};
    }
    return it->second;
  }

  std::vector<PluginEntityInfo> get_child_apps(const std::string & /*component_id*/) const override {
    return {};
  }

  LockAccessResult check_lock(const std::string & /*entity_id*/, const std::string & /*client_id*/,
                              const std::string & /*collection*/) const override {
    return LockAccessResult{true, "", "", ""};
  }

  tl::expected<LockInfo, LockError> acquire_lock(const std::string & /*entity_id*/, const std::string & /*client_id*/,
                                                 const std::vector<std::string> & /*scopes*/,
                                                 int /*expiration_seconds*/) override {
    return tl::make_unexpected(LockError{"lock-disabled", "Locking not available in test", 503, std::nullopt});
  }

  tl::expected<void, LockError> release_lock(const std::string & /*entity_id*/,
                                             const std::string & /*client_id*/) override {
    return tl::make_unexpected(LockError{"lock-disabled", "Locking not available in test", 503, std::nullopt});
  }

  IntrospectionInput get_entity_snapshot() const override {
    return snapshot_;
  }

  nlohmann::json list_all_faults() const override {
    return nlohmann::json::object();
  }

  void register_sampler(
      const std::string & /*collection*/,
      const std::function<tl::expected<nlohmann::json, std::string>(const std::string &, const std::string &)> &
      /*fn*/) override {
  }

  ResourceChangeNotifier * get_resource_change_notifier() override {
    return nullptr;
  }

  ConditionRegistry * get_condition_registry() override {
    return nullptr;
  }

  // Test helpers
  void add_entity(const std::string & id, SovdEntityType type, const std::string & fqn = "") {
    entities_[id] = PluginEntityInfo{type, id, "", fqn};
  }

  void set_entity_faults(const std::string & entity_id, const nlohmann::json & faults) {
    faults_[entity_id] = faults;
  }

  IntrospectionInput snapshot_;

 private:
  rclcpp::Node * node_{nullptr};
  std::unordered_map<std::string, PluginEntityInfo> entities_;
  std::unordered_map<std::string, nlohmann::json> faults_;
  std::map<SovdEntityType, std::vector<std::string>> type_capabilities_;
  std::unordered_map<std::string, std::vector<std::string>> entity_capabilities_;
};

class RclcppEnvironment : public ::testing::Environment {
 public:
  void SetUp() override {
    rclcpp::init(0, nullptr);
  }
  void TearDown() override {
    rclcpp::shutdown();
  }
};

testing::Environment * const rclcpp_env = testing::AddGlobalTestEnvironment(new RclcppEnvironment);

class SovdServiceInterfaceTest : public ::testing::Test {
 protected:
  void SetUp() override {
    node_ = std::make_shared<rclcpp::Node>("test_sovd_service_interface");
    context_ = std::make_unique<FakePluginContext>(node_.get());

    // Set up test entity tree
    Area area;
    area.id = "powertrain";
    area.name = "Powertrain System";

    Component comp;
    comp.id = "engine";
    comp.name = "Engine ECU";
    comp.area = "powertrain";
    comp.fqn = "/powertrain/engine";

    App app1;
    app1.id = "temp_sensor";
    app1.name = "Temperature Sensor";
    app1.component_id = "engine";

    App app2;
    app2.id = "rpm_sensor";
    app2.name = "RPM Sensor";
    app2.component_id = "engine";

    context_->snapshot_.areas = {area};
    context_->snapshot_.components = {comp};
    context_->snapshot_.apps = {app1, app2};

    context_->add_entity("powertrain", SovdEntityType::AREA);
    context_->add_entity("engine", SovdEntityType::COMPONENT, "/powertrain/engine");
    context_->add_entity("temp_sensor", SovdEntityType::APP);
    context_->add_entity("rpm_sensor", SovdEntityType::APP);

    // Create and configure plugin
    plugin_ = std::make_unique<SovdServiceInterface>();
    nlohmann::json config = {{"service_prefix", "/test_medkit"}};
    plugin_->configure(config);
    plugin_->set_context(*context_);
  }

  void TearDown() override {
    plugin_->shutdown();
    plugin_.reset();
    context_.reset();
    node_.reset();
  }

  std::shared_ptr<rclcpp::Node> node_;
  std::unique_ptr<FakePluginContext> context_;
  std::unique_ptr<SovdServiceInterface> plugin_;
};

TEST_F(SovdServiceInterfaceTest, ListAllEntities) {
  auto client = node_->create_client<ros2_medkit_msgs::srv::ListEntities>("/test_medkit/list_entities");

  auto request = std::make_shared<ros2_medkit_msgs::srv::ListEntities::Request>();
  // Empty filters = return all
  request->entity_type = "";
  request->parent_id = "";

  auto future = client->async_send_request(request);
  auto spin_result = rclcpp::spin_until_future_complete(node_, future, std::chrono::seconds(5));
  ASSERT_EQ(spin_result, rclcpp::FutureReturnCode::SUCCESS);
  auto response = future.get();

  ASSERT_TRUE(response->success);
  EXPECT_EQ(response->entities.size(), 4u);  // 1 area + 1 component + 2 apps
}

TEST_F(SovdServiceInterfaceTest, ListEntitiesFilterByType) {
  auto client = node_->create_client<ros2_medkit_msgs::srv::ListEntities>("/test_medkit/list_entities");

  auto request = std::make_shared<ros2_medkit_msgs::srv::ListEntities::Request>();
  request->entity_type = "app";
  request->parent_id = "";

  auto future = client->async_send_request(request);
  auto spin_result = rclcpp::spin_until_future_complete(node_, future, std::chrono::seconds(5));
  ASSERT_EQ(spin_result, rclcpp::FutureReturnCode::SUCCESS);
  auto response = future.get();

  ASSERT_TRUE(response->success);
  EXPECT_EQ(response->entities.size(), 2u);  // 2 apps
  EXPECT_EQ(response->entities[0].entity_type, "app");
  EXPECT_EQ(response->entities[1].entity_type, "app");
}

TEST_F(SovdServiceInterfaceTest, ListEntitiesFilterByParent) {
  auto client = node_->create_client<ros2_medkit_msgs::srv::ListEntities>("/test_medkit/list_entities");

  auto request = std::make_shared<ros2_medkit_msgs::srv::ListEntities::Request>();
  request->entity_type = "";
  request->parent_id = "engine";

  auto future = client->async_send_request(request);
  auto spin_result = rclcpp::spin_until_future_complete(node_, future, std::chrono::seconds(5));
  ASSERT_EQ(spin_result, rclcpp::FutureReturnCode::SUCCESS);
  auto response = future.get();

  ASSERT_TRUE(response->success);
  EXPECT_EQ(response->entities.size(), 2u);  // 2 apps under engine component
}

TEST_F(SovdServiceInterfaceTest, ListEntitiesFilterByTypeAndParent) {
  auto client = node_->create_client<ros2_medkit_msgs::srv::ListEntities>("/test_medkit/list_entities");

  auto request = std::make_shared<ros2_medkit_msgs::srv::ListEntities::Request>();
  request->entity_type = "app";
  request->parent_id = "engine";

  auto future = client->async_send_request(request);
  auto spin_result = rclcpp::spin_until_future_complete(node_, future, std::chrono::seconds(5));
  ASSERT_EQ(spin_result, rclcpp::FutureReturnCode::SUCCESS);
  auto response = future.get();

  ASSERT_TRUE(response->success);
  EXPECT_EQ(response->entities.size(), 2u);  // 2 apps under engine, both type "app"
  for (const auto & e : response->entities) {
    EXPECT_EQ(e.entity_type, "app");
    EXPECT_EQ(e.parent_id, "engine");
  }
}

TEST_F(SovdServiceInterfaceTest, ListEntitiesFilterByTypeAndParentNoMatch) {
  auto client = node_->create_client<ros2_medkit_msgs::srv::ListEntities>("/test_medkit/list_entities");

  auto request = std::make_shared<ros2_medkit_msgs::srv::ListEntities::Request>();
  request->entity_type = "component";
  request->parent_id = "engine";  // no components under engine

  auto future = client->async_send_request(request);
  auto spin_result = rclcpp::spin_until_future_complete(node_, future, std::chrono::seconds(5));
  ASSERT_EQ(spin_result, rclcpp::FutureReturnCode::SUCCESS);
  auto response = future.get();

  ASSERT_TRUE(response->success);
  EXPECT_TRUE(response->entities.empty());
}

TEST_F(SovdServiceInterfaceTest, ListEntitiesEmpty) {
  // Clear snapshot
  context_->snapshot_ = IntrospectionInput{};

  auto client = node_->create_client<ros2_medkit_msgs::srv::ListEntities>("/test_medkit/list_entities");

  auto request = std::make_shared<ros2_medkit_msgs::srv::ListEntities::Request>();
  auto future = client->async_send_request(request);
  auto spin_result = rclcpp::spin_until_future_complete(node_, future, std::chrono::seconds(5));
  ASSERT_EQ(spin_result, rclcpp::FutureReturnCode::SUCCESS);
  auto response = future.get();

  ASSERT_TRUE(response->success);
  EXPECT_TRUE(response->entities.empty());
}

TEST_F(SovdServiceInterfaceTest, GetCapabilitiesServerLevel) {
  auto client = node_->create_client<ros2_medkit_msgs::srv::GetCapabilities>("/test_medkit/get_capabilities");

  auto request = std::make_shared<ros2_medkit_msgs::srv::GetCapabilities::Request>();
  request->entity_id = "";  // Server-level

  auto future = client->async_send_request(request);
  auto spin_result = rclcpp::spin_until_future_complete(node_, future, std::chrono::seconds(5));
  ASSERT_EQ(spin_result, rclcpp::FutureReturnCode::SUCCESS);
  auto response = future.get();

  ASSERT_TRUE(response->success);
  EXPECT_FALSE(response->capabilities.empty());
  EXPECT_FALSE(response->resource_types.empty());
}

TEST_F(SovdServiceInterfaceTest, GetCapabilitiesEntityLevel) {
  // Register entity-specific capability
  context_->register_entity_capability("engine", "x-medkit-traces");
  context_->register_entity_capability("engine", "x-medkit-config");

  auto client = node_->create_client<ros2_medkit_msgs::srv::GetCapabilities>("/test_medkit/get_capabilities");

  auto request = std::make_shared<ros2_medkit_msgs::srv::GetCapabilities::Request>();
  request->entity_id = "engine";

  auto future = client->async_send_request(request);
  auto result = rclcpp::spin_until_future_complete(node_, future, std::chrono::seconds(5));
  ASSERT_EQ(result, rclcpp::FutureReturnCode::SUCCESS);
  auto response = future.get();

  ASSERT_TRUE(response->success);
  ASSERT_EQ(response->capabilities.size(), 2u);
  EXPECT_EQ(response->capabilities[0], "x-medkit-traces");
  EXPECT_EQ(response->capabilities[1], "x-medkit-config");
}

TEST_F(SovdServiceInterfaceTest, GetCapabilitiesTypeFallback) {
  // Register type-level capability (no entity-specific ones)
  context_->register_capability(SovdEntityType::COMPONENT, "faults");
  context_->register_capability(SovdEntityType::COMPONENT, "data");

  auto client = node_->create_client<ros2_medkit_msgs::srv::GetCapabilities>("/test_medkit/get_capabilities");

  auto request = std::make_shared<ros2_medkit_msgs::srv::GetCapabilities::Request>();
  request->entity_id = "engine";

  auto future = client->async_send_request(request);
  auto result = rclcpp::spin_until_future_complete(node_, future, std::chrono::seconds(5));
  ASSERT_EQ(result, rclcpp::FutureReturnCode::SUCCESS);
  auto response = future.get();

  ASSERT_TRUE(response->success);
  ASSERT_EQ(response->capabilities.size(), 2u);
  EXPECT_EQ(response->capabilities[0], "faults");
  EXPECT_EQ(response->capabilities[1], "data");
}

TEST_F(SovdServiceInterfaceTest, GetCapabilitiesEntityNotFound) {
  auto client = node_->create_client<ros2_medkit_msgs::srv::GetCapabilities>("/test_medkit/get_capabilities");

  auto request = std::make_shared<ros2_medkit_msgs::srv::GetCapabilities::Request>();
  request->entity_id = "nonexistent";

  auto future = client->async_send_request(request);
  auto spin_result = rclcpp::spin_until_future_complete(node_, future, std::chrono::seconds(5));
  ASSERT_EQ(spin_result, rclcpp::FutureReturnCode::SUCCESS);
  auto response = future.get();

  ASSERT_FALSE(response->success);
  EXPECT_FALSE(response->error_message.empty());
}

TEST_F(SovdServiceInterfaceTest, ListEntityFaults) {
  // Set up test faults
  nlohmann::json faults = nlohmann::json::array();
  faults.push_back({{"fault_code", "MOTOR_OVERHEAT"},
                    {"severity", 2},
                    {"description", "Motor temperature exceeds threshold"},
                    {"status", "CONFIRMED"},
                    {"occurrence_count", 3},
                    {"reporting_sources", {"temp_sensor"}}});
  context_->set_entity_faults("temp_sensor", faults);

  auto client = node_->create_client<ros2_medkit_msgs::srv::ListFaultsForEntity>("/test_medkit/list_entity_faults");

  auto request = std::make_shared<ros2_medkit_msgs::srv::ListFaultsForEntity::Request>();
  request->entity_id = "temp_sensor";

  auto future = client->async_send_request(request);
  auto spin_result = rclcpp::spin_until_future_complete(node_, future, std::chrono::seconds(5));
  ASSERT_EQ(spin_result, rclcpp::FutureReturnCode::SUCCESS);
  auto response = future.get();

  ASSERT_TRUE(response->success);
  ASSERT_EQ(response->faults.size(), 1u);
  EXPECT_EQ(response->faults[0].fault_code, "MOTOR_OVERHEAT");
  EXPECT_EQ(response->faults[0].severity, 2u);
  EXPECT_EQ(response->faults[0].status, "CONFIRMED");
  EXPECT_EQ(response->faults[0].occurrence_count, 3u);
}

TEST_F(SovdServiceInterfaceTest, ListEntityFaultsNotFound) {
  auto client = node_->create_client<ros2_medkit_msgs::srv::ListFaultsForEntity>("/test_medkit/list_entity_faults");

  auto request = std::make_shared<ros2_medkit_msgs::srv::ListFaultsForEntity::Request>();
  request->entity_id = "nonexistent";

  auto future = client->async_send_request(request);
  auto spin_result = rclcpp::spin_until_future_complete(node_, future, std::chrono::seconds(5));
  ASSERT_EQ(spin_result, rclcpp::FutureReturnCode::SUCCESS);
  auto response = future.get();

  ASSERT_FALSE(response->success);
  EXPECT_FALSE(response->error_message.empty());
}

TEST_F(SovdServiceInterfaceTest, GetEntityDataNotImplemented) {
  auto client = node_->create_client<ros2_medkit_msgs::srv::GetEntityData>("/test_medkit/get_entity_data");

  auto request = std::make_shared<ros2_medkit_msgs::srv::GetEntityData::Request>();
  request->entity_id = "temp_sensor";

  auto future = client->async_send_request(request);
  auto result = rclcpp::spin_until_future_complete(node_, future, std::chrono::seconds(5));
  ASSERT_EQ(result, rclcpp::FutureReturnCode::SUCCESS);
  auto response = future.get();

  // GetEntityData is not yet implemented - returns explicit failure
  ASSERT_FALSE(response->success);
  EXPECT_FALSE(response->error_message.empty());
  // data_json should still be valid JSON (empty object)
  auto parsed = nlohmann::json::parse(response->data_json, nullptr, false);
  EXPECT_FALSE(parsed.is_discarded());
}

TEST_F(SovdServiceInterfaceTest, GetEntityDataEntityNotFound) {
  auto client = node_->create_client<ros2_medkit_msgs::srv::GetEntityData>("/test_medkit/get_entity_data");

  auto request = std::make_shared<ros2_medkit_msgs::srv::GetEntityData::Request>();
  request->entity_id = "nonexistent";

  auto future = client->async_send_request(request);
  auto result = rclcpp::spin_until_future_complete(node_, future, std::chrono::seconds(5));
  ASSERT_EQ(result, rclcpp::FutureReturnCode::SUCCESS);
  auto response = future.get();

  ASSERT_FALSE(response->success);
  EXPECT_FALSE(response->error_message.empty());
}

TEST_F(SovdServiceInterfaceTest, PluginName) {
  EXPECT_EQ(plugin_->name(), "sovd_service_interface");
}

TEST_F(SovdServiceInterfaceTest, ServiceCallAfterShutdownReturnsFailure) {
  plugin_->shutdown();

  // Create client and call list_entities after shutdown
  auto client = node_->create_client<ros2_medkit_msgs::srv::ListEntities>("/test_medkit/list_entities");

  // Service should no longer be available after shutdown (services are reset)
  bool available = client->wait_for_service(std::chrono::milliseconds(100));
  EXPECT_FALSE(available) << "Service should not be available after shutdown";
}

}  // namespace
