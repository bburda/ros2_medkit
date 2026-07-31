// Copyright 2025 bburda
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

#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "ros2_medkit_gateway/core/managers/operation_manager.hpp"
#include "ros2_medkit_gateway/discovery/discovery_manager.hpp"
#include "ros2_medkit_gateway/ros2/transports/ros2_action_transport.hpp"
#include "ros2_medkit_gateway/ros2/transports/ros2_service_transport.hpp"
#include "ros2_medkit_gateway/ros2_common/callback_groups.hpp"

using namespace ros2_medkit_gateway;

class TestOperationManager : public ::testing::Test {
 protected:
  static void SetUpTestSuite() {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestSuite() {
    rclcpp::shutdown();
  }

  void SetUp() override {
    // Use short timeout for tests to avoid long waits on nonexistent services.
    node_ = std::make_shared<rclcpp::Node>("test_operation_manager_node");
    discovery_manager_ = std::make_unique<DiscoveryManager>(node_.get());
    auto groups = ros2_common::create_gateway_callback_groups(*node_);
    service_transport_ = std::make_shared<ros2::Ros2ServiceTransport>(node_.get(), groups.rpc_reentrant);
    action_transport_ =
        std::make_shared<ros2::Ros2ActionTransport>(node_.get(), groups.rpc_reentrant, groups.action_status);
    operation_manager_ = std::make_unique<OperationManager>(service_transport_, action_transport_,
                                                            discovery_manager_.get(), /*timeout=*/1);
  }

  void TearDown() override {
    operation_manager_.reset();
    discovery_manager_.reset();
    service_transport_.reset();
    action_transport_.reset();
    node_.reset();
  }

  std::shared_ptr<rclcpp::Node> node_;
  std::unique_ptr<DiscoveryManager> discovery_manager_;
  std::shared_ptr<ros2::Ros2ServiceTransport> service_transport_;
  std::shared_ptr<ros2::Ros2ActionTransport> action_transport_;
  std::unique_ptr<OperationManager> operation_manager_;
};

// ==================== TYPE VALIDATION TESTS ====================

TEST_F(TestOperationManager, test_is_valid_message_type_service) {
  // Valid service types
  EXPECT_TRUE(OperationManager::is_valid_message_type("std_srvs/srv/Trigger"));
  EXPECT_TRUE(OperationManager::is_valid_message_type("example_interfaces/srv/AddTwoInts"));
  EXPECT_TRUE(OperationManager::is_valid_message_type("my_package/srv/MyService"));
}

TEST_F(TestOperationManager, test_is_valid_message_type_action) {
  // Valid action types
  EXPECT_TRUE(OperationManager::is_valid_message_type("example_interfaces/action/Fibonacci"));
  EXPECT_TRUE(OperationManager::is_valid_message_type("nav2_msgs/action/NavigateToPose"));
  EXPECT_TRUE(OperationManager::is_valid_message_type("my_package/action/MyAction"));
}

TEST_F(TestOperationManager, test_is_valid_message_type_message) {
  // Valid message types
  EXPECT_TRUE(OperationManager::is_valid_message_type("std_msgs/msg/String"));
  EXPECT_TRUE(OperationManager::is_valid_message_type("sensor_msgs/msg/Image"));
  EXPECT_TRUE(OperationManager::is_valid_message_type("geometry_msgs/msg/Twist"));
}

TEST_F(TestOperationManager, test_is_valid_message_type_invalid) {
  // Invalid types
  EXPECT_FALSE(OperationManager::is_valid_message_type(""));
  EXPECT_FALSE(OperationManager::is_valid_message_type("invalid"));
  EXPECT_FALSE(OperationManager::is_valid_message_type("package/Type"));
  EXPECT_FALSE(OperationManager::is_valid_message_type("package/invalid/Type"));
  EXPECT_FALSE(OperationManager::is_valid_message_type("/srv/Type"));
  EXPECT_FALSE(OperationManager::is_valid_message_type("123package/srv/Type"));
  EXPECT_FALSE(OperationManager::is_valid_message_type("package/srv/"));
}

TEST_F(TestOperationManager, test_is_service_type) {
  EXPECT_TRUE(OperationManager::is_service_type("std_srvs/srv/Trigger"));
  EXPECT_TRUE(OperationManager::is_service_type("example_interfaces/srv/AddTwoInts"));

  EXPECT_FALSE(OperationManager::is_service_type("example_interfaces/action/Fibonacci"));
  EXPECT_FALSE(OperationManager::is_service_type("std_msgs/msg/String"));
  EXPECT_FALSE(OperationManager::is_service_type(""));
}

TEST_F(TestOperationManager, test_is_action_type) {
  EXPECT_TRUE(OperationManager::is_action_type("example_interfaces/action/Fibonacci"));
  EXPECT_TRUE(OperationManager::is_action_type("nav2_msgs/action/NavigateToPose"));

  EXPECT_FALSE(OperationManager::is_action_type("std_srvs/srv/Trigger"));
  EXPECT_FALSE(OperationManager::is_action_type("std_msgs/msg/String"));
  EXPECT_FALSE(OperationManager::is_action_type(""));
}

// ==================== UUID VALIDATION TESTS ====================

TEST_F(TestOperationManager, test_is_valid_uuid_hex_valid) {
  // Valid 32-character hex strings
  EXPECT_TRUE(OperationManager::is_valid_uuid_hex("00000000000000000000000000000000"));
  EXPECT_TRUE(OperationManager::is_valid_uuid_hex("ffffffffffffffffffffffffffffffff"));
  EXPECT_TRUE(OperationManager::is_valid_uuid_hex("0123456789abcdef0123456789abcdef"));
  EXPECT_TRUE(OperationManager::is_valid_uuid_hex("ABCDEF0123456789ABCDEF0123456789"));
}

TEST_F(TestOperationManager, test_is_valid_uuid_hex_invalid) {
  // Invalid UUIDs
  EXPECT_FALSE(OperationManager::is_valid_uuid_hex(""));
  EXPECT_FALSE(OperationManager::is_valid_uuid_hex("0"));
  EXPECT_FALSE(OperationManager::is_valid_uuid_hex("0000000000000000000000000000000"));     // 31 chars
  EXPECT_FALSE(OperationManager::is_valid_uuid_hex("000000000000000000000000000000000"));   // 33 chars
  EXPECT_FALSE(OperationManager::is_valid_uuid_hex("0000000000000000000000000000000g"));    // invalid char
  EXPECT_FALSE(OperationManager::is_valid_uuid_hex("0000-0000-0000-0000-0000-0000-0000"));  // dashes
}

// ==================== ACTION STATUS CONVERSION TESTS ====================

TEST_F(TestOperationManager, test_action_status_to_string) {
  EXPECT_EQ(action_status_to_string(ActionGoalStatus::UNKNOWN), "unknown");
  EXPECT_EQ(action_status_to_string(ActionGoalStatus::ACCEPTED), "accepted");
  EXPECT_EQ(action_status_to_string(ActionGoalStatus::EXECUTING), "executing");
  EXPECT_EQ(action_status_to_string(ActionGoalStatus::CANCELING), "canceling");
  EXPECT_EQ(action_status_to_string(ActionGoalStatus::SUCCEEDED), "succeeded");
  EXPECT_EQ(action_status_to_string(ActionGoalStatus::CANCELED), "canceled");
  EXPECT_EQ(action_status_to_string(ActionGoalStatus::ABORTED), "aborted");
}

// ==================== GOAL TRACKING TESTS ====================

TEST_F(TestOperationManager, test_list_tracked_goals_empty) {
  auto goals = operation_manager_->list_tracked_goals();
  EXPECT_TRUE(goals.empty());
}

TEST_F(TestOperationManager, test_get_tracked_goal_not_found) {
  auto goal = operation_manager_->get_tracked_goal("nonexistent_goal_id_12345678");
  EXPECT_FALSE(goal.has_value());
}

TEST_F(TestOperationManager, test_get_goals_for_action_empty) {
  auto goals = operation_manager_->get_goals_for_action("/test/action");
  EXPECT_TRUE(goals.empty());
}

TEST_F(TestOperationManager, test_get_latest_goal_for_action_empty) {
  auto goal = operation_manager_->get_latest_goal_for_action("/test/action");
  EXPECT_FALSE(goal.has_value());
}

TEST_F(TestOperationManager, test_cleanup_old_goals_empty) {
  // Should not throw on empty goals
  EXPECT_NO_THROW(operation_manager_->cleanup_old_goals(std::chrono::seconds(0)));
}

// ==================== SERVICE CALL TESTS ====================

TEST_F(TestOperationManager, test_call_service_unavailable) {
  // Call a service that doesn't exist
  auto result = operation_manager_->call_service("/nonexistent/service", "std_srvs/srv/Trigger", nlohmann::json{});

  EXPECT_FALSE(result.success);
  EXPECT_FALSE(result.error_message.empty());
}

TEST_F(TestOperationManager, test_call_component_service_invalid_type) {
  auto result = operation_manager_->call_component_service("/test", "operation", "invalid_type", nlohmann::json{});

  EXPECT_FALSE(result.success);
  EXPECT_TRUE(result.error_message.find("Invalid service type format") != std::string::npos);
}

TEST_F(TestOperationManager, test_call_component_service_wrong_type) {
  // Trying to call a service with an action type
  auto result = operation_manager_->call_component_service("/test", "operation", "example_interfaces/action/Fibonacci",
                                                           nlohmann::json{});

  EXPECT_FALSE(result.success);
  EXPECT_TRUE(result.error_message.find("not a service type") != std::string::npos);
}

// ==================== ACTION CALL TESTS ====================

TEST_F(TestOperationManager, test_send_component_action_goal_invalid_type) {
  auto result = operation_manager_->send_component_action_goal("/test", "operation", "invalid_type", nlohmann::json{});

  EXPECT_FALSE(result.success);
  EXPECT_TRUE(result.error_message.find("Invalid action type format") != std::string::npos);
}

TEST_F(TestOperationManager, test_send_component_action_goal_wrong_type) {
  // Trying to send action goal with a service type
  auto result =
      operation_manager_->send_component_action_goal("/test", "operation", "std_srvs/srv/Trigger", nlohmann::json{});

  EXPECT_FALSE(result.success);
  EXPECT_TRUE(result.error_message.find("not an action type") != std::string::npos);
}

TEST_F(TestOperationManager, test_cancel_action_goal_invalid_uuid) {
  auto result = operation_manager_->cancel_action_goal("/test/action", "invalid_uuid");

  EXPECT_FALSE(result.success);
  EXPECT_TRUE(result.error_message.find("Invalid goal_id format") != std::string::npos);
}

TEST_F(TestOperationManager, test_get_action_result_invalid_uuid) {
  auto result = operation_manager_->get_action_result("/test/action", "example_interfaces/action/Fibonacci", "invalid");

  EXPECT_FALSE(result.success);
  EXPECT_TRUE(result.error_message.find("Invalid goal_id format") != std::string::npos);
}

// ==================== UUID GENERATION AND CONVERSION TESTS ====================

TEST_F(TestOperationManager, test_is_valid_uuid_hex_boundary_cases) {
  // Test boundary - exactly 31 characters
  EXPECT_FALSE(OperationManager::is_valid_uuid_hex("0123456789abcdef0123456789abcde"));
  // Test boundary - exactly 33 characters
  EXPECT_FALSE(OperationManager::is_valid_uuid_hex("0123456789abcdef0123456789abcdef0"));
  // Mixed case should work
  EXPECT_TRUE(OperationManager::is_valid_uuid_hex("0123456789AbCdEf0123456789aBcDeF"));
}

// ==================== ACTION STATUS ADDITIONAL TESTS ====================

TEST_F(TestOperationManager, test_action_status_to_string_all_values) {
  // Test all status values including edge case
  EXPECT_EQ(action_status_to_string(static_cast<ActionGoalStatus>(99)), "unknown");
}

// ==================== GOAL TRACKING LIFECYCLE TESTS ====================

TEST_F(TestOperationManager, test_cleanup_old_goals_with_zero_timeout) {
  // Cleanup with 0 timeout should clean all completed goals immediately
  EXPECT_NO_THROW(operation_manager_->cleanup_old_goals(std::chrono::seconds(0)));
  EXPECT_TRUE(operation_manager_->list_tracked_goals().empty());
}

TEST_F(TestOperationManager, test_cleanup_old_goals_evicts_stuck_non_terminal) {
  // A goal stuck in ACCEPTED/EXECUTING past 2x max_age must be force-evicted
  // to prevent unbounded tracked_goals_ growth when the action server crashes
  // without publishing a terminal status.
  ActionGoalInfo stuck;
  stuck.goal_id = "stuck_goal_id_00000000000000000000";
  stuck.action_path = "/test/stuck_action";
  stuck.action_type = "example_interfaces/action/Fibonacci";
  stuck.entity_id = "test_entity";
  stuck.status = ActionGoalStatus::ACCEPTED;
  // last_update is more than 2x max_age in the past.
  const auto max_age = std::chrono::seconds(10);
  const auto now = std::chrono::system_clock::now();
  stuck.created_at = now - std::chrono::seconds(30);
  stuck.last_update = now - std::chrono::seconds(25);  // > 2 * 10s
  operation_manager_->inject_tracked_goal_for_testing(stuck);

  // A fresh (non-terminal) goal must NOT be evicted by the same call.
  ActionGoalInfo fresh;
  fresh.goal_id = "fresh_goal_id_000000000000000000000";
  fresh.action_path = "/test/fresh_action";
  fresh.action_type = "example_interfaces/action/Fibonacci";
  fresh.entity_id = "test_entity";
  fresh.status = ActionGoalStatus::EXECUTING;
  fresh.created_at = now;
  fresh.last_update = now;
  operation_manager_->inject_tracked_goal_for_testing(fresh);

  ASSERT_EQ(operation_manager_->list_tracked_goals().size(), 2u);

  operation_manager_->cleanup_old_goals(max_age);

  auto remaining = operation_manager_->list_tracked_goals();
  ASSERT_EQ(remaining.size(), 1u);
  EXPECT_EQ(remaining[0].goal_id, fresh.goal_id);
}

TEST_F(TestOperationManager, test_update_goal_status_nonexistent) {
  // Updating a nonexistent goal should not throw
  EXPECT_NO_THROW(operation_manager_->update_goal_status("nonexistent_goal", ActionGoalStatus::SUCCEEDED));
}

TEST_F(TestOperationManager, test_update_goal_feedback_nonexistent) {
  // Updating feedback for a nonexistent goal should not throw
  nlohmann::json feedback = {{"progress", 50}};
  EXPECT_NO_THROW(operation_manager_->update_goal_feedback("nonexistent_goal", feedback));
}

// ==================== SERVICE TYPE VALIDATION ADDITIONAL TESTS ====================

TEST_F(TestOperationManager, test_is_valid_message_type_edge_cases) {
  // Single character package name
  EXPECT_TRUE(OperationManager::is_valid_message_type("a/srv/Type"));
  // Single character type name
  EXPECT_TRUE(OperationManager::is_valid_message_type("pkg/srv/T"));
  // Package with numbers
  EXPECT_TRUE(OperationManager::is_valid_message_type("pkg2/srv/Type"));
  // Type with numbers
  EXPECT_TRUE(OperationManager::is_valid_message_type("pkg/srv/Type2"));
  // Underscore in package
  EXPECT_TRUE(OperationManager::is_valid_message_type("my_pkg/srv/Type"));
  // Underscore in type
  EXPECT_TRUE(OperationManager::is_valid_message_type("pkg/srv/My_Type"));
}

TEST_F(TestOperationManager, test_is_valid_message_type_invalid_edge_cases) {
  // Package starting with number
  EXPECT_FALSE(OperationManager::is_valid_message_type("2pkg/srv/Type"));
  // Type starting with number
  EXPECT_FALSE(OperationManager::is_valid_message_type("pkg/srv/2Type"));
  // Hyphen in package (not allowed in ROS2 naming)
  EXPECT_FALSE(OperationManager::is_valid_message_type("my-pkg/srv/Type"));
  // Empty type name
  EXPECT_FALSE(OperationManager::is_valid_message_type("pkg/srv/"));
  // Empty package name
  EXPECT_FALSE(OperationManager::is_valid_message_type("/srv/Type"));
  // Space in type
  EXPECT_FALSE(OperationManager::is_valid_message_type("pkg/srv/My Type"));
}

// ==================== SERVICE CALL ERROR HANDLING TESTS ====================

TEST_F(TestOperationManager, test_call_service_with_empty_request) {
  // Call with empty JSON object
  auto result = operation_manager_->call_service("/nonexistent/service", "std_srvs/srv/Trigger", nlohmann::json{});

  // Should fail because service doesn't exist
  EXPECT_FALSE(result.success);
}

TEST_F(TestOperationManager, test_call_service_with_null_request) {
  // Call with null JSON
  auto result =
      operation_manager_->call_service("/nonexistent/service", "std_srvs/srv/Trigger", nlohmann::json(nullptr));

  // Should fail because service doesn't exist
  EXPECT_FALSE(result.success);
}

TEST_F(TestOperationManager, test_call_component_service_not_found) {
  // Call service without providing type - should fail because discovery won't find it
  auto result = operation_manager_->call_component_service("/nonexistent/component", "nonexistent_service",
                                                           std::nullopt, nlohmann::json{});

  EXPECT_FALSE(result.success);
  EXPECT_TRUE(result.error_message.find("not found") != std::string::npos);
}

// ==================== ACTION CALL ERROR HANDLING TESTS ====================

TEST_F(TestOperationManager, test_send_action_goal_unavailable) {
  // Try to send goal to action that doesn't exist
  auto result = operation_manager_->send_action_goal("/nonexistent/action", "example_interfaces/action/Fibonacci",
                                                     nlohmann::json{{"order", 5}});

  EXPECT_FALSE(result.success);
  EXPECT_FALSE(result.goal_accepted);
  EXPECT_TRUE(result.error_message.find("not available") != std::string::npos);
}

TEST_F(TestOperationManager, test_send_component_action_goal_not_found) {
  // Try to send goal without providing type - should fail because discovery won't find it
  auto result = operation_manager_->send_component_action_goal("/nonexistent/component", "nonexistent_action",
                                                               std::nullopt, nlohmann::json{});

  EXPECT_FALSE(result.success);
  EXPECT_TRUE(result.error_message.find("not found") != std::string::npos);
}

TEST_F(TestOperationManager, test_cancel_action_goal_not_tracked) {
  // Valid UUID format but not tracked
  auto result = operation_manager_->cancel_action_goal("/test/action", "00000000000000000000000000000000");

  EXPECT_FALSE(result.success);
  EXPECT_TRUE(result.error_message.find("not tracked") != std::string::npos);
}

TEST_F(TestOperationManager, test_get_action_result_action_unavailable) {
  // Try to get result with valid UUID but unavailable action
  auto result = operation_manager_->get_action_result("/nonexistent/action", "example_interfaces/action/Fibonacci",
                                                      "00000000000000000000000000000000");

  EXPECT_FALSE(result.success);
  EXPECT_TRUE(result.error_message.find("not available") != std::string::npos);
}

// ==================== SUBSCRIPTION TESTS ====================

TEST_F(TestOperationManager, test_subscribe_unsubscribe_action_status) {
  // Subscribe and unsubscribe should not throw
  EXPECT_NO_THROW(operation_manager_->subscribe_to_action_status("/test/action"));
  EXPECT_NO_THROW(operation_manager_->unsubscribe_from_action_status("/test/action"));
}

TEST_F(TestOperationManager, test_subscribe_same_action_twice) {
  // Subscribing twice should be idempotent
  EXPECT_NO_THROW(operation_manager_->subscribe_to_action_status("/test/action"));
  EXPECT_NO_THROW(operation_manager_->subscribe_to_action_status("/test/action"));
  EXPECT_NO_THROW(operation_manager_->unsubscribe_from_action_status("/test/action"));
}

TEST_F(TestOperationManager, test_unsubscribe_without_subscribe) {
  // Unsubscribing without first subscribing should not throw
  EXPECT_NO_THROW(operation_manager_->unsubscribe_from_action_status("/never/subscribed"));
}

int main(int argc, char ** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
