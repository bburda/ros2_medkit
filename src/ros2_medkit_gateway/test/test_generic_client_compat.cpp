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

/// @file test_generic_client_compat.cpp
/// @brief Unit tests for the GenericServiceClient compatibility layer.
///
/// On Jazzy (rclcpp >= 21), these tests exercise the rclcpp::GenericClient alias path.
/// On Humble (rclcpp < 21), these tests exercise the custom shim implementation.
/// The tests verify the public API contract that OperationManager depends on.
///
// @verifies REQ_HUMBLE_COMPAT

#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <string>
#include <thread>

#include "ros2_medkit_gateway/compat/generic_client_compat.hpp"

using namespace ros2_medkit_gateway;

class TestGenericClientCompat : public ::testing::Test {
 protected:
  static void SetUpTestSuite() {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestSuite() {
    rclcpp::shutdown();
  }

  void SetUp() override {
    node_ = std::make_shared<rclcpp::Node>("test_generic_client_compat_node");
    // Register clients into a Reentrant group, matching production wiring
    // (issue #575): both compat paths must honour the group argument.
    rpc_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  }

  void TearDown() override {
    rpc_group_.reset();
    node_.reset();
  }

  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::CallbackGroup::SharedPtr rpc_group_;
};

// ==================== FACTORY TESTS ====================

/// Factory creates a valid non-null client
TEST_F(TestGenericClientCompat, factory_creates_valid_client) {
  auto client =
      compat::create_generic_service_client(node_.get(), "/test/trigger_service", "std_srvs/srv/Trigger", rpc_group_);
  ASSERT_NE(client, nullptr);
}

/// Factory works with different service types
TEST_F(TestGenericClientCompat, factory_works_with_set_bool) {
  auto client =
      compat::create_generic_service_client(node_.get(), "/test/set_bool_service", "std_srvs/srv/SetBool", rpc_group_);
  ASSERT_NE(client, nullptr);
}

/// Multiple clients can be created for different services
TEST_F(TestGenericClientCompat, multiple_clients_for_different_services) {
  auto client_a =
      compat::create_generic_service_client(node_.get(), "/test/service_a", "std_srvs/srv/Trigger", rpc_group_);
  auto client_b =
      compat::create_generic_service_client(node_.get(), "/test/service_b", "std_srvs/srv/Trigger", rpc_group_);
  auto client_c =
      compat::create_generic_service_client(node_.get(), "/test/service_c", "std_srvs/srv/SetBool", rpc_group_);

  ASSERT_NE(client_a, nullptr);
  ASSERT_NE(client_b, nullptr);
  ASSERT_NE(client_c, nullptr);

  // They should be distinct client instances
  EXPECT_NE(client_a.get(), client_b.get());
  EXPECT_NE(client_a.get(), client_c.get());
}

/// Multiple clients can be created for the same service (different consumers)
TEST_F(TestGenericClientCompat, multiple_clients_for_same_service) {
  auto client1 =
      compat::create_generic_service_client(node_.get(), "/test/shared_service", "std_srvs/srv/Trigger", rpc_group_);
  auto client2 =
      compat::create_generic_service_client(node_.get(), "/test/shared_service", "std_srvs/srv/Trigger", rpc_group_);

  ASSERT_NE(client1, nullptr);
  ASSERT_NE(client2, nullptr);
  EXPECT_NE(client1.get(), client2.get());
}

// ==================== SERVICE AVAILABILITY TESTS ====================

/// Client reports service as unavailable when no server exists
TEST_F(TestGenericClientCompat, service_not_available_for_nonexistent) {
  auto client = compat::create_generic_service_client(node_.get(), "/nonexistent/trigger_service",
                                                      "std_srvs/srv/Trigger", rpc_group_);
  ASSERT_NE(client, nullptr);

  bool available = client->wait_for_service(std::chrono::milliseconds(100));
  EXPECT_FALSE(available);
}

/// Client detects a running service server
TEST_F(TestGenericClientCompat, detects_running_service) {
  // Create a service server on the same node
  auto service = node_->create_service<std_srvs::srv::Trigger>(
      "/test/live_trigger_service", [](const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
                                       std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
        res->success = true;
        res->message = "ok";
      });

  auto client = compat::create_generic_service_client(node_.get(), "/test/live_trigger_service", "std_srvs/srv/Trigger",
                                                      rpc_group_);
  ASSERT_NE(client, nullptr);

  // Service should become available
  bool available = client->wait_for_service(std::chrono::seconds(2));
  EXPECT_TRUE(available);
}

/// Client detects a SetBool service server
TEST_F(TestGenericClientCompat, detects_running_set_bool_service) {
  auto service = node_->create_service<std_srvs::srv::SetBool>(
      "/test/live_set_bool_service", [](const std::shared_ptr<std_srvs::srv::SetBool::Request> /*req*/,
                                        std::shared_ptr<std_srvs::srv::SetBool::Response> res) {
        res->success = true;
        res->message = "done";
      });

  auto client = compat::create_generic_service_client(node_.get(), "/test/live_set_bool_service",
                                                      "std_srvs/srv/SetBool", rpc_group_);
  ASSERT_NE(client, nullptr);

  bool available = client->wait_for_service(std::chrono::seconds(2));
  EXPECT_TRUE(available);
}

// ==================== TYPE ALIAS VERIFICATION ====================

/// GenericServiceClient::SharedPtr is a valid shared_ptr type
TEST_F(TestGenericClientCompat, shared_ptr_type_is_valid) {
  compat::GenericServiceClient::SharedPtr client = compat::create_generic_service_client(
      node_.get(), "/test/type_alias_service", "std_srvs/srv/Trigger", rpc_group_);

  // Verify it's a proper shared_ptr (use_count should be >= 1)
  ASSERT_NE(client, nullptr);
  EXPECT_GE(client.use_count(), 1);
}

/// Client can be stored as a ClientBase shared pointer (polymorphism)
TEST_F(TestGenericClientCompat, can_cast_to_client_base) {
  auto client =
      compat::create_generic_service_client(node_.get(), "/test/cast_service", "std_srvs/srv/Trigger", rpc_group_);
  ASSERT_NE(client, nullptr);

  // The compat client (on both paths) should be castable to ClientBase
  std::shared_ptr<rclcpp::ClientBase> base_ptr = std::dynamic_pointer_cast<rclcpp::ClientBase>(client);
  EXPECT_NE(base_ptr, nullptr);
}

// ==================== COMPILE-TIME DETECTION ====================

/// Verify that HAS_GENERIC_CLIENT macro is defined consistently
TEST_F(TestGenericClientCompat, has_generic_client_macro_is_defined) {
#if HAS_GENERIC_CLIENT
  // On Iron+ / Jazzy: should use rclcpp::GenericClient
  SUCCEED() << "Running on Iron+ (HAS_GENERIC_CLIENT=1): using native rclcpp::GenericClient";
#else
  // On Humble: should use custom shim
  SUCCEED() << "Running on Humble (HAS_GENERIC_CLIENT=0): using compat shim";
#endif
}

int main(int argc, char ** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
