// Copyright 2025 mfaferek93
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

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "ros2_medkit_fault_reporter/fault_reporter.hpp"

using ros2_medkit_fault_reporter::FaultReporter;

namespace {

// The filter parameters the base constructor declares on the node during construction.
constexpr const char * kEnabledParam = "fault_reporter.local_filtering.enabled";

}  // namespace

class FaultReporterConstructionTest : public ::testing::Test {
 protected:
  void SetUp() override {
    rclcpp::init(0, nullptr);
  }

  void TearDown() override {
    rclcpp::shutdown();
  }
};

TEST_F(FaultReporterConstructionTest, ConstructsFromNodeReference) {
  auto node = std::make_shared<rclcpp::Node>("fault_reporter_node_ref");

  FaultReporter reporter(*node, "test_source");

  EXPECT_TRUE(node->has_parameter(kEnabledParam));
  EXPECT_NO_THROW((void)reporter.is_service_ready());
}

TEST_F(FaultReporterConstructionTest, ConstructsFromNodeSharedPtr) {
  auto node = std::make_shared<rclcpp::Node>("fault_reporter_node_shared");

  FaultReporter reporter(node, "test_source");

  EXPECT_TRUE(node->has_parameter(kEnabledParam));
  EXPECT_NO_THROW((void)reporter.is_service_ready());
}

TEST_F(FaultReporterConstructionTest, ConstructsFromLifecycleNodeReference) {
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("fault_reporter_lifecycle_ref");

  FaultReporter reporter(*node, "test_source");

  EXPECT_TRUE(node->has_parameter(kEnabledParam));
  EXPECT_NO_THROW((void)reporter.is_service_ready());
}

TEST_F(FaultReporterConstructionTest, ConstructsFromLifecycleNodeSharedPtr) {
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("fault_reporter_lifecycle_shared");

  FaultReporter reporter(node, "test_source");

  EXPECT_TRUE(node->has_parameter(kEnabledParam));
  EXPECT_NO_THROW((void)reporter.is_service_ready());
}

TEST_F(FaultReporterConstructionTest, ConstructsFromInterfaces) {
  auto node = std::make_shared<rclcpp::Node>("fault_reporter_interfaces");

  FaultReporter reporter(node->get_node_base_interface(), node->get_node_graph_interface(),
                         node->get_node_services_interface(), node->get_node_parameters_interface(), node->get_logger(),
                         "test_source");

  EXPECT_TRUE(node->has_parameter(kEnabledParam));
  EXPECT_NO_THROW((void)reporter.is_service_ready());
}

int main(int argc, char ** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
