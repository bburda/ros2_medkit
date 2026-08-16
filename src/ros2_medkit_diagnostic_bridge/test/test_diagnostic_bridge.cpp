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

#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "ros2_medkit_diagnostic_bridge/diagnostic_bridge_node.hpp"
#include "ros2_medkit_msgs/msg/fault.hpp"

using DiagStatus = diagnostic_msgs::msg::DiagnosticStatus;
using Fault = ros2_medkit_msgs::msg::Fault;
using ros2_medkit_diagnostic_bridge::DiagnosticBridgeNode;

class DiagnosticBridgeTest : public ::testing::Test {
 protected:
  void SetUp() override {
    rclcpp::init(0, nullptr);
  }

  void TearDown() override {
    rclcpp::shutdown();
  }
};

namespace {
diagnostic_msgs::msg::DiagnosticStatus diagnostic_status(const std::string & name, uint8_t level = 0,
                                                         const std::string & message = "",
                                                         std::vector<std::pair<std::string, std::string>> values = {}) {
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = name;
  status.level = level;
  status.message = message;
  for (const auto & [key, value] : values) {
    diagnostic_msgs::msg::KeyValue kv;
    kv.key = key;
    kv.value = value;
    status.values.push_back(kv);
  }
  return status;
}

std::shared_ptr<DiagnosticBridgeNode> make_node_with_keyvalue_codes(std::vector<std::string> keyvalue_codes) {
  rclcpp::NodeOptions options;
  options.append_parameter_override("keyvalue_codes", keyvalue_codes);
  return std::make_shared<DiagnosticBridgeNode>(options);
}
}  // namespace

// Test severity mapping
TEST_F(DiagnosticBridgeTest, MapToSeverity_Warn) {
  auto result = DiagnosticBridgeNode::map_to_severity(DiagStatus::WARN);
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(*result, Fault::SEVERITY_WARN);
}

TEST_F(DiagnosticBridgeTest, MapToSeverity_Error) {
  auto result = DiagnosticBridgeNode::map_to_severity(DiagStatus::ERROR);
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(*result, Fault::SEVERITY_ERROR);
}

TEST_F(DiagnosticBridgeTest, MapToSeverity_Stale) {
  auto result = DiagnosticBridgeNode::map_to_severity(DiagStatus::STALE);
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(*result, Fault::SEVERITY_CRITICAL);
}

TEST_F(DiagnosticBridgeTest, MapToSeverity_Ok) {
  // OK should return nullopt (use is_ok_level and send PASSED instead)
  auto result = DiagnosticBridgeNode::map_to_severity(DiagStatus::OK);
  EXPECT_FALSE(result.has_value());
}

TEST_F(DiagnosticBridgeTest, MapToSeverity_Unknown) {
  // Unknown level defaults to ERROR
  auto result = DiagnosticBridgeNode::map_to_severity(99);
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(*result, Fault::SEVERITY_ERROR);
}

// Test is_ok_level
TEST_F(DiagnosticBridgeTest, IsOkLevel_True) {
  EXPECT_TRUE(DiagnosticBridgeNode::is_ok_level(DiagStatus::OK));
}

TEST_F(DiagnosticBridgeTest, IsOkLevel_False) {
  EXPECT_FALSE(DiagnosticBridgeNode::is_ok_level(DiagStatus::WARN));
  EXPECT_FALSE(DiagnosticBridgeNode::is_ok_level(DiagStatus::ERROR));
  EXPECT_FALSE(DiagnosticBridgeNode::is_ok_level(DiagStatus::STALE));
}

// Node creation test
TEST_F(DiagnosticBridgeTest, NodeCreation) {
  auto node = std::make_shared<DiagnosticBridgeNode>();
  EXPECT_NE(node, nullptr);
  EXPECT_STREQ(node->get_name(), "diagnostic_bridge");
}

TEST_F(DiagnosticBridgeTest, SourceId_DefaultsToBridgeFqn) {
  auto node = std::make_shared<DiagnosticBridgeNode>();
  auto status = diagnostic_status("sensor", DiagStatus::ERROR);
  status.hardware_id = "/sensor_node";

  EXPECT_EQ(node->source_id_for(status), "/diagnostic_bridge");
}

TEST_F(DiagnosticBridgeTest, SourceId_UsesSlashContainingHardwareIdWhenEnabled) {
  rclcpp::NodeOptions options;
  options.append_parameter_override("use_hardware_id_as_source_id", true);
  auto node = std::make_shared<DiagnosticBridgeNode>(options);
  auto status = diagnostic_status("sensor", DiagStatus::ERROR);
  status.hardware_id = "/sensor_node";

  EXPECT_EQ(node->source_id_for(status), "/sensor_node");
}

TEST_F(DiagnosticBridgeTest, SourceId_NonSlashHardwareIdFallsBackToBridgeFqn) {
  rclcpp::NodeOptions options;
  options.append_parameter_override("use_hardware_id_as_source_id", true);
  auto node = std::make_shared<DiagnosticBridgeNode>(options);
  auto status = diagnostic_status("sensor", DiagStatus::ERROR);
  status.hardware_id = "SERIAL123";

  EXPECT_EQ(node->source_id_for(status), "/diagnostic_bridge");
}

TEST_F(DiagnosticBridgeTest, ReporterCache_IsBounded) {
  rclcpp::NodeOptions options;
  options.append_parameter_override("max_tracked_sources", 2);
  auto node = std::make_shared<DiagnosticBridgeNode>(options);

  ASSERT_EQ(node->tracked_reporter_count(), 0u);
  ASSERT_NE(node->reporter_for("/source_a"), nullptr);
  ASSERT_NE(node->reporter_for("/source_b"), nullptr);
  EXPECT_EQ(node->tracked_reporter_count(), 2u);
  ASSERT_NE(node->reporter_for("/source_c"), nullptr);
  EXPECT_EQ(node->tracked_reporter_count(), 2u);
}

// Test fault code mapping with auto-generate
TEST_F(DiagnosticBridgeTest, MapToFaultCode_AutoGenerate) {
  auto node = std::make_shared<DiagnosticBridgeNode>();

  // Auto-generated codes
  EXPECT_EQ(node->map_to_fault_code(diagnostic_status("motor temp")), "MOTOR_TEMP");
  EXPECT_EQ(node->map_to_fault_code(diagnostic_status("motor_temperature")), "MOTOR_TEMPERATURE");
  EXPECT_EQ(node->map_to_fault_code(diagnostic_status("motor: Status")), "MOTOR_STATUS");
  EXPECT_EQ(node->map_to_fault_code(diagnostic_status("/robot/sensor")), "ROBOT_SENSOR");
}

TEST_F(DiagnosticBridgeTest, MapToFaultCode_SpecialCharacters) {
  auto node = std::make_shared<DiagnosticBridgeNode>();

  // Multiple separators collapse to single underscore
  EXPECT_EQ(node->map_to_fault_code(diagnostic_status("motor::temp")), "MOTOR_TEMP");
  EXPECT_EQ(node->map_to_fault_code(diagnostic_status("motor  temp")), "MOTOR_TEMP");
  EXPECT_EQ(node->map_to_fault_code(diagnostic_status("motor - temp")), "MOTOR_TEMP");
}

TEST_F(DiagnosticBridgeTest, MapToFaultCode_LeadingTrailing) {
  auto node = std::make_shared<DiagnosticBridgeNode>();

  // Leading/trailing separators are removed
  EXPECT_EQ(node->map_to_fault_code(diagnostic_status("/motor")), "MOTOR");
  EXPECT_EQ(node->map_to_fault_code(diagnostic_status("motor/")), "MOTOR");
  EXPECT_EQ(node->map_to_fault_code(diagnostic_status("  motor  ")), "MOTOR");
}

TEST_F(DiagnosticBridgeTest, MapToFaultCode_FromAttribute) {
  auto node = make_node_with_keyvalue_codes({"code"});

  // Configured attribute key should take precedence over auto-generated codes.
  EXPECT_EQ(node->map_to_fault_code(diagnostic_status("/motor", 1, "", {{"code", "MOTOR_FAULT"}})), "MOTOR_FAULT");
}

TEST_F(DiagnosticBridgeTest, MapToFaultCode_KeyConfiguredButMissing_FallsBackToAutoGenerate) {
  auto node = make_node_with_keyvalue_codes({"fault_code", "error_id"});

  // No configured key exists in values, so mapping should fall back to auto-generated code.
  EXPECT_EQ(node->map_to_fault_code(diagnostic_status("/motor", 1, "", {{"code", "MOTOR_FAULT"}})), "MOTOR");
}

TEST_F(DiagnosticBridgeTest, MapToFaultCode_KeyConfiguredButMissing_AutoGenerateDisabledReturnsEmpty) {
  rclcpp::NodeOptions options;
  options.append_parameter_override("keyvalue_codes", std::vector<std::string>{"fault_code", "error_id"});
  options.append_parameter_override("auto_generate_codes", false);
  auto node = std::make_shared<DiagnosticBridgeNode>(options);

  // No configured key exists in values and auto-generate is disabled.
  EXPECT_EQ(node->map_to_fault_code(diagnostic_status("/motor", 1, "", {{"code", "MOTOR_FAULT"}})), "");
}

TEST_F(DiagnosticBridgeTest, MapToFaultCode_MultipleConfiguredKeys_UsesConfiguredKeyOrder) {
  auto node = make_node_with_keyvalue_codes({"primary_code", "secondary_code", "tertiary_code"});

  auto status = diagnostic_status(
      "/motor", 1, "", {{"secondary_code", "SECOND"}, {"primary_code", "PRIMARY"}, {"tertiary_code", "TERTIARY"}});

  // Key precedence should follow configured keyvalue_codes order.
  EXPECT_EQ(node->map_to_fault_code(status), "PRIMARY");
}

TEST_F(DiagnosticBridgeTest, MapToFaultCode_MultipleConfiguredKeys_ReorderedAttributesStillUsesConfiguredOrder) {
  auto node = make_node_with_keyvalue_codes({"primary_code", "secondary_code", "tertiary_code"});

  auto status = diagnostic_status(
      "/motor", 1, "", {{"tertiary_code", "TERTIARY"}, {"primary_code", "PRIMARY"}, {"secondary_code", "SECOND"}});

  // Reordering attributes should not change precedence when the same keys are present.
  EXPECT_EQ(node->map_to_fault_code(status), "PRIMARY");
}

TEST_F(DiagnosticBridgeTest, MapToFaultCode_KeyValueCodes_EmptyConfiguredKeysAreIgnored) {
  auto node = make_node_with_keyvalue_codes({"", "", "code"});

  // Empty configured keys are filtered during parameter loading.
  EXPECT_EQ(node->map_to_fault_code(diagnostic_status("/motor", 1, "", {{"code", "MOTOR_FAULT"}})), "MOTOR_FAULT");
}

TEST_F(DiagnosticBridgeTest, MapToFaultCode_NameToCodeOverride_PrecedesKeyValueCodes) {
  rclcpp::NodeOptions options;
  options.append_parameter_override("keyvalue_codes", std::vector<std::string>{"code"});
  options.append_parameter_override("name_to_code./motor", "OVERRIDE_CODE");
  auto node = std::make_shared<DiagnosticBridgeNode>(options);

  // Custom mapping has highest precedence even when keyvalue code is present.
  EXPECT_EQ(node->map_to_fault_code(diagnostic_status("/motor", 1, "", {{"code", "FROM_ATTRIBUTE"}})), "OVERRIDE_CODE");
}

int main(int argc, char ** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
