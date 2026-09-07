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

#include "ros2_medkit_opcua/node_map.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <fstream>
#include <string>
#include <unordered_map>
#include <vector>

namespace ros2_medkit_gateway {

class NodeMapTest : public ::testing::Test {
 protected:
  void SetUp() override {
    // Write a test YAML file
    yaml_path_ = "/tmp/test_node_map.yaml";
    std::ofstream f(yaml_path_);
    f << R"(
area_id: plc_systems
area_name: PLC Systems
component_id: openplc_runtime
component_name: OpenPLC Runtime
auto_browse: false

nodes:
  - node_id: "ns=1;s=TankLevel"
    entity_id: tank_process
    data_name: tank_level
    display_name: Tank Level
    unit: mm
    data_type: float
    writable: false
    alarm:
      fault_code: PLC_LOW_LEVEL
      severity: WARNING
      message: Tank level below minimum
      threshold: 100.0
      above_threshold: false

  - node_id: "ns=1;s=TankTemperature"
    entity_id: tank_process
    data_name: tank_temperature
    display_name: Tank Temperature
    unit: C
    data_type: float
    writable: false
    alarm:
      fault_code: PLC_HIGH_TEMP
      severity: ERROR
      message: Tank temperature above maximum
      threshold: 80.0
      above_threshold: true

  - node_id: "ns=1;s=PumpSpeed"
    entity_id: fill_pump
    data_name: pump_speed
    display_name: Pump Speed
    unit: "%"
    data_type: float
    writable: true

  - node_id: "ns=1;s=PumpRunning"
    entity_id: fill_pump
    data_name: pump_running
    display_name: Pump Running
    data_type: bool
    writable: false
)";
    f.close();
  }

  std::string yaml_path_;
};

TEST_F(NodeMapTest, LoadsYaml) {
  NodeMap map;
  ASSERT_TRUE(map.load(yaml_path_));
  EXPECT_EQ(map.entries().size(), 4u);
  EXPECT_EQ(map.area_id(), "plc_systems");
  EXPECT_EQ(map.component_id(), "openplc_runtime");
  EXPECT_FALSE(map.auto_browse());
}

TEST_F(NodeMapTest, EntriesForEntity) {
  NodeMap map;
  ASSERT_TRUE(map.load(yaml_path_));

  auto tank = map.entries_for_entity("tank_process");
  EXPECT_EQ(tank.size(), 2u);

  auto pump = map.entries_for_entity("fill_pump");
  EXPECT_EQ(pump.size(), 2u);

  auto empty = map.entries_for_entity("nonexistent");
  EXPECT_TRUE(empty.empty());
}

TEST_F(NodeMapTest, WritableEntries) {
  NodeMap map;
  ASSERT_TRUE(map.load(yaml_path_));

  auto writable = map.writable_entries_for_entity("fill_pump");
#if MEDKIT_OPCUA_READ_ONLY
  // The fixture map declares pump_speed as writable: true. A read-only build
  // has no write path, so the request is ignored and no entity has a writable
  // entry, whatever the file says.
  EXPECT_TRUE(writable.empty());
#else
  EXPECT_EQ(writable.size(), 1u);
  EXPECT_EQ(writable[0]->data_name, "pump_speed");
#endif

  auto tank_writable = map.writable_entries_for_entity("tank_process");
  EXPECT_TRUE(tank_writable.empty());
}

TEST_F(NodeMapTest, FindByDataName) {
  NodeMap map;
  ASSERT_TRUE(map.load(yaml_path_));

  auto * entry = map.find_by_data_name("tank_process", "tank_level");
  ASSERT_NE(entry, nullptr);
  EXPECT_EQ(entry->unit, "mm");
  EXPECT_FALSE(entry->writable);

  auto * missing = map.find_by_data_name("tank_process", "nonexistent");
  EXPECT_EQ(missing, nullptr);
}

TEST_F(NodeMapTest, FindByNodeId) {
  NodeMap map;
  ASSERT_TRUE(map.load(yaml_path_));

  auto * entry = map.find_by_node_id("ns=1;s=PumpSpeed");
  ASSERT_NE(entry, nullptr);
  EXPECT_EQ(entry->entity_id, "fill_pump");
#if MEDKIT_OPCUA_READ_ONLY
  EXPECT_FALSE(entry->writable) << "writable: true in the file must not survive a read-only build";
#else
  EXPECT_TRUE(entry->writable);
#endif
}

TEST_F(NodeMapTest, AlarmEntries) {
  NodeMap map;
  ASSERT_TRUE(map.load(yaml_path_));

  auto alarms = map.alarm_entries();
  EXPECT_EQ(alarms.size(), 2u);

  // Check alarm config
  bool found_low_level = false;
  bool found_high_temp = false;
  for (const auto * e : alarms) {
    if (e->alarm->fault_code == "PLC_LOW_LEVEL") {
      EXPECT_EQ(e->alarm->severity, "WARNING");
      EXPECT_DOUBLE_EQ(e->alarm->threshold, 100.0);
      EXPECT_FALSE(e->alarm->above_threshold);
      found_low_level = true;
    }
    if (e->alarm->fault_code == "PLC_HIGH_TEMP") {
      EXPECT_EQ(e->alarm->severity, "ERROR");
      EXPECT_DOUBLE_EQ(e->alarm->threshold, 80.0);
      EXPECT_TRUE(e->alarm->above_threshold);
      found_high_temp = true;
    }
  }
  EXPECT_TRUE(found_low_level);
  EXPECT_TRUE(found_high_temp);
}

TEST_F(NodeMapTest, EntityDefs) {
  NodeMap map;
  ASSERT_TRUE(map.load(yaml_path_));

  auto & defs = map.entity_defs();
  EXPECT_EQ(defs.size(), 2u);

  // Find tank_process def
  const PlcEntityDef * tank_def = nullptr;
  const PlcEntityDef * pump_def = nullptr;
  for (const auto & d : defs) {
    if (d.id == "tank_process") {
      tank_def = &d;
    }
    if (d.id == "fill_pump") {
      pump_def = &d;
    }
  }

  ASSERT_NE(tank_def, nullptr);
  EXPECT_EQ(tank_def->data_names.size(), 2u);
  EXPECT_TRUE(tank_def->writable_names.empty());
  EXPECT_TRUE(tank_def->has_faults);

  ASSERT_NE(pump_def, nullptr);
  EXPECT_EQ(pump_def->data_names.size(), 2u);
#if MEDKIT_OPCUA_READ_ONLY
  // No writable entry means no set_* operation and no x-plc-operations
  // capability: the tree cannot advertise a write this build cannot perform.
  EXPECT_TRUE(pump_def->writable_names.empty());
#else
  EXPECT_EQ(pump_def->writable_names.size(), 1u);
#endif
  EXPECT_FALSE(pump_def->has_faults);
}

TEST_F(NodeMapTest, LoadNonexistentFile) {
  NodeMap map;
  EXPECT_FALSE(map.load("/nonexistent/path.yaml"));
}

// -- ros2_topic tests --

TEST_F(NodeMapTest, Ros2TopicAutoGenerated) {
  NodeMap map;
  ASSERT_TRUE(map.load(yaml_path_));

  auto * entry = map.find_by_data_name("tank_process", "tank_level");
  ASSERT_NE(entry, nullptr);
  EXPECT_EQ(entry->ros2_topic, "/plc/tank_process/tank_level");

  auto * pump = map.find_by_data_name("fill_pump", "pump_speed");
  ASSERT_NE(pump, nullptr);
  EXPECT_EQ(pump->ros2_topic, "/plc/fill_pump/pump_speed");
}

TEST_F(NodeMapTest, Ros2TopicExplicitOverride) {
  std::string path = "/tmp/test_node_map_override.yaml";
  std::ofstream f(path);
  f << R"(
area_id: test
component_id: test
nodes:
  - node_id: "ns=1;s=Val"
    entity_id: ent
    data_name: val
    ros2_topic: /custom/my_topic
)";
  f.close();

  NodeMap map;
  ASSERT_TRUE(map.load(path));
  EXPECT_EQ(map.entries()[0].ros2_topic, "/custom/my_topic");
}

TEST_F(NodeMapTest, Ros2TopicSanitizesSpecialChars) {
  std::string path = "/tmp/test_node_map_sanitize.yaml";
  std::ofstream f(path);
  f << R"(
area_id: test
component_id: test
nodes:
  - node_id: "ns=1;s=V1"
    entity_id: tank-process
    data_name: tank.level
  - node_id: "ns=1;s=V2"
    entity_id: "my entity"
    data_name: "data name"
  - node_id: "ns=1;s=V3"
    entity_id: ok_name
    data_name: 123sensor
)";
  f.close();

  NodeMap map;
  ASSERT_TRUE(map.load(path));

  // Hyphens and dots replaced with underscores
  EXPECT_EQ(map.entries()[0].ros2_topic, "/plc/tank_process/tank_level");

  // Spaces replaced with underscores
  EXPECT_EQ(map.entries()[1].ros2_topic, "/plc/my_entity/data_name");

  // Leading digit gets underscore prefix
  EXPECT_EQ(map.entries()[2].ros2_topic, "/plc/ok_name/_123sensor");
}

TEST_F(NodeMapTest, Ros2TopicInvalidCustomFallsBackToAuto) {
  std::string path = "/tmp/test_node_map_invalid_topic.yaml";
  std::ofstream f(path);
  f << R"(
area_id: test
component_id: test
nodes:
  - node_id: "ns=1;s=V1"
    entity_id: ent
    data_name: val
    ros2_topic: "not a valid topic!"
)";
  f.close();

  NodeMap map;
  ASSERT_TRUE(map.load(path));
  // Invalid custom topic should fall back to auto-generated
  EXPECT_EQ(map.entries()[0].ros2_topic, "/plc/ent/val");
}

TEST_F(NodeMapTest, Ros2TopicEmptyEntityId) {
  std::string path = "/tmp/test_node_map_empty.yaml";
  std::ofstream f(path);
  f << R"(
area_id: test
component_id: test
nodes:
  - node_id: "ns=1;s=V1"
    entity_id: ""
    data_name: val
)";
  f.close();

  NodeMap map;
  ASSERT_TRUE(map.load(path));
  // Empty entity_id should become _unnamed
  EXPECT_EQ(map.entries()[0].ros2_topic, "/plc/_unnamed/val");
}

// -- parse_node_id tests --
//
// OPC-UA Node IDs per OPC 10000-6 section 5.3.1.10. The parser supports all
// four identifier types (i, s, g, b) plus the optional namespace prefix.

TEST(ParseNodeIdTest, NumericWithNamespace) {
  const auto nid = NodeMap::parse_node_id("ns=2;i=42");
  EXPECT_EQ(nid.getNamespaceIndex(), 2);
  ASSERT_EQ(nid.getIdentifierType(), opcua::NodeIdType::Numeric);
  EXPECT_EQ(nid.getIdentifierAs<uint32_t>(), 42u);
}

TEST(ParseNodeIdTest, NumericDefaultNamespace) {
  const auto nid = NodeMap::parse_node_id("i=2253");  // Server_ServerStatus
  EXPECT_EQ(nid.getNamespaceIndex(), 0);
  ASSERT_EQ(nid.getIdentifierType(), opcua::NodeIdType::Numeric);
  EXPECT_EQ(nid.getIdentifierAs<uint32_t>(), 2253u);
}

TEST(ParseNodeIdTest, StringSiemensStyle) {
  // Siemens TIA Portal uses quoted DB addresses with embedded dots.
  const auto nid = NodeMap::parse_node_id("ns=3;s=\"Tank_DB\".\"level_mm\"");
  EXPECT_EQ(nid.getNamespaceIndex(), 3);
  ASSERT_EQ(nid.getIdentifierType(), opcua::NodeIdType::String);
  EXPECT_EQ(nid.getIdentifierAs<opcua::String>(), std::string_view{"\"Tank_DB\".\"level_mm\""});
}

TEST(ParseNodeIdTest, StringBeckhoffStyle) {
  const auto nid = NodeMap::parse_node_id("ns=4;s=MAIN.Tank.level");
  EXPECT_EQ(nid.getNamespaceIndex(), 4);
  ASSERT_EQ(nid.getIdentifierType(), opcua::NodeIdType::String);
  EXPECT_EQ(nid.getIdentifierAs<opcua::String>(), std::string_view{"MAIN.Tank.level"});
}

TEST(ParseNodeIdTest, StringWithEmbeddedSemicolon) {
  // Pathological but legal: string identifier can contain semicolons.
  // The parser takes the rest of the string after `;s=` verbatim.
  const auto nid = NodeMap::parse_node_id("ns=2;s=value;with;semicolons");
  EXPECT_EQ(nid.getNamespaceIndex(), 2);
  EXPECT_EQ(nid.getIdentifierAs<opcua::String>(), std::string_view{"value;with;semicolons"});
}

TEST(ParseNodeIdTest, GuidWithNamespace) {
  const auto nid = NodeMap::parse_node_id("ns=3;g=09087e75-8e5e-499e-954f-f2a9603db28a");
  EXPECT_EQ(nid.getNamespaceIndex(), 3);
  ASSERT_EQ(nid.getIdentifierType(), opcua::NodeIdType::Guid);
}

TEST(ParseNodeIdTest, GuidWithBracesMicrosoftForm) {
  // Microsoft registry GUID format with curly braces is accepted.
  const auto nid = NodeMap::parse_node_id("ns=3;g={09087e75-8e5e-499e-954f-f2a9603db28a}");
  EXPECT_EQ(nid.getNamespaceIndex(), 3);
  EXPECT_EQ(nid.getIdentifierType(), opcua::NodeIdType::Guid);
}

TEST(ParseNodeIdTest, OpaqueBase64) {
  // Base64 of "helloworld12345!" (16 bytes).
  const auto nid = NodeMap::parse_node_id("ns=2;b=aGVsbG93b3JsZDEyMzQ1IQ==");
  EXPECT_EQ(nid.getNamespaceIndex(), 2);
  EXPECT_EQ(nid.getIdentifierType(), opcua::NodeIdType::ByteString);
}

TEST(ParseNodeIdTest, MalformedMissingType) {
  const auto nid = NodeMap::parse_node_id("ns=2");
  // Should return default-constructed (empty) NodeId on parse failure.
  EXPECT_EQ(nid.getIdentifierType(), opcua::NodeIdType::Numeric);
  EXPECT_EQ(nid.getIdentifierAs<uint32_t>(), 0u);
}

TEST(ParseNodeIdTest, MalformedBadNamespace) {
  const auto nid = NodeMap::parse_node_id("ns=abc;i=1");
  EXPECT_EQ(nid.getNamespaceIndex(), 0);
  EXPECT_EQ(nid.getIdentifierAs<uint32_t>(), 0u);
}

TEST(ParseNodeIdTest, MalformedBadGuid) {
  const auto nid = NodeMap::parse_node_id("ns=3;g=not-a-valid-guid");
  EXPECT_EQ(nid.getNamespaceIndex(), 0);
  EXPECT_EQ(nid.getIdentifierAs<uint32_t>(), 0u);
}

TEST(ParseNodeIdTest, MalformedUnknownTypeChar) {
  const auto nid = NodeMap::parse_node_id("ns=2;x=42");
  // Unknown identifier type falls through to default.
  EXPECT_EQ(nid.getNamespaceIndex(), 0);
  EXPECT_EQ(nid.getIdentifierAs<uint32_t>(), 0u);
}

// -- YAML validation edge cases (#17, #19) --

TEST_F(NodeMapTest, MalformedYamlSyntax) {
  std::string path = "/tmp/test_node_map_malformed.yaml";
  std::ofstream f(path);
  f << "{{{{this is not valid yaml at all";
  f.close();

  NodeMap map;
  EXPECT_FALSE(map.load(path));
}

TEST_F(NodeMapTest, MissingRequiredFields) {
  std::string path = "/tmp/test_node_map_missing.yaml";
  std::ofstream f(path);
  f << R"(
area_id: test
component_id: test
nodes:
  - entity_id: ent1
    data_name: val1
)";
  f.close();

  NodeMap map;
  ASSERT_TRUE(map.load(path));
  // Entry without node_id should be skipped
  EXPECT_TRUE(map.entries().empty());
}

TEST_F(NodeMapTest, DuplicateNodeId) {
  std::string path = "/tmp/test_node_map_dup.yaml";
  std::ofstream f(path);
  f << R"(
area_id: test
component_id: test
nodes:
  - node_id: "ns=1;i=1"
    entity_id: ent1
    data_name: first
  - node_id: "ns=1;i=1"
    entity_id: ent1
    data_name: second
)";
  f.close();

  NodeMap map;
  ASSERT_TRUE(map.load(path));
  // Duplicate node_id - first wins, second skipped
  ASSERT_EQ(map.entries().size(), 1u);
  EXPECT_EQ(map.entries()[0].data_name, "first");
}

TEST_F(NodeMapTest, UnknownDataTypeDefaultsToFloat) {
  std::string path = "/tmp/test_node_map_badtype.yaml";
  std::ofstream f(path);
  f << R"(
area_id: test
component_id: test
nodes:
  - node_id: "ns=1;i=1"
    entity_id: ent1
    data_name: val1
    data_type: custom_weird_type
)";
  f.close();

  NodeMap map;
  ASSERT_TRUE(map.load(path));
  EXPECT_EQ(map.entries()[0].data_type, "float");
}

TEST_F(NodeMapTest, RangeValidationFieldsParsed) {
  std::string path = "/tmp/test_node_map_range.yaml";
  std::ofstream f(path);
  f << R"(
area_id: test
component_id: test
nodes:
  - node_id: "ns=1;i=1"
    entity_id: ent1
    data_name: val1
    min_value: -10.5
    max_value: 100.0
)";
  f.close();

  NodeMap map;
  ASSERT_TRUE(map.load(path));
  ASSERT_TRUE(map.entries()[0].min_value.has_value());
  ASSERT_TRUE(map.entries()[0].max_value.has_value());
  EXPECT_DOUBLE_EQ(*map.entries()[0].min_value, -10.5);
  EXPECT_DOUBLE_EQ(*map.entries()[0].max_value, 100.0);
  EXPECT_TRUE(map.entries()[0].has_range());
}

TEST_F(NodeMapTest, AlarmPartialConfigDefaultsSeverity) {
  std::string path = "/tmp/test_node_map_alarm.yaml";
  std::ofstream f(path);
  f << R"(
area_id: test
component_id: test
nodes:
  - node_id: "ns=1;i=1"
    entity_id: ent1
    data_name: val1
    alarm:
      fault_code: TEST_FAULT
      threshold: 50.0
)";
  f.close();

  NodeMap map;
  ASSERT_TRUE(map.load(path));
  ASSERT_TRUE(map.entries()[0].alarm.has_value());
  EXPECT_EQ(map.entries()[0].alarm->fault_code, "TEST_FAULT");
  // severity defaults to "ERROR" when not specified
  EXPECT_EQ(map.entries()[0].alarm->severity, "ERROR");
  EXPECT_DOUBLE_EQ(map.entries()[0].alarm->threshold, 50.0);
  // above_threshold defaults to true
  EXPECT_TRUE(map.entries()[0].alarm->above_threshold);
}

TEST_F(NodeMapTest, RejectsThresholdEventAlarmCollision) {
  // bburda review on PR #387: the genuine schema collision worth checking
  // is not "alarm_source under nodes" (covered separately) but the same
  // ``(entity_id, fault_code)`` declared by both a threshold alarm under
  // ``nodes[*].alarm`` and a subscription entry under ``event_alarms``.
  // fault_manager would receive both pipelines' calls and the resulting
  // status flapping is impossible to debug at runtime; the two paths have
  // different semantics (debounced vs state-machine) so a merge is not
  // even well-defined. Loader rejects the whole file.
  std::string path = "/tmp/test_node_map_alarm_collision.yaml";
  std::ofstream f(path);
  f << R"(
area_id: test
component_id: test
nodes:
  - node_id: "ns=1;i=1"
    entity_id: tank_process
    data_name: pressure
    alarm:
      fault_code: PLC_OVERPRESSURE
      threshold: 90.0
event_alarms:
  - alarm_source: "ns=2;s=Alarms.Overpressure"
    entity_id: tank_process
    fault_code: PLC_OVERPRESSURE
)";
  f.close();

  NodeMap map;
  EXPECT_FALSE(map.load(path));
}

TEST_F(NodeMapTest, AcceptsDifferentFaultCodesAcrossPipelines) {
  // Same entity, *different* fault_codes across pipelines is fine - each
  // fault_manager entry is keyed on fault_code, so no collision. Locks
  // the contract that the rejection above is precise (won't false-positive
  // when only entity overlaps).
  std::string path = "/tmp/test_node_map_alarm_no_collision.yaml";
  std::ofstream f(path);
  f << R"(
area_id: test
component_id: test
nodes:
  - node_id: "ns=1;i=1"
    entity_id: tank_process
    data_name: pressure
    alarm:
      fault_code: PLC_PRESSURE_HIGH
      threshold: 90.0
event_alarms:
  - alarm_source: "ns=2;s=Alarms.Overheat"
    entity_id: tank_process
    fault_code: PLC_OVERHEAT
)";
  f.close();

  NodeMap map;
  EXPECT_TRUE(map.load(path));
}

TEST_F(NodeMapTest, EventAlarmSeverityAliasSetsOverride) {
  // Regression: an ``event_alarms`` entry configured with ``severity:`` (the
  // key the threshold ``alarm:`` block uses) must not be silently dropped. It
  // is accepted as an alias for ``severity_override`` so the configured
  // severity reaches the fault instead of falling back to the live event
  // Severity band mapping.
  std::string path = "/tmp/test_node_map_event_alarm_severity_alias.yaml";
  std::ofstream f(path);
  f << R"(
area_id: test
component_id: test
event_alarms:
  - alarm_source: "ns=3;s=Program_Alarm"
    entity_id: plc_program
    fault_code: PLC_PROGRAM_ALARM
    severity: ERROR
)";
  f.close();

  NodeMap map;
  ASSERT_TRUE(map.load(path));
  ASSERT_EQ(map.event_alarms().size(), 1u);
  EXPECT_EQ(map.event_alarms()[0].severity_override, "ERROR");
}

TEST_F(NodeMapTest, EventAlarmSeverityOverrideWinsOverAlias) {
  // ``severity_override`` is the canonical key; when both are present it wins.
  std::string path = "/tmp/test_node_map_event_alarm_severity_precedence.yaml";
  std::ofstream f(path);
  f << R"(
area_id: test
component_id: test
event_alarms:
  - alarm_source: "ns=3;s=Program_Alarm"
    entity_id: plc_program
    fault_code: PLC_PROGRAM_ALARM
    severity_override: CRITICAL
    severity: WARNING
)";
  f.close();

  NodeMap map;
  ASSERT_TRUE(map.load(path));
  ASSERT_EQ(map.event_alarms().size(), 1u);
  EXPECT_EQ(map.event_alarms()[0].severity_override, "CRITICAL");
}

TEST_F(NodeMapTest, EventAlarmMappingSeverityAliasSetsOverride) {
  // The alias also applies to per-condition ``mappings`` entries (issue #389).
  std::string path = "/tmp/test_node_map_event_alarm_mapping_severity_alias.yaml";
  std::ofstream f(path);
  f << R"(
area_id: test
component_id: test
event_alarms:
  - alarm_source: "ns=3;s=Program_Alarm"
    entity_id: plc_program
    mappings:
      - condition_name: Overtemp
        fault_code: PLC_OVERTEMP
        severity: WARNING
)";
  f.close();

  NodeMap map;
  ASSERT_TRUE(map.load(path));
  ASSERT_EQ(map.event_alarms().size(), 1u);
  ASSERT_EQ(map.event_alarms()[0].mappings.size(), 1u);
  EXPECT_EQ(map.event_alarms()[0].mappings[0].severity_override, "WARNING");
}

TEST_F(NodeMapTest, EventAlarmNullSeverityOverrideFallsBackToAlias) {
  // A present-but-null ``severity_override:`` is reported as defined by yaml-cpp
  // but must not shadow a real ``severity:`` sibling (nor emit a spurious
  // "non-string severity_override" warning): the alias still wins.
  std::string path = "/tmp/test_node_map_event_alarm_null_override.yaml";
  std::ofstream f(path);
  f << R"(
area_id: test
component_id: test
event_alarms:
  - alarm_source: "ns=3;s=Program_Alarm"
    entity_id: plc_program
    fault_code: PLC_PROGRAM_ALARM
    severity_override:
    severity: WARNING
)";
  f.close();

  NodeMap map;
  ASSERT_TRUE(map.load(path));
  ASSERT_EQ(map.event_alarms().size(), 1u);
  EXPECT_EQ(map.event_alarms()[0].severity_override, "WARNING");
}

TEST_F(NodeMapTest, EventAlarmSeverityAliasInvalidBucketDefaultsToError) {
  // The alias is *validated* (validate_severity), not passed through raw: an
  // unknown bucket is defaulted to ERROR so a typo cannot misroute the fault to
  // a bogus severity. Pins the validation behaviour the alias path adds.
  std::string path = "/tmp/test_node_map_event_alarm_severity_invalid.yaml";
  std::ofstream f(path);
  f << R"(
area_id: test
component_id: test
event_alarms:
  - alarm_source: "ns=3;s=Program_Alarm"
    entity_id: plc_program
    fault_code: PLC_PROGRAM_ALARM
    severity: notabucket
)";
  f.close();

  NodeMap map;
  ASSERT_TRUE(map.load(path));
  ASSERT_EQ(map.event_alarms().size(), 1u);
  EXPECT_EQ(map.event_alarms()[0].severity_override, "ERROR");
}

TEST_F(NodeMapTest, RejectsAlarmSourceUnderNodes) {
  // Schema validation: ``alarm_source`` is only valid in the top-level
  // ``event_alarms:`` section. Used to be silently ignored when not paired
  // with ``alarm.threshold``, which let a config typo land an alarm that
  // never fires. Loader must now reject the whole file with an error so
  // the typo is visible in the manifest-load log line. (Copilot review on
  // PR #387.)
  std::string path = "/tmp/test_node_map_misplaced_alarm_source.yaml";
  std::ofstream f(path);
  f << R"(
area_id: test
component_id: test
nodes:
  - node_id: "ns=1;i=1"
    entity_id: ent1
    data_name: val1
    alarm_source: "ns=2;s=Alarms.Misplaced"
)";
  f.close();

  NodeMap map;
  EXPECT_FALSE(map.load(path));
}

// -- Shared fault-detection node-map wiring (issue #481) ---------------------

namespace fd = ros2_medkit::fault_detection;

TEST_F(NodeMapTest, ThresholdAlarmLoweredToDetectionRule) {
  std::string path = "/tmp/test_node_map_threshold_detect.yaml";
  std::ofstream f(path);
  f << R"(
area_id: test
component_id: test
nodes:
  - node_id: "ns=1;i=1"
    entity_id: ent1
    data_name: temp
    alarm:
      fault_code: HIGH_TEMP
      severity: ERROR
      message: "Over temp"
      threshold: 80.0
      above_threshold: true
)";
  f.close();

  NodeMap map;
  ASSERT_TRUE(map.load(path));
  ASSERT_EQ(map.detection_entries().size(), 1u);
  const auto & det = map.detection_entries()[0]->detection;
  ASSERT_TRUE(det.has_value());
  ASSERT_TRUE(std::holds_alternative<fd::ThresholdRule>(*det));
  const auto & r = std::get<fd::ThresholdRule>(*det);
  EXPECT_EQ(r.fault.fault_code, "HIGH_TEMP");
  EXPECT_DOUBLE_EQ(r.threshold, 80.0);
  EXPECT_TRUE(r.above);
}

TEST_F(NodeMapTest, StatusBitsParsedIntoDetection) {
  std::string path = "/tmp/test_node_map_status_bits.yaml";
  std::ofstream f(path);
  f << R"(
area_id: test
component_id: test
nodes:
  - node_id: "ns=1;s=StatusWord"
    entity_id: pump
    data_name: status_word
    data_type: int
    status_bits:
      - bit: 3
        fault_code: PUMP_OVERLOAD
        severity: ERROR
        message: "Pump overload"
      - bit: 7
        fault_code: FILTER_DIRTY
        severity: WARNING
)";
  f.close();

  NodeMap map;
  ASSERT_TRUE(map.load(path));
  ASSERT_EQ(map.detection_entries().size(), 1u);
  const auto & det = map.detection_entries()[0]->detection;
  ASSERT_TRUE(det.has_value());
  ASSERT_TRUE(std::holds_alternative<fd::StatusWordRule>(*det));
  const auto & r = std::get<fd::StatusWordRule>(*det);
  ASSERT_EQ(r.bits.size(), 2u);
  EXPECT_EQ(r.bits[0].bit, 3u);
  EXPECT_EQ(r.bits[0].fault.fault_code, "PUMP_OVERLOAD");
  EXPECT_EQ(r.bits[1].bit, 7u);
  // Message defaults to the fault code when omitted.
  EXPECT_EQ(r.bits[1].fault.message, "FILTER_DIRTY");

  auto signals = fd::evaluate(fd::Value{static_cast<std::int64_t>(0b1000)}, *det);
  ASSERT_EQ(signals.size(), 2u);
  EXPECT_TRUE(signals[0].active);   // bit 3 set
  EXPECT_FALSE(signals[1].active);  // bit 7 clear
}

TEST_F(NodeMapTest, FaultEnumParsedIntoDetection) {
  std::string path = "/tmp/test_node_map_fault_enum.yaml";
  std::ofstream f(path);
  f << R"(
area_id: test
component_id: test
nodes:
  - node_id: "ns=1;s=FaultCode"
    entity_id: vfd
    data_name: fault_code
    data_type: int
    fault_enum:
      ok_value: 0
      codes:
        - code: 10
          fault_code: VFD_OVERVOLTAGE
          severity: ERROR
          message: "DC bus overvoltage"
        - code: 11
          fault_code: VFD_OVERCURRENT
          severity: ERROR
)";
  f.close();

  NodeMap map;
  ASSERT_TRUE(map.load(path));
  ASSERT_EQ(map.detection_entries().size(), 1u);
  const auto & det = map.detection_entries()[0]->detection;
  ASSERT_TRUE(det.has_value());
  ASSERT_TRUE(std::holds_alternative<fd::EnumMapRule>(*det));
  const auto & r = std::get<fd::EnumMapRule>(*det);
  EXPECT_EQ(r.ok_value, 0);
  ASSERT_EQ(r.codes.size(), 2u);
  EXPECT_EQ(r.codes[0].code, 10);
  EXPECT_EQ(r.codes[0].fault.message, "DC bus overvoltage");

  auto signals = fd::evaluate(fd::Value{static_cast<std::int64_t>(10)}, *det);
  // Two configured codes plus the auto-added unmapped catch-all.
  ASSERT_EQ(signals.size(), 3u);
  EXPECT_TRUE(signals[0].active);   // code 10
  EXPECT_FALSE(signals[1].active);  // code 11
  EXPECT_FALSE(signals[2].active);  // catch-all (a mapped code is not "unmapped")
}

TEST_F(NodeMapTest, RejectsDuplicateDetectionFaultCode) {
  // The poller shares one FaultTransitionTracker keyed by fault_code alone, so
  // two detection entries emitting the same code (here on different entities)
  // would flap raise/clear every cycle. The loader must reject the whole file
  // at load time. (issue #481)
  std::string path = "/tmp/test_node_map_dup_fault_code.yaml";
  std::ofstream f(path);
  f << R"(
area_id: test
component_id: test
nodes:
  - node_id: "ns=1;i=1"
    entity_id: ent_a
    data_name: temp_a
    alarm:
      fault_code: SHARED_CODE
      threshold: 80.0
  - node_id: "ns=1;i=2"
    entity_id: ent_b
    data_name: temp_b
    alarm:
      fault_code: SHARED_CODE
      threshold: 90.0
)";
  f.close();

  NodeMap map;
  EXPECT_FALSE(map.load(path));
}

TEST_F(NodeMapTest, RejectsDuplicateCodeAcrossStatusBitsAndEnum) {
  // Uniqueness spans every emitted code, not just threshold alarms: a bit code
  // colliding with an enum code on another node is still a shared-tracker
  // collision. (issue #481)
  std::string path = "/tmp/test_node_map_dup_mixed.yaml";
  std::ofstream f(path);
  f << R"(
area_id: test
component_id: test
nodes:
  - node_id: "ns=1;s=Status"
    entity_id: pump
    data_name: status_word
    data_type: int
    status_bits:
      - bit: 2
        fault_code: COLLIDE
  - node_id: "ns=1;s=Fault"
    entity_id: vfd
    data_name: fault_code
    data_type: int
    fault_enum:
      codes:
        - code: 7
          fault_code: COLLIDE
)";
  f.close();

  NodeMap map;
  EXPECT_FALSE(map.load(path));
}

TEST_F(NodeMapTest, RejectsStatusBitsCollidingWithEventAlarm) {
  // The author's original collision check only looked at nodes[*].alarm; a
  // status_bits (or fault_enum) code colliding with an event_alarms
  // (entity_id, fault_code) must be rejected too, otherwise fault_manager gets
  // both a polled report and a state-machine report for one SOVD fault.
  // (issue #481)
  std::string path = "/tmp/test_node_map_bits_event_collision.yaml";
  std::ofstream f(path);
  f << R"(
area_id: test
component_id: test
nodes:
  - node_id: "ns=1;s=Status"
    entity_id: tank_process
    data_name: status_word
    data_type: int
    status_bits:
      - bit: 1
        fault_code: PLC_OVERPRESSURE
event_alarms:
  - alarm_source: "ns=2;s=Alarms.Overpressure"
    entity_id: tank_process
    fault_code: PLC_OVERPRESSURE
)";
  f.close();

  NodeMap map;
  EXPECT_FALSE(map.load(path));
}

TEST_F(NodeMapTest, AcceptsDistinctCodesAcrossDetectionModes) {
  // The uniqueness rule must not false-positive on genuinely distinct codes
  // spread across threshold / status_bits / fault_enum entries.
  std::string path = "/tmp/test_node_map_distinct_modes.yaml";
  std::ofstream f(path);
  f << R"(
area_id: test
component_id: test
nodes:
  - node_id: "ns=1;i=1"
    entity_id: tank
    data_name: temp
    alarm:
      fault_code: HIGH_TEMP
      threshold: 80.0
  - node_id: "ns=1;s=Status"
    entity_id: pump
    data_name: status_word
    data_type: int
    status_bits:
      - bit: 3
        fault_code: PUMP_OVERLOAD
      - bit: 7
        fault_code: FILTER_DIRTY
  - node_id: "ns=1;s=Fault"
    entity_id: vfd
    data_name: fault_code
    data_type: int
    fault_enum:
      codes:
        - code: 10
          fault_code: VFD_OVERVOLTAGE
)";
  f.close();

  NodeMap map;
  ASSERT_TRUE(map.load(path));
  EXPECT_EQ(map.detection_entries().size(), 3u);
}

TEST_F(NodeMapTest, RejectsCodeAcrossPipelinesDifferentEntities) {
  // Global-by-code uniqueness (issue #481): a polled detection code on
  // entity_a and an event_alarms code on entity_b share the SAME fault_code.
  // fault_manager keys and clears by code alone, so the two collide even
  // though the entities differ; the (entity_id, fault_code) pair the earlier
  // guard keyed on let this through. The loader must reject the whole file.
  std::string path = "/tmp/test_node_map_cross_entity_pipeline.yaml";
  std::ofstream f(path);
  f << R"(
area_id: test
component_id: test
nodes:
  - node_id: "ns=1;i=1"
    entity_id: entity_a
    data_name: pressure
    alarm:
      fault_code: SHARED_CODE
      threshold: 90.0
event_alarms:
  - alarm_source: "ns=2;s=Alarms.Something"
    entity_id: entity_b
    fault_code: SHARED_CODE
)";
  f.close();

  NodeMap map;
  EXPECT_FALSE(map.load(path));
}

TEST_F(NodeMapTest, MalformedDetectionStringSkipsRuleNotFile) {
  // issue #481: a wrong-typed string field (here a sequence where a scalar
  // fault_code is expected) must warn and skip just that rule, not throw
  // YAML::TypedBadConversion into the outer catch and discard every other
  // node in the file.
  std::string path = "/tmp/test_node_map_malformed_string.yaml";
  std::ofstream f(path);
  f << R"(
area_id: test
component_id: test
nodes:
  - node_id: "ns=1;s=BadEnum"
    entity_id: vfd
    data_name: fault_code
    data_type: int
    fault_enum:
      codes:
        - code: 5
          fault_code: [not, a, scalar]
  - node_id: "ns=1;s=GoodTemp"
    entity_id: tank
    data_name: temp
    alarm:
      fault_code: GOOD_HIGH_TEMP
      threshold: 80.0
)";
  f.close();

  NodeMap map;
  // Whole file still loads; the malformed enum code is dropped so that node
  // just has no detection, and the second valid node keeps its rule.
  ASSERT_TRUE(map.load(path));
  EXPECT_EQ(map.entries().size(), 2u);
  const auto * bad = map.find_by_node_id("ns=1;s=BadEnum");
  ASSERT_NE(bad, nullptr);
  EXPECT_FALSE(bad->detection.has_value());
  const auto * good = map.find_by_node_id("ns=1;s=GoodTemp");
  ASSERT_NE(good, nullptr);
  ASSERT_TRUE(good->detection.has_value());
  EXPECT_TRUE(std::holds_alternative<fd::ThresholdRule>(*good->detection));
}

TEST_F(NodeMapTest, AlarmMissingFaultCodeSkipsDetectionNotFile) {
  // issue #481: an ``alarm:`` block with no fault_code used to abort the whole
  // file via an unguarded .as<std::string>(). Now it warns, skips detection
  // for that point, and keeps loading the rest.
  std::string path = "/tmp/test_node_map_alarm_no_code.yaml";
  std::ofstream f(path);
  f << R"(
area_id: test
component_id: test
nodes:
  - node_id: "ns=1;i=1"
    entity_id: tank
    data_name: pressure
    alarm:
      threshold: 90.0
  - node_id: "ns=1;i=2"
    entity_id: tank
    data_name: level
    alarm:
      fault_code: LEVEL_LOW
      threshold: 10.0
      above_threshold: false
)";
  f.close();

  NodeMap map;
  ASSERT_TRUE(map.load(path));
  EXPECT_EQ(map.entries().size(), 2u);
  const auto * p = map.find_by_node_id("ns=1;i=1");
  ASSERT_NE(p, nullptr);
  EXPECT_FALSE(p->detection.has_value());
  const auto * l = map.find_by_node_id("ns=1;i=2");
  ASSERT_NE(l, nullptr);
  EXPECT_TRUE(l->detection.has_value());
}

TEST_F(NodeMapTest, StatusBitOutOfRangeSkipped) {
  // A bit position >= 64 can never be set in the 64-bit decode register, so it
  // is dead config: warn and drop the bit, keep the rest. (issue #481)
  std::string path = "/tmp/test_node_map_bit_oob.yaml";
  std::ofstream f(path);
  f << R"(
area_id: test
component_id: test
nodes:
  - node_id: "ns=1;s=Status"
    entity_id: pump
    data_name: status_word
    data_type: int
    status_bits:
      - bit: 64
        fault_code: NEVER_FIRES
      - bit: 3
        fault_code: PUMP_OVERLOAD
)";
  f.close();

  NodeMap map;
  ASSERT_TRUE(map.load(path));
  ASSERT_EQ(map.detection_entries().size(), 1u);
  const auto & det = map.detection_entries()[0]->detection;
  ASSERT_TRUE(std::holds_alternative<fd::StatusWordRule>(*det));
  const auto & r = std::get<fd::StatusWordRule>(*det);
  ASSERT_EQ(r.bits.size(), 1u);
  EXPECT_EQ(r.bits[0].bit, 3u);
  EXPECT_EQ(r.bits[0].fault.fault_code, "PUMP_OVERLOAD");
}

// -- Poller-level shared-tracker integration (issue #481) --------------------
//
// Exercises the exact loop OpcuaPoller::evaluate_alarms runs: one shared
// FaultTransitionTracker, iterating detection entries built by the real
// NodeMap loader and evaluated by the shared fault_detection evaluator. Proves
// there is no cross-entry flapping and that status-word bits transition
// independently. Driven without a live OPC UA server by feeding synthetic
// values keyed by node_id, mirroring the poller's snapshot.

namespace {

struct DetectionChange {
  std::string entity_id;
  fd::FaultSignal signal;
};

// Mirror of OpcuaPoller::evaluate_alarms: for each detection entry, look up its
// value, evaluate, and push the value through the SHARED tracker; return the
// raise/clear edges. Entries whose node has no value yet are skipped.
std::vector<DetectionChange> evaluate_cycle(const NodeMap & map, fd::FaultTransitionTracker & tracker,
                                            const std::unordered_map<std::string, fd::Value> & values) {
  std::vector<DetectionChange> changes;
  for (const auto * entry : map.detection_entries()) {
    auto it = values.find(entry->node_id_str);
    if (it == values.end()) {
      continue;
    }
    auto signals = fd::evaluate(it->second, *entry->detection);
    for (auto & edge : tracker.apply(signals)) {
      changes.push_back({entry->entity_id, std::move(edge)});
    }
  }
  return changes;
}

const DetectionChange * find_change(const std::vector<DetectionChange> & v, const std::string & code) {
  for (const auto & c : v) {
    if (c.signal.fault_code == code) {
      return &c;
    }
  }
  return nullptr;
}

}  // namespace

TEST_F(NodeMapTest, PollerNoCrossEntryFlapping) {
  std::string path = "/tmp/test_node_map_poller_two_entries.yaml";
  std::ofstream f(path);
  f << R"(
area_id: test
component_id: test
nodes:
  - node_id: "ns=1;s=TempA"
    entity_id: ent_a
    data_name: temp_a
    data_type: float
    alarm:
      fault_code: A_HIGH
      threshold: 80.0
  - node_id: "ns=1;s=TempB"
    entity_id: ent_b
    data_name: temp_b
    data_type: float
    alarm:
      fault_code: B_HIGH
      threshold: 80.0
)";
  f.close();

  NodeMap map;
  ASSERT_TRUE(map.load(path));
  ASSERT_EQ(map.detection_entries().size(), 2u);

  fd::FaultTransitionTracker tracker;
  std::unordered_map<std::string, fd::Value> values;

  // Both healthy: no transitions.
  values["ns=1;s=TempA"] = fd::Value{70.0};
  values["ns=1;s=TempB"] = fd::Value{70.0};
  EXPECT_TRUE(evaluate_cycle(map, tracker, values).empty());

  // A crosses high, B stays healthy: exactly one raise for A.
  values["ns=1;s=TempA"] = fd::Value{90.0};
  {
    auto changes = evaluate_cycle(map, tracker, values);
    ASSERT_EQ(changes.size(), 1u);
    EXPECT_EQ(changes[0].entity_id, "ent_a");
    EXPECT_EQ(changes[0].signal.fault_code, "A_HIGH");
    EXPECT_TRUE(changes[0].signal.active);
  }

  // Steady state with A still high: the shared tracker must NOT clear B (which
  // it never raised) nor re-emit A. This is the regression the fix guards.
  for (int i = 0; i < 5; ++i) {
    EXPECT_TRUE(evaluate_cycle(map, tracker, values).empty());
  }

  // Now B crosses high too: exactly one raise for B, A untouched.
  values["ns=1;s=TempB"] = fd::Value{95.0};
  {
    auto changes = evaluate_cycle(map, tracker, values);
    ASSERT_EQ(changes.size(), 1u);
    EXPECT_EQ(changes[0].entity_id, "ent_b");
    EXPECT_EQ(changes[0].signal.fault_code, "B_HIGH");
    EXPECT_TRUE(changes[0].signal.active);
  }

  // A clears, B stays high: exactly one clear for A.
  values["ns=1;s=TempA"] = fd::Value{60.0};
  {
    auto changes = evaluate_cycle(map, tracker, values);
    ASSERT_EQ(changes.size(), 1u);
    EXPECT_EQ(changes[0].signal.fault_code, "A_HIGH");
    EXPECT_FALSE(changes[0].signal.active);
  }
}

TEST_F(NodeMapTest, PollerStatusWordBitsTransitionIndependently) {
  std::string path = "/tmp/test_node_map_poller_status_word.yaml";
  std::ofstream f(path);
  f << R"(
area_id: test
component_id: test
nodes:
  - node_id: "ns=1;s=StatusWord"
    entity_id: pump
    data_name: status_word
    data_type: int
    status_bits:
      - bit: 0
        fault_code: PUMP_OVERLOAD
      - bit: 3
        fault_code: FILTER_DIRTY
)";
  f.close();

  NodeMap map;
  ASSERT_TRUE(map.load(path));

  fd::FaultTransitionTracker tracker;
  std::unordered_map<std::string, fd::Value> values;

  // All clear.
  values["ns=1;s=StatusWord"] = fd::Value{static_cast<std::int64_t>(0)};
  EXPECT_TRUE(evaluate_cycle(map, tracker, values).empty());

  // Bit 0 set only: raise PUMP_OVERLOAD, FILTER_DIRTY stays clear.
  values["ns=1;s=StatusWord"] = fd::Value{static_cast<std::int64_t>(0b0001)};
  {
    auto changes = evaluate_cycle(map, tracker, values);
    ASSERT_EQ(changes.size(), 1u);
    const auto * overload = find_change(changes, "PUMP_OVERLOAD");
    ASSERT_NE(overload, nullptr);
    EXPECT_TRUE(overload->signal.active);
  }

  // Bit 3 also set: raise FILTER_DIRTY only, PUMP_OVERLOAD stays raised.
  values["ns=1;s=StatusWord"] = fd::Value{static_cast<std::int64_t>(0b1001)};
  {
    auto changes = evaluate_cycle(map, tracker, values);
    ASSERT_EQ(changes.size(), 1u);
    const auto * dirty = find_change(changes, "FILTER_DIRTY");
    ASSERT_NE(dirty, nullptr);
    EXPECT_TRUE(dirty->signal.active);
  }

  // Bit 0 clears, bit 3 stays: clear PUMP_OVERLOAD only.
  values["ns=1;s=StatusWord"] = fd::Value{static_cast<std::int64_t>(0b1000)};
  {
    auto changes = evaluate_cycle(map, tracker, values);
    ASSERT_EQ(changes.size(), 1u);
    const auto * overload = find_change(changes, "PUMP_OVERLOAD");
    ASSERT_NE(overload, nullptr);
    EXPECT_FALSE(overload->signal.active);
  }
}

// -- Undecidable input, catch-all, and malformed-config handling (issue #481) -

TEST_F(NodeMapTest, EmptyStatusBitsBlockLoadsWithoutDetection) {
  // A declared-but-empty status_bits block yields zero rules. The loader must
  // keep the data point but emit no detection (warned), not silently ship a
  // non-functional fault map and not reject the whole file.
  std::string path = "/tmp/test_node_map_empty_status_bits.yaml";
  std::ofstream f(path);
  f << R"(
area_id: test
component_id: test
nodes:
  - node_id: "ns=1;s=Status"
    entity_id: pump
    data_name: status_word
    data_type: int
    status_bits: []
)";
  f.close();

  NodeMap map;
  ASSERT_TRUE(map.load(path));
  ASSERT_EQ(map.entries().size(), 1u);
  EXPECT_TRUE(map.detection_entries().empty());
}

TEST_F(NodeMapTest, FaultEnumWithoutCodesLoadsWithoutDetection) {
  std::string path = "/tmp/test_node_map_empty_enum.yaml";
  std::ofstream f(path);
  f << R"(
area_id: test
component_id: test
nodes:
  - node_id: "ns=1;s=Fault"
    entity_id: vfd
    data_name: fault_code
    data_type: int
    fault_enum:
      ok_value: 0
)";
  f.close();

  NodeMap map;
  ASSERT_TRUE(map.load(path));
  ASSERT_EQ(map.entries().size(), 1u);
  EXPECT_TRUE(map.detection_entries().empty());
}

TEST_F(NodeMapTest, WrongTypedBitSkippedNotAborted) {
  // A single mistyped bit must skip just that bit (consistent with the
  // missing-field path), never tear down the entire node map.
  std::string path = "/tmp/test_node_map_bad_bit.yaml";
  std::ofstream f(path);
  f << R"(
area_id: test
component_id: test
nodes:
  - node_id: "ns=1;s=Status"
    entity_id: pump
    data_name: status_word
    data_type: int
    status_bits:
      - bit: "three"
        fault_code: BAD_BIT
      - bit: 3
        fault_code: PUMP_OVERLOAD
)";
  f.close();

  NodeMap map;
  ASSERT_TRUE(map.load(path));
  ASSERT_EQ(map.detection_entries().size(), 1u);
  const auto & det = map.detection_entries()[0]->detection;
  ASSERT_TRUE(std::holds_alternative<fd::StatusWordRule>(*det));
  const auto & r = std::get<fd::StatusWordRule>(*det);
  ASSERT_EQ(r.bits.size(), 1u);
  EXPECT_EQ(r.bits[0].fault.fault_code, "PUMP_OVERLOAD");
}

TEST_F(NodeMapTest, WrongTypedEnumCodeSkippedNotAborted) {
  std::string path = "/tmp/test_node_map_bad_code.yaml";
  std::ofstream f(path);
  f << R"(
area_id: test
component_id: test
nodes:
  - node_id: "ns=1;s=Fault"
    entity_id: vfd
    data_name: fault_code
    data_type: int
    fault_enum:
      codes:
        - code: high
          fault_code: BAD_CODE
        - code: 10
          fault_code: VFD_OVERVOLTAGE
)";
  f.close();

  NodeMap map;
  ASSERT_TRUE(map.load(path));
  ASSERT_EQ(map.detection_entries().size(), 1u);
  const auto & det = map.detection_entries()[0]->detection;
  ASSERT_TRUE(std::holds_alternative<fd::EnumMapRule>(*det));
  const auto & r = std::get<fd::EnumMapRule>(*det);
  ASSERT_EQ(r.codes.size(), 1u);
  EXPECT_EQ(r.codes[0].code, 10);
}

TEST_F(NodeMapTest, WrongTypedEventAlarmStringSkipsEntryNotFile) {
  // A present-but-wrong-typed required string on one ``event_alarms`` entry
  // must skip just that entry, consistent with the nodes block, rather than
  // throwing into the outer catch, which would discard every valid node and
  // alarm and disable the whole plugin. (#481)
  std::string path = "/tmp/test_node_map_bad_event_alarm.yaml";
  std::ofstream f(path);
  f << R"(
area_id: test
component_id: test
nodes:
  - node_id: "ns=1;s=Pressure"
    entity_id: tank_process
    data_name: pressure
    alarm:
      fault_code: PLC_OVERPRESSURE
      threshold: 90.0
event_alarms:
  - alarm_source: "ns=2;s=Alarms.Bad"
    entity_id: [not, a, string]
    fault_code: BAD_ALARM
  - alarm_source: "ns=2;s=Alarms.Good"
    entity_id: tank_process
    fault_code: PLC_OVERHEAT
)";
  f.close();

  NodeMap map;
  ASSERT_TRUE(map.load(path));
  // Valid node survived the malformed event_alarms entry.
  ASSERT_EQ(map.detection_entries().size(), 1u);
  // Only the well-formed alarm was kept; the wrong-typed one was skipped.
  ASSERT_EQ(map.event_alarms().size(), 1u);
  EXPECT_EQ(map.event_alarms()[0].fault_code, "PLC_OVERHEAT");
}

TEST_F(NodeMapTest, FaultEnumGetsCatchAllUnknownFault) {
  std::string path = "/tmp/test_node_map_enum_catchall.yaml";
  std::ofstream f(path);
  f << R"(
area_id: test
component_id: test
nodes:
  - node_id: "ns=1;s=Fault"
    entity_id: vfd
    data_name: fault_code
    data_type: int
    fault_enum:
      ok_value: 0
      codes:
        - code: 10
          fault_code: VFD_OVERVOLTAGE
)";
  f.close();

  NodeMap map;
  ASSERT_TRUE(map.load(path));
  ASSERT_EQ(map.detection_entries().size(), 1u);
  const auto & det = map.detection_entries()[0]->detection;
  ASSERT_TRUE(std::holds_alternative<fd::EnumMapRule>(*det));
  const auto & r = std::get<fd::EnumMapRule>(*det);
  ASSERT_FALSE(r.unknown_fault.fault_code.empty());

  // An unenumerated non-ok value raises the catch-all rather than reading ok.
  auto out = fd::evaluate(fd::Value{static_cast<std::int64_t>(77)}, *det);
  const fd::FaultSignal * unk = nullptr;
  for (const auto & s : out) {
    if (s.fault_code == r.unknown_fault.fault_code) {
      unk = &s;
    }
  }
  ASSERT_NE(unk, nullptr);
  EXPECT_TRUE(unk->active);
  EXPECT_EQ(unk->message, "unmapped fault code 77");
}

TEST_F(NodeMapTest, NonNumericThresholdFallsBackAndLoads) {
  // A present-but-unparseable threshold must warn and fall back, not silently
  // invert/disable detection and not abort the load.
  std::string path = "/tmp/test_node_map_bad_threshold.yaml";
  std::ofstream f(path);
  f << R"(
area_id: test
component_id: test
nodes:
  - node_id: "ns=1;i=1"
    entity_id: ent1
    data_name: val1
    alarm:
      fault_code: BAD_THRESH
      threshold: "100 bar"
)";
  f.close();

  NodeMap map;
  ASSERT_TRUE(map.load(path));
  ASSERT_TRUE(map.entries()[0].alarm.has_value());
  EXPECT_DOUBLE_EQ(map.entries()[0].alarm->threshold, 0.0);
}

TEST_F(NodeMapTest, InvalidSeverityDefaultsToError) {
  std::string path = "/tmp/test_node_map_bad_severity.yaml";
  std::ofstream f(path);
  f << R"(
area_id: test
component_id: test
nodes:
  - node_id: "ns=1;i=1"
    entity_id: ent1
    data_name: val1
    alarm:
      fault_code: SEV_TEST
      severity: SEVERE
      threshold: 50.0
)";
  f.close();

  NodeMap map;
  ASSERT_TRUE(map.load(path));
  ASSERT_TRUE(map.entries()[0].alarm.has_value());
  EXPECT_EQ(map.entries()[0].alarm->severity, "ERROR");
}

TEST_F(NodeMapTest, StatusWordWidthMasksSignExtension) {
  std::string path = "/tmp/test_node_map_status_width.yaml";
  std::ofstream f(path);
  f << R"(
area_id: test
component_id: test
nodes:
  - node_id: "ns=1;s=Status"
    entity_id: drive
    data_name: status_word
    data_type: int
    status_word_width: 16
    status_bits:
      - bit: 15
        fault_code: DRIVE_FAULT
)";
  f.close();

  NodeMap map;
  ASSERT_TRUE(map.load(path));
  ASSERT_EQ(map.detection_entries().size(), 1u);
  const auto & det = map.detection_entries()[0]->detection;
  ASSERT_TRUE(std::holds_alternative<fd::StatusWordRule>(*det));
  const auto & r = std::get<fd::StatusWordRule>(*det);
  EXPECT_EQ(r.width, 16u);

  // bit15 set in a sign-extended 16-bit word: only DRIVE_FAULT, no spurious bits.
  const auto sign_extended = static_cast<std::int64_t>(0xFFFFFFFFFFFF8000ULL);
  auto out = fd::evaluate(fd::Value{sign_extended}, *det);
  ASSERT_EQ(out.size(), 1u);
  EXPECT_TRUE(out[0].active);
}

TEST_F(NodeMapTest, RejectsMultipleDetectionModesOnOneNode) {
  std::string path = "/tmp/test_node_map_multi_mode.yaml";
  std::ofstream f(path);
  f << R"(
area_id: test
component_id: test
nodes:
  - node_id: "ns=1;i=1"
    entity_id: ent1
    data_name: val1
    alarm:
      fault_code: A
      threshold: 1.0
    status_bits:
      - bit: 0
        fault_code: B
)";
  f.close();

  NodeMap map;
  EXPECT_FALSE(map.load(path));
}

// ---------------------------------------------------------------------------
// Issue #389: multi-alarm mapping + associated values.
// ---------------------------------------------------------------------------

// AlarmMapping brace-init order:
// {match_condition_name, match_source_node, match_event_type, match_message,
//  fault_code, severity_override, message_override}
TEST(ResolveAlarmTest, FallsBackToSourceFaultCode) {
  AlarmEventConfig cfg;
  cfg.entity_id = "tank";
  cfg.fault_code = "PLC_GENERIC";
  cfg.severity_override = "WARNING";
  auto r = NodeMap::resolve_alarm(cfg, "AnyName", "ns=2;s=Src", "ns=0;i=2915", "any msg");
  EXPECT_TRUE(r.matched);
  EXPECT_EQ(r.fault_code, "PLC_GENERIC");
  EXPECT_EQ(r.severity_override, "WARNING");
}

TEST(ResolveAlarmTest, NoMappingNoFallbackIsUnmatched) {
  AlarmEventConfig cfg;
  cfg.entity_id = "tank";  // no fault_code, no mappings
  auto r = NodeMap::resolve_alarm(cfg, "X", "Y", "Z", "msg");
  EXPECT_FALSE(r.matched);
}

TEST(ResolveAlarmTest, FirstMatchingMappingWinsByConditionName) {
  AlarmEventConfig cfg;
  cfg.entity_id = "tank";
  cfg.fault_code = "PLC_CATCHALL";
  cfg.mappings.push_back({/*cond*/ "Overpressure", "", "", "", "PLC_OVERPRESSURE", "ERROR", "Overpressure!"});
  cfg.mappings.push_back({/*cond*/ "LowLevel", "", "", "", "PLC_LOW_LEVEL", "WARNING", "Low level"});

  auto r1 = NodeMap::resolve_alarm(cfg, "Overpressure", "ns=2;s=Tank", "ns=0;i=2915", "");
  EXPECT_TRUE(r1.matched);
  EXPECT_EQ(r1.fault_code, "PLC_OVERPRESSURE");
  EXPECT_EQ(r1.severity_override, "ERROR");
  EXPECT_EQ(r1.message_override, "Overpressure!");

  auto r2 = NodeMap::resolve_alarm(cfg, "LowLevel", "ns=2;s=Tank", "ns=0;i=2915", "");
  EXPECT_EQ(r2.fault_code, "PLC_LOW_LEVEL");

  // Unmatched condition name -> source-level catch-all.
  auto r3 = NodeMap::resolve_alarm(cfg, "Unknown", "ns=2;s=Tank", "ns=0;i=2915", "");
  EXPECT_TRUE(r3.matched);
  EXPECT_EQ(r3.fault_code, "PLC_CATCHALL");
}

TEST(ResolveAlarmTest, MatchBySourceNodeAndEventType) {
  AlarmEventConfig cfg;
  cfg.entity_id = "tank";
  cfg.mappings.push_back({"", "ns=2;s=PumpA", "ns=0;i=2915", "", "PLC_PUMP_A", "", ""});
  // SourceNode mismatch -> unmatched (no fallback fault_code).
  EXPECT_FALSE(NodeMap::resolve_alarm(cfg, "X", "ns=2;s=PumpB", "ns=0;i=2915", "").matched);
  // Both match.
  auto r = NodeMap::resolve_alarm(cfg, "X", "ns=2;s=PumpA", "ns=0;i=2915", "");
  EXPECT_TRUE(r.matched);
  EXPECT_EQ(r.fault_code, "PLC_PUMP_A");
}

TEST(ResolveAlarmTest, MappingInheritsSourceLevelOverridesWhenUnset) {
  AlarmEventConfig cfg;
  cfg.entity_id = "tank";
  cfg.severity_override = "CRITICAL";
  cfg.message_override = "src-msg";
  cfg.mappings.push_back({"Cond", "", "", "", "PLC_X", "", ""});  // no per-mapping overrides
  auto r = NodeMap::resolve_alarm(cfg, "Cond", "s", "e", "");
  EXPECT_EQ(r.severity_override, "CRITICAL");
  EXPECT_EQ(r.message_override, "src-msg");
}

TEST(ResolveAlarmTest, MatchByMessageSubstring) {
  // A PLC (e.g. Siemens S7-1500) emits many Program_Alarms sharing one
  // EventType and SourceNode, differing only by alarm text; a case-sensitive
  // Message substring match splits them into distinct SOVD faults.
  AlarmEventConfig cfg;
  cfg.entity_id = "cpu";
  cfg.fault_code = "PLC_PROGRAM_ALARM";  // source-level catch-all
  cfg.mappings.push_back({"", "", "", "trigger active", "PLC_ALARM_TRIGGER", "ERROR", ""});
  cfg.mappings.push_back({"", "", "", "Second alarm", "PLC_ALARM_SECOND", "WARNING", ""});

  // Same EventType + SourceNode for both; only the message differs.
  auto r1 = NodeMap::resolve_alarm(cfg, "", "ns=3;i=1845", "ns=3;i=1805", "Test Program Alert - trigger active");
  EXPECT_TRUE(r1.matched);
  EXPECT_EQ(r1.fault_code, "PLC_ALARM_TRIGGER");
  EXPECT_EQ(r1.severity_override, "ERROR");

  auto r2 = NodeMap::resolve_alarm(cfg, "", "ns=3;i=1845", "ns=3;i=1805", "Second alarm fired");
  EXPECT_TRUE(r2.matched);
  EXPECT_EQ(r2.fault_code, "PLC_ALARM_SECOND");

  // No substring match -> source-level catch-all.
  auto r3 = NodeMap::resolve_alarm(cfg, "", "ns=3;i=1845", "ns=3;i=1805", "Unrelated text");
  EXPECT_TRUE(r3.matched);
  EXPECT_EQ(r3.fault_code, "PLC_PROGRAM_ALARM");

  // Empty event message never matches a non-empty match_message -> catch-all.
  auto r4 = NodeMap::resolve_alarm(cfg, "", "ns=3;i=1845", "ns=3;i=1805", "");
  EXPECT_EQ(r4.fault_code, "PLC_PROGRAM_ALARM");

  // Case sensitivity: the substring match is case-sensitive, so a case-mismatched
  // event must NOT match and falls to the catch-all, while the exact case routes
  // to the distinct fault. A future case-insensitive find regresses r5 red.
  cfg.mappings.push_back({"", "", "", "Overtemp", "PLC_ALARM_OVERTEMP", "ERROR", ""});
  auto r5 = NodeMap::resolve_alarm(cfg, "", "ns=3;i=1845", "ns=3;i=1805", "overtemp fault");
  EXPECT_EQ(r5.fault_code, "PLC_PROGRAM_ALARM");  // wrong case -> catch-all
  auto r6 = NodeMap::resolve_alarm(cfg, "", "ns=3;i=1845", "ns=3;i=1805", "Overtemp fault");
  EXPECT_EQ(r6.fault_code, "PLC_ALARM_OVERTEMP");  // exact case -> distinct fault
}

TEST_F(NodeMapTest, LoadsMappingsAndAssociatedValues) {
  std::string path = "/tmp/test_node_map_mappings.yaml";
  std::ofstream f(path);
  f << R"(
area_id: test
component_id: test
event_alarms:
  - alarm_source: "ns=0;i=2253"
    entity_id: plant
    fault_code: PLC_GENERIC
    mappings:
      - condition_name: "Overpressure"
        fault_code: PLC_OVERPRESSURE
        severity_override: ERROR
        message: "Tank overpressure"
      - condition_name: "LowLevel"
        fault_code: PLC_LOW_LEVEL
    associated_values:
      - "SD_1"
      - name: "SD_2"
        label: "Setpoint"
)";
  f.close();

  NodeMap map;
  ASSERT_TRUE(map.load(path));
  ASSERT_EQ(map.event_alarms().size(), 1u);
  const auto & cfg = map.event_alarms()[0];
  EXPECT_EQ(cfg.fault_code, "PLC_GENERIC");
  ASSERT_EQ(cfg.mappings.size(), 2u);
  EXPECT_EQ(cfg.mappings[0].match_condition_name, "Overpressure");
  EXPECT_EQ(cfg.mappings[0].fault_code, "PLC_OVERPRESSURE");
  ASSERT_EQ(cfg.associated_values.size(), 2u);
  EXPECT_EQ(cfg.associated_values[0].name, "SD_1");
  EXPECT_EQ(cfg.associated_values[0].label, "SD_1");  // defaults to name
  EXPECT_EQ(cfg.associated_values[1].label, "Setpoint");

  // find_event_alarm resolves a mapping fault_code, not just the source code.
  EXPECT_NE(map.find_event_alarm("plant", "PLC_LOW_LEVEL"), nullptr);
  EXPECT_NE(map.find_event_alarm("plant", "PLC_GENERIC"), nullptr);
}

TEST_F(NodeMapTest, MappingOnlyEntryNeedsNoSourceFaultCode) {
  std::string path = "/tmp/test_node_map_mapping_only.yaml";
  std::ofstream f(path);
  f << R"(
area_id: test
component_id: test
event_alarms:
  - alarm_source: "ns=0;i=2253"
    entity_id: plant
    mappings:
      - condition_name: "A"
        fault_code: PLC_A
)";
  f.close();

  NodeMap map;
  ASSERT_TRUE(map.load(path));
  ASSERT_EQ(map.event_alarms().size(), 1u);
  EXPECT_TRUE(map.event_alarms()[0].fault_code.empty());
  EXPECT_EQ(map.event_alarms()[0].mappings.size(), 1u);
}

TEST_F(NodeMapTest, RejectsMappingFaultCodeCollisionWithThreshold) {
  // A mapping fault_code that collides with a threshold alarm fault_code on
  // the same entity must be rejected, same as a source-level collision.
  std::string path = "/tmp/test_node_map_mapping_collision.yaml";
  std::ofstream f(path);
  f << R"(
area_id: test
component_id: test
nodes:
  - node_id: "ns=1;i=1"
    entity_id: tank
    data_name: pressure
    alarm:
      fault_code: PLC_OVERPRESSURE
      threshold: 90.0
event_alarms:
  - alarm_source: "ns=0;i=2253"
    entity_id: tank
    mappings:
      - condition_name: "Overpressure"
        fault_code: PLC_OVERPRESSURE
)";
  f.close();

  NodeMap map;
  EXPECT_FALSE(map.load(path));
}

TEST_F(NodeMapTest, RejectsDuplicateFaultCodeAcrossEventAlarms) {
  // Issue #480: two event_alarms resolving to the same (entity_id, fault_code)
  // must be rejected, not silently deduped. At runtime distinct ConditionIds
  // are tracked separately and each reports, but a ClearFault is keyed on
  // fault_code alone, so one condition clearing would wipe the other's fault
  // while it is still active (flap).
  std::string path = "/tmp/test_node_map_dup_event_alarm.yaml";
  std::ofstream f(path);
  f << R"(
area_id: test
component_id: test
event_alarms:
  - alarm_source: "ns=2;s=Alarms.PumpA"
    entity_id: pump
    fault_code: PLC_PUMP_FAULT
  - alarm_source: "ns=2;s=Alarms.PumpB"
    entity_id: pump
    fault_code: PLC_PUMP_FAULT
)";
  f.close();

  NodeMap map;
  EXPECT_FALSE(map.load(path));
}

TEST_F(NodeMapTest, RejectsDuplicateFaultCodeAcrossEventAlarmMappings) {
  // Same flap hazard when the duplicate comes from two mappings (in different
  // sources) rather than two source-level fault_codes.
  std::string path = "/tmp/test_node_map_dup_event_alarm_mapping.yaml";
  std::ofstream f(path);
  f << R"(
area_id: test
component_id: test
event_alarms:
  - alarm_source: "ns=2;s=Alarms.A"
    entity_id: line
    mappings:
      - condition_name: "Jam"
        fault_code: PLC_LINE_STOP
  - alarm_source: "ns=2;s=Alarms.B"
    entity_id: line
    mappings:
      - condition_name: "EStop"
        fault_code: PLC_LINE_STOP
)";
  f.close();

  NodeMap map;
  EXPECT_FALSE(map.load(path));
}

TEST_F(NodeMapTest, RejectsSameFaultCodeOnDifferentEntities) {
  // Uniqueness is GLOBAL by fault_code, not per (entity_id, fault_code):
  // fault_manager keys clear_fault/get_fault by code alone, so the same code
  // on two entities still collides at the manager (one source's clear wipes
  // the other's fault). The loader must reject it. (issue #481)
  std::string path = "/tmp/test_node_map_same_code_diff_entity.yaml";
  std::ofstream f(path);
  f << R"(
area_id: test
component_id: test
event_alarms:
  - alarm_source: "ns=2;s=Alarms.PumpA"
    entity_id: pump_a
    fault_code: PLC_PUMP_FAULT
  - alarm_source: "ns=2;s=Alarms.PumpB"
    entity_id: pump_b
    fault_code: PLC_PUMP_FAULT
)";
  f.close();

  NodeMap map;
  EXPECT_FALSE(map.load(path));
}

// ---- Zero-config native A&C (auto_alarms) --------------------------------

TEST(DeriveAutoFaultCodeTest, UsesConditionNameWhenPresent) {
  // Tier 1 = slug(ConditionName) + a canonical-SourceNode hash suffix (the
  // suffix keeps same-named conditions on different sources distinct).
  const std::string code =
      NodeMap::derive_auto_fault_code("Overpressure", "SourceX", "ns=2;s=Tank", "ns=0;i=2915", "msg");
  EXPECT_EQ(code.rfind("PLC_ALARM_OVERPRESSURE_", 0), 0u);
}

TEST(DeriveAutoFaultCodeTest, ConditionNameSlugified) {
  // Dots, spaces and punctuation collapse to single underscores; case folds
  // to upper, matching every hand-written fault_code in this codebase
  // (PLC_OVERPRESSURE, PLC_COMMS_LOST, ...).
  const std::string code = NodeMap::derive_auto_fault_code("Tank.Overpressure!!", "", "ns=2;s=Tank", "ns=0;i=2915", "");
  EXPECT_EQ(code.rfind("PLC_ALARM_TANK_OVERPRESSURE_", 0), 0u);
}

TEST(DeriveAutoFaultCodeTest, FallsBackToSourceNameWhenConditionNameEmpty) {
  const std::string code = NodeMap::derive_auto_fault_code("", "Pump A Fault", "ns=2;s=PumpA", "ns=0;i=2915", "msg");
  EXPECT_EQ(code.rfind("PLC_ALARM_PUMP_A_FAULT_", 0), 0u);
}

TEST(DeriveAutoFaultCodeTest, HashFallbackWhenBothNamesEmpty) {
  const std::string code = NodeMap::derive_auto_fault_code("", "", "ns=3;i=1845", "ns=0;i=2915", "pa");
  EXPECT_EQ(code.rfind("PLC_ALARM_", 0), 0u);
  EXPECT_GT(code.size(), std::string("PLC_ALARM_").size());
  // Deterministic: same inputs -> same code.
  EXPECT_EQ(code, NodeMap::derive_auto_fault_code("", "", "ns=3;i=1845", "ns=0;i=2915", "pa"));
}

TEST(DeriveAutoFaultCodeTest, SharedSourceNodeDisambiguatedByMessage) {
  // Real Siemens S7-1500 quirk: every Program_Alarm of one FB shares a single
  // SourceNode (e.g. i=1845) with no ConditionName/SourceName, differing
  // only by Message ("pa" vs "pa2"). Without folding Message into the hash
  // fallback, both would collapse onto the same fault_code.
  const std::string pa = NodeMap::derive_auto_fault_code("", "", "ns=3;i=1845", "ns=0;i=2915", "pa");
  const std::string pa2 = NodeMap::derive_auto_fault_code("", "", "ns=3;i=1845", "ns=0;i=2915", "pa2");
  EXPECT_NE(pa, pa2);
}

TEST(DeriveAutoFaultCodeTest, EmptyConditionNameSlugFallsThroughToSourceName) {
  // A ConditionName that slugifies to nothing (pure punctuation) must not
  // stick with an empty-but-nonempty-input tier - it falls through exactly
  // like an absent ConditionName.
  const std::string code = NodeMap::derive_auto_fault_code("!!!", "PumpA", "ns=2;s=PumpA", "ns=0;i=2915", "");
  EXPECT_EQ(code.rfind("PLC_ALARM_PUMPA_", 0), 0u);
}

TEST(DeriveAutoFaultCodeTest, SameConditionNameDifferentSourcesGetDistinctCodes) {
  // Two identical FB instances each raising "Overpressure" from a DIFFERENT
  // SourceNode must not collapse onto one code: fault_manager keys/clears by
  // code alone, so a collision would let one condition's clear wipe the
  // other's still-active fault. This is the whole point of folding the
  // SourceNode into tiers 1/2.
  const std::string a = NodeMap::derive_auto_fault_code("Overpressure", "", "ns=2;s=PumpA", "ns=0;i=2915", "msg");
  const std::string b = NodeMap::derive_auto_fault_code("Overpressure", "", "ns=2;s=PumpB", "ns=0;i=2915", "msg");
  EXPECT_EQ(a.rfind("PLC_ALARM_OVERPRESSURE_", 0), 0u);
  EXPECT_EQ(b.rfind("PLC_ALARM_OVERPRESSURE_", 0), 0u);
  EXPECT_NE(a, b);
}

TEST(DeriveAutoFaultCodeTest, SameSourceNameDifferentSourcesGetDistinctCodes) {
  // Tier 2 (SourceName fallback) mirrors tier 1: distinct sources sharing a
  // SourceName stay distinct.
  const std::string a = NodeMap::derive_auto_fault_code("", "Pump Fault", "ns=2;s=PumpA", "ns=0;i=2915", "msg");
  const std::string b = NodeMap::derive_auto_fault_code("", "Pump Fault", "ns=2;s=PumpB", "ns=0;i=2915", "msg");
  EXPECT_NE(a, b);
}

TEST(DeriveAutoFaultCodeTest, StableAcrossMessageDriftForSameCondition) {
  // Tiers 1/2 must NOT fold Message: OPC-UA lets a condition's Message differ
  // between its active and inactive notifications, and the code must stay the
  // same so the clear matches the raise (fault_manager keys by code).
  const std::string active =
      NodeMap::derive_auto_fault_code("Overpressure", "", "ns=2;s=PumpA", "ns=0;i=2915", "active text");
  const std::string cleared =
      NodeMap::derive_auto_fault_code("Overpressure", "", "ns=2;s=PumpA", "ns=0;i=2915", "cleared text");
  EXPECT_EQ(active, cleared);
}

TEST(DeriveAutoFaultCodeTest, EquivalentSourceSpellingsProduceSameCode) {
  // The SourceNode is canonicalized before hashing, so the default numeric
  // Server object and its explicit namespaced spelling map to one code.
  const std::string numeric = NodeMap::derive_auto_fault_code("Overpressure", "", "i=2253", "ns=0;i=2915", "msg");
  const std::string namespaced =
      NodeMap::derive_auto_fault_code("Overpressure", "", "ns=0;i=2253", "ns=0;i=2915", "msg");
  EXPECT_EQ(numeric, namespaced);
}

TEST(AutoAlarmPassesFiltersTest, NoPatternsAdmitsEverything) {
  EXPECT_TRUE(NodeMap::auto_alarm_passes_filters("Overpressure", "Tank", "msg", {}, {}));
}

TEST(AutoAlarmPassesFiltersTest, ExcludeDropsMatchingSystemMessage) {
  EXPECT_FALSE(NodeMap::auto_alarm_passes_filters("", "Server", "CPU not in RUN", {}, {"CPU not in RUN"}));
}

TEST(AutoAlarmPassesFiltersTest, ExcludeWinsOverInclude) {
  EXPECT_FALSE(NodeMap::auto_alarm_passes_filters("Overpressure", "", "msg", {"Overpressure"}, {"Overpressure"}));
}

TEST(AutoAlarmPassesFiltersTest, NonEmptyIncludeRequiresAMatch) {
  EXPECT_FALSE(NodeMap::auto_alarm_passes_filters("Underpressure", "", "msg", {"Overpressure"}, {}));
  EXPECT_TRUE(NodeMap::auto_alarm_passes_filters("Overpressure", "", "msg", {"Overpressure"}, {}));
}

TEST(AutoAlarmPassesFiltersTest, IncludeMatchesAnyOfConditionNameSourceNameMessage) {
  EXPECT_TRUE(NodeMap::auto_alarm_passes_filters("", "", "contains PumpA text", {"PumpA"}, {}));
}

TEST(MapAutoSeverityTest, DefaultBandsMatchOpcuaPluginConvention) {
  AutoAlarmsSeverityBands bands;
  EXPECT_EQ(NodeMap::map_auto_severity(1000, bands), "CRITICAL");
  EXPECT_EQ(NodeMap::map_auto_severity(801, bands), "CRITICAL");
  EXPECT_EQ(NodeMap::map_auto_severity(800, bands), "ERROR");
  EXPECT_EQ(NodeMap::map_auto_severity(501, bands), "ERROR");
  EXPECT_EQ(NodeMap::map_auto_severity(500, bands), "WARNING");
  EXPECT_EQ(NodeMap::map_auto_severity(201, bands), "WARNING");
  EXPECT_EQ(NodeMap::map_auto_severity(200, bands), "INFO");
  EXPECT_EQ(NodeMap::map_auto_severity(1, bands), "INFO");
}

TEST(MapAutoSeverityTest, CustomBandsOverrideThresholds) {
  AutoAlarmsSeverityBands bands;
  bands.critical_min = 900;
  bands.error_min = 700;
  bands.warning_min = 400;
  EXPECT_EQ(NodeMap::map_auto_severity(850, bands), "ERROR");  // below the raised critical bar
  EXPECT_EQ(NodeMap::map_auto_severity(900, bands), "CRITICAL");
  EXPECT_EQ(NodeMap::map_auto_severity(399, bands), "INFO");
}

TEST_F(NodeMapTest, AutoAlarmsDefaultsOff) {
  NodeMap map;
  ASSERT_TRUE(map.load(yaml_path_));
  EXPECT_FALSE(map.auto_alarms().enabled);
}

TEST_F(NodeMapTest, AutoAlarmsBareBooleanEnablesWithDefaults) {
  std::string path = "/tmp/test_node_map_auto_alarms_bool.yaml";
  std::ofstream f(path);
  f << R"(
area_id: test
component_id: plc_runtime
nodes:
  - node_id: "ns=1;s=TankLevel"
    entity_id: tank_process
    data_name: tank_level
auto_alarms: true
)";
  f.close();

  NodeMap map;
  ASSERT_TRUE(map.load(path));
  const auto & cfg = map.auto_alarms();
  EXPECT_TRUE(cfg.enabled);
  EXPECT_EQ(cfg.source_node_id_str, "i=2253");
  // Defaults to "<component_id>_alarms" when not overridden - NOT the bare
  // component_id, which already names the PLC runtime's own Component entity
  // and would collide with it in the SOVD merge pipeline (reproduced against
  // a real Siemens S7-1500 during development).
  EXPECT_EQ(cfg.entity_id, "plc_runtime_alarms");
  EXPECT_TRUE(cfg.auto_clear);
  EXPECT_EQ(cfg.severity_bands.critical_min, 801);
  EXPECT_EQ(cfg.severity_bands.error_min, 501);
  EXPECT_EQ(cfg.severity_bands.warning_min, 201);
  EXPECT_TRUE(cfg.include_patterns.empty());
  EXPECT_TRUE(cfg.exclude_patterns.empty());
  // A file with only auto_alarms (no event_alarms) still gets its fallback
  // entity registered as fault-bearing for SOVD discovery.
  bool found = false;
  for (const auto & def : map.entity_defs()) {
    if (def.id == "plc_runtime_alarms") {
      found = true;
      EXPECT_TRUE(def.has_faults);
    }
  }
  EXPECT_TRUE(found);
}

TEST_F(NodeMapTest, AutoAlarmsOnlyNoNodesOrEventAlarmsIsValid) {
  // A config that declares ONLY auto_alarms (the true zero-config case) must
  // load successfully - the emptiness guard must not reject it.
  std::string path = "/tmp/test_node_map_auto_alarms_only.yaml";
  std::ofstream f(path);
  f << R"(
area_id: test
component_id: plc_runtime
auto_alarms: true
)";
  f.close();

  NodeMap map;
  EXPECT_TRUE(map.load(path));
}

TEST_F(NodeMapTest, AutoAlarmsMapFormOverridesEverything) {
  std::string path = "/tmp/test_node_map_auto_alarms_map.yaml";
  std::ofstream f(path);
  f << R"(
area_id: test
component_id: plc_runtime
auto_alarms:
  enabled: true
  source_node_id: "ns=3;i=1000"
  entity_id: alarms_catchall
  auto_clear: false
  severity_bands:
    critical: 900
    error: 700
    warning: 400
  include: ["Program_Alarm"]
  exclude: ["CPU not in RUN"]
)";
  f.close();

  NodeMap map;
  ASSERT_TRUE(map.load(path));
  const auto & cfg = map.auto_alarms();
  EXPECT_TRUE(cfg.enabled);
  EXPECT_EQ(cfg.source_node_id_str, "ns=3;i=1000");
  EXPECT_EQ(cfg.entity_id, "alarms_catchall");
  EXPECT_FALSE(cfg.auto_clear);
  EXPECT_EQ(cfg.severity_bands.critical_min, 900);
  EXPECT_EQ(cfg.severity_bands.error_min, 700);
  EXPECT_EQ(cfg.severity_bands.warning_min, 400);
  ASSERT_EQ(cfg.include_patterns.size(), 1u);
  EXPECT_EQ(cfg.include_patterns[0], "Program_Alarm");
  ASSERT_EQ(cfg.exclude_patterns.size(), 1u);
  EXPECT_EQ(cfg.exclude_patterns[0], "CPU not in RUN");
}

TEST_F(NodeMapTest, AutoAlarmsEnabledDefaultsTrueInMapForm) {
  // ``enabled:`` may be omitted inside the map form - presence of the map
  // itself implies intent to turn it on (matches the bare-boolean form).
  std::string path = "/tmp/test_node_map_auto_alarms_map_no_enabled.yaml";
  std::ofstream f(path);
  f << R"(
area_id: test
component_id: plc_runtime
auto_alarms:
  entity_id: alarms_catchall
)";
  f.close();

  NodeMap map;
  ASSERT_TRUE(map.load(path));
  EXPECT_TRUE(map.auto_alarms().enabled);
}

TEST_F(NodeMapTest, AutoAlarmsInvalidSeverityBandOrderResetsToDefaults) {
  std::string path = "/tmp/test_node_map_auto_alarms_bad_bands.yaml";
  std::ofstream f(path);
  f << R"(
area_id: test
component_id: plc_runtime
auto_alarms:
  severity_bands:
    critical: 300
    error: 500
    warning: 700
)";
  f.close();

  NodeMap map;
  ASSERT_TRUE(map.load(path));
  const auto & bands = map.auto_alarms().severity_bands;
  EXPECT_EQ(bands.critical_min, 801);
  EXPECT_EQ(bands.error_min, 501);
  EXPECT_EQ(bands.warning_min, 201);
}

TEST_F(NodeMapTest, AutoAlarmsFalseStaysDisabled) {
  std::string path = "/tmp/test_node_map_auto_alarms_false.yaml";
  std::ofstream f(path);
  f << R"(
area_id: test
component_id: plc_runtime
nodes:
  - node_id: "ns=1;s=TankLevel"
    entity_id: tank_process
    data_name: tank_level
auto_alarms: false
)";
  f.close();

  NodeMap map;
  ASSERT_TRUE(map.load(path));
  EXPECT_FALSE(map.auto_alarms().enabled);
}

// ---- Param-only zero-config overlay (finalize_auto_alarms_overlay) --------
// The true zero-config path: no node-map YAML is ever loaded; the plugin sets
// the AutoAlarmsConfig straight from the plugins.opcua.auto_alarms param via
// mutable_auto_alarms() and then finalize_auto_alarms_overlay() re-derives the
// fields load() normally fills in and rebuilds entity_defs.

TEST(AutoAlarmsFinalizeOverlayTest, ParamOnlyNoNodeMapProducesActiveConfigAndEntity) {
  NodeMap map;  // never loaded - no YAML at all
  ASSERT_FALSE(map.auto_alarms().enabled);

  auto & cfg = map.mutable_auto_alarms();
  cfg.enabled = true;  // as the plugin would set from `auto_alarms: true`

  ASSERT_TRUE(map.finalize_auto_alarms_overlay());

  const auto & finalized = map.auto_alarms();
  EXPECT_TRUE(finalized.enabled);
  // Default source parsed to a real NodeId (not the null default) so the
  // poller subscribes to the right EventNotifier in the no-node_map path.
  EXPECT_EQ(finalized.source_node_id_str, "i=2253");
  EXPECT_EQ(finalized.source_node_id.toString(), NodeMap::parse_node_id("i=2253").toString());
  // Entity defaulted from the neutral default component_id (config-less
  // discovery overrides it from device identity at runtime), and registered
  // as fault-bearing so SOVD discovery surfaces the alarms App with no node map.
  EXPECT_EQ(finalized.entity_id, "opcua_device_alarms");
  bool found = false;
  for (const auto & def : map.entity_defs()) {
    if (def.id == "opcua_device_alarms") {
      found = true;
      EXPECT_TRUE(def.has_faults);
    }
  }
  EXPECT_TRUE(found);
}

TEST(AutoAlarmsFinalizeOverlayTest, ComponentIdentityOverrideFlowsIntoAlarmEntity) {
  // Config-less discovery: the plugin sets the component id/name from the
  // device's own identity before enabling auto_alarms. The derived alarms
  // entity must key off the overridden id, not the neutral default.
  NodeMap map;
  map.set_component_identity("siemens_ag_cpu_1505sp_f", "Siemens AG CPU 1505SP F");
  EXPECT_EQ(map.component_id(), "siemens_ag_cpu_1505sp_f");
  EXPECT_EQ(map.component_name(), "Siemens AG CPU 1505SP F");

  map.mutable_auto_alarms().enabled = true;
  ASSERT_TRUE(map.finalize_auto_alarms_overlay());
  EXPECT_EQ(map.auto_alarms().entity_id, "siemens_ag_cpu_1505sp_f_alarms");

  // Empty id is a no-op guard (keeps the current identity).
  map.set_component_identity("", "ignored");
  EXPECT_EQ(map.component_id(), "siemens_ag_cpu_1505sp_f");
}

TEST(AutoAlarmsFinalizeOverlayTest, ParamOnlyRespectsOverriddenSourceAndEntity) {
  NodeMap map;
  auto & cfg = map.mutable_auto_alarms();
  cfg.enabled = true;
  cfg.source_node_id_str = "ns=3;i=1000";
  cfg.entity_id = "custom_alarms";

  ASSERT_TRUE(map.finalize_auto_alarms_overlay());

  EXPECT_EQ(map.auto_alarms().source_node_id.toString(), NodeMap::parse_node_id("ns=3;i=1000").toString());
  EXPECT_EQ(map.auto_alarms().entity_id, "custom_alarms");
}

TEST(AutoAlarmsFinalizeOverlayTest, EmptySourceNodeDisablesRatherThanCrashing) {
  NodeMap map;
  auto & cfg = map.mutable_auto_alarms();
  cfg.enabled = true;
  cfg.source_node_id_str = "";  // invalid

  EXPECT_FALSE(map.finalize_auto_alarms_overlay());
  EXPECT_FALSE(map.auto_alarms().enabled);
}

}  // namespace ros2_medkit_gateway
