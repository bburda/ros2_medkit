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
#include "ros2_medkit_graph_watchdog/detector_config.hpp"

#include <string>
#include <vector>

#include <gtest/gtest.h>

namespace {
using ros2_medkit_graph_watchdog::collect_unknown_detector_keys;
using ros2_medkit_graph_watchdog::DetectorMode;
using ros2_medkit_graph_watchdog::extract_detector_config;
using ros2_medkit_graph_watchdog::parse_detector_mode;

// The gateway delivers NESTED config (extract_plugin_config un-nests dotted keys).
nlohmann::json nested() {
  return {{"detectors",
           {{"qos", {{"mode", "off"}, {"threshold", 5}}}, {"tf", {{"mode", "advisory"}}}, {"other", {{"x", 1}}}}}};
}

TEST(DetectorConfig, ParsesModeFromNested) {
  EXPECT_EQ(parse_detector_mode(nested(), "qos"), DetectorMode::Off);
  EXPECT_EQ(parse_detector_mode(nested(), "tf"), DetectorMode::Advisory);
  EXPECT_EQ(parse_detector_mode(nested(), "absent"), DetectorMode::Raise);  // missing -> Raise
}
TEST(DetectorConfig, UnknownModeFallsBackToRaiseAndWarns) {
  const nlohmann::json cfg = {{"detectors", {{"qos", {{"mode", "bogus"}}}}}};
  std::vector<std::string> warnings;
  EXPECT_EQ(parse_detector_mode(cfg, "qos", &warnings), DetectorMode::Raise);
  ASSERT_EQ(warnings.size(), 1u);  // a silent fallback to Raise is the costly direction
  EXPECT_NE(warnings[0].find("qos"), std::string::npos);
}

// A BARE `off` in YAML is typed BOOL false by rcl_yaml_param_parser (YAML 1.1), so it
// never arrives as a string. Reading only strings left the detector the operator just
// disabled still raising - the exact failure direction that must not be silent.
TEST(DetectorConfig, YamlBooleanFalseMeansOff) {
  const nlohmann::json cfg = {{"detectors", {{"qos", {{"mode", false}}}}}};
  std::vector<std::string> warnings;
  EXPECT_EQ(parse_detector_mode(cfg, "qos", &warnings), DetectorMode::Off);
  EXPECT_TRUE(warnings.empty());  // a recognized form must not warn
}

TEST(DetectorConfig, YamlBooleanTrueMeansRaise) {
  const nlohmann::json cfg = {{"detectors", {{"qos", {{"mode", true}}}}}};
  std::vector<std::string> warnings;
  EXPECT_EQ(parse_detector_mode(cfg, "qos", &warnings), DetectorMode::Raise);
  EXPECT_TRUE(warnings.empty());
}

TEST(DetectorConfig, ExplicitRaiseStringIsRecognized) {
  const nlohmann::json cfg = {{"detectors", {{"qos", {{"mode", "raise"}}}}}};
  std::vector<std::string> warnings;
  EXPECT_EQ(parse_detector_mode(cfg, "qos", &warnings), DetectorMode::Raise);
  EXPECT_TRUE(warnings.empty());  // spelled out explicitly, so not a typo
}

TEST(DetectorConfig, NonScalarModeWarnsRatherThanBeingDropped) {
  const nlohmann::json cfg = {{"detectors", {{"qos", {{"mode", nlohmann::json::array({"off"})}}}}}};
  std::vector<std::string> warnings;
  EXPECT_EQ(parse_detector_mode(cfg, "qos", &warnings), DetectorMode::Raise);
  EXPECT_EQ(warnings.size(), 1u);
}

TEST(DetectorConfig, AbsentModeNeverWarns) {
  std::vector<std::string> warnings;
  EXPECT_EQ(parse_detector_mode(nested(), "absent", &warnings), DetectorMode::Raise);
  EXPECT_TRUE(warnings.empty());  // not configuring a detector is not an error
}

TEST(DetectorConfig, UnknownKeysAreReportedAgainstTheKnownSet) {
  const nlohmann::json cfg = {{"ignore_globs", nlohmann::json::array({"*_stamp"})}, {"baseline", false}};
  std::vector<std::string> warnings;
  collect_unknown_detector_keys(cfg, {"baseline", "ignore"}, warnings);
  ASSERT_EQ(warnings.size(), 1u);  // `baseline` is known; only the misspelt key warns
  EXPECT_NE(warnings[0].find("ignore_globs"), std::string::npos);
  EXPECT_NE(warnings[0].find("ignore"), std::string::npos);  // names the set it should have matched
}

TEST(DetectorConfig, NoUnknownKeyWarningsWhenEverythingIsKnown) {
  const nlohmann::json cfg = {{"baseline", false}, {"ignore", nlohmann::json::array()}};
  std::vector<std::string> warnings;
  collect_unknown_detector_keys(cfg, {"baseline", "ignore"}, warnings);
  EXPECT_TRUE(warnings.empty());
}
TEST(DetectorConfig, ExtractsOwnFieldsExcludingMode) {
  const auto cfg = extract_detector_config(nested(), "qos");
  EXPECT_EQ(cfg.value("threshold", 0), 5);
  EXPECT_FALSE(cfg.contains("mode"));  // mode is consumed by parse_detector_mode
  EXPECT_FALSE(cfg.contains("x"));     // other detector's field must not leak
}
TEST(DetectorConfig, MissingDetectorYieldsEmpty) {
  EXPECT_TRUE(extract_detector_config(nested(), "absent").empty());
  EXPECT_TRUE(extract_detector_config(nlohmann::json::object(), "qos").empty());
}
}  // namespace
