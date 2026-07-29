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
#pragma once

#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "ros2_medkit_graph_watchdog/detector.hpp"
#include "ros2_medkit_graph_watchdog/detector_config_keys.hpp"  // collect_unknown_detector_keys

// The gateway delivers per-plugin config as a NESTED json:
// extract_plugin_config (param_utils.hpp) un-nests dotted keys, so a YAML param
// plugins.graph_watchdog.detectors.<id>.<field> arrives as
// config["detectors"]["<id>"]["<field>"], NOT the literal flat key
// "detectors.<id>.<field>". These helpers read that nested shape.
namespace ros2_medkit_graph_watchdog {

/// Resolve a detector's mode from nested plugin config (config["detectors"][id]["mode"]).
/// Returns Raise when absent. When `warnings` is non-null, a present-but-unrecognized
/// value appends one entry to it instead of being dropped silently.
///
/// A BARE `off` in YAML never arrives here as a string. rcl_yaml_param_parser follows
/// YAML 1.1, so `off`/`Off`/`OFF`/`no`/`n` are typed BOOL false and `on`/`yes` BOOL true;
/// only a QUOTED "off" stays a STRING. Since `off` is the value operators actually write,
/// a string-only reader silently leaves the detector in Raise - i.e. the detector the
/// operator just disabled keeps pushing faults at the robot, with nothing in the log to
/// connect the two. Reading the boolean form is what makes
///   detectors.<id>.mode: off
/// mean what it says. `true` maps to Raise so `mode: on` reads naturally too.
inline DetectorMode parse_detector_mode(const nlohmann::json & config, const std::string & id,
                                        std::vector<std::string> * warnings = nullptr) {
  if (!config.contains("detectors") || !config["detectors"].is_object()) {
    return DetectorMode::Raise;
  }
  const auto & detectors = config["detectors"];
  if (!detectors.contains(id) || !detectors[id].is_object()) {
    return DetectorMode::Raise;
  }
  const auto & entry = detectors[id];
  if (!entry.contains("mode")) {
    return DetectorMode::Raise;
  }
  const auto & mode_value = entry["mode"];
  if (mode_value.is_boolean()) {
    return mode_value.get<bool>() ? DetectorMode::Raise : DetectorMode::Off;
  }
  if (mode_value.is_string()) {
    const std::string mode = mode_value.get<std::string>();
    if (mode == "raise") {
      return DetectorMode::Raise;
    }
    if (mode == "advisory") {
      return DetectorMode::Advisory;
    }
    if (mode == "off") {
      return DetectorMode::Off;
    }
  }
  if (warnings != nullptr) {
    warnings->push_back("detector '" + id + "': unrecognized mode " + mode_value.dump() +
                        "; expected \"raise\", \"advisory\" or \"off\" - falling back to raise");
  }
  return DetectorMode::Raise;
}

/// Collect a detector's own fields (config["detectors"][id] minus the reserved "mode").
inline nlohmann::json extract_detector_config(const nlohmann::json & config, const std::string & id) {
  nlohmann::json out = nlohmann::json::object();
  if (!config.contains("detectors") || !config["detectors"].is_object()) {
    return out;
  }
  const auto & detectors = config["detectors"];
  if (!detectors.contains(id) || !detectors[id].is_object()) {
    return out;
  }
  for (const auto & item : detectors[id].items()) {
    if (item.key() != "mode") {
      out[item.key()] = item.value();
    }
  }
  return out;
}

}  // namespace ros2_medkit_graph_watchdog
