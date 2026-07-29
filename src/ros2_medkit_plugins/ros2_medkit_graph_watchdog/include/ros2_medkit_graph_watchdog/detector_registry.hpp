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

#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "ros2_medkit_graph_watchdog/detector.hpp"

namespace ros2_medkit_graph_watchdog {

using DetectorFactory = std::function<std::unique_ptr<Detector>()>;

/// Process-wide registry of detector factories. Populated at dlopen time by the
/// REGISTER_DETECTOR static initializers compiled into the plugin .so.
class DetectorRegistry {
 public:
  static DetectorRegistry & instance();

  ~DetectorRegistry() = default;
  DetectorRegistry(const DetectorRegistry &) = delete;
  DetectorRegistry & operator=(const DetectorRegistry &) = delete;
  DetectorRegistry(DetectorRegistry &&) = delete;
  DetectorRegistry & operator=(DetectorRegistry &&) = delete;

  void register_factory(const std::string & id, DetectorFactory factory);
  std::vector<std::unique_ptr<Detector>> create_all() const;

 private:
  DetectorRegistry() = default;
  std::vector<std::pair<std::string, DetectorFactory>> factories_;
};

}  // namespace ros2_medkit_graph_watchdog

/// Declare + self-register a detector. One detector per file in src/detectors/;
/// add this macro at file scope. The static initializer runs at dlopen; no CMake
/// edit and no registry edit are needed (sources are globbed).
#define REGISTER_DETECTOR(CLASS, ID)                                                       \
  namespace {                                                                              \
  const bool CLASS##_registered = [] {                                                     \
    ::ros2_medkit_graph_watchdog::DetectorRegistry::instance().register_factory((ID), [] { \
      return std::make_unique<CLASS>();                                                    \
    });                                                                                    \
    return true;                                                                           \
  }();                                                                                     \
  }  // namespace
