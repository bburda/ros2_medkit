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
#include "ros2_medkit_graph_watchdog/detector_registry.hpp"

namespace ros2_medkit_graph_watchdog {

DetectorRegistry & DetectorRegistry::instance() {
  static DetectorRegistry registry;  // Meyers singleton: safe under static-init order.
  return registry;
}

void DetectorRegistry::register_factory(const std::string & id, DetectorFactory factory) {
  factories_.emplace_back(id, std::move(factory));
}

std::vector<std::unique_ptr<Detector>> DetectorRegistry::create_all() const {
  std::vector<std::unique_ptr<Detector>> detectors;
  detectors.reserve(factories_.size());
  for (const auto & [id, factory] : factories_) {
    (void)id;
    detectors.push_back(factory());
  }
  return detectors;
}

}  // namespace ros2_medkit_graph_watchdog
