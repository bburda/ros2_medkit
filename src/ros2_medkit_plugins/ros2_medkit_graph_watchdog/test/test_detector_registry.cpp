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
#include <gtest/gtest.h>

#include <algorithm>

#include "ros2_medkit_graph_watchdog/detector.hpp"
#include "ros2_medkit_graph_watchdog/detector_registry.hpp"

namespace {

// Test-only detector proving registration -> create -> tick. Not shipped.
class NoopDetector : public ros2_medkit_graph_watchdog::Detector {
 public:
  std::string id() const override {
    return "noop";
  }
  void tick(ros2_medkit_graph_watchdog::DetectorContext & /*ctx*/) override {
  }
};

REGISTER_DETECTOR(NoopDetector, "noop")

}  // namespace

TEST(DetectorRegistry, SelfRegistersAndCreates) {
  auto detectors = ros2_medkit_graph_watchdog::DetectorRegistry::instance().create_all();
  const bool has_noop = std::any_of(detectors.begin(), detectors.end(), [](const auto & d) {
    return d->id() == "noop";
  });
  EXPECT_TRUE(has_noop);
}

TEST(DetectorRegistry, CreatedDetectorTicksWithoutThrow) {
  auto detectors = ros2_medkit_graph_watchdog::DetectorRegistry::instance().create_all();
  ros2_medkit_graph_watchdog::DetectorContext ctx;  // empty context is fine for a no-op
  for (auto & d : detectors) {
    if (d->id() == "noop") {
      EXPECT_NO_THROW(d->tick(ctx));
    }
  }
}
