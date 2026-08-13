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

#include <chrono>
#include <cstdint>

#include <rclcpp/rclcpp.hpp>

namespace ros2_medkit_graph_watchdog {

/// Thin wrapper over the owning node's clock. Honors use_sim_time (node clock).
/// time_is_valid() flags a paused/absent /clock so time-based detectors can skip
/// their age/grace math instead of firing a false positive.
class WatchdogClock {
 public:
  explicit WatchdogClock(rclcpp::Node * node)
    : clock_(node->get_clock()), use_sim_time_(node->get_parameter_or("use_sim_time", false)) {
  }

  rclcpp::Time now() const {
    return clock_->now();
  }

  /// Called once per tick by the plugin. Samples sim + wall time to detect a stall.
  void mark_tick() {
    const rclcpp::Time sim_now = clock_->now();
    const auto wall_now = std::chrono::steady_clock::now();
    if (have_sample_ && use_sim_time_) {
      const bool wall_advanced = (wall_now - last_wall_) > std::chrono::milliseconds(1);
      const bool sim_advanced = sim_now.nanoseconds() > last_sim_ns_;
      // Wall time moving while sim time stands still is the stall. Anything else is fine:
      // wall not advancing says nothing, and sim advancing proves the clock is live.
      valid_ = !wall_advanced || sim_advanced;
    }
    last_sim_ns_ = sim_now.nanoseconds();
    last_wall_ = wall_now;
    have_sample_ = true;
  }

  bool time_is_valid() const {
    return valid_;
  }

 private:
  rclcpp::Clock::SharedPtr clock_;
  bool use_sim_time_;
  bool have_sample_ = false;
  int64_t last_sim_ns_ = 0;
  std::chrono::steady_clock::time_point last_wall_{};
  bool valid_ = true;
};

}  // namespace ros2_medkit_graph_watchdog
