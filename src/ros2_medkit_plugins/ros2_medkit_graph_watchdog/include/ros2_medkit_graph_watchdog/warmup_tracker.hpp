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

#include <cstdint>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace ros2_medkit_graph_watchdog {

/// Per-entity bringup-quiesce. An entity is "armed" once it has been continuously
/// present for `warmup_cycles` ticks. An entity that vanishes for more than
/// `forget_grace` ticks and later reappears is treated as a restart and re-warmed; a
/// shorter gap is treated as transient discovery churn and does NOT re-warm. Pure; no ROS.
class WarmupTracker {
 public:
  /// A one-tick ROS 2 graph-poll gap is indistinguishable from a real restart, and DDS
  /// discovery churn routinely drops a node from a single poll without it restarting.
  /// Absorbing a few missed ticks stops that churn from perpetually re-arming (and thus
  /// permanently suppressing) an entity whose gaps recur faster than the warmup window.
  static constexpr uint64_t kDefaultForgetGrace = 2;

  struct Entry {
    uint64_t first_seen;  ///< tick the current continuous-presence streak began (drives arming)
    uint64_t last_seen;   ///< most recent tick the entity was present (drives forget grace)
  };

  explicit WarmupTracker(int warmup_cycles, uint64_t forget_grace = kDefaultForgetGrace)
    : warmup_cycles_(warmup_cycles < 0 ? 0 : warmup_cycles), forget_grace_(forget_grace) {
  }

  /// Record the entity ids present at `tick`. New ids get first_seen=tick; a still-known
  /// id keeps its original first_seen and refreshes last_seen; an id missing for longer
  /// than forget_grace_ ticks is forgotten so a later reappearance re-warms.
  void update(const std::vector<std::string> & present_ids, uint64_t tick) {
    std::unordered_set<std::string> present(present_ids.begin(), present_ids.end());
    for (auto it = entries_.begin(); it != entries_.end();) {
      const bool missing = present.count(it->first) == 0;
      if (missing && (tick - it->second.last_seen) > forget_grace_) {
        it = entries_.erase(it);
      } else {
        ++it;
      }
    }
    for (const auto & id : present_ids) {
      auto [it, inserted] = entries_.try_emplace(id, Entry{tick, tick});
      if (!inserted) {
        it->second.last_seen = tick;  // continuous (or within-grace) presence keeps first_seen
      }
    }
  }

  bool is_known(const std::string & id) const {
    return entries_.count(id) > 0;
  }

  bool is_armed(const std::string & id, uint64_t tick) const {
    auto it = entries_.find(id);
    if (it == entries_.end()) {
      return false;
    }
    return (tick - it->second.first_seen) >= static_cast<uint64_t>(warmup_cycles_);
  }

  const std::unordered_map<std::string, Entry> & entries() const {
    return entries_;
  }

 private:
  int warmup_cycles_;
  uint64_t forget_grace_;
  std::unordered_map<std::string, Entry> entries_;
};

}  // namespace ros2_medkit_graph_watchdog
