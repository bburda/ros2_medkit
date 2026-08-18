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

#include <cstddef>
#include <map>
#include <string>
#include <vector>

#include "ros2_medkit_graph_watchdog/detector.hpp"  // DetectorContext

namespace ros2_medkit_graph_watchdog {

/// One vote on whether a single candidate entity should be dropped from an aggregated
/// fault this tick. A detector holds a CHAIN of these and drops a candidate the moment
/// any of them votes yes - see apply_suppressors() below.
///
/// Pure by contract: an implementation reads only the key it is asked about and the
/// DetectorContext it is handed (gate, snapshot, clock), and never raises or clears a
/// fault itself. The interface exists so more than one detector can share the same
/// suppression mechanism (AllowlistSuppressor already serves node_death and tf_stale)
/// without either one owning the other's config surface.
class Suppressor {
 public:
  virtual ~Suppressor() = default;

  /// True to suppress `entity_key` this tick; false to abstain (say nothing about it).
  virtual bool suppresses(const std::string & entity_key, const DetectorContext & ctx) const = 0;

  /// Whether a "yes" from this suppressor is a standing fact about the key rather than a
  /// condition that can later stop holding.
  ///
  /// This is what a detector's own pruning may safely rely on: reclaiming a tracked key's
  /// bookkeeping while it is suppressed only avoids a false report FOR AS LONG AS the
  /// suppression keeps holding. A veto that CAN lift (a live condition on the current
  /// graph, say) offers no such guarantee - the moment it lifts, the key would be a live,
  /// unsuppressed death with no tracking left to report it, which is a false heal that
  /// silently outlives the very condition that caused it. A veto that is durable never
  /// lifts for a given key once it has fired, so reclaiming under it loses nothing: the
  /// key would never be reported again regardless of whether its bookkeeping survives.
  ///
  /// Defaults to false, which is the safe assumption for a suppressor nobody has reasoned
  /// about yet. Override to true only for a suppressor whose "yes" is permanent per key.
  virtual bool durable() const {
    return false;
  }
};

/// Drop every key in `affected` that any suppressor in `chain` votes to suppress.
///
/// Order-independent: a key survives only if EVERY suppressor abstains, so which one
/// happens to run first never changes the result. Returns how many keys were dropped, for
/// callers that want to know without re-diffing `affected` themselves.
inline std::size_t apply_suppressors(std::map<std::string, std::string> & affected,
                                     const std::vector<const Suppressor *> & chain, const DetectorContext & ctx) {
  if (chain.empty()) {
    return 0;
  }
  std::size_t dropped = 0;
  for (auto it = affected.begin(); it != affected.end();) {
    bool suppressed = false;
    for (const Suppressor * s : chain) {
      if (s != nullptr && s->suppresses(it->first, ctx)) {
        suppressed = true;
        break;
      }
    }
    if (suppressed) {
      it = affected.erase(it);
      ++dropped;
    } else {
      ++it;
    }
  }
  return dropped;
}

}  // namespace ros2_medkit_graph_watchdog
