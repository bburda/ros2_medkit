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

#include "ros2_medkit_graph_watchdog/reliability_gate.hpp"  // DepartedLifecycle
#include "ros2_medkit_graph_watchdog/suppressor.hpp"

namespace ros2_medkit_graph_watchdog {

/// Self-suppression: a managed lifecycle node that reached a clean shutdown before it left
/// the graph was stopped on purpose, so node_death must not report its departure. Reads
/// `ReliabilityGate::departed_lifecycle_state_of(fqn)` - the seam that header documents for
/// exactly this - rather than owning any state of its own.
///
/// A departure counts as clean only for two labels, and one of them still needs corroborating
/// evidence:
///   - `shuttingdown`: only reachable via a deliberate SHUTDOWN transition, so the label alone
///     is enough.
///   - `finalized`: NOT enough on its own. It is also where a node's on_error override lands
///     after ON_ERROR_FAILURE/ON_ERROR_ERROR out of `errorprocessing` - the standard way a
///     driver reports a hardware fault it cannot recover from, which is precisely the death
///     an operator needs reported, not silenced. So `finalized` only counts when the watcher
///     actually saw a transition into it (`saw_transition`) and never saw it pass through the
///     error branch (`!error_terminated`). Without an observed transition history the
///     departure is unclassified and stays reported.
///
/// `unconfigured` is deliberately excluded: it is also the resting state of a node whose
/// configure() failed or that never activated at all, and suppressing on it would hide exactly
/// that startup failure.
///
/// Stateless by design: a detector's own `configure()` (where the suppressor chain is built)
/// runs before any DetectorContext exists, so this class stores nothing at construction and
/// reads the gate fresh on every call instead.
class LifecycleShutdownSuppressor : public Suppressor {
 public:
  bool suppresses(const std::string & fqn, const DetectorContext & ctx) const override {
    if (ctx.gate == nullptr) {
      return false;  // not yet wired (e.g. a bare-context test) -> nothing to read, abstain
    }
    const auto departed = ctx.gate->departed_lifecycle_state_of(fqn);
    if (!departed.has_value()) {
      return false;  // never departed within retention, or this fqn was never lifecycle-tracked
    }
    if (departed->label == "shuttingdown") {
      return true;
    }
    if (departed->label == "finalized") {
      return departed->saw_transition && !departed->error_terminated;
    }
    return false;
  }

  /// A departure's shutdown shape does not change after the fact - once a fqn's last
  /// observed transition qualifies as clean, it stays clean for as long as the gate's
  /// retention window remembers it at all. Reclaiming a suppressed key's tracker
  /// bookkeeping is therefore sound PROVIDED that window outlives node_death's own reclaim
  /// tick, which is what GraphWatchdogPlugin::compute_departed_retention_ticks() sizes it
  /// for.
  bool durable() const override {
    return true;
  }
};

}  // namespace ros2_medkit_graph_watchdog
