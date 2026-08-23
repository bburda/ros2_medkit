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

namespace ros2_medkit_graph_watchdog {

/// On what GROUNDS the presence detector (node_death) owns an entity's departure. The two
/// positive answers are not interchangeable, and collapsing them into one boolean is what
/// makes a node the graph later proves inactive stay owned by the wrong detector.
///
/// Stickiness is earned by KNOWLEDGE, never by IGNORANCE:
///
///   - kEarned survives anything the node does afterwards. It was granted because the node's
///     lifecycle state was read and said so, or because the node carries no lifecycle at all,
///     and a later deactivate does not hand the node back - that is the boundary this plugin
///     has always drawn (a node that ran and then stopped running is a death, whatever state
///     it stopped in).
///   - kProvisional is granted for the opposite reason: nothing is known, and nothing more
///     will be asked, so leaving the node unowned would mean nobody reports it at all. That
///     ground evaporates the instant a real label arrives, because the label tells us who
///     the node belongs to. A provisional owner must therefore hand the node back while it
///     is still alive, rather than discover at its death that it never had a claim.
///
/// The two NEGATIVE answers are just as different from each other, and conflating them is its
/// own defect: kUnclaimed says nothing is known yet, kDisowned says the graph has said whose
/// node this is. Only the second may take a key away from a caller that already holds it. A
/// node that is merely re-warming after a restart answers kUnclaimed, and treating that as a
/// verdict releases a key the detector must keep - its next death would then be reported by
/// nobody, and the detector would call an outage healthy every tick it lasted.
///
/// See ReliabilityGate::presence_ownership() for the exact per-state mapping.
enum class PresenceOwnership : std::uint8_t {
  kUnclaimed,    ///< nothing known either way yet: still warming up, or still being asked about
  kDisowned,     ///< measured, and measured as a state this detector does not report - another detector's
  kProvisional,  ///< owned only because the asking stopped with no answer; withdrawn as soon as one arrives
  kEarned,       ///< owned on a measurement, or on the absence of any lifecycle to measure; never withdrawn
};

}  // namespace ros2_medkit_graph_watchdog
