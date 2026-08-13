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

// Frozen GRAPH_* fault-code namespace. All silent-fault
// detectors and downstream consumers reference these exact strings,
// so a code here is an external contract: never rename or repurpose one.
namespace ros2_medkit_graph_watchdog::graph_fault_codes {
inline constexpr const char * kQosMismatch = "GRAPH_QOS_MISMATCH";
inline constexpr const char * kOrphan = "GRAPH_ORPHAN";
inline constexpr const char * kNodeDisappeared = "GRAPH_NODE_DISAPPEARED";
inline constexpr const char * kTfStale = "GRAPH_TF_STALE";
inline constexpr const char * kParamDrift = "GRAPH_PARAM_DRIFT";
inline constexpr const char * kLatencyBudget = "GRAPH_LATENCY_BUDGET";
// Extension of the frozen namespace above (new capability, beyond the original
// six classes): a node the operator declared must-be-active is present but not active.
inline constexpr const char * kNodeInactive = "GRAPH_NODE_INACTIVE";
// Second extension, alongside kNodeInactive: a node the operator declared must-be-active
// is present and matched, but its lifecycle label has never been read - an unverified
// promise, not a confirmed violation, and deliberately its own record rather than content
// folded into kNodeInactive.
inline constexpr const char * kNodeUnreadable = "GRAPH_NODE_UNREADABLE";
// Third extension, sibling of kNodeUnreadable: a node the operator declared must-be-active
// is present and matched, but carries no tracked lifecycle at all (`lifecycle_state_of()`
// returns nullopt - a typo, or a plain non-lifecycle node). Like kNodeUnreadable this is an
// unverified promise, not a confirmed violation, and shares the same cause-blind unmeasured
// clock as kNodeUnreadable - the two differ only in which cause the clock matured under.
inline constexpr const char * kNodeNotManaged = "GRAPH_NODE_NOT_MANAGED";
}  // namespace ros2_medkit_graph_watchdog::graph_fault_codes
