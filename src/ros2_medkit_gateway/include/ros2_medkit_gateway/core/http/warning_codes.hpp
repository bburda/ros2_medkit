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

namespace ros2_medkit_gateway {

/// Warning codes surfaced on /health.warnings.
///
/// These describe deployment anomalies the gateway chooses to flag without
/// taking itself offline (contrast with error codes in error_codes.hpp which
/// fail individual requests). Operators can key on these strings to drive
/// dashboards, alerts, or automated remediation.
///
/// Keep this list in sync with docs/api/warning_codes.rst. All codes MUST be
/// stable strings; rename = breaking change for downstream consumers.

/// A leaf (non-hierarchical) Component ID was announced by more than one peer
/// during aggregation merge. Routing falls back to last-writer-wins, so
/// requests for the affected Component reach one peer non-deterministically.
/// Resolve by renaming the Component on one side or by modelling it as a
/// hierarchical parent (declare a child Component with
/// ``parentComponentId`` pointing at the colliding ID on the owning peer).
inline constexpr const char * WARN_LEAF_ID_COLLISION = "leaf_id_collision";

/// One or more running ROS nodes are not declared in the manifest while
/// unmanifested_nodes is set to "error". The gateway keeps serving; the node
/// fully-qualified names are listed in ``ros_node_fqns`` and ``entity_ids`` is
/// empty, because a node FQN is not an addressable SOVD entity id. Resolve by
/// declaring them in the manifest, or by relaxing the policy to "warn" or
/// "ignore".
inline constexpr const char * WARN_UNMANIFESTED_NODES = "unmanifested_nodes";

/// Schema version for the ``warnings`` array on ``GET /health``. Clients can
/// key on this integer to detect supported warning codes without
/// string-matching on individual codes. Increment whenever a code is added,
/// removed, or the shape of an individual warning object changes. Keep in
/// sync with docs/api/warning_codes.rst.
///
/// Version 2 added ``unmanifested_nodes`` and the ``ros_node_fqns`` field,
/// and made the array unconditional: it used to be emitted only when
/// aggregation was active, which left every non-aggregating gateway unable to
/// report anything at all.
///
/// Invariant across all codes: ``entity_ids`` carries addressable SOVD entity
/// ids only, ``ros_node_fqns`` carries ROS node names, ``peer_names`` carries
/// aggregation peers, and all three are always present - empty when a code has
/// nothing to say in one of them.
inline constexpr int kWarningSchemaVersion = 2;

}  // namespace ros2_medkit_gateway
