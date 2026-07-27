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

#include <nlohmann/json.hpp>
#include <set>
#include <string>

#include "ros2_medkit_gateway/core/models/entity_types.hpp"

namespace ros2_medkit_gateway {

class ThreadSafeEntityCache;

namespace faults {

/// Resolve the single fault-reporting source an app owns, or "" when it owns
/// none. An external app (a plugin-introspected asset with no ROS binding)
/// reports under its bare entity id; every other app owns its `effective_fqn()`.
/// An unbound non-external app owns nothing - granting it its bare id would let
/// it claim faults it never reported.
std::string resolve_app_source_fqn(const ThreadSafeEntityCache & cache, const std::string & app_id);

/// Resolve the set of fault-reporting sources that fall within an entity's
/// scope by walking the entity cache. Each hosted app contributes what
/// `resolve_app_source_fqn` yields (bare entity id for an external app,
/// effective FQN otherwise):
///   - APP: the app's own source, if it owns one
///   - COMPONENT: every hosted app's source, plus the component's own id when
///     the component is external (protocol bridges report under that id)
///   - AREA: every source under the area and its (recursive) subareas
///   - FUNCTION: every source hosted directly or via a hosted component
/// Returns an empty set for SERVER / UNKNOWN.
///
/// Shared by the HTTP fault handlers (`GET /{entity}/faults`) and the ROS 2
/// plugin-context fault path so both agree on entity -> source-set resolution.
std::set<std::string> resolve_entity_source_fqns(const ThreadSafeEntityCache & cache, SovdEntityType type,
                                                 const std::string & entity_id);

/// True when `fault` has at least one reporting source and *every* reporting
/// source is within `source_fqns` (exact match or a path-boundary prefix).
/// An empty scope set or an empty/absent source list returns false.
bool fault_in_source_scope(const nlohmann::json & fault, const std::set<std::string> & source_fqns);

/// Subset of `faults_array` whose faults satisfy `fault_in_source_scope`.
nlohmann::json filter_faults_by_sources(const nlohmann::json & faults_array, const std::set<std::string> & source_fqns);

}  // namespace faults
}  // namespace ros2_medkit_gateway
