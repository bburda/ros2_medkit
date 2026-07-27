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
#include <tl/expected.hpp>

#include "ros2_medkit_gateway/core/models/entity_types.hpp"

namespace ros2_medkit_gateway {

class LogManager;
class ThreadSafeEntityCache;

namespace logs {

/// Query every log entry within an entity's scope.
///
/// Exact-match sources come from `faults::resolve_entity_source_fqns`, so the
/// scope rule matches faults and bulk-data: an external app stores entries
/// under its bare entity id, an external component also owns entries stored
/// under its own id, areas recurse subareas and functions follow component
/// hosts. COMPONENT and AREA additionally run `entity_fqn` as a namespace
/// prefix query - hosted-app resolution must not disable it, or a manifest
/// component that hosts an external app loses the ROS node entries under its
/// namespace. APP falls back to `entity_fqn` (exact) when the resolver yields
/// nothing. Both result sets are merged, deduped by entry id, sorted by id
/// ascending and re-capped to the entity's configured max_entries.
///
/// `resolved_sources` (optional out) receives the exact-match source set for
/// response metadata. Shared by the HTTP log handlers and the cyclic
/// subscription "logs" sampler so both agree on entity -> log-source scoping.
tl::expected<nlohmann::json, std::string>
query_entity_logs(LogManager & log_mgr, const ThreadSafeEntityCache & cache, SovdEntityType type,
                  const std::string & entity_id, const std::string & entity_fqn, const std::string & min_severity,
                  const std::string & context_filter, std::set<std::string> * resolved_sources = nullptr);

}  // namespace logs
}  // namespace ros2_medkit_gateway
