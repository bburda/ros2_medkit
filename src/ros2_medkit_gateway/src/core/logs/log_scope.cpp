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

#include "ros2_medkit_gateway/core/logs/log_scope.hpp"

#include <algorithm>
#include <cstdlib>
#include <unordered_set>
#include <utility>
#include <vector>

#include "ros2_medkit_gateway/core/faults/fault_scope.hpp"
#include "ros2_medkit_gateway/core/managers/log_manager.hpp"
#include "ros2_medkit_gateway/core/models/thread_safe_entity_cache.hpp"

namespace ros2_medkit_gateway {
namespace logs {

namespace {

using json = nlohmann::json;

// Entry ids are "log_<n>" with a monotonic n (LogManager::entry_to_json).
unsigned long long entry_id_num(const json & item) {
  const std::string id = item.value("id", "");
  const std::size_t off = id.rfind("log_", 0) == 0 ? 4 : 0;
  return std::strtoull(id.c_str() + off, nullptr, 10);
}

}  // namespace

tl::expected<json, std::string> query_entity_logs(LogManager & log_mgr, const ThreadSafeEntityCache & cache,
                                                  SovdEntityType type, const std::string & entity_id,
                                                  const std::string & entity_fqn, const std::string & min_severity,
                                                  const std::string & context_filter,
                                                  std::set<std::string> * resolved_sources) {
  auto sources = faults::resolve_entity_source_fqns(cache, type, entity_id);
  if (resolved_sources) {
    *resolved_sources = sources;
  }

  std::vector<std::string> exact(sources.begin(), sources.end());
  // Legacy APP fallback: an app that is not in the cache (or resolves to no
  // source) keeps the entity.fqn exact query it always had.
  if (type == SovdEntityType::APP && exact.empty() && !entity_fqn.empty()) {
    exact.push_back(entity_fqn);
  }
  // Functions are pure aggregated views - no namespace fallback. APP has a
  // single source. Only grouping entities own a namespace prefix.
  const bool run_prefix = (type == SovdEntityType::COMPONENT || type == SovdEntityType::AREA) && !entity_fqn.empty();

  json merged = json::array();
  if (!exact.empty()) {
    auto result = log_mgr.get_logs(exact, false, min_severity, context_filter, entity_id);
    if (!result) {
      return tl::make_unexpected(result.error());
    }
    merged = std::move(*result);
  }

  if (run_prefix) {
    auto result = log_mgr.get_logs({entity_fqn}, true, min_severity, context_filter, entity_id);
    if (!result) {
      return tl::make_unexpected(result.error());
    }
    // Dedupe: an app FQN under the entity namespace is hit by both queries.
    std::unordered_set<std::string> seen;
    for (const auto & item : merged) {
      seen.insert(item.value("id", ""));
    }
    for (auto & item : *result) {
      if (seen.insert(item.value("id", "")).second) {
        merged.push_back(std::move(item));
      }
    }
    std::sort(merged.begin(), merged.end(), [](const json & a, const json & b) {
      return entry_id_num(a) < entry_id_num(b);
    });
    // Each get_logs call caps individually; re-cap the union to the entity
    // budget, keeping the most recent entries.
    auto cfg = log_mgr.get_config(entity_id);
    if (cfg && merged.size() > cfg->max_entries) {
      merged.erase(merged.begin(), merged.begin() + static_cast<std::ptrdiff_t>(merged.size() - cfg->max_entries));
    }
  }

  return merged;
}

}  // namespace logs
}  // namespace ros2_medkit_gateway
