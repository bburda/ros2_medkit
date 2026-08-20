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

#include <map>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

#include "ros2_medkit_gateway/core/http/member_qualified_id.hpp"
#include "ros2_medkit_gateway/core/models/thread_safe_entity_cache.hpp"

namespace ros2_medkit_gateway {
namespace http {

/**
 * @brief Which item half an operation's id carries.
 *
 * An operation's wire id is the last segment of its ROS path, so a short name
 * is unique only as far as its provider's own graph makes it so. Two axes
 * decide the id, and they are independent:
 *
 *  - the MEMBER half, from `member_qualified_id.hpp`, separates copies that
 *    belong to different members;
 *  - the ITEM half, decided here, separates copies that belong to the SAME
 *    member - there the member half names one member twice and separates
 *    nothing, so the ROS path stands in for the short name.
 *
 * Everything that emits an operation id - the collection, the per-entity
 * OpenAPI document - and everything that resolves one has to read the same
 * rule, or the gateway documents and lists a request it then refuses.
 */

/// The full paths whose short name their OWN provider carries more than once.
///
/// Every other operation keeps its short name, which is the id every current
/// client and the generated OpenAPI document send.
///
/// An entity that exposes its operations directly records no owner for them.
/// The empty owner is one provider, which is exactly what a plain App is.
inline std::unordered_set<std::string> operation_paths_addressed_by_path(const AggregatedOperations & ops) {
  std::map<std::pair<std::string, std::string>, std::vector<std::string>> paths_by_provider;
  const auto record = [&ops, &paths_by_provider](const std::string & name, const std::string & full_path) {
    auto owner = ops.owner_by_path.find(full_path);
    const std::string member = owner != ops.owner_by_path.end() ? owner->second : std::string{};
    paths_by_provider[std::make_pair(member, name)].push_back(full_path);
  };
  for (const auto & svc : ops.services) {
    record(svc.name, svc.full_path);
  }
  for (const auto & act : ops.actions) {
    record(act.name, act.full_path);
  }

  std::unordered_set<std::string> shared;
  for (const auto & entry : paths_by_provider) {
    if (entry.second.size() < 2) {
      continue;
    }
    shared.insert(entry.second.begin(), entry.second.end());
  }
  return shared;
}

/// The item half `full_path` is addressed by, given the paths its provider
/// carries under a shared short name.
inline std::string operation_item_half(const std::string & name, const std::string & full_path,
                                       const std::unordered_set<std::string> & addressed_by_path) {
  return addressed_by_path.count(full_path) > 0u ? path_item_id(full_path) : name;
}

/// True when `item_id` addresses this operation.
///
/// Two forms, both exact: the short name, and the ROS path without its leading
/// slash. A short name never contains a slash, so neither form can be read as
/// the other.
inline bool operation_item_id_names(const std::string & name, const std::string & full_path,
                                    const std::string & item_id) {
  return item_id == name || item_id == path_item_id(full_path);
}

}  // namespace http
}  // namespace ros2_medkit_gateway
