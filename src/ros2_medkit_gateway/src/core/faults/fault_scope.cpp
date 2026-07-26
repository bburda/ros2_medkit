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

#include "ros2_medkit_gateway/core/faults/fault_scope.hpp"

#include <utility>
#include <vector>

#include "ros2_medkit_gateway/core/models/thread_safe_entity_cache.hpp"

namespace ros2_medkit_gateway {
namespace faults {

namespace {

void collect_app_fqn(const ThreadSafeEntityCache & cache, const std::string & app_id, std::set<std::string> & out) {
  auto fqn = resolve_app_source_fqn(cache, app_id);
  if (fqn.empty()) {
    return;
  }
  out.insert(std::move(fqn));
}

void collect_component_app_fqns(const ThreadSafeEntityCache & cache, const std::string & comp_id,
                                std::set<std::string> & out) {
  // An external component is a reporting source in its own right, even when
  // child apps also contribute: protocol bridge plugins raise faults (e.g.
  // PLC_COMMS_LOST) with the component's own id as source_id, and those must
  // not vanish from /components/<id>/faults just because the component hosts
  // apps. A non-external component never claims its bare id - an unbound ROS
  // component must not own faults it never reported (#516 guardrail).
  auto comp = cache.get_component(comp_id);
  if (comp && comp->external.value_or(false)) {
    out.insert(comp_id);
  }
  for (const auto & app_id : cache.get_apps_for_component(comp_id)) {
    collect_app_fqn(cache, app_id, out);
  }
}

void collect_area_app_fqns(const ThreadSafeEntityCache & cache, const std::string & area_id,
                           std::set<std::string> & out) {
  // BFS over (area, subareas...) so a top-level area whose components live in
  // nested subareas still resolves to the union of every descendant's apps.
  // Without the recursion, e.g. `/areas/powertrain/...` returns an empty set
  // when components are attached to `engine` (subarea of `powertrain`).
  std::vector<std::string> pending = {area_id};
  std::set<std::string> visited;
  while (!pending.empty()) {
    auto current = std::move(pending.back());
    pending.pop_back();
    if (!visited.insert(current).second) {
      continue;
    }
    for (const auto & comp_id : cache.get_components_for_area(current)) {
      collect_component_app_fqns(cache, comp_id, out);
    }
    for (const auto & sub_id : cache.get_subareas(current)) {
      pending.push_back(sub_id);
    }
  }
}

void collect_function_app_fqns(const ThreadSafeEntityCache & cache, const std::string & function_id,
                               std::set<std::string> & out) {
  // Function.hosts can contain either App IDs or Component IDs; the indexed
  // lookups in the cache only resolve the App-host case (function_to_apps_),
  // so we walk the raw `hosts` list and dispatch per host kind ourselves.
  auto func = cache.get_function(function_id);
  if (!func) {
    return;
  }
  for (const auto & host_id : func->hosts) {
    if (cache.get_app(host_id)) {
      collect_app_fqn(cache, host_id, out);
    } else if (cache.get_component(host_id)) {
      collect_component_app_fqns(cache, host_id, out);
    }
    // Unknown host - silently skip; it would have been flagged by manifest validation.
  }
}

bool source_matches_scope(const std::string & src, const std::set<std::string> & scope_fqns) {
  for (const auto & fqn : scope_fqns) {
    if (src == fqn) {
      return true;
    }
    if (src.size() > fqn.size() && src.compare(0, fqn.size(), fqn) == 0 && src[fqn.size()] == '/') {
      return true;
    }
  }
  return false;
}

}  // namespace

std::string resolve_app_source_fqn(const ThreadSafeEntityCache & cache, const std::string & app_id) {
  auto app = cache.get_app(app_id);
  if (!app) {
    return "";
  }
  // External assets introspected by a protocol plugin (e.g. a PLC over OPC UA)
  // report faults to the fault_manager under their own entity id, so that id is
  // their sole fault-scope owner. This must be checked before effective_fqn():
  // a manifest may declare a stray ros_binding on an external app, and the
  // derived FQN would otherwise shadow the bare id and drop its faults from
  // every rollup (#517, neighbor case).
  if (app->external.value_or(false)) {
    return app_id;
  }
  // A non-external app that merely failed to bind owns nothing: granting an
  // unbound ROS app its bare id would let it claim faults it never reported.
  return app->effective_fqn();
}

std::set<std::string> resolve_entity_source_fqns(const ThreadSafeEntityCache & cache, SovdEntityType type,
                                                 const std::string & entity_id) {
  std::set<std::string> fqns;
  switch (type) {
    case SovdEntityType::APP:
      collect_app_fqn(cache, entity_id, fqns);
      break;
    case SovdEntityType::COMPONENT:
      collect_component_app_fqns(cache, entity_id, fqns);
      break;
    case SovdEntityType::AREA:
      collect_area_app_fqns(cache, entity_id, fqns);
      break;
    case SovdEntityType::FUNCTION:
      collect_function_app_fqns(cache, entity_id, fqns);
      break;
    case SovdEntityType::SERVER:
    case SovdEntityType::UNKNOWN:
      break;
  }
  return fqns;
}

bool fault_in_source_scope(const nlohmann::json & fault, const std::set<std::string> & source_fqns) {
  if (source_fqns.empty()) {
    return false;
  }
  if (!fault.contains("reporting_sources") || !fault["reporting_sources"].is_array()) {
    return false;
  }
  const auto & sources = fault["reporting_sources"];
  if (sources.empty()) {
    return false;
  }
  for (const auto & src : sources) {
    if (!src.is_string()) {
      return false;
    }
    if (!source_matches_scope(src.get<std::string>(), source_fqns)) {
      return false;
    }
  }
  return true;
}

nlohmann::json filter_faults_by_sources(const nlohmann::json & faults_array,
                                        const std::set<std::string> & source_fqns) {
  nlohmann::json filtered = nlohmann::json::array();
  for (const auto & fault : faults_array) {
    if (fault_in_source_scope(fault, source_fqns)) {
      filtered.push_back(fault);
    }
  }
  return filtered;
}

}  // namespace faults
}  // namespace ros2_medkit_gateway
