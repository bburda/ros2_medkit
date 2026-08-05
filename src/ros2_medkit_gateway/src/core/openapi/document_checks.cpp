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

#include "ros2_medkit_gateway/core/openapi/document_checks.hpp"

#include <string>
#include <vector>

namespace ros2_medkit_gateway {
namespace openapi {

namespace {

constexpr const char * kRefPrefix = "#/components/";

/// Collect every `#/components/<section>/<name>` reference found anywhere in a
/// subtree, at any depth and under any key. Walking the parsed tree rather
/// than regex-matching a dump keeps a `$ref`-looking *string value* elsewhere
/// in the document (a description, an example) from counting as a reference.
void collect_refs(const nlohmann::json & node, std::vector<std::string> & out) {
  if (node.is_object()) {
    for (const auto & [key, value] : node.items()) {
      if (key == "$ref" && value.is_string()) {
        const auto & ref = value.get_ref<const std::string &>();
        if (ref.rfind(kRefPrefix, 0) == 0) {
          out.push_back(ref);
        }
        continue;
      }
      collect_refs(value, out);
    }
    return;
  }
  if (node.is_array()) {
    for (const auto & element : node) {
      collect_refs(element, out);
    }
  }
}

/// Split `#/components/<section>/<name>` into its two halves. Returns false for
/// a reference that is not two segments deep (a pointer into a sub-property,
/// which no emitter here produces).
bool split_ref(const std::string & ref, std::string & section, std::string & name) {
  const std::string tail = ref.substr(std::string(kRefPrefix).size());
  const auto slash = tail.find('/');
  if (slash == std::string::npos || slash == 0 || slash + 1 >= tail.size()) {
    return false;
  }
  section = tail.substr(0, slash);
  name = tail.substr(slash + 1);
  return true;
}

}  // namespace

std::set<std::string> unreachable_schemas(const nlohmann::json & document) {
  if (!document.is_object() || !document.contains("components")) {
    return {};
  }
  // Bound to const references rather than walked through iterators: GCC's
  // -Wnull-dereference fires on nlohmann's inlined `end()` when a json is
  // reached via `find()`, and the warning is not worth carrying for a lookup
  // `contains()` has already proven safe.
  const nlohmann::json & components = document["components"];
  if (!components.is_object() || !components.contains("schemas")) {
    return {};
  }
  const nlohmann::json & schemas = components["schemas"];
  if (!schemas.is_object()) {
    return {};
  }

  // Frontier starts at every reference an operation makes; a schema is reached
  // when some chain of $refs from there arrives at it. Components referenced
  // only by other unreached components stay unreached, which is the point:
  // a client generator walks the same graph.
  std::vector<std::string> frontier;
  if (document.contains("paths")) {
    collect_refs(document["paths"], frontier);
  }

  std::set<std::string> reached_schemas;
  std::set<std::string> seen;
  while (!frontier.empty()) {
    const std::string ref = std::move(frontier.back());
    frontier.pop_back();
    if (!seen.insert(ref).second) {
      continue;
    }
    std::string section;
    std::string name;
    if (!split_ref(ref, section, name)) {
      continue;
    }
    if (section == "schemas") {
      reached_schemas.insert(name);
    }
    if (!components.contains(section)) {
      continue;  // dangling $ref - a different defect, reported by its own test
    }
    const nlohmann::json & section_obj = components[section];
    if (!section_obj.is_object() || !section_obj.contains(name)) {
      continue;
    }
    collect_refs(section_obj[name], frontier);
  }

  std::set<std::string> unreachable;
  for (const auto & entry : schemas.items()) {
    if (reached_schemas.count(entry.key()) == 0) {
      unreachable.insert(entry.key());
    }
  }
  return unreachable;
}

}  // namespace openapi
}  // namespace ros2_medkit_gateway
