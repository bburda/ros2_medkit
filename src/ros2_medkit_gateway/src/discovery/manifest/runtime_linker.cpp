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

#include "ros2_medkit_gateway/discovery/manifest/runtime_linker.hpp"

#include "ros2_medkit_gateway/core/discovery/merge_types.hpp"
#include "ros2_medkit_gateway/core/http/warning_codes.hpp"

#include <algorithm>

namespace ros2_medkit_gateway {
namespace discovery {

namespace {

/// Path-segment-boundary namespace match: "/nav" matches "/nav" and "/nav/sub" but NOT "/navigation"
bool namespace_matches(const std::string & actual_ns, const std::string & expected_ns) {
  if (actual_ns == expected_ns) {
    return true;
  }
  if (actual_ns.size() > expected_ns.size() && actual_ns.compare(0, expected_ns.size(), expected_ns) == 0 &&
      actual_ns[expected_ns.size()] == '/') {
    return true;
  }
  return false;
}

/// Extract the last path segment from a FQN (e.g., "/ns/node_name" -> "node_name")
std::string extract_node_name(const std::string & fqn) {
  auto pos = fqn.rfind('/');
  if (pos == std::string::npos) {
    return fqn;
  }
  return fqn.substr(pos + 1);
}

/// Extract namespace from a FQN (e.g., "/ns/sub/node" -> "/ns/sub", "/node" -> "/")
std::string extract_namespace(const std::string & fqn) {
  auto pos = fqn.rfind('/');
  if (pos == std::string::npos || pos == 0) {
    return "/";
  }
  return fqn.substr(0, pos);
}

/// Path-segment-boundary topic match: "/state" matches "/state/x" but NOT "/statement/x"
bool topic_path_matches(const std::string & topic, const std::string & topic_namespace) {
  if (topic == topic_namespace) {
    return true;
  }
  if (topic.size() > topic_namespace.size() && topic.compare(0, topic_namespace.size(), topic_namespace) == 0 &&
      topic[topic_namespace.size()] == '/') {
    return true;
  }
  return false;
}

}  // namespace

RuntimeLinker::RuntimeLinker(rclcpp::Node * node) : node_(node) {
}

LinkingResult RuntimeLinker::link(const std::vector<App> & manifest_apps, const std::vector<App> & runtime_apps,
                                  const ManifestConfig & config,
                                  const std::unordered_set<std::string> & protected_ids) {
  LinkingResult result;

  // Track which runtime nodes have been matched
  std::set<std::string> matched_nodes;

  // Process each manifest app
  for (const auto & manifest_app : manifest_apps) {
    App linked_app = manifest_app;  // Copy
    linked_app.is_online = false;

    // External apps don't need ROS binding
    if (manifest_app.external.value_or(false)) {
      result.linked_apps.push_back(linked_app);
      continue;
    }

    // If no ROS binding, app can't be linked
    if (!manifest_app.ros_binding.has_value() || manifest_app.ros_binding->is_empty()) {
      result.unlinked_app_ids.push_back(manifest_app.id);
      result.linked_apps.push_back(linked_app);
      continue;
    }

    const auto & binding = manifest_app.ros_binding.value();
    bool found = false;

    // Collect candidates, excluding already-bound nodes
    std::vector<const App *> candidates;

    for (const auto & rt_app : runtime_apps) {
      if (!rt_app.bound_fqn.has_value()) {
        continue;
      }
      const auto & fqn = rt_app.bound_fqn.value();
      if (matched_nodes.count(fqn)) {
        continue;  // Node already bound to another app
      }
      auto node_name = extract_node_name(fqn);
      auto node_ns = extract_namespace(fqn);
      if (matches_binding(binding, fqn, node_name, node_ns)) {
        candidates.push_back(&rt_app);
      } else if (!binding.topic_namespace.empty() && matches_topic_namespace(binding.topic_namespace, rt_app)) {
        candidates.push_back(&rt_app);
      }
    }

    // Check if any candidates were excluded due to binding conflicts
    if (candidates.empty()) {
      // Check if there WOULD have been a match without exclusivity
      for (const auto & rt_app : runtime_apps) {
        if (!rt_app.bound_fqn.has_value()) {
          continue;
        }
        const auto & fqn = rt_app.bound_fqn.value();
        if (matched_nodes.count(fqn)) {
          auto node_name = extract_node_name(fqn);
          auto node_ns = extract_namespace(fqn);
          if (matches_binding(binding, fqn, node_name, node_ns) ||
              (!binding.topic_namespace.empty() && matches_topic_namespace(binding.topic_namespace, rt_app))) {
            result.binding_conflicts++;
            result.warnings.push_back("App '" + manifest_app.id + "' cannot bind to '" + fqn +
                                      "' - already bound to app '" + result.node_to_app.at(fqn) + "'");
            log_warn(result.warnings.back());
            break;
          }
        }
      }
    }

    // Sort candidates by FQN for deterministic selection
    std::sort(candidates.begin(), candidates.end(), [](const App * a, const App * b) {
      return a->bound_fqn.value() < b->bound_fqn.value();
    });

    if (!candidates.empty()) {
      if (candidates.size() > 1 && binding.namespace_pattern == "*") {
        result.wildcard_multi_match++;
        log_warn("App '" + manifest_app.id + "' wildcard matched " + std::to_string(candidates.size()) +
                 " nodes, selecting '" + candidates[0]->bound_fqn.value() + "'");
      }

      const auto & match = *candidates[0];
      const auto & match_fqn = match.bound_fqn.value();
      linked_app.bound_fqn = match_fqn;
      linked_app.is_online = true;
      // inherit_runtime_resources=false keeps the manifest's declaration of
      // what the app exposes: the binding still says which node it is, but the
      // node's live topics, services and actions are not copied onto it.
      if (config.inherit_runtime_resources) {
        enrich_app(linked_app, match);
      }

      result.app_to_node[manifest_app.id] = match_fqn;
      result.node_to_app[match_fqn] = manifest_app.id;
      matched_nodes.insert(match_fqn);
      found = true;

      log_debug("Linked app '" + manifest_app.id + "' to node '" + match_fqn + "'");
    }

    if (!found) {
      result.unlinked_app_ids.push_back(manifest_app.id);
      log_debug("App '" + manifest_app.id + "' not linked (no matching node)");
    }

    result.linked_apps.push_back(linked_app);
  }

  // Find orphan nodes (runtime apps not matching any manifest app)
  for (const auto & rt_app : runtime_apps) {
    if (rt_app.bound_fqn.has_value() && matched_nodes.find(rt_app.bound_fqn.value()) == matched_nodes.end()) {
      result.orphan_nodes.push_back(rt_app.bound_fqn.value());
    }
  }

  // Suppress runtime-origin apps that duplicate linked manifest apps (#307).
  // After merge_entities(), both manifest apps and runtime synthetic apps may
  // appear in the merged input. Remove runtime apps whose bound_fqn matches
  // a successfully linked manifest app to avoid duplicates.
  std::set<std::string> linked_fqns;
  for (const auto & [app_id, node_fqn] : result.app_to_node) {
    linked_fqns.insert(node_fqn);
  }

  auto it = std::remove_if(result.linked_apps.begin(), result.linked_apps.end(), [&](const App & app) {
    // Same two grounds the pipeline's orphan sweep uses, from the same set:
    // the owning layer declares its entities protected, or the source tag is
    // one the whitelist knows. Keying on the tag alone deleted entities from
    // any provider that stamped a tag the whitelist had never heard of.
    if (protected_ids.count(app.id) > 0 || is_protected_source(app.source)) {
      return false;
    }
    if (!app.bound_fqn.has_value()) {
      return false;  // Keep apps without bound_fqn
    }
    return linked_fqns.count(app.bound_fqn.value()) > 0;  // Remove if FQN is linked
  });
  result.linked_apps.erase(it, result.linked_apps.end());

  // Log summary
  log_info("Runtime linking: " + result.summary());

  // Handle orphan nodes according to policy
  if (!result.orphan_nodes.empty()) {
    switch (config.unmanifested_nodes) {
      case ManifestConfig::UnmanifestedNodePolicy::IGNORE:
        log_debug("Ignoring " + std::to_string(result.orphan_nodes.size()) + " orphan nodes");
        break;

      case ManifestConfig::UnmanifestedNodePolicy::WARN:
        for (const auto & orphan : result.orphan_nodes) {
          log_warn("Orphan node (not in manifest): " + orphan);
        }
        break;

      case ManifestConfig::UnmanifestedNodePolicy::ERROR:
        log_error("Orphan nodes detected with 'error' policy: " + std::to_string(result.orphan_nodes.size()) +
                  " node(s) are not declared in the manifest. The gateway keeps serving; they are reported on "
                  "GET /health as a '" +
                  std::string(WARN_UNMANIFESTED_NODES) + "' warning.");
        break;

      case ManifestConfig::UnmanifestedNodePolicy::INCLUDE_AS_ORPHAN:
        log_info("Including " + std::to_string(result.orphan_nodes.size()) +
                 " orphan node(s), same as the 'warn' policy but logged at info level");
        break;
    }
  }

  last_result_ = result;
  return result;
}

bool RuntimeLinker::matches_binding(const App::RosBinding & binding, const std::string & node_fqn,
                                    const std::string & /*node_name*/, const std::string & node_namespace) const {
  // Check node name match using last FQN segment (exact match only)
  if (binding.node_name.empty()) {
    return false;
  }

  std::string actual_name = extract_node_name(node_fqn);
  if (actual_name != binding.node_name) {
    return false;
  }

  // Check namespace match
  if (binding.namespace_pattern == "*") {
    // Wildcard matches any namespace
    return true;
  }

  // Normalize namespaces for comparison
  std::string expected_ns = binding.namespace_pattern.empty() ? "/" : binding.namespace_pattern;
  std::string actual_ns = node_namespace.empty() ? "/" : node_namespace;

  // Path-segment-boundary match
  return namespace_matches(actual_ns, expected_ns);
}

bool RuntimeLinker::matches_topic_namespace(const std::string & topic_ns, const App & runtime_app) const {
  // Check if any topic matches with path-segment boundary
  for (const auto & topic : runtime_app.topics.publishes) {
    if (topic_path_matches(topic, topic_ns)) {
      return true;
    }
  }
  for (const auto & topic : runtime_app.topics.subscribes) {
    if (topic_path_matches(topic, topic_ns)) {
      return true;
    }
  }
  return false;
}

void RuntimeLinker::enrich_app(App & app, const App & runtime_app) {
  app.topics = runtime_app.topics;
  app.services = runtime_app.services;
  app.actions = runtime_app.actions;
}

bool RuntimeLinker::is_app_online(const std::string & app_id) const {
  return last_result_.app_to_node.find(app_id) != last_result_.app_to_node.end();
}

std::optional<std::string> RuntimeLinker::get_bound_node(const std::string & app_id) const {
  auto it = last_result_.app_to_node.find(app_id);
  if (it != last_result_.app_to_node.end()) {
    return it->second;
  }
  return std::nullopt;
}

std::optional<std::string> RuntimeLinker::get_app_for_node(const std::string & node_fqn) const {
  auto it = last_result_.node_to_app.find(node_fqn);
  if (it != last_result_.node_to_app.end()) {
    return it->second;
  }
  return std::nullopt;
}

void RuntimeLinker::log_info(const std::string & msg) const {
  if (node_) {
    RCLCPP_INFO(node_->get_logger(), "%s", msg.c_str());
  }
}

void RuntimeLinker::log_debug(const std::string & msg) const {
  if (node_) {
    RCLCPP_DEBUG(node_->get_logger(), "%s", msg.c_str());
  }
}

void RuntimeLinker::log_warn(const std::string & msg) const {
  if (node_) {
    RCLCPP_WARN(node_->get_logger(), "%s", msg.c_str());
  }
}

void RuntimeLinker::log_error(const std::string & msg) const {
  if (node_) {
    RCLCPP_ERROR(node_->get_logger(), "%s", msg.c_str());
  }
}

}  // namespace discovery
}  // namespace ros2_medkit_gateway
