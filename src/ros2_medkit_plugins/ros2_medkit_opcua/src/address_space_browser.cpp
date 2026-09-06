// Copyright 2026 mfaferek93
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

#include "ros2_medkit_opcua/address_space_browser.hpp"

#include <algorithm>
#include <cctype>
#include <unordered_map>

#include <rclcpp/rclcpp.hpp>

namespace ros2_medkit_gateway {

namespace {

inline rclcpp::Logger auto_browse_logger() {
  static auto logger = rclcpp::get_logger("opcua.auto_browse");
  return logger;
}

}  // namespace

AutoBrowser::AutoBrowser(AutoBrowseSource & source, AutoBrowseConfig config)
  : source_(source), config_(std::move(config)) {
}

bool AutoBrowser::namespace_allowed(uint16_t ns, const AutoBrowseConfig & cfg) {
  if (!cfg.namespace_allow.empty()) {
    return std::find(cfg.namespace_allow.begin(), cfg.namespace_allow.end(), ns) != cfg.namespace_allow.end();
  }
  return std::find(cfg.namespace_deny.begin(), cfg.namespace_deny.end(), ns) == cfg.namespace_deny.end();
}

std::string AutoBrowser::map_opcua_type(const std::string & opcua_type_name) {
  static const std::unordered_map<std::string, std::string> kMap = {
      {"Boolean", "bool"}, {"SByte", "int"},     {"Byte", "int"},        {"Int16", "int"},  {"UInt16", "int"},
      {"Int32", "int"},    {"UInt32", "int"},    {"Int64", "int"},       {"UInt64", "int"}, {"Float", "float"},
      {"Double", "float"}, {"String", "string"}, {"DateTime", "string"},
  };
  const auto it = kMap.find(opcua_type_name);
  return it == kMap.end() ? std::string{} : it->second;
}

std::string AutoBrowser::sanitize_segment(const std::string & s) {
  if (s.empty()) {
    return "_unnamed";
  }
  std::string out;
  out.reserve(s.size());
  for (char c : s) {
    const unsigned char uc = static_cast<unsigned char>(c);
    if (std::isalnum(uc) != 0) {
      out += static_cast<char>(std::tolower(uc));
    } else if (c == '_' || c == '-') {
      out += c;
    } else {
      out += '_';
    }
  }
  if (out.empty()) {
    return "_unnamed";
  }
  if (std::isdigit(static_cast<unsigned char>(out[0])) != 0) {
    out.insert(out.begin(), '_');
  }
  return out;
}

std::string AutoBrowser::join_entity_id(const std::vector<std::string> & segments) {
  if (segments.empty()) {
    return "root";
  }
  std::string out;
  for (const auto & seg : segments) {
    if (!out.empty()) {
      out += '_';
    }
    out += sanitize_segment(seg);
  }
  return out;
}

std::string AutoBrowser::join_display_name(const std::vector<std::string> & segments) {
  if (segments.empty()) {
    return "Root";
  }
  std::string out;
  for (const auto & seg : segments) {
    if (!out.empty()) {
      out += " / ";
    }
    out += seg;
  }
  return out;
}

namespace {

// A sanitized SOVD segment keeps '-' (valid in a SOVD id), but '-' is illegal
// in a ROS 2 topic token, which must match [A-Za-z_][A-Za-z0-9_]*. Map every
// char that is not alnum or '_' (including '-' and '.') to '_' so the
// auto-generated /plc topic is always a valid ROS 2 name. Mirrors the
// hand-written path's sanitize_topic_segment in node_map.cpp; the SOVD
// entity_id / data_name keep their '-'.
std::string to_topic_segment(const std::string & s) {
  if (s.empty()) {
    return "_unnamed";
  }
  std::string out = s;
  for (char & c : out) {
    if (std::isalnum(static_cast<unsigned char>(c)) == 0 && c != '_') {
      c = '_';
    }
  }
  if (std::isdigit(static_cast<unsigned char>(out[0])) != 0) {
    out.insert(out.begin(), '_');
  }
  return out;
}

// entity_id must stay unique across the whole walk (it doubles as a SOVD
// path segment / ROS 2 topic segment); a collision after sanitization (two
// differently-named branches folding to the same slug) gets a numeric
// suffix, deterministic by first-seen order.
std::string unique_entity_id(const std::string & candidate, std::unordered_set<std::string> & seen) {
  if (seen.insert(candidate).second) {
    return candidate;
  }
  for (int suffix = 2;; ++suffix) {
    std::string next = candidate + "_" + std::to_string(suffix);
    if (seen.insert(next).second) {
      return next;
    }
  }
}

}  // namespace

void AutoBrowser::visit_object(const opcua::NodeId & node, const std::vector<std::string> & path_ids,
                               const std::vector<std::string> & path_display, int depth, AutoBrowseResult & result,
                               std::unordered_set<std::string> & seen_entity_ids,
                               std::unordered_set<std::string> & visited_nodes) {
  // HierarchicalReferences (Organizes included, includeSubtypes=true) can form
  // shared subtrees or cycles: a node reachable from two parents, or one
  // referencing an ancestor. Without a visited set the walk would re-descend
  // the same subtree, inflating nodes_visited until the node cap trips and
  // silently truncates unrelated branches. Skip a node already walked.
  if (!visited_nodes.insert(node.toString()).second) {
    return;
  }
  if (depth > config_.max_depth) {
    result.depth_cap_hit = true;
    return;
  }
  if (result.nodes_visited >= config_.max_nodes) {
    result.node_cap_hit = true;
    return;
  }

  const auto children = source_.browse_children(node);

  std::vector<NodeMapEntry> local_entries;
  std::unordered_set<std::string> seen_data_names;

  for (const auto & child : children) {
    if (result.nodes_visited >= config_.max_nodes) {
      result.node_cap_hit = true;
      break;
    }
    ++result.nodes_visited;

    if (!namespace_allowed(child.browse_name_ns, config_)) {
      continue;  // infra/denied namespace: neither mapped nor recursed into
    }

    const std::string display = child.display_name.empty() ? child.browse_name : child.display_name;

    if (child.node_class == opcua::NodeClass::Object) {
      std::vector<std::string> next_ids = path_ids;
      next_ids.push_back(child.browse_name.empty() ? display : child.browse_name);
      // Parallel raw-DisplayName path, kept unsanitized for the human-readable
      // entity name ("PLC_1 / DB_Test").
      std::vector<std::string> next_display = path_display;
      next_display.push_back(display);
      visit_object(child.node_id, next_ids, next_display, depth + 1, result, seen_entity_ids, visited_nodes);
    } else if (child.node_class == opcua::NodeClass::Variable) {
      const std::string opcua_type = source_.read_type_name(child.node_id);
      const std::string medkit_type = map_opcua_type(opcua_type);
      if (medkit_type.empty()) {
        RCLCPP_DEBUG(auto_browse_logger(), "auto_browse: skipping %s (%s) - unsupported/unresolved DataType",
                     child.node_id.toString().c_str(), display.c_str());
        continue;
      }
      if (config_.read_initial_values && !source_.read_initial_value(child.node_id).has_value()) {
        RCLCPP_DEBUG(auto_browse_logger(), "auto_browse: skipping %s (%s) - Value not readable",
                     child.node_id.toString().c_str(), display.c_str());
        continue;
      }

      NodeMapEntry entry;
      entry.node_id_str = child.node_id.toString();
      entry.node_id = child.node_id;
      entry.data_name = sanitize_segment(child.browse_name.empty() ? display : child.browse_name);
      // Keep data_name unique within this entity, same collision rule as
      // entity_id (deterministic numeric suffix by first-seen order).
      if (!seen_data_names.insert(entry.data_name).second) {
        int suffix = 2;
        std::string base = entry.data_name;
        while (!seen_data_names.insert(base + "_" + std::to_string(suffix)).second) {
          ++suffix;
        }
        entry.data_name = base + "_" + std::to_string(suffix);
      }
      entry.display_name = display;
      entry.data_type = medkit_type;
      // Writability comes from the server itself: read the node's
      // AccessLevel/UserAccessLevel and expose a write surface exactly when the
      // CurrentWrite bit is set for this session. A failed attribute read (or
      // infer_writable disabled) keeps the safe read-only default; an explicit
      // node_map `nodes:` entry still takes precedence downstream.
      //
      // A read-only build has no write path at all, so the inference is not
      // compiled in and the server's CurrentWrite bit is never consulted: every
      // discovered point stays read-only whatever infer_writable says. The
      // operator hears about the ignored setting once, at startup, from
      // OpcuaPlugin::set_context.
      entry.writable = false;
#if !MEDKIT_OPCUA_READ_ONLY
      if (config_.infer_writable) {
        const std::optional<bool> writable = source_.read_writable(child.node_id);
        if (writable.has_value()) {
          entry.writable = *writable;
        }
        RCLCPP_DEBUG(auto_browse_logger(), "auto_browse: %s (%s) writable=%s", child.node_id.toString().c_str(),
                     display.c_str(), entry.writable ? "true" : (writable.has_value() ? "false" : "unknown"));
      }
#endif
      local_entries.push_back(std::move(entry));
    }
    // Other node classes cannot reach here - browse_children already
    // server-side filters to Object | Variable.
  }

  if (!local_entries.empty()) {
    const std::string entity_id = unique_entity_id(join_entity_id(path_ids), seen_entity_ids);
    // Human-readable entity name = the raw DisplayName path ("PLC_1 / DB_Test").
    // Without this, discovery falls back to title-casing the slug in
    // build_entity_defs, discarding the server's real DisplayNames.
    const std::string entity_display = join_display_name(path_display);
    for (auto & entry : local_entries) {
      entry.entity_id = entity_id;
      entry.entity_display_name = entity_display;
      // The SOVD entity_id / data_name may contain '-'; the ROS 2 topic must not.
      entry.ros2_topic = "/plc/" + to_topic_segment(entity_id) + "/" + to_topic_segment(entry.data_name);
      result.entries.push_back(std::move(entry));
    }
  }
}

AutoBrowseResult AutoBrowser::browse() {
  AutoBrowseResult result;
  std::unordered_set<std::string> seen_entity_ids;
  // Shared across all roots so a node reachable from more than one configured
  // root is still walked only once.
  std::unordered_set<std::string> visited_nodes;

  for (const auto & root_str : config_.root_node_ids) {
    const opcua::NodeId root = NodeMap::parse_node_id(root_str);
    visit_object(root, {}, {}, 0, result, seen_entity_ids, visited_nodes);
  }

  if (result.node_cap_hit) {
    RCLCPP_WARN(auto_browse_logger(),
                "auto_browse: node cap (%zu) reached - address-space walk truncated, the SOVD tree may be "
                "incomplete; raise auto_browse.max_nodes if this server has a larger address space",
                config_.max_nodes);
  }
  if (result.depth_cap_hit) {
    RCLCPP_WARN(auto_browse_logger(),
                "auto_browse: depth cap (%d) reached on at least one branch - some nested objects were not "
                "explored; raise auto_browse.max_depth if this server nests deeper",
                config_.max_depth);
  }

  return result;
}

}  // namespace ros2_medkit_gateway
