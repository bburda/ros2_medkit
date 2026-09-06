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

// Unit tests for the recursive OPC-UA address-space walker (auto_browse).
// Exercises AutoBrowser against an injected FakeAutoBrowseSource - an
// in-memory tree, no network, no server - covering hierarchy mapping, type
// inference, depth/node caps, namespace filtering and node_map precedence.

#include "ros2_medkit_opcua/address_space_browser.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <atomic>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <optional>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <unistd.h>

namespace ros2_medkit_gateway {

namespace {

// RAII-scoped YAML fixture written to a unique path under the system temp
// dir (PID + per-process counter, so two test binaries - or two tests in the
// same binary - never collide on the same file), removed on scope exit.
class TempYamlFile {
 public:
  explicit TempYamlFile(const std::string & contents) {
    static std::atomic<int> counter{0};
    path_ = (std::filesystem::temp_directory_path() / ("ros2_medkit_opcua_test_" + std::to_string(::getpid()) + "_" +
                                                       std::to_string(counter.fetch_add(1)) + ".yaml"))
                .string();
    std::ofstream f(path_);
    f << contents;
  }
  ~TempYamlFile() {
    std::error_code ec;
    std::filesystem::remove(path_, ec);
  }
  TempYamlFile(const TempYamlFile &) = delete;
  TempYamlFile & operator=(const TempYamlFile &) = delete;

  const std::string & path() const {
    return path_;
  }

 private:
  std::string path_;
};

OpcuaClient::BrowseChild make_child(const std::string & node_id_str, uint16_t bn_ns, const std::string & browse_name,
                                    const std::string & display_name, opcua::NodeClass node_class) {
  OpcuaClient::BrowseChild c;
  c.node_id = NodeMap::parse_node_id(node_id_str);
  c.browse_name_ns = bn_ns;
  c.browse_name = browse_name;
  c.display_name = display_name;
  c.node_class = node_class;
  return c;
}

// In-memory AutoBrowseSource: an explicit parent(NodeId string) -> children
// tree, a NodeId-string -> OPC-UA type-name map, and a set of node ids whose
// Value read is made to fail (for the read_initial_values gate).
class FakeAutoBrowseSource : public AutoBrowseSource {
 public:
  void add_child(const opcua::NodeId & parent, const OpcuaClient::BrowseChild & child) {
    tree_[parent.toString()].push_back(child);
  }
  void set_type(const opcua::NodeId & node, std::string type_name) {
    types_[node.toString()] = std::move(type_name);
  }
  void set_unreadable(const opcua::NodeId & node) {
    unreadable_.insert(node.toString());
  }
  // Mark a node the server reports as writable (CurrentWrite set).
  void set_writable(const opcua::NodeId & node) {
    writable_.insert(node.toString());
  }
  // Mark a node whose AccessLevel attribute read fails (read_writable -> nullopt).
  void set_access_level_unreadable(const opcua::NodeId & node) {
    access_unreadable_.insert(node.toString());
  }

  std::vector<OpcuaClient::BrowseChild> browse_children(const opcua::NodeId & parent) override {
    ++browse_calls;
    auto it = tree_.find(parent.toString());
    return it == tree_.end() ? std::vector<OpcuaClient::BrowseChild>{} : it->second;
  }

  std::string read_type_name(const opcua::NodeId & variable_node) override {
    auto it = types_.find(variable_node.toString());
    return it == types_.end() ? std::string{} : it->second;
  }

  std::optional<OpcuaValue> read_initial_value(const opcua::NodeId & variable_node) override {
    if (unreadable_.count(variable_node.toString()) > 0) {
      return std::nullopt;
    }
    return OpcuaValue{1.0};
  }

  std::optional<bool> read_writable(const opcua::NodeId & variable_node) override {
    if (access_unreadable_.count(variable_node.toString()) > 0) {
      return std::nullopt;
    }
    return writable_.count(variable_node.toString()) > 0;
  }

  int browse_calls{0};

 private:
  std::unordered_map<std::string, std::vector<OpcuaClient::BrowseChild>> tree_;
  std::unordered_map<std::string, std::string> types_;
  std::unordered_set<std::string> unreadable_;
  std::unordered_set<std::string> writable_;
  std::unordered_set<std::string> access_unreadable_;
};

const NodeMapEntry * find_entry(const std::vector<NodeMapEntry> & entries, const std::string & node_id_str) {
  auto it = std::find_if(entries.begin(), entries.end(), [&](const NodeMapEntry & e) {
    return e.node_id_str == node_id_str;
  });
  return it == entries.end() ? nullptr : &(*it);
}

}  // namespace

// -- Hierarchy mapping + type inference --------------------------------

TEST(AutoBrowserTest, MapsObjectHierarchyToEntityAndVariablesToDataPoints) {
  FakeAutoBrowseSource source;
  const auto root = NodeMap::parse_node_id("i=85");
  const auto plc1 = NodeMap::parse_node_id("ns=2;s=PLC_1");
  const auto db_test = NodeMap::parse_node_id("ns=2;s=DB_Test");
  const auto level = NodeMap::parse_node_id("ns=2;s=DB_Test.Level");
  const auto running = NodeMap::parse_node_id("ns=2;s=DB_Test.Running");

  source.add_child(root, make_child("ns=2;s=PLC_1", 2, "PLC_1", "PLC_1", opcua::NodeClass::Object));
  source.add_child(plc1, make_child("ns=2;s=DB_Test", 2, "DB_Test", "DB_Test", opcua::NodeClass::Object));
  source.add_child(db_test, make_child("ns=2;s=DB_Test.Level", 2, "Level", "Level", opcua::NodeClass::Variable));
  source.add_child(db_test, make_child("ns=2;s=DB_Test.Running", 2, "Running", "Running", opcua::NodeClass::Variable));
  source.set_type(level, "Float");
  source.set_type(running, "Boolean");
  // The server marks Running writable (CurrentWrite) but Level read-only.
  source.set_writable(running);

  AutoBrowseConfig cfg;
  AutoBrowser browser(source, cfg);
  auto result = browser.browse();

  ASSERT_EQ(result.entries.size(), 2u);
  const auto * level_entry = find_entry(result.entries, level.toString());
  const auto * running_entry = find_entry(result.entries, running.toString());
  ASSERT_NE(level_entry, nullptr);
  ASSERT_NE(running_entry, nullptr);

  EXPECT_EQ(level_entry->entity_id, "plc_1_db_test");
  EXPECT_EQ(level_entry->data_name, "level");
  EXPECT_EQ(level_entry->display_name, "Level");
  EXPECT_EQ(level_entry->data_type, "float");
  EXPECT_EQ(level_entry->ros2_topic, "/plc/plc_1_db_test/level");
  EXPECT_FALSE(level_entry->writable);  // server: read-only

  EXPECT_EQ(running_entry->entity_id, "plc_1_db_test");
  EXPECT_EQ(running_entry->data_name, "running");
  EXPECT_EQ(running_entry->data_type, "bool");
#if MEDKIT_OPCUA_READ_ONLY
  // A read-only build never consults the server's CurrentWrite bit: the
  // inference is not compiled in, so a point the server would let us write
  // still comes back read-only.
  EXPECT_FALSE(running_entry->writable);
#else
  EXPECT_TRUE(running_entry->writable);  // server: CurrentWrite set
#endif

  EXPECT_FALSE(result.node_cap_hit);
  EXPECT_FALSE(result.depth_cap_hit);
}

// -- writable inferred from the server's AccessLevel -------------------

TEST(AutoBrowserTest, WritableInferredFromServerAccessLevel) {
  FakeAutoBrowseSource source;
  const auto root = NodeMap::parse_node_id("i=85");
  const auto rw = NodeMap::parse_node_id("ns=2;s=Setpoint");  // writable
  const auto ro = NodeMap::parse_node_id("ns=2;s=Measured");  // read-only
  const auto unk = NodeMap::parse_node_id("ns=2;s=Unknown");  // AccessLevel read fails

  source.add_child(root, make_child("ns=2;s=Setpoint", 2, "Setpoint", "Setpoint", opcua::NodeClass::Variable));
  source.add_child(root, make_child("ns=2;s=Measured", 2, "Measured", "Measured", opcua::NodeClass::Variable));
  source.add_child(root, make_child("ns=2;s=Unknown", 2, "Unknown", "Unknown", opcua::NodeClass::Variable));
  source.set_type(rw, "Float");
  source.set_type(ro, "Float");
  source.set_type(unk, "Float");
  source.set_writable(rw);
  source.set_access_level_unreadable(unk);

  AutoBrowseConfig cfg;
  AutoBrowser browser(source, cfg);
  auto result = browser.browse();

  ASSERT_EQ(result.entries.size(), 3u);
#if MEDKIT_OPCUA_READ_ONLY
  EXPECT_FALSE(find_entry(result.entries, rw.toString())->writable)
      << "infer_writable must not produce a writable point in a read-only build";
#else
  EXPECT_TRUE(find_entry(result.entries, rw.toString())->writable);
#endif
  EXPECT_FALSE(find_entry(result.entries, ro.toString())->writable);
  // AccessLevel read failed -> safe read-only default, never a false-positive.
  EXPECT_FALSE(find_entry(result.entries, unk.toString())->writable);
}

TEST(AutoBrowserTest, InferWritableDisabledKeepsEverythingReadOnly) {
  FakeAutoBrowseSource source;
  const auto root = NodeMap::parse_node_id("i=85");
  const auto rw = NodeMap::parse_node_id("ns=2;s=Setpoint");
  source.add_child(root, make_child("ns=2;s=Setpoint", 2, "Setpoint", "Setpoint", opcua::NodeClass::Variable));
  source.set_type(rw, "Float");
  source.set_writable(rw);

  AutoBrowseConfig cfg;
  cfg.infer_writable = false;
  AutoBrowser browser(source, cfg);
  auto result = browser.browse();

  ASSERT_EQ(result.entries.size(), 1u);
  EXPECT_FALSE(find_entry(result.entries, rw.toString())->writable);
}

TEST(AutoBrowserTest, PureObjectsWithoutVariablesDoNotBecomeEntities) {
  // PLC_1 -> Empty (Object, no variables anywhere under it) should never
  // appear as an entity_id/App; it is only a path segment.
  FakeAutoBrowseSource source;
  const auto root = NodeMap::parse_node_id("i=85");
  const auto plc1 = NodeMap::parse_node_id("ns=2;s=PLC_1");
  const auto empty = NodeMap::parse_node_id("ns=2;s=Empty");

  source.add_child(root, make_child("ns=2;s=PLC_1", 2, "PLC_1", "PLC_1", opcua::NodeClass::Object));
  source.add_child(plc1, make_child("ns=2;s=Empty", 2, "Empty", "Empty", opcua::NodeClass::Object));
  (void)empty;  // no children registered - a genuinely empty subtree

  AutoBrowseConfig cfg;
  AutoBrowser browser(source, cfg);
  auto result = browser.browse();

  EXPECT_TRUE(result.entries.empty());
}

// -- Variable -> data point type inference (map_opcua_type) -------------

TEST(AutoBrowserTest, MapsSupportedOpcuaBuiltinTypes) {
  EXPECT_EQ(AutoBrowser::map_opcua_type("Boolean"), "bool");
  EXPECT_EQ(AutoBrowser::map_opcua_type("SByte"), "int");
  EXPECT_EQ(AutoBrowser::map_opcua_type("Byte"), "int");
  EXPECT_EQ(AutoBrowser::map_opcua_type("Int16"), "int");
  EXPECT_EQ(AutoBrowser::map_opcua_type("UInt16"), "int");
  EXPECT_EQ(AutoBrowser::map_opcua_type("Int32"), "int");
  EXPECT_EQ(AutoBrowser::map_opcua_type("UInt32"), "int");
  EXPECT_EQ(AutoBrowser::map_opcua_type("Int64"), "int");
  EXPECT_EQ(AutoBrowser::map_opcua_type("UInt64"), "int");
  EXPECT_EQ(AutoBrowser::map_opcua_type("Float"), "float");
  EXPECT_EQ(AutoBrowser::map_opcua_type("Double"), "float");
  EXPECT_EQ(AutoBrowser::map_opcua_type("String"), "string");
  EXPECT_EQ(AutoBrowser::map_opcua_type("DateTime"), "string");
}

TEST(AutoBrowserTest, UnsupportedTypesMapToEmpty) {
  EXPECT_EQ(AutoBrowser::map_opcua_type("Guid"), "");
  EXPECT_EQ(AutoBrowser::map_opcua_type("ByteString"), "");
  EXPECT_EQ(AutoBrowser::map_opcua_type("SomeStructuredType"), "");
  EXPECT_EQ(AutoBrowser::map_opcua_type(""), "");
}

TEST(AutoBrowserTest, VariableWithUnsupportedTypeIsSkipped) {
  FakeAutoBrowseSource source;
  const auto root = NodeMap::parse_node_id("i=85");
  const auto guid_var = NodeMap::parse_node_id("ns=2;s=SomeGuid");
  const auto ok_var = NodeMap::parse_node_id("ns=2;s=Ok");

  source.add_child(root, make_child("ns=2;s=SomeGuid", 2, "SomeGuid", "SomeGuid", opcua::NodeClass::Variable));
  source.add_child(root, make_child("ns=2;s=Ok", 2, "Ok", "Ok", opcua::NodeClass::Variable));
  source.set_type(guid_var, "Guid");
  source.set_type(ok_var, "Int32");

  AutoBrowseConfig cfg;
  AutoBrowser browser(source, cfg);
  auto result = browser.browse();

  ASSERT_EQ(result.entries.size(), 1u);
  EXPECT_EQ(result.entries[0].node_id_str, ok_var.toString());
  // Variables directly on the root (empty path) fall back to entity_id "root".
  EXPECT_EQ(result.entries[0].entity_id, "root");
}

// -- read_initial_values gate --------------------------------------------

TEST(AutoBrowserTest, UnreadableVariableSkippedWhenReadInitialValuesEnabled) {
  FakeAutoBrowseSource source;
  const auto root = NodeMap::parse_node_id("i=85");
  const auto var = NodeMap::parse_node_id("ns=2;s=Broken");

  source.add_child(root, make_child("ns=2;s=Broken", 2, "Broken", "Broken", opcua::NodeClass::Variable));
  source.set_type(var, "Float");
  source.set_unreadable(var);

  AutoBrowseConfig cfg;
  cfg.read_initial_values = true;
  AutoBrowser browser_gated(source, cfg);
  EXPECT_TRUE(browser_gated.browse().entries.empty());

  cfg.read_initial_values = false;
  AutoBrowser browser_ungated(source, cfg);
  EXPECT_EQ(browser_ungated.browse().entries.size(), 1u);
}

// -- Depth cap -------------------------------------------------------------

TEST(AutoBrowserTest, DepthCapStopsDescentAndIsReported) {
  FakeAutoBrowseSource source;
  const auto root = NodeMap::parse_node_id("i=85");

  // Chain: root -> L1 -> L2 -> L3 -> L4 (Variable at the end)
  opcua::NodeId parent = root;
  for (int i = 1; i <= 4; ++i) {
    const std::string id = "ns=2;s=Level" + std::to_string(i);
    const auto node_class = (i == 4) ? opcua::NodeClass::Variable : opcua::NodeClass::Object;
    source.add_child(parent, make_child(id, 2, "Level" + std::to_string(i), "Level" + std::to_string(i), node_class));
    parent = NodeMap::parse_node_id(id);
    if (i < 4) {
      source.set_type(parent, "");  // objects have no type; harmless
    } else {
      source.set_type(parent, "Float");
    }
  }

  AutoBrowseConfig cfg;
  cfg.max_depth = 2;  // root's children are depth 1; Level3 (depth 3) is already over budget
  AutoBrowser browser(source, cfg);
  auto result = browser.browse();

  EXPECT_TRUE(result.entries.empty());
  EXPECT_TRUE(result.depth_cap_hit);
}

TEST(AutoBrowserTest, DepthWithinBudgetIsNotReportedAsCapHit) {
  FakeAutoBrowseSource source;
  const auto root = NodeMap::parse_node_id("i=85");
  const auto obj = NodeMap::parse_node_id("ns=2;s=Obj");
  const auto var = NodeMap::parse_node_id("ns=2;s=Obj.Var");
  source.add_child(root, make_child("ns=2;s=Obj", 2, "Obj", "Obj", opcua::NodeClass::Object));
  source.add_child(obj, make_child("ns=2;s=Obj.Var", 2, "Var", "Var", opcua::NodeClass::Variable));
  source.set_type(var, "Int32");

  AutoBrowseConfig cfg;
  cfg.max_depth = 8;
  AutoBrowser browser(source, cfg);
  auto result = browser.browse();

  EXPECT_EQ(result.entries.size(), 1u);
  EXPECT_FALSE(result.depth_cap_hit);
}

// -- Node cap ---------------------------------------------------------------

TEST(AutoBrowserTest, NodeCapTruncatesWalkAndIsReported) {
  FakeAutoBrowseSource source;
  const auto root = NodeMap::parse_node_id("i=85");
  for (int i = 0; i < 20; ++i) {
    const std::string id = "ns=2;s=Var" + std::to_string(i);
    source.add_child(
        root, make_child(id, 2, "Var" + std::to_string(i), "Var" + std::to_string(i), opcua::NodeClass::Variable));
    source.set_type(NodeMap::parse_node_id(id), "Int32");
  }

  AutoBrowseConfig cfg;
  cfg.max_nodes = 5;
  AutoBrowser browser(source, cfg);
  auto result = browser.browse();

  EXPECT_LE(result.entries.size(), 5u);
  EXPECT_TRUE(result.node_cap_hit);
}

// -- Namespace filtering -----------------------------------------------

TEST(AutoBrowserTest, DefaultDenyListSkipsNamespaceZeroInfraNodes) {
  FakeAutoBrowseSource source;
  const auto root = NodeMap::parse_node_id("i=85");
  const auto server_obj = NodeMap::parse_node_id("i=2253");  // ns=0 Server object
  const auto device_set = NodeMap::parse_node_id("ns=3;s=DeviceSet");
  const auto plc_var = NodeMap::parse_node_id("ns=3;s=DeviceSet.Var");

  // ns=0 "Server" child: must be skipped by the default deny={0} filter, so
  // its own children (which would otherwise be a variable) never get visited.
  source.add_child(root, make_child("i=2253", 0, "Server", "Server", opcua::NodeClass::Object));
  source.add_child(server_obj,
                   make_child("i=9999", 0, "ShouldNeverBeVisited", "ShouldNeverBeVisited", opcua::NodeClass::Variable));

  // ns=3 (vendor/DI) "DeviceSet" child: allowed by default.
  source.add_child(root, make_child("ns=3;s=DeviceSet", 3, "DeviceSet", "DeviceSet", opcua::NodeClass::Object));
  source.add_child(device_set, make_child("ns=3;s=DeviceSet.Var", 3, "Var", "Var", opcua::NodeClass::Variable));
  source.set_type(plc_var, "Int32");

  AutoBrowseConfig cfg;  // default namespace_deny = {0}
  AutoBrowser browser(source, cfg);
  auto result = browser.browse();

  ASSERT_EQ(result.entries.size(), 1u);
  EXPECT_EQ(result.entries[0].node_id_str, plc_var.toString());
  EXPECT_EQ(browser.namespace_allowed(0, cfg), false);
  EXPECT_EQ(browser.namespace_allowed(3, cfg), true);
}

TEST(AutoBrowserTest, NamespaceAllowListOverridesDenyList) {
  AutoBrowseConfig cfg;
  cfg.namespace_allow = {3};
  cfg.namespace_deny = {};  // irrelevant while allow-list is set
  EXPECT_TRUE(AutoBrowser::namespace_allowed(3, cfg));
  EXPECT_FALSE(AutoBrowser::namespace_allowed(0, cfg));
  EXPECT_FALSE(AutoBrowser::namespace_allowed(4, cfg));
}

// -- Entity id / segment sanitization ------------------------------------

TEST(AutoBrowserTest, SanitizeSegmentLowercasesAndReplacesInvalidChars) {
  EXPECT_EQ(AutoBrowser::sanitize_segment("DB_Test"), "db_test");
  EXPECT_EQ(AutoBrowser::sanitize_segment("Line-A"), "line-a");
  EXPECT_EQ(AutoBrowser::sanitize_segment("Tag.With.Dots"), "tag_with_dots");
  EXPECT_EQ(AutoBrowser::sanitize_segment("3Phase"), "_3phase");
  EXPECT_EQ(AutoBrowser::sanitize_segment(""), "_unnamed");
}

TEST(AutoBrowserTest, JoinEntityIdEmptyFallsBackToRoot) {
  EXPECT_EQ(AutoBrowser::join_entity_id({}), "root");
  EXPECT_EQ(AutoBrowser::join_entity_id({"PLC_1", "DB_Test"}), "plc_1_db_test");
}

TEST(AutoBrowserTest, JoinDisplayNameEmptyFallsBackToRoot) {
  EXPECT_EQ(AutoBrowser::join_display_name({}), "Root");
  EXPECT_EQ(AutoBrowser::join_display_name({"PLC_1", "DB_Test"}), "PLC_1 / DB_Test");
}

TEST(AutoBrowserTest, CollidingSanitizedEntityIdsGetDeterministicSuffix) {
  // "Line.A" (dot -> '_') and "Line_A" both sanitize to "line_a"; the second
  // branch encountered must not silently merge into the first's data points.
  FakeAutoBrowseSource source;
  const auto root = NodeMap::parse_node_id("i=85");
  const auto branch_a = NodeMap::parse_node_id("ns=2;s=Line.A");
  const auto branch_b = NodeMap::parse_node_id("ns=2;s=Line_A");
  const auto var_a = NodeMap::parse_node_id("ns=2;s=Line.A.X");
  const auto var_b = NodeMap::parse_node_id("ns=2;s=Line_A.X");

  source.add_child(root, make_child("ns=2;s=Line.A", 2, "Line.A", "Line.A", opcua::NodeClass::Object));
  source.add_child(root, make_child("ns=2;s=Line_A", 2, "Line_A", "Line_A", opcua::NodeClass::Object));
  source.add_child(branch_a, make_child("ns=2;s=Line.A.X", 2, "X", "X", opcua::NodeClass::Variable));
  source.add_child(branch_b, make_child("ns=2;s=Line_A.X", 2, "X", "X", opcua::NodeClass::Variable));
  source.set_type(var_a, "Int32");
  source.set_type(var_b, "Int32");

  AutoBrowseConfig cfg;
  AutoBrowser browser(source, cfg);
  auto result = browser.browse();

  ASSERT_EQ(result.entries.size(), 2u);
  std::unordered_set<std::string> entity_ids;
  for (const auto & e : result.entries) {
    entity_ids.insert(e.entity_id);
  }
  EXPECT_EQ(entity_ids.size(), 2u) << "colliding sanitized entity ids must be disambiguated, not merged";
}

// -- ROS 2 topic safety (hyphens) --------------------------------------

TEST(AutoBrowserTest, HyphenatedNamesYieldValidRos2TopicButKeepSovdHyphen) {
  // '-' is valid in a SOVD id but illegal in a ROS 2 topic token. Common PLC
  // tags like "Line-A" / "Flow-Rate" must still produce a valid /plc topic.
  FakeAutoBrowseSource source;
  const auto root = NodeMap::parse_node_id("i=85");
  const auto line = NodeMap::parse_node_id("ns=2;s=Line-A");
  const auto flow = NodeMap::parse_node_id("ns=2;s=Line-A.Flow-Rate");
  source.add_child(root, make_child("ns=2;s=Line-A", 2, "Line-A", "Line-A", opcua::NodeClass::Object));
  source.add_child(line,
                   make_child("ns=2;s=Line-A.Flow-Rate", 2, "Flow-Rate", "Flow-Rate", opcua::NodeClass::Variable));
  source.set_type(flow, "Float");

  AutoBrowseConfig cfg;
  AutoBrowser browser(source, cfg);
  auto result = browser.browse();

  ASSERT_EQ(result.entries.size(), 1u);
  const auto & e = result.entries[0];
  // SOVD id / data_name keep the hyphen ...
  EXPECT_EQ(e.entity_id, "line-a");
  EXPECT_EQ(e.data_name, "flow-rate");
  // ... but the ROS 2 topic must be a valid name (no '-').
  EXPECT_EQ(e.ros2_topic, "/plc/line_a/flow_rate");
}

// -- Cyclic / shared subtree guard (visited-NodeId set) -----------------

TEST(AutoBrowserTest, CyclicHierarchyIsWalkedOnceAndTerminates) {
  // root -> A (Object, owns a Variable) -> B (Object) -> back to A (cycle via a
  // HierarchicalReference). Without a visited set the walk would re-descend A
  // until the node cap trips; with it, each Object is entered exactly once.
  FakeAutoBrowseSource source;
  const auto root = NodeMap::parse_node_id("i=85");
  const auto a = NodeMap::parse_node_id("ns=2;s=A");
  const auto b = NodeMap::parse_node_id("ns=2;s=B");
  const auto var = NodeMap::parse_node_id("ns=2;s=A.V");
  source.add_child(root, make_child("ns=2;s=A", 2, "A", "A", opcua::NodeClass::Object));
  source.add_child(a, make_child("ns=2;s=A.V", 2, "V", "V", opcua::NodeClass::Variable));
  source.add_child(a, make_child("ns=2;s=B", 2, "B", "B", opcua::NodeClass::Object));
  source.add_child(b, make_child("ns=2;s=A", 2, "A", "A", opcua::NodeClass::Object));  // B -> A back-edge
  source.set_type(var, "Int32");

  AutoBrowseConfig cfg;
  AutoBrowser browser(source, cfg);
  auto result = browser.browse();

  EXPECT_FALSE(result.node_cap_hit);
  ASSERT_EQ(result.entries.size(), 1u);
  EXPECT_EQ(result.entries[0].entity_id, "a");
  // browse_children entered once each for root, A, B - never A twice.
  EXPECT_EQ(source.browse_calls, 3);
}

// -- Human-readable entity display name (join_display_name in the walk) --

TEST(AutoBrowserTest, AttachesHumanReadableEntityDisplayNamePath) {
  FakeAutoBrowseSource source;
  const auto root = NodeMap::parse_node_id("i=85");
  const auto plc1 = NodeMap::parse_node_id("ns=2;s=PLC_1");
  const auto db_test = NodeMap::parse_node_id("ns=2;s=DB_Test");
  const auto level = NodeMap::parse_node_id("ns=2;s=DB_Test.Level");
  source.add_child(root, make_child("ns=2;s=PLC_1", 2, "PLC_1", "PLC_1", opcua::NodeClass::Object));
  source.add_child(plc1, make_child("ns=2;s=DB_Test", 2, "DB_Test", "DB_Test", opcua::NodeClass::Object));
  source.add_child(db_test, make_child("ns=2;s=DB_Test.Level", 2, "Level", "Level", opcua::NodeClass::Variable));
  source.set_type(level, "Float");

  AutoBrowseConfig cfg;
  AutoBrowser browser(source, cfg);
  auto result = browser.browse();

  ASSERT_EQ(result.entries.size(), 1u);
  EXPECT_EQ(result.entries[0].entity_id, "plc_1_db_test");
  EXPECT_EQ(result.entries[0].entity_display_name, "PLC_1 / DB_Test");
}

// -- Multiple configured roots ----------------------------------------

TEST(AutoBrowserTest, MultipleRootsAreAllWalked) {
  FakeAutoBrowseSource source;
  const auto root_a = NodeMap::parse_node_id("ns=2;s=RootA");
  const auto root_b = NodeMap::parse_node_id("ns=2;s=RootB");
  const auto var_a = NodeMap::parse_node_id("ns=2;s=RootA.V");
  const auto var_b = NodeMap::parse_node_id("ns=2;s=RootB.V");

  source.add_child(root_a, make_child("ns=2;s=RootA.V", 2, "V", "V", opcua::NodeClass::Variable));
  source.add_child(root_b, make_child("ns=2;s=RootB.V", 2, "V", "V", opcua::NodeClass::Variable));
  source.set_type(var_a, "Int32");
  source.set_type(var_b, "Int32");

  AutoBrowseConfig cfg;
  cfg.root_node_ids = {"ns=2;s=RootA", "ns=2;s=RootB"};
  AutoBrowser browser(source, cfg);
  auto result = browser.browse();

  EXPECT_EQ(result.entries.size(), 2u);
}

// -- node_map precedence (explicit config wins over auto_browse) --------

TEST(NodeMapAutoBrowseMergeTest, ExplicitNodeMapEntryTakesPrecedenceOverAutoBrowsed) {
  NodeMap node_map;
  const TempYamlFile yaml_file(R"(
component_id: test_plc
auto_browse: true
nodes:
  - node_id: "ns=2;s=DB_Test.Level"
    entity_id: hand_authored_entity
    data_name: hand_authored_name
    data_type: float
    writable: true
)");
  ASSERT_TRUE(node_map.load(yaml_file.path()));
  EXPECT_TRUE(node_map.auto_browse());
  ASSERT_EQ(node_map.entries().size(), 1u);

  std::vector<NodeMapEntry> auto_entries;
  NodeMapEntry collide;
  collide.node_id_str = "ns=2;s=DB_Test.Level";  // same node as the explicit entry above
  collide.node_id = NodeMap::parse_node_id(collide.node_id_str);
  collide.entity_id = "auto_browsed_entity";
  collide.data_name = "auto_browsed_name";
  collide.data_type = "float";
  auto_entries.push_back(collide);

  NodeMapEntry new_one;
  new_one.node_id_str = "ns=2;s=DB_Test.Running";
  new_one.node_id = NodeMap::parse_node_id(new_one.node_id_str);
  new_one.entity_id = "auto_browsed_entity";
  new_one.data_name = "running";
  new_one.data_type = "bool";
  auto_entries.push_back(new_one);

  const std::size_t added = node_map.merge_auto_browsed_entries(std::move(auto_entries));
  EXPECT_EQ(added, 1u);  // only "Running" was added; "Level" was dropped (collision)
  ASSERT_EQ(node_map.entries().size(), 2u);

  const auto * level = node_map.find_by_node_id("ns=2;s=DB_Test.Level");
  ASSERT_NE(level, nullptr);
  EXPECT_EQ(level->entity_id, "hand_authored_entity");
  EXPECT_EQ(level->data_name, "hand_authored_name");
#if MEDKIT_OPCUA_READ_ONLY
  EXPECT_FALSE(level->writable);
#else
  EXPECT_TRUE(level->writable);
#endif

  const auto * running = node_map.find_by_node_id("ns=2;s=DB_Test.Running");
  ASSERT_NE(running, nullptr);
  EXPECT_EQ(running->entity_id, "auto_browsed_entity");
}

TEST(NodeMapAutoBrowseMergeTest, AutoBrowseOnlyConfigWithNoNodesSectionIsValid) {
  NodeMap node_map;
  const TempYamlFile yaml_file(R"(
component_id: test_plc
auto_browse:
  max_depth: 5
  max_nodes: 1000
  namespace_deny: [0]
  read_initial_values: false
)");
  ASSERT_TRUE(node_map.load(yaml_file.path()));
  EXPECT_TRUE(node_map.auto_browse());
  EXPECT_EQ(node_map.auto_browse_config().max_depth, 5);
  EXPECT_EQ(node_map.auto_browse_config().max_nodes, 1000u);
  EXPECT_FALSE(node_map.auto_browse_config().read_initial_values);
  EXPECT_TRUE(node_map.entries().empty());
}

TEST(NodeMapAutoBrowseMergeTest, EntityDefNameUsesAutoBrowsedDisplayNamePath) {
  // An auto-browsed entry carries the raw DisplayName path; build_entity_defs
  // must surface it as the entity name instead of title-casing the slug.
  NodeMap node_map;
  std::vector<NodeMapEntry> auto_entries;
  NodeMapEntry entry;
  entry.node_id_str = "ns=2;s=DB_Test.Level";
  entry.node_id = NodeMap::parse_node_id(entry.node_id_str);
  entry.entity_id = "plc_1_db_test";
  entry.entity_display_name = "PLC_1 / DB_Test";
  entry.data_name = "level";
  entry.data_type = "float";
  auto_entries.push_back(entry);

  ASSERT_EQ(node_map.merge_auto_browsed_entries(std::move(auto_entries)), 1u);
  ASSERT_EQ(node_map.entity_defs().size(), 1u);
  EXPECT_EQ(node_map.entity_defs()[0].id, "plc_1_db_test");
  EXPECT_EQ(node_map.entity_defs()[0].name, "PLC_1 / DB_Test");
}

}  // namespace ros2_medkit_gateway
