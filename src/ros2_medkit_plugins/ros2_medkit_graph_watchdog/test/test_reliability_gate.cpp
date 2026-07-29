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
#include <gtest/gtest.h>

#include <rclcpp/rclcpp.hpp>

#include "ros2_medkit_gateway/core/providers/introspection_provider.hpp"
#include "ros2_medkit_graph_watchdog/detector.hpp"  // reliability_allows() free function
#include "ros2_medkit_graph_watchdog/reliability_gate.hpp"

using ros2_medkit_gateway::App;
using ros2_medkit_gateway::IntrospectionInput;
using ros2_medkit_graph_watchdog::reliability_allows;
using ros2_medkit_graph_watchdog::ReliabilityGate;

class ReliabilityGateTest : public ::testing::Test {
 protected:
  void SetUp() override {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
    node_ = std::make_shared<rclcpp::Node>("reliability_gate_test_node");
  }
  void TearDown() override {
    node_.reset();
  }
  static IntrospectionInput snap(std::vector<std::string> app_ids) {
    IntrospectionInput in;
    for (auto & id : app_ids) {
      App a;
      a.id = id;
      in.apps.push_back(a);
    }
    return in;
  }
  rclcpp::Node::SharedPtr node_;
  std::mutex mtx_;
};

TEST_F(ReliabilityGateTest, SuppressedUntilArmed) {
  ReliabilityGate g(3, node_.get(), &mtx_);
  g.update(snap({"/a"}), 5);
  EXPECT_FALSE(g.allows_raise("/a"));  // tick 5, first_seen 5
  g.update(snap({"/a"}), 8);           // 3 elapsed
  EXPECT_TRUE(g.allows_raise("/a"));
}

TEST_F(ReliabilityGateTest, UnknownSourceFallsBackToGlobalWarmup) {
  ReliabilityGate g(3, node_.get(), &mtx_);
  g.update(snap({"/a"}), 1);                    // graph first non-empty at tick 1
  EXPECT_FALSE(g.allows_raise("/some/topic"));  // unknown, 0 elapsed globally
  g.update(snap({"/a"}), 4);
  EXPECT_TRUE(g.allows_raise("/some/topic"));  // 3 elapsed globally
}

TEST_F(ReliabilityGateTest, StatusJsonShape) {
  ReliabilityGate g(3, node_.get(), &mtx_);
  g.update(snap({"/a"}), 1);
  g.update(snap({"/a"}), 10);
  const auto j = g.status_json();
  ASSERT_TRUE(j.contains("x-medkit-watchdog"));
  const auto & body = j["x-medkit-watchdog"];
  EXPECT_TRUE(body.contains("entities"));
  EXPECT_EQ(body["warmup_cycles"], 3);
}

// The gate composes warmup AND lifecycle: an armed entity whose managed lifecycle is
// non-active must still be suppressed, and reported "warming_up" in status_json. Without
// this the `&& node_ok` conjunct in allows_raise() is unreachable from a gate-level test
// (every other test builds Apps with no services, so nothing is lifecycle-tracked and
// node_ok is always true - dropping the conjunct would leave the suite green).
TEST_F(ReliabilityGateTest, ArmedButLifecycleInactiveIsSuppressed) {
  ReliabilityGate g(3, node_.get(), &mtx_);
  g.update(snap({"/a"}), 5);
  g.update(snap({"/a"}), 8);          // 3 elapsed -> armed
  EXPECT_TRUE(g.allows_raise("/a"));  // armed, nothing lifecycle-tracked -> allowed

  g.set_lifecycle_state_for_test("/a", "inactive");
  EXPECT_FALSE(g.allows_raise("/a"));  // armed but lifecycle non-active -> suppressed

  const auto j = g.status_json();
  bool found = false;
  for (const auto & e : j["x-medkit-watchdog"]["entities"]) {
    if (e["id"] == "/a") {
      found = true;
      EXPECT_EQ(e["state"], "warming_up");  // suppressed by lifecycle, though armed
      EXPECT_EQ(e["lifecycle"], "inactive");
      EXPECT_EQ(e["armed"], true);
    }
  }
  EXPECT_TRUE(found);

  // Once the node reports active, the same armed entity is allowed.
  g.set_lifecycle_state_for_test("/a", "active");
  EXPECT_TRUE(g.allows_raise("/a"));
}

// A null gate (not yet wired by the plugin) must never suppress a raise.
TEST(ReliabilityAllows, NullGateAlwaysAllows) {
  EXPECT_TRUE(reliability_allows(nullptr, "x"));
}

// The free function must mirror ReliabilityGate::allows_raise for a real gate:
// suppressed while warming up, allowed once armed.
TEST_F(ReliabilityGateTest, MirrorsGateArmedState) {
  ReliabilityGate g(3, node_.get(), &mtx_);
  g.update(snap({"/a"}), 5);
  EXPECT_FALSE(reliability_allows(&g, "/a"));  // not armed yet (0 elapsed)
  g.update(snap({"/a"}), 8);                   // 3 elapsed -> armed
  EXPECT_TRUE(reliability_allows(&g, "/a"));
}
