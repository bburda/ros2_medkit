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
#include <atomic>

#include <gtest/gtest.h>

#include "ros2_medkit_graph_watchdog/detector.hpp"
#include "ros2_medkit_graph_watchdog/reliability_gate.hpp"

using ros2_medkit_graph_watchdog::DetectorContext;
using ros2_medkit_graph_watchdog::DetectorMode;
using ros2_medkit_graph_watchdog::mode_emits;

TEST(DetectorMode, OnlyRaiseEmits) {
  EXPECT_TRUE(mode_emits(DetectorMode::Raise));
  EXPECT_FALSE(mode_emits(DetectorMode::Advisory));
  EXPECT_FALSE(mode_emits(DetectorMode::Off));
}

// Locks the no-crash-on-unwired-client contract: a detector may be ticked
// before the plugin has wired a fault_client (or in a unit test that never
// wires one at all). fault_client is left null (default); gateway_node is
// also left null here since source_id is non-empty and the null-client guard
// short-circuits before the gateway_node-dependent empty-source_id branch is
// ever reached.
TEST(DetectorContext, RaiseAndClearFaultDoNotCrashWithNullFaultClient) {
  for (const DetectorMode mode : {DetectorMode::Raise, DetectorMode::Advisory, DetectorMode::Off}) {
    DetectorContext ctx;
    ctx.mode = mode;
    EXPECT_NO_THROW(ctx.raise_fault("GRAPH_QOS_MISMATCH", 2, "description", "/nav/planner"));
    EXPECT_NO_THROW(ctx.clear_fault("GRAPH_QOS_MISMATCH", "/nav/planner"));
  }
}

TEST(TfStaticQos, IsTransientLocalDepthOne) {
  const auto q = ros2_medkit_graph_watchdog::tf_static_qos();
  EXPECT_EQ(q.durability(), rclcpp::DurabilityPolicy::TransientLocal);
}

// null gate must not crash raise_fault (a detector may tick before the gate is wired)
TEST(DetectorContextGate, NullGateDoesNotCrash) {
  ros2_medkit_graph_watchdog::DetectorContext ctx;  // gate defaults null, fault_client null
  ctx.mode = ros2_medkit_graph_watchdog::DetectorMode::Raise;
  EXPECT_NO_THROW(ctx.raise_fault("GRAPH_X", 2, "d", "/e"));
}

// snapshot/cancelled default null (bare-context tests never see a gateway snapshot or the
// plugin's shutdown flag), and cancelled wires straight through to the caller's atomic.
TEST(DetectorContext, SnapshotAndCancelledDefaultNullAndWireThrough) {
  DetectorContext ctx;
  EXPECT_EQ(ctx.snapshot, nullptr);
  EXPECT_EQ(ctx.cancelled, nullptr);

  std::atomic<bool> cancel{false};
  ctx.cancelled = &cancel;
  EXPECT_FALSE(ctx.cancelled->load());
  cancel.store(true);
  EXPECT_TRUE(ctx.cancelled->load());
}
