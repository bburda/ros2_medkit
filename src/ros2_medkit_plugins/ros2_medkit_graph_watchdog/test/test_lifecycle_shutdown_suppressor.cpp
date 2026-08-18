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
//
// Drives a REAL ReliabilityGate (needs a real rclcpp::Node for its LifecycleWatcher), fed
// through its set_departed_lifecycle_state_for_test() seam rather than a live managed
// node - that seam exists precisely so a suppressor test can drive
// departed_lifecycle_state_of() without a real GetState/transition_event round trip.
#include <gtest/gtest.h>

#include <memory>
#include <mutex>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "ros2_medkit_graph_watchdog/lifecycle_shutdown_suppressor.hpp"
#include "ros2_medkit_graph_watchdog/reliability_gate.hpp"

using ros2_medkit_graph_watchdog::DetectorContext;
using ros2_medkit_graph_watchdog::LifecycleShutdownSuppressor;
using ros2_medkit_graph_watchdog::ReliabilityGate;

class LifecycleShutdownSuppressorTest : public ::testing::Test {
 protected:
  static void SetUpTestSuite() {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  void SetUp() override {
    node_ = std::make_shared<rclcpp::Node>("lifecycle_shutdown_suppressor_test_node");
    gate_ = std::make_unique<ReliabilityGate>(0, node_.get(), &mtx_);
  }

  void TearDown() override {
    gate_.reset();
    node_.reset();
  }

  DetectorContext ctx_with_gate() {
    DetectorContext ctx;
    ctx.gate = gate_.get();
    return ctx;
  }

  rclcpp::Node::SharedPtr node_;
  std::mutex mtx_;
  std::unique_ptr<ReliabilityGate> gate_;
};

TEST_F(LifecycleShutdownSuppressorTest, NullGateAbstains) {
  LifecycleShutdownSuppressor s;
  DetectorContext ctx;  // ctx.gate stays nullptr
  EXPECT_FALSE(s.suppresses("/anything", ctx));
}

TEST_F(LifecycleShutdownSuppressorTest, AFqnThatNeverDepartedAbstains) {
  LifecycleShutdownSuppressor s;
  EXPECT_FALSE(s.suppresses("/never/departed", ctx_with_gate()));
}

TEST_F(LifecycleShutdownSuppressorTest, ShuttingdownLabelAloneSuppresses) {
  LifecycleShutdownSuppressor s;
  gate_->set_departed_lifecycle_state_for_test("/clean", "shuttingdown");
  EXPECT_TRUE(s.suppresses("/clean", ctx_with_gate()));
}

TEST_F(LifecycleShutdownSuppressorTest, FinalizedWithAnObservedTransitionAndNoErrorSuppresses) {
  LifecycleShutdownSuppressor s;
  gate_->set_departed_lifecycle_state_for_test("/clean", "finalized", /*saw_transition=*/true,
                                               /*error_terminated=*/false);
  EXPECT_TRUE(s.suppresses("/clean", ctx_with_gate()));
}

TEST_F(LifecycleShutdownSuppressorTest, FinalizedWithoutAnObservedTransitionIsNotSuppressed) {
  // An unobserved "finalized" cannot be told apart from a node that failed on_error before
  // this watcher ever saw a transition - the departure stays unclassified, so it is
  // reported rather than trusted.
  LifecycleShutdownSuppressor s;
  gate_->set_departed_lifecycle_state_for_test("/unseen", "finalized", /*saw_transition=*/false,
                                               /*error_terminated=*/false);
  EXPECT_FALSE(s.suppresses("/unseen", ctx_with_gate()));
}

TEST_F(LifecycleShutdownSuppressorTest, FinalizedThroughTheErrorBranchIsNotSuppressed) {
  LifecycleShutdownSuppressor s;
  gate_->set_departed_lifecycle_state_for_test("/crashed", "finalized", /*saw_transition=*/true,
                                               /*error_terminated=*/true);
  EXPECT_FALSE(s.suppresses("/crashed", ctx_with_gate()));
}

TEST_F(LifecycleShutdownSuppressorTest, UnconfiguredIsNeverTreatedAsClean) {
  // unconfigured is also the resting state of a failed configure() or a node that never
  // activated at all - treating it as clean would hide exactly that startup failure.
  LifecycleShutdownSuppressor s;
  gate_->set_departed_lifecycle_state_for_test("/never/started", "unconfigured");
  EXPECT_FALSE(s.suppresses("/never/started", ctx_with_gate()));
}

TEST_F(LifecycleShutdownSuppressorTest, InactiveIsNotSuppressed) {
  // A managed node paused mid-lifecycle (not shut down at all) is not a clean departure.
  LifecycleShutdownSuppressor s;
  gate_->set_departed_lifecycle_state_for_test("/paused", "inactive");
  EXPECT_FALSE(s.suppresses("/paused", ctx_with_gate()));
}

TEST_F(LifecycleShutdownSuppressorTest, IsDurable) {
  LifecycleShutdownSuppressor s;
  EXPECT_TRUE(s.durable());
}
