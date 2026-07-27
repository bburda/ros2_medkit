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

// Contract coverage for Ros2ParameterTransport's teardown ordering.
//
// HONEST SCOPE: these tests do NOT reproduce the SIGABRT that motivated the pre-shutdown
// callback, and they are not expected to. They pass with and without it. The abort needs a
// first-ever graph use to race the GraphListener's shutdown so the node ends up
// half-registered (see the header comment on context_ / pre_shutdown_handle_), and this
// single-threaded sequence cannot produce that race. Destroying a FULLY registered node
// after rclcpp::shutdown() is a clean operation in rclcpp, so nothing here can abort.
//
// What these tests do pin is the teardown order in both directions - transport destroyed
// after context shutdown, and before it - so a refactor that reorders it has something to
// fail against. Do not read a green run here as proof the abort is fixed, and do not
// conclude from it that the callback is dead code. The evidence for the fix is the
// end-to-end measurement in the commit message: 0 aborts in 40 gateway shutdowns against a
// 6-in-33 baseline on the same build.
//
// This test lives in its own translation unit because it must own the whole
// init/shutdown lifecycle of the context - it cannot share the process-wide
// SetUpTestSuite/TearDownTestSuite bracket the other transport tests use. It runs two
// init/shutdown cycles in one process, which is also what makes the destructor's
// remove_pre_shutdown_callback() observable: a callback left registered by the first cycle
// would fire during the second one against a freed transport.

#include <gtest/gtest.h>

#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "ros2_medkit_gateway/ros2/transports/ros2_parameter_transport.hpp"

using ros2_medkit_gateway::ros2::Ros2ParameterTransport;

namespace {
constexpr double kServiceTimeoutSec = 0.2;
constexpr double kNegativeCacheTtlSec = 1.0;
}  // namespace

TEST(Ros2ParameterTransportShutdown, DestroyingTheTransportAfterContextShutdownDoesNotAbort) {
  rclcpp::init(0, nullptr);
  auto owner = std::make_shared<rclcpp::Node>("param_transport_shutdown_owner");
  auto transport = std::make_unique<Ros2ParameterTransport>(owner.get(), kServiceTimeoutSec, kNegativeCacheTtlSec);

  // Required for ~NodeGraph to call remove_node() at all: rclcpp registers a node with the
  // GraphListener lazily, on the first graph query, and ~NodeGraph only removes a node whose
  // should_add_to_graph_listener_ flag was consumed. One round trip (against a node that
  // does not exist - it only has to reach wait_for_service, bounded by kServiceTimeoutSec)
  // puts _param_client_node in the listener, which is the state a real gateway is in by
  // shutdown time. Without it this test would exercise nothing.
  transport->list_parameters("/no_such_node_for_graph_registration");

  // Invalidate the context while the transport still holds its internal node. The
  // pre-shutdown callback registered in the constructor releases it here.
  rclcpp::shutdown();

  // Dropping a fully registered node after shutdown is clean in rclcpp (remove_node() takes
  // its is_shutdown() branch and erases from a list __shutdown() never touched), so this
  // does not abort with or without the callback. It pins the order, nothing more.
  transport.reset();
  owner.reset();
  SUCCEED() << "transport destroyed after rclcpp::shutdown() without aborting";
}

// The ordinary case: the transport dies while the context is still valid, so the
// destructor deregisters its pre-shutdown callback and runs the real teardown itself.
//
// This test also depends on the one above having run first, and that is the point.
// Context::shutdown() fires pre-shutdown callbacks off a copy and never erases them, and
// the default context is a process-lifetime static that rclcpp::init() re-initializes
// without clearing the callback lists. So the callback the first test FIRED is still
// registered when this second init/shutdown cycle starts. If the destructor did not erase
// it, the rclcpp::shutdown() below would invoke it on the first test's freed transport.
TEST(Ros2ParameterTransportShutdown, DestroyingTheTransportBeforeContextShutdownIsClean) {
  rclcpp::init(0, nullptr);
  {
    auto owner = std::make_shared<rclcpp::Node>("param_transport_ordinary_owner");
    auto transport = std::make_unique<Ros2ParameterTransport>(owner.get(), kServiceTimeoutSec, kNegativeCacheTtlSec);
    transport.reset();
  }
  // If the destructor had left its callback registered, this shutdown would run it
  // against a destroyed transport.
  rclcpp::shutdown();
  SUCCEED() << "context shutdown after the transport was destroyed without aborting";
}
