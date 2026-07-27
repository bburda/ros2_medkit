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
// HONEST SCOPE: these two tests do NOT reproduce the abort they describe - they pass
// with and without the fix. A single-node process tears down cleanly either way; the
// abort needs the real multi-node + executor teardown. The evidence for the fix is the
// end-to-end measurement recorded in the commit message (0 aborts in 40 gateway
// shutdowns, against a 6-in-33 baseline on the same build). What these tests DO pin is
// the contract in both directions - transport destroyed after context shutdown, and
// before it - so a future refactor that reorders this teardown has something to fail
// against. Do not read them as proof the abort is gone.
//
// Ros2ParameterTransport owns an internal rclcpp::Node ("_param_client_node"). rclcpp's
// ~NodeGraph calls GraphListener::remove_node(), which throws NodeNotFoundError once the
// context has been shut down and the listener has dropped its node list. ~NodeGraph is
// implicitly noexcept, so that throw is an immediate std::terminate - the process dies
// with SIGABRT and no try/catch at any call site can intercept it.
//
// This is reachable whenever an owner outlives rclcpp::shutdown(): the gateway's own
// managers, and any plugin whose objects are destroyed from ~GatewayNode (which runs
// after rclcpp::shutdown() in main.cpp's teardown). In the field it surfaced as an
// intermittent exit -6 on Ctrl+C, because whether the listener had already cleared its
// list by then is a race. Here it is deterministic: rclcpp::shutdown() has returned, so
// the listener is definitely down before the transport is dropped.
//
// This test lives in its own translation unit because it must own the whole
// init/shutdown lifecycle of the context - it cannot share the process-wide
// SetUpTestSuite/TearDownTestSuite bracket the other transport tests use.

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

  // REQUIRED to reproduce: rclcpp registers a node with the GraphListener lazily, on the
  // first graph query - and ~NodeGraph only calls remove_node() for a node that was
  // registered. A transport that never talked to anyone therefore tears down cleanly even
  // without the fix. One round trip (against a node that does not exist - it just has to
  // reach wait_for_service, bounded by kServiceTimeoutSec) puts _param_client_node in the
  // listener, which is the state every real gateway is in by shutdown time.
  transport->list_parameters("/no_such_node_for_graph_registration");

  // Invalidate the context while the transport still holds its internal node. The
  // pre-shutdown callback registered in the constructor must release it here.
  rclcpp::shutdown();

  // Before the fix this dropped param_node_ with the GraphListener already down:
  // ~NodeGraph -> remove_node() -> NodeNotFoundError -> std::terminate. The process
  // aborted instead of reaching any assertion below.
  transport.reset();
  owner.reset();
  SUCCEED() << "transport destroyed after rclcpp::shutdown() without aborting";
}

// The ordinary case must keep working: the transport dies while the context is still
// valid, so the destructor deregisters the (never-fired) pre-shutdown callback and runs
// the real teardown itself. A stale callback left on the context would fire later
// against freed memory.
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
