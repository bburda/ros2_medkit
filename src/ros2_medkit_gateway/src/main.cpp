// Copyright 2025-2026 bburda, mfaferek93
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

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "ros2_medkit_gateway/core/thread_pool_config.hpp"
#include "ros2_medkit_gateway/data/ros2_topic_data_provider.hpp"
#include "ros2_medkit_gateway/gateway_node.hpp"
#include "ros2_medkit_gateway/ros2_common/ros2_subscription_executor.hpp"
#include "ros2_medkit_serialization/json_serializer.hpp"

namespace {

// Declare operator-tunable parameters for Ros2SubscriptionExecutor and
// Ros2TopicDataProvider. Every knob is clamped to a conservative range
// because a runaway value here (e.g. queue_depth=1e9) can starve the gateway
// process for memory. Defaults mirror the Config{} constructors so the stock
// deployment behaviour is unchanged.
ros2_medkit_gateway::ros2_common::Ros2SubscriptionExecutor::Config declare_executor_config(rclcpp::Node & node) {
  ros2_medkit_gateway::ros2_common::Ros2SubscriptionExecutor::Config cfg;
  const auto queue_depth = node.declare_parameter<int64_t>("subscription_executor.max_queue_depth", 256);
  cfg.max_queue_depth = static_cast<std::size_t>(std::clamp<int64_t>(queue_depth, 16, 4096));
  const auto watchdog_ms = node.declare_parameter<int64_t>("subscription_executor.watchdog_threshold_ms", 5000);
  cfg.watchdog_threshold = std::chrono::milliseconds{std::clamp<int64_t>(watchdog_ms, 100, 60000)};
  const auto watchdog_tick = node.declare_parameter<int64_t>("subscription_executor.watchdog_tick_ms", 1000);
  cfg.watchdog_tick = std::chrono::milliseconds{std::clamp<int64_t>(watchdog_tick, 10, 10000)};
  const auto graph_tick = node.declare_parameter<int64_t>("subscription_executor.graph_poll_tick_ms", 100);
  cfg.graph_poll_tick = std::chrono::milliseconds{std::clamp<int64_t>(graph_tick, 10, 10000)};
  return cfg;
}

ros2_medkit_gateway::Ros2TopicDataProvider::Config declare_data_provider_config(rclcpp::Node & node) {
  ros2_medkit_gateway::Ros2TopicDataProvider::Config cfg;
  const auto pool_size = node.declare_parameter<int64_t>("data_provider.max_pool_size", 256);
  cfg.max_pool_size = static_cast<std::size_t>(std::clamp<int64_t>(pool_size, 1, 4096));
  const auto cold_cap = node.declare_parameter<int64_t>("data_provider.cold_wait_cap", 4);
  cfg.cold_wait_cap = static_cast<std::size_t>(std::clamp<int64_t>(cold_cap, 0, 1024));
  const auto max_parallel = node.declare_parameter<int64_t>("data_provider.max_parallel_samples", 8);
  cfg.max_parallel_samples = static_cast<std::size_t>(std::clamp<int64_t>(max_parallel, 1, 256));
  const auto idle_safety_s = node.declare_parameter<int64_t>("data_provider.idle_safety_net_sec", 15 * 60);
  cfg.idle_safety_net = std::chrono::seconds{std::clamp<int64_t>(idle_safety_s, 0, 24 * 3600)};
  const auto idle_sweep_s = node.declare_parameter<int64_t>("data_provider.idle_sweep_tick_sec", 60);
  cfg.idle_sweep_tick = std::chrono::seconds{std::clamp<int64_t>(idle_sweep_s, 0, 3600)};
  return cfg;
}

}  // namespace

int main(int argc, char ** argv) {
  bool ros_inited = false;
  try {
    rclcpp::init(argc, argv);
    ros_inited = true;

    auto node = std::make_shared<ros2_medkit_gateway::GatewayNode>();

    // MultiThreadedExecutor for the gateway node. Issue #440: the thread count
    // is bounded by the server.executor_threads parameter (default 2) instead of
    // rclcpp's default (host cores, minimum 2), so the footprint does not grow
    // with the host core count.
    //
    // What the count buys (issue #575): the response callbacks of the blocking
    // RPC clients (generic service clients + the per-action client trio) live
    // in a shared Reentrant callback group (ros2_common/callback_groups.hpp),
    // so with 2+ threads a service/action response is dispatched even while a
    // default-group callback (e.g. a discovery refresh pass) is running. The
    // node's timers, graph events and the SSE-fault / trigger-fault / rosout
    // subscriptions stay in the default MutuallyExclusive group by design -
    // refresh passes must stay serialized and those subscriptions rely on
    // in-order delivery; per-action status subscriptions sit in their own
    // MutuallyExclusive group, ordered but decoupled from the default group.
    // A single thread is still safe: a Reentrant group does not require a
    // second thread, and the blocking wait for an RPC runs on the cpp-httplib
    // pool thread (a separate server_thread_), never on an executor thread,
    // so it cannot deadlock the executor; the fault transport additionally
    // uses its own private executor. The Ros2SubscriptionExecutor built below
    // owns its own internal single-threaded executor (spun from its worker
    // thread); the subscription node is intentionally not added here.
    const auto executor_threads =
        ros2_medkit_gateway::clamp_thread_count(node->get_parameter("server.executor_threads").as_int(), 1, 256);
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), executor_threads);
    executor.add_node(node);
    RCLCPP_INFO(node->get_logger(), "Main executor bounded to %zu threads", executor_threads);

    // Stand up the ROS 2 subscription executor + topic data provider.
    // Issue #375: all subscription create/destroy calls are funneled through the
    // serial worker owned by sub_exec, eliminating the rcl hash-map race that
    // previously killed /data on Rolling (now Lyrical) when concurrent HTTP handler threads
    // created subscriptions on the same node.
    const auto exec_cfg = declare_executor_config(*node);
    const auto dp_cfg = declare_data_provider_config(*node);

    // Issue #440: warn if the HTTP pool cannot cover the documented worst case.
    // Every SSE stream pins a worker for its lifetime and every cold-/data wait
    // parks one for up to topic_sample_timeout_sec, so a pool below
    // sse.max_clients + data_provider.cold_wait_cap can be fully starved (slow
    // bulk-data downloads also hold a worker, uncounted, so leave extra headroom
    // if you serve those concurrently). This is a misconfiguration warning, not a
    // hard error - an operator who knows their traffic stays under the cap may
    // run a smaller pool. Values are clamped the same way their consumers clamp
    // them, so the comparison reflects effective sizes. cold_wait_cap is read
    // from dp_cfg (already clamped above).
    {
      const auto http_pool = ros2_medkit_gateway::clamp_thread_count(
          node->get_parameter("server.http_thread_pool_size").as_int(), 1, 1024);
      const auto sse_clients =
          ros2_medkit_gateway::clamp_thread_count(node->get_parameter("sse.max_clients").as_int(), 1, 1024);
      const auto needed = sse_clients + dp_cfg.cold_wait_cap;
      if (http_pool < needed) {
        RCLCPP_WARN(node->get_logger(),
                    "server.http_thread_pool_size (%zu) is below sse.max_clients (%zu) + "
                    "data_provider.cold_wait_cap (%zu) = %zu: a burst of SSE streams and cold /data "
                    "requests can starve the HTTP worker pool. Raise http_thread_pool_size, or lower "
                    "sse.max_clients / data_provider.cold_wait_cap, so the pool covers their sum.",
                    http_pool, sse_clients, dp_cfg.cold_wait_cap, needed);
      }
    }

    auto sub_exec = std::make_shared<ros2_medkit_gateway::ros2_common::Ros2SubscriptionExecutor>(node, exec_cfg);
    auto serializer = std::make_shared<ros2_medkit_serialization::JsonSerializer>();
    auto data_provider = std::make_shared<ros2_medkit_gateway::Ros2TopicDataProvider>(sub_exec, serializer, dp_cfg);
    node->set_topic_data_provider(data_provider);

    // Zero-config entity freeze-frames subscribe through the same serial worker
    // (issue #375). Wired here because the executor is built after the node; the
    // slot is dropped in ~GatewayNode, which runs after sub_exec.reset() below
    // and so takes the executor's shutdown fast path.
    node->init_entity_freeze_frame_capture(*sub_exec);

    // Route per-trigger topic subscriptions through the same serial worker
    // (issue #548): their callbacks are then dispatched on - and drained by -
    // the executor at teardown, so a subscription cannot fire on a partially
    // destroyed subscriber. shutdown_trigger_subscriber() below drops those
    // subscriptions while sub_exec is still alive.
    node->set_trigger_subscription_executor(*sub_exec);
    // Config-less threshold-rule (fault-trigger) engine (issue #235). Started
    // here so its evaluation loop sees loaded plugins; joined in ~GatewayNode.
    node->init_fault_trigger_engine();

    // Spin in a try/catch so an uncaught handler exception falls through to the
    // explicit teardown block below. Without this, an escaping throw bypasses
    // the teardown ordering and triggers exactly the rclcpp abort described
    // there (~GatewayNode running against a dead executor, exit -6).
    try {
      executor.spin();
    } catch (const std::exception & ex) {
      RCLCPP_ERROR(node->get_logger(), "Executor.spin() threw an unhandled exception: %s. Falling through to teardown.",
                   ex.what());
    } catch (...) {
      RCLCPP_ERROR(node->get_logger(), "Executor.spin() threw an unknown exception. Falling through to teardown.");
    }

    // Teardown order (issue #375): stack-unwind destructs executor before node
    // which leaves ~GatewayNode running against a dead executor. Newer rclcpp
    // (Lyrical; recent Jazzy patch releases) asserts 'node needs to be
    // associated with an executor' and aborts with exit -6 when the managers'
    // shutdown paths touch service clients. Explicit teardown avoids that:
    //   1. stop + join the REST server FIRST, so no httplib handler thread can
    //      be mid-subscribe()/unsubscribe() (which dereference sub_exec through
    //      the trigger subscriber, and the data provider likewise) when the
    //      subscription executor is freed in step 4 (issue #548). ~GatewayNode
    //      calls stop_rest_server() again; it is idempotent.
    //   2. detach the provider from GatewayNode so the managers stop using it
    //      before we drop it (GatewayNode otherwise holds a shared_ptr that
    //      would keep the provider alive past data_provider.reset()).
    //   3. shut down the trigger topic subscriber so its per-trigger
    //      subscriptions are destroyed on - and drained by - the subscription
    //      worker while sub_exec is still alive (issue #548).
    //   4. drop the provider (clears pool entries via the subscription worker)
    //   5. reset sub_exec (joins worker, tears down internal subscription executor)
    //   6. remove the gateway node from the executor and drop our ref so
    //      ~GatewayNode runs with the executor still alive.
    node->stop_rest_server();
    node->set_topic_data_provider(nullptr);
    node->shutdown_trigger_subscriber();
    data_provider.reset();
    sub_exec.reset();
    executor.remove_node(node);
    node.reset();

    rclcpp::shutdown();
  } catch (const std::exception & ex) {
    fprintf(stderr, "[ros2_medkit_gateway] Fatal exception in main: %s\n", ex.what());
    if (ros_inited && rclcpp::ok()) {
      rclcpp::shutdown();
    }
    return 1;
  } catch (...) {
    fprintf(stderr, "[ros2_medkit_gateway] Fatal unknown exception in main\n");
    if (ros_inited && rclcpp::ok()) {
      rclcpp::shutdown();
    }
    return 1;
  }
  return 0;
}
