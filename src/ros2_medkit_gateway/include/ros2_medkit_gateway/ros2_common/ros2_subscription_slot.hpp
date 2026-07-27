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

#pragma once

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <utility>

#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rosidl_runtime_cpp/traits.hpp>
#include <tl/expected.hpp>

#include "ros2_medkit_gateway/ros2_common/ros2_subscription_executor.hpp"

namespace ros2_medkit_gateway::ros2_common {

/**
 * @brief RAII handle for a single rclcpp subscription whose creation and destruction
 *        are serialized on the Ros2SubscriptionExecutor worker thread.
 *
 * Create via `create_typed<MessageT>` (for known message types) or `create_generic`
 * (for runtime-typed topics). The returned `unique_ptr` owns the subscription; when
 * it drops, the destructor either:
 *   - posts a destroy task on the executor worker (normal path, bounded 30s deadline), or
 *   - if `Ros2SubscriptionExecutor::is_shutting_down()` is true, releases the shared
 *     pointer on the current thread. The subscription node's `rcl_node_fini` during
 *     the executor's teardown cleans up the rcl handle atomically.
 *
 * @par Thread safety
 * Factories and destructor are safe to call from any thread. Callbacks run on the
 * Ros2SubscriptionExecutor worker thread - the same thread that creates and destroys
 * subscriptions - so user-provided callbacks must keep work brief and must not call
 * back into Ros2SubscriptionSlot::create_typed/create_generic synchronously.
 *
 * @par No public escape hatch
 * The "release without executor" behavior is not exposed as a public method - the
 * destructor makes the decision internally based on the executor's shutdown state.
 * This shrinks the API surface and avoids footguns for safety analysis.
 */
class Ros2SubscriptionSlot final {
 public:
  /// Deadline applied to the worker-posted destroy task in the normal (non-shutdown) path.
  static constexpr std::chrono::milliseconds kDestroyDeadline{30000};

  template <typename MessageT>
  [[nodiscard]] static tl::expected<std::unique_ptr<Ros2SubscriptionSlot>, std::string>
  create_typed(Ros2SubscriptionExecutor & exec, const std::string & topic, const rclcpp::QoS & qos,
               std::function<void(std::shared_ptr<const MessageT>)> cb);

  [[nodiscard]] static tl::expected<std::unique_ptr<Ros2SubscriptionSlot>, std::string>
  create_generic(Ros2SubscriptionExecutor & exec, const std::string & topic, const std::string & type_name,
                 const rclcpp::QoS & qos, std::function<void(std::shared_ptr<const rclcpp::SerializedMessage>)> cb);

  ~Ros2SubscriptionSlot() noexcept;

  Ros2SubscriptionSlot(const Ros2SubscriptionSlot &) = delete;
  Ros2SubscriptionSlot & operator=(const Ros2SubscriptionSlot &) = delete;
  Ros2SubscriptionSlot(Ros2SubscriptionSlot &&) = delete;
  Ros2SubscriptionSlot & operator=(Ros2SubscriptionSlot &&) = delete;

  [[nodiscard]] const std::string & topic() const noexcept {
    return topic_;
  }
  [[nodiscard]] const std::string & type_name() const noexcept {
    return type_name_;
  }
  /// Publishers currently matched to this subscription (rcl graph query,
  /// callable from any thread while the slot is alive; may throw on rcl error).
  [[nodiscard]] size_t publisher_count() const {
    return sub_->get_publisher_count();
  }

 private:
  Ros2SubscriptionSlot(Ros2SubscriptionExecutor & exec, std::string topic, std::string type_name,
                       rclcpp::SubscriptionBase::SharedPtr sub);

  // Shared ownership of the shutdown flag keeps it readable even if the
  // executor has already been destroyed (fast-path scenario in tests and
  // at process teardown). See Ros2SubscriptionExecutor::shutdown_flag_ptr().
  std::shared_ptr<std::atomic<bool>> shutdown_flag_;
  // Non-owning pointer used only when shutdown_flag_ is false (executor alive).
  Ros2SubscriptionExecutor * exec_;
  std::string topic_;
  std::string type_name_;
  rclcpp::SubscriptionBase::SharedPtr sub_;
};

// ---------------------------------------------------------------------------
// Template implementation

template <typename MessageT>
tl::expected<std::unique_ptr<Ros2SubscriptionSlot>, std::string>
Ros2SubscriptionSlot::create_typed(Ros2SubscriptionExecutor & exec, const std::string & topic, const rclcpp::QoS & qos,
                                   std::function<void(std::shared_ptr<const MessageT>)> cb) {
  if (exec.is_shutting_down()) {
    return tl::unexpected(std::string{"executor shutting down"});
  }
  rclcpp::Node * node = exec.node();
  if (node == nullptr) {
    return tl::unexpected(std::string{"executor has no subscription node"});
  }

  auto result = exec.run_sync<rclcpp::SubscriptionBase::SharedPtr>(std::function<rclcpp::SubscriptionBase::SharedPtr()>(
      [node, topic, qos, cb_owned = std::move(cb)]() -> rclcpp::SubscriptionBase::SharedPtr {
        return node->create_subscription<MessageT>(topic, qos, cb_owned);
      }));
  if (!result) {
    return tl::unexpected(result.error());
  }
  std::string type_name{rosidl_generator_traits::name<MessageT>()};
  // std::make_unique cannot call the private ctor; use new + unique_ptr here.
  return std::unique_ptr<Ros2SubscriptionSlot>(
      new Ros2SubscriptionSlot(exec, topic, std::move(type_name), std::move(*result)));
}

}  // namespace ros2_medkit_gateway::ros2_common
