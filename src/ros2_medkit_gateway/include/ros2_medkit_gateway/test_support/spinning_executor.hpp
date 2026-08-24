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

#include <atomic>
#include <chrono>
#include <cstdio>
#include <exception>
#include <memory>
#include <thread>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"

/// Support for this package's tests and the tests of packages that extend it.
/// Not part of the gateway's supported API: it exists to be included from a
/// test translation unit and carries no stability promise.
namespace ros2_medkit_gateway::test_support {

/**
 * @brief An executor whose node set is fixed for as long as its thread runs.
 *
 * An executor rebuilds its entity collection whenever a node joins or leaves
 * it, and a thread already inside spin()/spin_some() reads that collection
 * through the executor's notify waitable. The two overlap without a lock the
 * caller can take, and ThreadSanitizer reports it as a write in operator
 * delete against ExecutorNotifyWaitable::is_ready(). The overlap is what this
 * type removes: every node is added before the thread starts, and the thread is
 * cancelled and joined before any node leaves.
 *
 * A test that needs a node for part of its run gives that node its own
 * instance. Reaching into an executor another thread is spinning is the shape
 * this exists to replace, so nothing here exposes a way to do it.
 */
class SpinningExecutor {
 public:
  explicit SpinningExecutor(std::vector<rclcpp::Node::SharedPtr> nodes)
    : executor_(std::make_shared<rclcpp::executors::SingleThreadedExecutor>()), nodes_(std::move(nodes)) {
    for (const auto & node : nodes_) {
      executor_->add_node(node);
    }
    thread_ = std::thread([this]() {
      try {
        executor_->spin();
      } catch (...) {
        // A callback that throws would otherwise leave the thread's entry
        // function by exception, which is std::terminate: the whole test binary
        // dies with no test named. Carry it out to stop() instead.
        spin_exception_ = std::current_exception();
      }
      spin_returned_.store(true);
    });
  }

  SpinningExecutor(const SpinningExecutor &) = delete;
  SpinningExecutor & operator=(const SpinningExecutor &) = delete;
  SpinningExecutor(SpinningExecutor &&) = delete;
  SpinningExecutor & operator=(SpinningExecutor &&) = delete;

  ~SpinningExecutor() {
    try {
      stop();
    } catch (const std::exception & e) {
      std::fprintf(stderr, "SpinningExecutor: spin() threw: %s\n", e.what());
    } catch (...) {
      std::fprintf(stderr, "SpinningExecutor: spin() threw a non-std exception\n");
    }
  }

  /// Stop the spin, join the thread, release the nodes, and rethrow whatever
  /// spin() threw. Idempotent; the destructor calls it and reports instead.
  void stop() {
    if (thread_.joinable()) {
      // On Jazzy, cancel() clears the same `spinning` flag that spin() sets on
      // entry, so a cancel issued before the worker reaches spin() is
      // overwritten and lost - and join() then waits for a spin nobody asked to
      // stop. Repeating it until the worker is provably out costs one sleep in
      // the ordinary case and cannot deadlock in the racing one.
      while (!spin_returned_.load()) {
        executor_->cancel();
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
      }
      thread_.join();
      for (const auto & node : nodes_) {
        executor_->remove_node(node);
      }
      nodes_.clear();
    }
    if (spin_exception_) {
      auto pending = spin_exception_;
      spin_exception_ = nullptr;
      std::rethrow_exception(pending);
    }
  }

 private:
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::vector<rclcpp::Node::SharedPtr> nodes_;
  std::atomic<bool> spin_returned_{false};
  std::exception_ptr spin_exception_;
  std::thread thread_;
};

}  // namespace ros2_medkit_gateway::test_support
