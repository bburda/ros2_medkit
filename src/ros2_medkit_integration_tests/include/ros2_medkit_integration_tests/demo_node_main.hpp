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

#include <csignal>
#include <functional>
#include <memory>
#include <thread>

#include <rclcpp/rclcpp.hpp>

#include "ros2_medkit_integration_tests/crash_backtrace.hpp"

namespace ros2_medkit_integration_tests {

/**
 * @brief Run a demo node with a graceful, race-free shutdown.
 *
 * Integration-test shutdowns used to flake on a rotating cast of demo
 * binaries with errors like:
 *
 *   "Unexpected condition: string capacity was zero for allocated data!
 *    Exiting."  (rosidl_runtime_c / exit 255)
 *
 * Root cause: rclcpp's default SIGINT handler calls rclcpp::shutdown()
 * asynchronously from a signal context, which invalidates the shared
 * rcl context mid-run. An in-flight timer callback can then be holding
 * a rosidl message whose string fields point at allocator state that is
 * already partially torn down; destruction aborts.
 *
 * Pattern used here (following "signalfd / sigwait thread" best practice):
 *
 *   1. Block SIGINT / SIGTERM in the main thread before any rclcpp work.
 *   2. Call rclcpp::init with `SignalHandlerOptions::None` so rclcpp does
 *      NOT install its own handlers on top.
 *   3. Construct the node + executor with signals still blocked.
 *   4. Spawn a dedicated sigwait thread that blocks on sigwait() for
 *      SIGINT / SIGTERM and, on arrival, calls executor.cancel() to wake
 *      spin() from a known-good thread context.
 *   5. executor.spin() returns cleanly; the main thread then removes the
 *      node from the executor and resets the shared_ptr while the rcl
 *      context is still valid. Message / publisher destructors therefore
 *      run against a consistent allocator and cannot trip the rosidl
 *      sequence-invariant abort.
 *   6. rclcpp::shutdown() is called last, AFTER everything is destroyed.
 *
 * Usage:
 *
 *   int main(int argc, char ** argv) {
 *     return ros2_medkit_integration_tests::run_demo_node(argc, argv, [] {
 *       return std::make_shared<MyDemoNode>();
 *     });
 *   }
 *
 * The factory is called with signals already blocked and rclcpp::init
 * already complete, so node constructors can create publishers / timers
 * freely.
 */
inline int run_demo_node(int argc, char ** argv, const std::function<std::shared_ptr<rclcpp::Node>()> & node_factory) {
  // Installed first: a fatal signal anywhere below, including inside the DDS
  // stack while the participant is being created, then names its frames instead
  // of leaving an exit status and no output.
  install_crash_backtrace();

  sigset_t mask;
  sigset_t old;
  sigemptyset(&mask);
  sigaddset(&mask, SIGINT);
  sigaddset(&mask, SIGTERM);
  if (pthread_sigmask(SIG_BLOCK, &mask, &old) != 0) {
    return 1;
  }

  rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);

  int exit_code = 0;
  {
    auto node = node_factory();
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);

    std::thread signal_thread([&executor, &mask] {
      int signum = 0;
      // sigwait blocks until SIGINT / SIGTERM arrives. Every thread inherits
      // the blocked mask from the main thread so the signal is queued until
      // this one consumes it, no matter which thread the kernel delivers it
      // to.
      while (true) {
        const int rc = sigwait(&mask, &signum);
        if (rc == 0) {
          break;
        }
        // EINTR can theoretically fire; retry.
      }
      executor.cancel();
    });

    executor.spin();

    // spin() returned via executor.cancel(); signal_thread has already
    // consumed the signal and is about to exit.
    if (signal_thread.joinable()) {
      signal_thread.join();
    }

    executor.remove_node(node);
    node.reset();
  }

  rclcpp::shutdown();
  // Restore the caller's signal mask for completeness.
  pthread_sigmask(SIG_SETMASK, &old, nullptr);
  return exit_code;
}

}  // namespace ros2_medkit_integration_tests
