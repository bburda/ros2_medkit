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
#include <cstddef>

#include <execinfo.h>
#include <unistd.h>

// A sanitizer owns the fatal signals and reports far more than a bare stack, so
// this file stands down when one is present. GCC and Clang announce that
// differently, and __has_feature has to be probed in its own directive because
// GCC expands it eagerly inside a compound #if.
#if defined(__SANITIZE_ADDRESS__) || defined(__SANITIZE_THREAD__)
#define MEDKIT_CRASH_BACKTRACE_SANITIZED 1
#elif defined(__has_feature)
#if __has_feature(address_sanitizer) || __has_feature(thread_sanitizer)
#define MEDKIT_CRASH_BACKTRACE_SANITIZED 1
#endif
#endif

namespace ros2_medkit_integration_tests {

/// Marker that prefixes every frame, so one grep separates a crash report from
/// the surrounding node output.
inline constexpr const char kCrashMarker[] = "MEDKIT-CRASH";

namespace detail {

inline constexpr int kMaxFrames = 64;

/// Storage for the frame addresses. A signal handler must not allocate, so the
/// buffer is reserved up front and reused.
inline void ** crash_frame_buffer() {
  static void * frames[kMaxFrames];
  return frames;
}

/// Writes a string literal. The length comes from the type, so nothing in the
/// handler calls strlen - which POSIX does not list as async-signal-safe.
template <std::size_t N>
inline void write_literal(const char (&text)[N]) {
  const ssize_t written = ::write(STDERR_FILENO, text, N - 1);
  static_cast<void>(written);
}

/// Writes the marker, the signal number and the backtrace, then returns so the
/// default disposition installed by SA_RESETHAND can terminate the process with
/// the original signal. That keeps the exit status the test harness sees
/// unchanged: a segfault still reports as -11, now with frames attached.
inline void crash_handler(int signum) {
  // Unwinding from a signal handler is not async-signal-safe: backtrace() can
  // need the loader or an allocator lock, and if the faulting thread already
  // held one it deadlocks here - in exactly the startup paths this exists to
  // diagnose. alarm() is async-signal-safe and SIGALRM's disposition is the
  // default, so a wedged unwinder becomes a dead process after five seconds
  // instead of a test that hangs to its timeout with nothing to read. The exit
  // status is then SIGALRM rather than the original signal, which is the
  // trade: a wrong status beats no output and no status at all.
  ::alarm(5);

  write_literal(kCrashMarker);
  switch (signum) {
    case SIGSEGV:
      write_literal(" signal=SIGSEGV\n");
      break;
    case SIGBUS:
      write_literal(" signal=SIGBUS\n");
      break;
    case SIGABRT:
      write_literal(" signal=SIGABRT\n");
      break;
    default:
      write_literal(" signal=other\n");
      break;
  }

  void ** frames = crash_frame_buffer();
  const int depth = ::backtrace(frames, kMaxFrames);
  // backtrace_symbols_fd writes through the raw fd and allocates nothing, which
  // is what makes it usable here; backtrace_symbols would call malloc.
  ::backtrace_symbols_fd(frames, depth, STDERR_FILENO);
  write_literal(kCrashMarker);
  write_literal(" end\n");
}

}  // namespace detail

/// Report the stack on a fatal signal instead of dying silently.
///
/// A process killed by SIGSEGV during startup leaves nothing behind: no output,
/// no core file (a container cannot set the host's core_pattern), and the test
/// harness reports only the exit status. Without frames there is no way to tell
/// a defect in this repository from one below it, in rclcpp, rmw or the DDS
/// implementation.
///
/// Call this before any other work in main(). The first `backtrace` call
/// resolves the unwinder's lazy relocations, which does allocate - so it is made
/// here, at install time, and never inside the handler.
inline void install_crash_backtrace() {
#ifdef MEDKIT_CRASH_BACKTRACE_SANITIZED
  return;
#else
  void ** frames = detail::crash_frame_buffer();
  static_cast<void>(::backtrace(frames, 1));

  struct sigaction action {};
  action.sa_handler = &detail::crash_handler;
  // The handler writes into one static frame buffer, and SA_RESETHAND only
  // resets the signal that fired. Without this, a second thread taking a
  // DIFFERENT fatal signal would re-enter concurrently and interleave the two
  // reports.
  ::sigemptyset(&action.sa_mask);
  ::sigaddset(&action.sa_mask, SIGSEGV);
  ::sigaddset(&action.sa_mask, SIGBUS);
  ::sigaddset(&action.sa_mask, SIGABRT);
  // SA_RESETHAND restores the default disposition before the handler runs, so
  // returning from it re-raises the signal and the process dies the way it
  // would have without us.
  action.sa_flags = SA_RESETHAND;

  ::sigaction(SIGSEGV, &action, nullptr);
  ::sigaction(SIGBUS, &action, nullptr);
  ::sigaction(SIGABRT, &action, nullptr);
#endif
}

/// True when install_crash_backtrace() installs handlers in this build.
inline constexpr bool crash_backtrace_is_active() {
#ifdef MEDKIT_CRASH_BACKTRACE_SANITIZED
  return false;
#else
  return true;
#endif
}

}  // namespace ros2_medkit_integration_tests
