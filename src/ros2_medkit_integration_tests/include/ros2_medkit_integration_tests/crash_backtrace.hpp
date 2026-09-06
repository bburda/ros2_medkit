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

/// Lets exactly one thread write a report.
///
/// sa_mask does NOT do this: it blocks signals for the thread running the
/// handler only. Thread A faulting on SIGSEGV while thread B takes SIGBUS gives
/// two concurrent handlers writing one static frame buffer and interleaving
/// their output. The second entrant says so in one line and gets out of the way.
inline std::atomic_flag & crash_reporting_flag() {
  static std::atomic_flag reporting = ATOMIC_FLAG_INIT;
  return reporting;
}

/// A stack of its own for the handler to run on.
///
/// The commonest silent SIGSEGV is a stack overflow, and that is exactly the one
/// a handler on the ordinary stack cannot report: it needs stack to run, there is
/// none, so it faults again and - the disposition already reset - the process
/// dies having written nothing. Measured before this existed: a null dereference
/// produced 452 bytes and two markers, a stack overflow produced zero of both.
///
/// SIGSTKSZ is not a compile-time constant on current glibc, so the size is
/// fixed here; 64 KB is far more than backtrace_symbols_fd needs.
inline constexpr std::size_t kAltStackBytes = 64 * 1024;

inline char * crash_alt_stack() {
  static char storage[kAltStackBytes];
  return storage;
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

  if (crash_reporting_flag().test_and_set()) {
    write_literal(kCrashMarker);
    write_literal(" second thread also faulted; its frames are not reported\n");
    ::raise(signum);
    return;
  }

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

  // Explicit, not implicit. Returning re-executes the faulting instruction,
  // which re-raises a genuine SIGSEGV or SIGBUS - but a signal delivered by
  // kill(), raise() or pthread_kill() has no instruction to retry, so the
  // process would simply resume and then be killed by the alarm above five
  // seconds later, reporting SIGALRM instead of the signal that happened.
  // Measured before this line existed: raise(SIGSEGV) gave exit status 142, not
  // 139. SA_RESETHAND has already restored the default disposition, so this
  // terminates the process with the right status in both cases.
  ::raise(signum);
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

  // sigaltstack is per-thread, and this runs on the thread that calls it - the
  // main thread, where a node is constructed and where the startup crashes this
  // exists for happen. A stack overflow on a DDS thread is still silent; that is
  // a real limit, not an oversight.
  stack_t alt{};
  alt.ss_sp = detail::crash_alt_stack();
  alt.ss_size = detail::kAltStackBytes;
  alt.ss_flags = 0;
  ::sigaltstack(&alt, nullptr);

  struct sigaction action {};
  action.sa_handler = &detail::crash_handler;
  // Blocks the other fatal signals for the duration of the handler ON THIS
  // THREAD. That is all sa_mask can do - the cross-thread case is what
  // crash_reporting_flag() above is for.
  ::sigemptyset(&action.sa_mask);
  ::sigaddset(&action.sa_mask, SIGSEGV);
  ::sigaddset(&action.sa_mask, SIGBUS);
  ::sigaddset(&action.sa_mask, SIGABRT);
  // SA_RESETHAND restores the default disposition before the handler runs, so
  // returning from it re-raises the signal and the process dies the way it
  // would have without us.
  // SA_ONSTACK puts the handler on the alternate stack above, which is what lets
  // it run at all when the ordinary stack is the thing that overflowed.
  // static_cast, because these constants are unsigned and sa_flags is int:
  // -Wsign-conversion, which this workspace builds with, reports the change of
  // value once per translation unit that includes this header.
  action.sa_flags = static_cast<int>(SA_RESETHAND | SA_ONSTACK);

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
