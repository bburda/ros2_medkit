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

#include <climits>
#include <csignal>
#include <cstdlib>
#include <string>

#include <unistd.h>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "ros2_medkit_integration_tests/crash_backtrace.hpp"

using ros2_medkit_integration_tests::crash_backtrace_is_active;
using ros2_medkit_integration_tests::install_crash_backtrace;

namespace {

constexpr bool kHandlerActive = crash_backtrace_is_active();

// Recursion the optimiser cannot flatten into a loop. Measured across -O0 to
// -O3: without the escaping address GCC turns this into a loop at -O2 and the
// process spins forever instead of overflowing, which made an earlier version of
// the test below hang rather than fail.
volatile char * stack_probe_sink = nullptr;

__attribute__((noinline)) int recurse_until_the_stack_runs_out(int x) {
  volatile char pad[4096];
  pad[0] = static_cast<char>(x);
  stack_probe_sink = pad;
  // The barrier is what makes this recurse rather than loop. `noinline` and an
  // escaping address are not enough inside a translation unit where the
  // function has internal linkage: measured, GCC still turned it into a loop at
  // -O2 and the process spun instead of overflowing.
  asm volatile("" : : "r"(pad) : "memory");
  return recurse_until_the_stack_runs_out(x + pad[0]) + 1;
}

/// Faults on an unmapped address that is NOT null.
///
/// Null would be simpler, and was what this did first, but UBSan diagnoses a
/// store through a null pointer before the store happens - so under the ASan job,
/// which builds `asan,ubsan`, no signal was ever raised and four of the cases
/// below passed their "no marker" assertion because nothing crashed rather than
/// because the handler stood down. A non-null unmapped address gives UBSan
/// nothing to object to and still segfaults.
///
/// Both volatile qualifiers are load-bearing and were picked by measuring: a
/// store through a plain `T * volatile` is deleted at -O2 as an erroneous path
/// and the process then does not crash at all.
/// A regex matching a backtrace frame that belongs to this test binary.
///
/// The object and offset pair is what addr2line turns back into a location, and
/// glibc's spacing between the offset and the address differs by release
/// (resolute writes ") [0x", noble writes ")[0x"), so the separator is loose. A
/// frame carries a symbol name before the "+" whenever the binary exports its
/// symbols, which CMake does by default, so that half is loose too.
std::string own_binary_frame_pattern() {
  char exe[PATH_MAX] = {};
  const ssize_t len = ::readlink("/proc/self/exe", exe, sizeof(exe) - 1);
  const std::string self = len > 0 ? std::string(exe, static_cast<std::size_t>(len)) : std::string();
  return self + R"(\([^)]*\+0x[0-9a-fA-F]+\) ?\[0x[0-9a-fA-F]+\])";
}

void crash_by_unmapped_write() {
  install_crash_backtrace();
  volatile int * volatile target = reinterpret_cast<volatile int *>(0x1000);
  *target = 1;
}

}  // namespace

// The claim under test is not "a handler is installed" but "a process that dies
// on a fatal signal leaves frames behind". Only killing a process proves it,
// which is what a death test does: the body runs in a forked child and the
// assertion matches that child's stderr.
//
// Every case asserts in both build configurations rather than standing down
// under one of them. In a sanitizer build the promise is the opposite one - the
// sanitizer owns the fatal signals and this handler must stay out of its way -
// so the absence of our marker is the thing worth pinning there. Replacing a
// sanitizer's report with a plainer stack is the way this file could do harm,
// and a test that went quiet under sanitizers would be blind to exactly that.
TEST(CrashBacktrace, SegvIsReported) {
  if constexpr (kHandlerActive) {
    ASSERT_DEATH(crash_by_unmapped_write(), "MEDKIT-CRASH signal=SIGSEGV");
  } else {
    ASSERT_DEATH(crash_by_unmapped_write(), ::testing::AllOf(::testing::HasSubstr("Sanitizer"),
                                                             ::testing::Not(::testing::HasSubstr("MEDKIT-CRASH"))));
  }
}

TEST(CrashBacktrace, SegvReportsResolvableFrames) {
  if constexpr (kHandlerActive) {
    // The marker alone would be satisfied by an empty stack. What makes a
    // report useful is a frame carrying an object and an offset, because that
    // pair is what addr2line turns back into a location. Asserting on a symbol
    // NAME would pin the wrong thing: a release build without -rdynamic reports
    // offsets for this binary's own frames, and the frames worth reading here
    // belong to libraries below us anyway.
    //
    // Two things are matched loosely on purpose. glibc's spacing between the
    // offset and the address differs by release (resolute writes ") [0x",
    // noble writes ")[0x"), and a frame carries a symbol name before the "+"
    // whenever the binary exports its symbols - a link flag away, and not what
    // this test is about.
    // Anchored on THIS binary's own path, read at runtime rather than written
    // down: a bare offset pattern is satisfied by any frame, and libc's frames
    // alone would pass it while saying nothing about whether our own frames came
    // back resolvable. The path is not a name pin - it is whatever the test was
    // built as.
    ASSERT_DEATH(crash_by_unmapped_write(), own_binary_frame_pattern());
  } else {
    ASSERT_DEATH(crash_by_unmapped_write(), ::testing::AllOf(::testing::HasSubstr("Sanitizer"),
                                                             ::testing::Not(::testing::HasSubstr("MEDKIT-CRASH"))));
  }
}

// A stack overflow is the commonest silent SIGSEGV, and it is the one a handler
// on the ordinary stack cannot report - it needs stack to run and there is none.
// This case shipped broken until it was measured: the null dereference above
// produced 452 bytes, and an overflow produced zero.
TEST(CrashBacktrace, SegvOnAnOverflowedStackIsStillReported) {
  const auto overflow_the_stack = [] {
    install_crash_backtrace();
    static_cast<void>(recurse_until_the_stack_runs_out(1));
  };
  if constexpr (kHandlerActive) {
    ASSERT_DEATH(overflow_the_stack(), "MEDKIT-CRASH signal=SIGSEGV");
  } else {
    ASSERT_DEATH(overflow_the_stack(), ::testing::AllOf(::testing::HasSubstr("Sanitizer"),
                                                        ::testing::Not(::testing::HasSubstr("MEDKIT-CRASH"))));
  }
}

TEST(CrashBacktrace, AbortIsReportedToo) {
  if constexpr (kHandlerActive) {
    ASSERT_DEATH(
        {
          install_crash_backtrace();
          std::abort();
        },
        "MEDKIT-CRASH signal=SIGABRT");
  } else {
    // Only the absence of our marker here, unlike the SEGV cases: a sanitizer
    // does not take SIGABRT by default (ASan's handle_abort is off), so there is
    // no sanitizer report to require - and none for us to have clobbered.
    ASSERT_DEATH(
        {
          install_crash_backtrace();
          std::abort();
        },
        ::testing::Not(::testing::HasSubstr("MEDKIT-CRASH")));
  }
}

// Exit status is what ctest and launch_testing report, and a handler that
// swallowed the signal would turn a crash into a clean exit and hide it.
TEST(CrashBacktrace, ProcessStillDiesFromTheOriginalSignal) {
  if constexpr (kHandlerActive) {
    EXPECT_EXIT(crash_by_unmapped_write(), ::testing::KilledBySignal(SIGSEGV), "MEDKIT-CRASH end");
  } else {
    // A sanitizer reports first and then exits on its own terms, so the death
    // itself is what stays assertable here.
    EXPECT_DEATH(crash_by_unmapped_write(), ".*");
  }
}

// The crash helper installs the handler itself, so a test that merely calls
// install twice beforehand proves nothing - the third install inside the child
// would carry it. The double install has to happen INSIDE the dying process.
TEST(CrashBacktrace, InstallingTwiceIsHarmless) {
  const auto crash_after_installing_twice = [] {
    install_crash_backtrace();
    install_crash_backtrace();
    volatile int * volatile target = nullptr;
    *target = 1;
  };
  if constexpr (kHandlerActive) {
    EXPECT_EXIT(crash_after_installing_twice(), ::testing::KilledBySignal(SIGSEGV), "MEDKIT-CRASH end");
  } else {
    ASSERT_DEATH(
        crash_after_installing_twice(),
        ::testing::AllOf(::testing::HasSubstr("Sanitizer"), ::testing::Not(::testing::HasSubstr("MEDKIT-CRASH"))));
  }
}
