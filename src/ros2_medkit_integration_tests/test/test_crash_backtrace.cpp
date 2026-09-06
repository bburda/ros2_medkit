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

using ros2_medkit_integration_tests::install_crash_backtrace;

namespace {

// Recursion the optimiser cannot flatten into a loop. Measured across -O0 to
// -O3: without the escaping address GCC turns this into a loop at -O2 and the
// process spins forever instead of overflowing, which made an earlier version of
// the test below hang rather than fail.
volatile char * stack_probe_sink = nullptr;

// The recursion is the point of this function, so the compiler's warning about
// it is noise.
#if defined(__GNUC__) && !defined(__clang__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Winfinite-recursion"
#endif
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
#if defined(__GNUC__) && !defined(__clang__)
#pragma GCC diagnostic pop
#endif

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
/// Anchored on the basename, not on /proc/self/exe: glibc prints argv[0] for the
/// main executable rather than its resolved path, so a symlinked workspace or a
/// plain `./test_crash_backtrace` would not match a realpath and the test would
/// fail on a perfectly good report.
///
/// The object and offset pair is what addr2line turns back into a location.
/// glibc's spacing between the offset and the address differs by release
/// (resolute writes ") [0x", noble writes ")[0x"), and a frame carries a symbol
/// name before the "+" whenever the binary exports its symbols, which CMake does
/// by default - so both of those are matched loosely.
const char * own_binary_frame_pattern() {
  return R"(test_crash_backtrace\([^)]*\+0x[0-9a-fA-F]+\) ?\[0x[0-9a-fA-F]+\])";
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
// These cases cover the configuration this package is built in, and only that
// one. The header also stands down under a sanitizer so it cannot replace a
// sanitizer's own report - but sanitizers are opt-in per package here, through
// include(ROS2MedkitSanitizers), and this package does not opt in. So
// __SANITIZE_ADDRESS__ is never defined for this translation unit, the
// stand-down never engages, and a test written for it would assert against a
// branch that cannot be reached. The stand-down stays in the header for the day
// this package does opt in; it is not claimed to be tested.
TEST(CrashBacktrace, SegvIsReported) {
  ASSERT_DEATH(crash_by_unmapped_write(), "MEDKIT-CRASH signal=SIGSEGV");
}

TEST(CrashBacktrace, SegvReportsResolvableFrames) {
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
  ASSERT_DEATH(overflow_the_stack(), "MEDKIT-CRASH signal=SIGSEGV");
}

TEST(CrashBacktrace, AbortIsReportedToo) {
  ASSERT_DEATH(
      {
        install_crash_backtrace();
        std::abort();
      },
      "MEDKIT-CRASH signal=SIGABRT");
}

// Exit status is what ctest and launch_testing report, and a handler that
// swallowed the signal would turn a crash into a clean exit and hide it.
TEST(CrashBacktrace, ProcessStillDiesFromTheOriginalSignal) {
  EXPECT_EXIT(crash_by_unmapped_write(), ::testing::KilledBySignal(SIGSEGV), "MEDKIT-CRASH end");
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
  EXPECT_EXIT(crash_after_installing_twice(), ::testing::KilledBySignal(SIGSEGV), "MEDKIT-CRASH end");
}
