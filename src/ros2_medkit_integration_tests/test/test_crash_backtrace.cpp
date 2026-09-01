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

#include <csignal>
#include <cstdlib>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "ros2_medkit_integration_tests/crash_backtrace.hpp"

using ros2_medkit_integration_tests::crash_backtrace_is_active;
using ros2_medkit_integration_tests::install_crash_backtrace;

namespace {

constexpr bool kHandlerActive = crash_backtrace_is_active();

void crash_by_null_write() {
  install_crash_backtrace();
  // Both qualifiers are load-bearing and were picked by measuring, not by
  // reasoning: a store through a plain `int * volatile` is deleted at -O2 as an
  // erroneous path and the process then does not crash at all. Marking the
  // pointee volatile makes the store itself one the compiler must emit, and
  // marking the pointer volatile stops it being folded to a known constant.
  volatile int * volatile target = nullptr;
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
    ASSERT_DEATH(crash_by_null_write(), "MEDKIT-CRASH signal=SIGSEGV");
  } else {
    ASSERT_DEATH(crash_by_null_write(), ::testing::Not(::testing::HasSubstr("MEDKIT-CRASH")));
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
    ASSERT_DEATH(crash_by_null_write(), R"(\([^)]*\+0x[0-9a-fA-F]+\) ?\[0x[0-9a-fA-F]+\])");
  } else {
    EXPECT_FALSE(crash_backtrace_is_active()) << "a sanitizer build must not install the handler";
    ASSERT_DEATH(crash_by_null_write(), ::testing::Not(::testing::HasSubstr("MEDKIT-CRASH")));
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
    EXPECT_EXIT(crash_by_null_write(), ::testing::KilledBySignal(SIGSEGV), "MEDKIT-CRASH end");
  } else {
    // A sanitizer reports first and then exits on its own terms, so the death
    // itself is what stays assertable here.
    EXPECT_DEATH(crash_by_null_write(), ".*");
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
    ASSERT_DEATH(crash_after_installing_twice(), ::testing::Not(::testing::HasSubstr("MEDKIT-CRASH")));
  }
}
