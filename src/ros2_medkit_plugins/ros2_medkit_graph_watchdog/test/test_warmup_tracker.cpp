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
#include <gtest/gtest.h>

#include "ros2_medkit_graph_watchdog/warmup_tracker.hpp"

using ros2_medkit_graph_watchdog::WarmupTracker;

TEST(WarmupTracker, UnknownUntilSeen) {
  WarmupTracker w(3);
  EXPECT_FALSE(w.is_known("/a"));
  EXPECT_FALSE(w.is_armed("/a", 10));
}

TEST(WarmupTracker, ArmsAfterNCycles) {
  WarmupTracker w(3);
  w.update({"/a"}, 5);  // first seen at tick 5
  EXPECT_TRUE(w.is_known("/a"));
  EXPECT_FALSE(w.is_armed("/a", 5));  // 0 elapsed
  EXPECT_FALSE(w.is_armed("/a", 7));  // 2 elapsed
  EXPECT_TRUE(w.is_armed("/a", 8));   // 3 elapsed >= 3
}

TEST(WarmupTracker, ZeroCyclesArmsImmediately) {
  WarmupTracker w(0);
  w.update({"/a"}, 5);
  EXPECT_TRUE(w.is_armed("/a", 5));
}

TEST(WarmupTracker, ReappearingEntityRewarms) {
  WarmupTracker w(3);
  w.update({"/a"}, 1);
  EXPECT_TRUE(w.is_armed("/a", 10));   // armed
  w.update({}, 11);                    // /a gone: 11 - last_seen(1) = 10 > grace(2) -> forgotten
  w.update({"/a"}, 12);                // reappears -> fresh first_seen at 12
  EXPECT_FALSE(w.is_armed("/a", 13));  // 1 elapsed since reappearance
  EXPECT_TRUE(w.is_armed("/a", 15));   // 3 elapsed
}

// A transient one-tick discovery gap (DDS churn, not a real restart) must NOT re-warm an
// already-armed entity: gaps recurring faster than the warmup window would otherwise
// suppress it forever. The forget grace absorbs short gaps.
TEST(WarmupTracker, TransientGapWithinGraceDoesNotRewarm) {
  WarmupTracker w(3);  // default forget grace = 2 ticks
  w.update({"/a"}, 1);
  w.update({"/a"}, 2);
  w.update({"/a"}, 3);
  EXPECT_TRUE(w.is_armed("/a", 4));  // armed (>= 3 elapsed since first_seen 1)
  w.update({}, 5);                   // one-tick gap: 5 - last_seen(3) = 2, not > grace(2) -> kept
  w.update({"/a"}, 6);               // back within grace: original first_seen(1) preserved
  EXPECT_TRUE(w.is_armed("/a", 6));  // still armed - not re-warmed
}

// A gap longer than the forget grace is a real restart and DOES re-warm.
TEST(WarmupTracker, GapBeyondGraceRewarms) {
  WarmupTracker w(3);  // default forget grace = 2 ticks
  w.update({"/a"}, 1);
  EXPECT_TRUE(w.is_armed("/a", 5));
  w.update({}, 6);                    // 6 - last_seen(1) = 5 > grace(2) -> forgotten
  w.update({"/a"}, 7);                // reappears -> re-warmed from tick 7
  EXPECT_FALSE(w.is_armed("/a", 9));  // 2 elapsed since reappearance
  EXPECT_TRUE(w.is_armed("/a", 10));  // 3 elapsed
}

// Custom (zero) forget grace: any single missed tick forgets immediately.
TEST(WarmupTracker, ZeroForgetGraceForgetsOnFirstMiss) {
  WarmupTracker w(3, /*forget_grace=*/0);
  w.update({"/a"}, 1);
  w.update({}, 2);  // 2 - last_seen(1) = 1 > grace(0) -> forgotten
  EXPECT_FALSE(w.is_known("/a"));
}
