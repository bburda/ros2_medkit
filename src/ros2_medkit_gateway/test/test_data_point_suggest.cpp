// Copyright 2026 mfaferek93
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

#include <string>
#include <vector>

#include "ros2_medkit_gateway/core/data_point_suggest.hpp"

using ros2_medkit_gateway::closest_data_points;
using ros2_medkit_gateway::edit_distance;
using ros2_medkit_gateway::sanitized_leaf;
using ros2_medkit_gateway::suggest_data_point;

TEST(EditDistanceTest, Basics) {
  EXPECT_EQ(edit_distance("", ""), 0u);
  EXPECT_EQ(edit_distance("abc", "abc"), 0u);
  EXPECT_EQ(edit_distance("abc", ""), 3u);
  EXPECT_EQ(edit_distance("", "abc"), 3u);
  EXPECT_EQ(edit_distance("counter", "countr"), 1u);
  EXPECT_EQ(edit_distance("kitten", "sitting"), 3u);
}

TEST(SanitizedLeafTest, StripsNamespaceAndSanitizes) {
  EXPECT_EQ(sanitized_leaf("MAIN.counter"), "counter");
  EXPECT_EQ(sanitized_leaf("MAIN.Counter"), "counter");
  EXPECT_EQ(sanitized_leaf("GVL.Motor Speed"), "motor_speed");
  EXPECT_EQ(sanitized_leaf("counter"), "counter");
  EXPECT_EQ(sanitized_leaf("ns1.ns2.leaf"), "leaf");
  EXPECT_EQ(sanitized_leaf("/plc/main/counter"), "counter");
  EXPECT_EQ(sanitized_leaf(""), "");
}

TEST(SuggestDataPointTest, DeterministicLeafMappingWinsExactly) {
  const std::vector<std::string> names{"counter", "level", "adsigrp_sym_uploadinfo"};
  EXPECT_EQ(suggest_data_point("MAIN.counter", names), "counter");
  EXPECT_EQ(suggest_data_point("GVL.Level", names), "level");
}

TEST(SuggestDataPointTest, CloseTypoSuggested) {
  const std::vector<std::string> names{"counter", "level"};
  EXPECT_EQ(suggest_data_point("countr", names), "counter");
  EXPECT_EQ(suggest_data_point("MAIN.countr", names), "counter");
}

TEST(SuggestDataPointTest, NothingCloseReturnsEmpty) {
  const std::vector<std::string> names{"counter", "level"};
  EXPECT_EQ(suggest_data_point("hydraulic_pressure", names), "");
  EXPECT_EQ(suggest_data_point("xyz", names), "");
}

TEST(SuggestDataPointTest, EmptyNamesReturnsEmpty) {
  EXPECT_EQ(suggest_data_point("counter", {}), "");
}

TEST(ClosestDataPointsTest, RanksByDistanceThenName) {
  const std::vector<std::string> names{"zeta", "counter", "counters", "level"};
  const auto ranked = closest_data_points("counter", names, 3);
  ASSERT_EQ(ranked.size(), 3u);
  EXPECT_EQ(ranked[0], "counter");
  EXPECT_EQ(ranked[1], "counters");
}

TEST(ClosestDataPointsTest, TruncatesToN) {
  std::vector<std::string> names;
  for (int i = 0; i < 50; ++i) {
    names.push_back("sym_" + std::to_string(i));
  }
  EXPECT_EQ(closest_data_points("sym_1", names, 20).size(), 20u);
  EXPECT_EQ(closest_data_points("sym_1", names, 100).size(), 50u);
}

TEST(ClosestDataPointsTest, LeafOfNamespacedInputDrivesRanking) {
  std::vector<std::string> names;
  for (int i = 10; i < 40; ++i) {
    names.push_back("adsigrp_sym_" + std::to_string(i));
  }
  names.push_back("counter");
  const auto ranked = closest_data_points("MAIN.counter", names, 20);
  ASSERT_FALSE(ranked.empty());
  EXPECT_EQ(ranked[0], "counter") << "sanitized leaf must outrank the alphabetical prefix";
}
