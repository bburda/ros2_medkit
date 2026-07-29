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

#include "ros2_medkit_graph_watchdog/aggregated_fault.hpp"

#include <gtest/gtest.h>

#include <map>
#include <string>

using ros2_medkit_graph_watchdog::AggregatedFault;

namespace {

std::map<std::string, std::string> affected_entries(int count, std::size_t detail_chars) {
  std::map<std::string, std::string> affected;
  for (int i = 0; i < count; ++i) {
    // Zero-padded so map order is the insertion order, keeping the test deterministic.
    const std::string key = "entity_" + std::string(4 - std::to_string(i).size(), '0') + std::to_string(i);
    affected.emplace(key, std::string(detail_chars, 'x'));
  }
  return affected;
}

}  // namespace

TEST(AggregatedFaultDescribe, JoinsEntriesWithSemicolons) {
  const auto desc = AggregatedFault::describe({{"a", "first"}, {"b", "second"}});
  EXPECT_EQ(desc, "first; second");
}

TEST(AggregatedFaultDescribe, FallsBackToTheKeyWhenTheDetailIsEmpty) {
  const auto desc = AggregatedFault::describe({{"/scan", ""}, {"/tf", "tf detail"}});
  EXPECT_EQ(desc, "/scan; tf detail");
}

TEST(AggregatedFaultDescribe, ShortDescriptionIsNotTruncated) {
  const auto desc = AggregatedFault::describe(affected_entries(3, 10));
  EXPECT_LE(desc.size(), AggregatedFault::kMaxDescriptionChars);
  EXPECT_EQ(desc.find("..."), std::string::npos);
}

// One aggregated fault names EVERY affected entity, so a large graph would otherwise
// produce an unbounded ReportFault description that travels into every /faults payload.
TEST(AggregatedFaultDescribe, ManyEntitiesAreCappedWithAnEllipsis) {
  const auto desc = AggregatedFault::describe(affected_entries(500, 40));
  EXPECT_EQ(desc.size(), AggregatedFault::kMaxDescriptionChars);  // cap INCLUDES the marker
  EXPECT_EQ(desc.substr(desc.size() - 3), "...");
}

// A single oversized entry must be capped too - the truncation cannot rely on there
// being a later entry to trip the check.
TEST(AggregatedFaultDescribe, ASingleOversizedEntryIsCapped) {
  const auto desc = AggregatedFault::describe(affected_entries(1, AggregatedFault::kMaxDescriptionChars * 2));
  EXPECT_EQ(desc.size(), AggregatedFault::kMaxDescriptionChars);
  EXPECT_EQ(desc.substr(desc.size() - 3), "...");
}

// Exactly at the cap must NOT gain an ellipsis: truncating a description that already
// fits would silently drop real content.
TEST(AggregatedFaultDescribe, ExactlyAtTheCapIsLeftAlone) {
  const auto desc = AggregatedFault::describe(affected_entries(1, AggregatedFault::kMaxDescriptionChars));
  EXPECT_EQ(desc.size(), AggregatedFault::kMaxDescriptionChars);
  EXPECT_EQ(desc.find("..."), std::string::npos);
}
