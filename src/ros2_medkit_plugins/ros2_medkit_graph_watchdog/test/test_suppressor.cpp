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
//
// Pure interface/logic tests: the Suppressor contract and apply_suppressors() over
// hand-built stand-in suppressors. No rclcpp::init() needed anywhere here - every
// DetectorContext used is default-constructed (all pointers null), which the interface
// contract already requires a Suppressor to tolerate (see suppresses()'s own doc).
#include <gtest/gtest.h>

#include <map>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "ros2_medkit_graph_watchdog/suppressor.hpp"

using ros2_medkit_graph_watchdog::apply_suppressors;
using ros2_medkit_graph_watchdog::DetectorContext;
using ros2_medkit_graph_watchdog::Suppressor;

namespace {
/// Votes yes for every key in `veto`, abstains otherwise. `durable_flag` lets a test
/// stand in for either shape the interface describes without needing a real suppressor.
class FixedSuppressor : public Suppressor {
 public:
  explicit FixedSuppressor(std::set<std::string> veto, bool durable_flag = false)
    : veto_(std::move(veto)), durable_flag_(durable_flag) {
  }
  bool suppresses(const std::string & entity_key, const DetectorContext & /*ctx*/) const override {
    return veto_.count(entity_key) > 0;
  }
  bool durable() const override {
    return durable_flag_;
  }

 private:
  std::set<std::string> veto_;
  bool durable_flag_;
};
}  // namespace

TEST(SuppressorInterface, DurableDefaultsFalse) {
  FixedSuppressor s({"/a"});
  EXPECT_FALSE(s.durable());
}

TEST(SuppressorInterface, DurableCanBeOverriddenTrue) {
  FixedSuppressor s({"/a"}, /*durable_flag=*/true);
  EXPECT_TRUE(s.durable());
}

TEST(ApplySuppressors, EmptyChainSuppressesNothing) {
  std::map<std::string, std::string> affected{{"/a", "reason a"}, {"/b", "reason b"}};
  DetectorContext ctx;
  EXPECT_EQ(apply_suppressors(affected, {}, ctx), 0u);
  EXPECT_EQ(affected.size(), 2u);
}

TEST(ApplySuppressors, DropsOnlyTheVetoedKeys) {
  std::map<std::string, std::string> affected{{"/a", "reason a"}, {"/b", "reason b"}, {"/c", "reason c"}};
  FixedSuppressor vetoes_b({"/b"});
  DetectorContext ctx;
  const std::vector<const Suppressor *> chain{&vetoes_b};
  EXPECT_EQ(apply_suppressors(affected, chain, ctx), 1u);
  EXPECT_EQ(affected.count("/a"), 1u);
  EXPECT_EQ(affected.count("/b"), 0u);
  EXPECT_EQ(affected.count("/c"), 1u);
}

TEST(ApplySuppressors, AnySingleSuppressorInTheChainIsEnough) {
  std::map<std::string, std::string> affected{{"/a", "reason a"}, {"/b", "reason b"}};
  FixedSuppressor abstains_on_everything({});
  FixedSuppressor vetoes_b({"/b"});
  DetectorContext ctx;
  const std::vector<const Suppressor *> chain{&abstains_on_everything, &vetoes_b};
  apply_suppressors(affected, chain, ctx);
  EXPECT_EQ(affected.count("/a"), 1u);
  EXPECT_EQ(affected.count("/b"), 0u);
}

TEST(ApplySuppressors, ANullEntryInTheChainIsSkippedNotDereferenced) {
  std::map<std::string, std::string> affected{{"/a", "reason a"}};
  DetectorContext ctx;
  const std::vector<const Suppressor *> chain{nullptr};
  EXPECT_EQ(apply_suppressors(affected, chain, ctx), 0u);
  EXPECT_EQ(affected.size(), 1u);
}

TEST(ApplySuppressors, ReturnsTheDroppedCount) {
  std::map<std::string, std::string> affected{{"/a", "x"}, {"/b", "y"}, {"/c", "z"}};
  FixedSuppressor vetoes_everything({"/a", "/b", "/c"});
  DetectorContext ctx;
  const std::vector<const Suppressor *> chain{&vetoes_everything};
  EXPECT_EQ(apply_suppressors(affected, chain, ctx), 3u);
  EXPECT_TRUE(affected.empty());
}
