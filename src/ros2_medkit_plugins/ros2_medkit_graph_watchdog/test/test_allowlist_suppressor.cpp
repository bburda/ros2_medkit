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
// Pure logic: no rclcpp::init() needed - AllowlistSuppressor::suppresses() ignores ctx
// entirely, so a default-constructed DetectorContext (every pointer null) is enough.
#include <gtest/gtest.h>

#include <set>
#include <string>

#include "ros2_medkit_graph_watchdog/allowlist_suppressor.hpp"

using ros2_medkit_graph_watchdog::AllowlistSuppressor;
using ros2_medkit_graph_watchdog::DetectorContext;

TEST(AllowlistSuppressor, ExactMatchSuppresses) {
  AllowlistSuppressor s({"/a", "/b"});
  DetectorContext ctx;
  EXPECT_TRUE(s.suppresses("/a", ctx));
  EXPECT_TRUE(s.suppresses("/b", ctx));
}

TEST(AllowlistSuppressor, NonMemberAbstains) {
  AllowlistSuppressor s({"/a"});
  DetectorContext ctx;
  EXPECT_FALSE(s.suppresses("/c", ctx));
}

TEST(AllowlistSuppressor, PrefixOfAnAllowedEntryIsNotAMatch) {
  AllowlistSuppressor s({"/r1/x"});
  DetectorContext ctx;
  EXPECT_FALSE(s.suppresses("/r1/x/child", ctx));
  EXPECT_FALSE(s.suppresses("/r1", ctx));
}

TEST(AllowlistSuppressor, ASharedSuffixAcrossDifferentNamespacesIsNotAMatch) {
  // /r1/x on the list must never reach /r2/x - the whole reason this is an exact-match
  // set rather than a leaf/substring comparison.
  AllowlistSuppressor s({"/r1/x"});
  DetectorContext ctx;
  EXPECT_FALSE(s.suppresses("/r2/x", ctx));
}

TEST(AllowlistSuppressor, EmptyAllowSetSuppressesNothing) {
  AllowlistSuppressor s({});
  DetectorContext ctx;
  EXPECT_FALSE(s.suppresses("", ctx));
  EXPECT_FALSE(s.suppresses("/a", ctx));
}

TEST(AllowlistSuppressor, IsDurable) {
  AllowlistSuppressor s({"/a"});
  EXPECT_TRUE(s.durable());
}

// The three matching forms this class deliberately mirrors from
// lifecycle_expectation_detector.cpp's own require_active matching (id / effective_fqn() /
// bare leaf) - see the class doc. suppresses() itself only ever sees one string (the entity
// key) and so only ever proves the two forms derivable from that string alone; the id form
// is proven separately below through allows(), the method a caller holding a captured
// App::id uses instead - see node_death_detector.cpp's tick().

TEST(AllowlistSuppressor, FullFqnFormSuppressesViaExactMatch) {
  // The form the ORIGINAL exact-match tests above already exercise, named here so all three
  // forms appear together as one group.
  AllowlistSuppressor s({"/powertrain/engine/calibration"});
  DetectorContext ctx;
  EXPECT_TRUE(s.suppresses("/powertrain/engine/calibration", ctx));
}

TEST(AllowlistSuppressor, BareLeafFormSuppressesANamespacedKey) {
  // The bug this fix closes: an operator writes the bare node name they see in the graph,
  // not its full namespaced fqn.
  AllowlistSuppressor s({"calibration"});
  DetectorContext ctx;
  EXPECT_TRUE(s.suppresses("/powertrain/engine/calibration", ctx));
}

TEST(AllowlistSuppressor, BareLeafFormMatchesEveryNamespaceOfThatName) {
  // Deliberately fleet-wide, mirroring lifecycle_expectation's own require_active: a bare
  // name matches every namespace's node of that name. An operator who wants to pin one
  // namespace uses a full fqn entry instead (see FullFqnFormSuppressesViaExactMatch).
  AllowlistSuppressor s({"calibration"});
  DetectorContext ctx;
  EXPECT_TRUE(s.suppresses("/coll_a/calibration", ctx));
  EXPECT_TRUE(s.suppresses("/coll_b/calibration", ctx));
}

TEST(AllowlistSuppressor, KeyWithNoSlashIsAbstainedWhenNotListed) {
  // Exercises the slash == npos branch: entity_key is already its own bare leaf, so there is
  // no second form left to try once the exact-match check above has already missed.
  AllowlistSuppressor s({"other"});
  DetectorContext ctx;
  EXPECT_FALSE(s.suppresses("calibration", ctx));
}

TEST(AllowlistSuppressor, IdFormMatchesThroughAllowsDirectly) {
  // The id form: a caller (node_death_detector.cpp) that has captured App::id while an
  // entity was still present checks it against allows() directly, bypassing suppresses()
  // entirely - see the class doc for why suppresses() alone cannot answer this for a dead
  // key. The id used here is what a bare-name collision produces
  // ('<namespace-without-slashes>_<name>'), proven end to end (including the sibling that
  // must NOT match) in test_node_death_integration.cpp's own collision test.
  AllowlistSuppressor s({"coll_a_calibration"});
  EXPECT_TRUE(s.allows("coll_a_calibration"));
}

TEST(AllowlistSuppressor, AllowsIsAnExactCheckWithNoLeafDerivation) {
  // allows() is the raw building block suppresses() itself is built from - it does not also
  // derive a bare leaf the way suppresses() does, so a candidate that would only match via
  // leaf-derivation must not match here.
  AllowlistSuppressor s({"calibration"});
  EXPECT_FALSE(s.allows("/powertrain/engine/calibration"));
}
