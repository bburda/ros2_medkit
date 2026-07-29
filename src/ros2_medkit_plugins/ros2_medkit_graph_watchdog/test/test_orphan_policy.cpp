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
#include "ros2_medkit_graph_watchdog/orphan_policy.hpp"

#include <gtest/gtest.h>

namespace {
using ros2_medkit_graph_watchdog::find_orphans;
using ros2_medkit_graph_watchdog::TopicEndpointCounts;

TEST(OrphanPolicy, TypoPairReportedOnceNamingBothSides) {
  std::vector<TopicEndpointCounts> topics = {
      {"/scann", "sensor_msgs/msg/LaserScan", 1, 0},  // typo: pub-only
      {"/scan", "sensor_msgs/msg/LaserScan", 0, 1},   // intended target: sub-only
  };
  auto out = find_orphans(topics, 1);
  ASSERT_EQ(out.size(), 1u);  // ONE finding, not two
  EXPECT_EQ(out[0].publisher_topic, "/scann");
  EXPECT_EQ(out[0].subscriber_topic, "/scan");
  EXPECT_NE(out[0].reason.find("/scann"), std::string::npos);
  EXPECT_NE(out[0].reason.find("/scan"), std::string::npos);  // names both, suggests neither
}
TEST(OrphanPolicy, LoneDebugTopicWithNoCounterpartIsNotOrphan) {
  std::vector<TopicEndpointCounts> topics = {{"/debug_markers", "visualization_msgs/msg/Marker", 1, 0}};
  EXPECT_TRUE(find_orphans(topics, 1).empty());
}
TEST(OrphanPolicy, NearMissButDifferentTypeIsNotOrphan) {
  std::vector<TopicEndpointCounts> topics = {
      {"/scann", "sensor_msgs/msg/LaserScan", 1, 0},
      {"/scan", "sensor_msgs/msg/PointCloud2", 0, 1},
  };
  EXPECT_TRUE(find_orphans(topics, 1).empty());
}
TEST(OrphanPolicy, FullyConnectedTopicIsNotOrphan) {
  std::vector<TopicEndpointCounts> topics = {{"/scan", "sensor_msgs/msg/LaserScan", 1, 1}};
  EXPECT_TRUE(find_orphans(topics, 1).empty());
}
TEST(OrphanPolicy, NumericSuffixWithinDistance2ButNotDefaultDistance) {
  // /scan vs /scan_1 is Levenshtein 2 (two trailing inserts); at the detector default
  // (max_edit_distance=1) this legitimate sibling topic must NOT be flagged.
  std::vector<TopicEndpointCounts> topics = {
      {"/scan", "sensor_msgs/msg/LaserScan", 1, 0},
      {"/scan_1", "sensor_msgs/msg/LaserScan", 0, 1},
  };
  EXPECT_TRUE(find_orphans(topics, 1).empty());
  EXPECT_FALSE(find_orphans(topics, 2).empty());  // only fires if an operator opts into distance 2
}
TEST(OrphanPolicy, PerRobotTopicsInDifferentNamespacesAreNotOrphans) {
  // /robot1/scan vs /robot2/scan is distance 1 but a fleet layout, not a typo.
  std::vector<TopicEndpointCounts> topics = {
      {"/robot1/scan", "sensor_msgs/msg/LaserScan", 1, 0},
      {"/robot2/scan", "sensor_msgs/msg/LaserScan", 0, 1},
  };
  EXPECT_TRUE(find_orphans(topics, 1).empty());
}

// The case above is now rejected by the numeric-field rule as well, so on its own it no
// longer proves the same-namespace guard does anything. A fleet is not always numbered:
// these two namespaces differ by a letter, and the pair must still be rejected because the
// difference sits in the namespace, not in the leaf.
TEST(OrphanPolicy, ALetterDifferenceInTheNamespaceIsStillNotAnOrphan) {
  std::vector<TopicEndpointCounts> topics = {
      {"/left/scan", "sensor_msgs/msg/LaserScan", 1, 0},
      {"/lift/scan", "sensor_msgs/msg/LaserScan", 0, 1},
  };
  EXPECT_TRUE(find_orphans(topics, 1).empty());
}
TEST(OrphanPolicy, OneSidedNextToHealthyConnectedSiblingIsNotOrphan) {
  // /lidar_1 is pub-only; /lidar_2 is fully connected and HEALTHY (same type, distance 1,
  // same namespace). /lidar_2 is not waiting for /lidar_1's data, so this is a normal
  // multi-sensor layout, not a typo - the counterpart must be strictly one-sided to pair.
  std::vector<TopicEndpointCounts> topics = {
      {"/lidar_1", "sensor_msgs/msg/LaserScan", 1, 0},
      {"/lidar_2", "sensor_msgs/msg/LaserScan", 1, 1},
  };
  EXPECT_TRUE(find_orphans(topics, 1).empty());
}
TEST(OrphanPolicy, SystemTopicsAreSkippedButOnlyByExactName) {
  // /rosout and /parameter_events are absolute and global: exactly two names, never a prefix
  // class. A user topic that merely starts with the same text must stay under detection - a
  // prefix match would silently exempt it and hide a real typo.
  std::vector<TopicEndpointCounts> system_pair = {
      {"/parameter_eventt", "rcl_interfaces/msg/ParameterEvent", 1, 0},
      {"/parameter_events", "rcl_interfaces/msg/ParameterEvent", 0, 1},
  };
  EXPECT_TRUE(find_orphans(system_pair, 1).empty());  // the real system topic is skipped

  std::vector<TopicEndpointCounts> user_pair = {
      {"/parameter_events_debugg", "rcl_interfaces/msg/ParameterEvent", 1, 0},
      {"/parameter_events_debug", "rcl_interfaces/msg/ParameterEvent", 0, 1},
  };
  auto out = find_orphans(user_pair, 1);
  ASSERT_EQ(out.size(), 1u) << "a user topic starting with /parameter_events must still be checked";
  EXPECT_EQ(out[0].publisher_topic, "/parameter_events_debugg");
  EXPECT_EQ(out[0].subscriber_topic, "/parameter_events_debug");
}
// Two independently one-sided members of a sensor enumeration is an ordinary state on a
// multi-sensor robot: nothing is recording /lidar_1 while /lidar_2's driver is down. Naming
// them as a possible typo, at ERROR, is exactly the false positive this detector must not
// produce. The same-namespace guard does not catch these, because it only helps when the
// differing character sits in a namespace segment; in all of these it sits in the leaf.
TEST(OrphanPolicy, ANumericFieldDifferenceIsAnEnumerationNotATypo) {
  struct Case {
    const char * pub;
    const char * sub;
  };
  const Case cases[] = {
      {"/lidar_1", "/lidar_2"},
      {"/image1", "/image2"},
      {"/cameras/image1", "/cameras/image2"},
      {"/camera1/image", "/camera2/image"},
  };
  for (const auto & c : cases) {
    std::vector<TopicEndpointCounts> topics = {
        {c.pub, "sensor_msgs/msg/Image", 1, 0},
        {c.sub, "sensor_msgs/msg/Image", 0, 1},
    };
    EXPECT_TRUE(find_orphans(topics, 1).empty()) << c.pub << " + " << c.sub << " is an enumeration, not a typo";
  }
}

// The cost of the rule above, stated as a test so it cannot be forgotten: a typo that IS a
// digit is no longer reported. Accepted deliberately - a false ERROR on every multi-sensor
// robot is worse than this miss.
TEST(OrphanPolicy, ADigitTypoIsKnowinglyNotReported) {
  std::vector<TopicEndpointCounts> topics = {
      {"/lidar_1", "sensor_msgs/msg/LaserScan", 1, 0},
      {"/lidar_2", "sensor_msgs/msg/LaserScan", 0, 1},
  };
  EXPECT_TRUE(find_orphans(topics, 1).empty());
}

// A letter difference is still a typo and must keep firing, otherwise the rule above has
// swallowed the whole detector.
TEST(OrphanPolicy, LetterDifferenceIsStillReported) {
  std::vector<TopicEndpointCounts> topics = {
      {"/scan_raw", "sensor_msgs/msg/LaserScan", 1, 0},
      {"/scan_row", "sensor_msgs/msg/LaserScan", 0, 1},
  };
  auto out = find_orphans(topics, 1);
  ASSERT_EQ(out.size(), 1u);
  EXPECT_EQ(out[0].publisher_topic, "/scan_raw");
}

// A name that differs by adding or removing characters is not a digit substitution, even when
// the changed characters are digits, so it must stay under detection.
// The tenth sensor of a family is two edits from the ninth, so this pair only becomes a
// candidate at max_edit_distance 2 - and there the numeric-field rule is the only thing left
// that spares it. Comparing character positions instead of collapsed names would report it.
TEST(OrphanPolicy, ANumericFieldDifferenceIsSparedWhenTheIndexChangesLength) {
  std::vector<TopicEndpointCounts> topics = {
      {"/lidar_9", "sensor_msgs/msg/LaserScan", 1, 0},
      {"/lidar_10", "sensor_msgs/msg/LaserScan", 0, 1},
  };
  EXPECT_TRUE(find_orphans(topics, 2).empty());
}

// An appended digit is a different name, not another member of a family: "/scan1"
// collapses to "/scan#", which is not "/scan".
TEST(OrphanPolicy, AnAppendedDigitIsStillReported) {
  std::vector<TopicEndpointCounts> topics = {
      {"/scan1", "sensor_msgs/msg/LaserScan", 1, 0},
      {"/scan", "sensor_msgs/msg/LaserScan", 0, 1},
  };
  EXPECT_EQ(find_orphans(topics, 1).size(), 1u);
}
}  // namespace

// A misspelled namespace is the one remap mistake the exact-namespace guard hides, so the
// budget that reveals it needs its own coverage on both sides of the default.
TEST(OrphanPolicy, AMisspelledNamespaceIsInvisibleAtTheDefaultBudget) {
  std::vector<TopicEndpointCounts> topics = {
      {"/robott/scan", "sensor_msgs/msg/LaserScan", 1, 0},
      {"/robot/scan", "sensor_msgs/msg/LaserScan", 0, 1},
  };
  EXPECT_TRUE(find_orphans(topics, 1).empty());
}

TEST(OrphanPolicy, AMisspelledNamespaceIsReportedOnceTheNamespaceBudgetAllowsIt) {
  std::vector<TopicEndpointCounts> topics = {
      {"/robott/scan", "sensor_msgs/msg/LaserScan", 1, 0},
      {"/robot/scan", "sensor_msgs/msg/LaserScan", 0, 1},
  };
  const auto found = find_orphans(topics, 1, 1);
  ASSERT_EQ(found.size(), 1u);
  EXPECT_EQ(found[0].publisher_topic, "/robott/scan");
  EXPECT_EQ(found[0].subscriber_topic, "/robot/scan");
}

// A transposition costs two edits, so the namespace budget is a real bound and not a
// yes/no switch: the same pair stays hidden at 1 and appears at 2.
TEST(OrphanPolicy, TheNamespaceBudgetIsABoundNotASwitch) {
  std::vector<TopicEndpointCounts> topics = {
      {"/robto/scan", "sensor_msgs/msg/LaserScan", 1, 0},
      {"/robot/scan", "sensor_msgs/msg/LaserScan", 0, 1},
  };
  EXPECT_TRUE(find_orphans(topics, 1, 1).empty());
  EXPECT_EQ(find_orphans(topics, 1, 2).size(), 1u);
}

// The digit guard outranks the namespace budget: a numbered fleet must stay quiet even for
// an operator who opted into namespace matching.
TEST(OrphanPolicy, ANumberedFleetIsStillSparedUnderANamespaceBudget) {
  std::vector<TopicEndpointCounts> topics = {
      {"/robot1/scan", "sensor_msgs/msg/LaserScan", 1, 0},
      {"/robot2/scan", "sensor_msgs/msg/LaserScan", 0, 1},
  };
  EXPECT_TRUE(find_orphans(topics, 1, 1).empty());
}

// The two budgets are independent: spending the namespace one does not buy extra leaf edits.
TEST(OrphanPolicy, TheLeafBudgetIsNotWidenedByTheNamespaceBudget) {
  std::vector<TopicEndpointCounts> topics = {
      {"/robott/scan", "sensor_msgs/msg/LaserScan", 1, 0},
      {"/robot/odometry", "sensor_msgs/msg/LaserScan", 0, 1},
  };
  EXPECT_TRUE(find_orphans(topics, 1, 1).empty());
}
