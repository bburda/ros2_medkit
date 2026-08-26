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

#include <string>

#include "ros2_medkit_gateway/core/http/operation_item_id.hpp"

namespace ros2_medkit_gateway {
namespace {

ServiceInfo service(const std::string & name, const std::string & full_path) {
  ServiceInfo svc;
  svc.name = name;
  svc.full_path = full_path;
  return svc;
}

ActionInfo action(const std::string & name, const std::string & full_path) {
  ActionInfo act;
  act.name = name;
  act.full_path = full_path;
  return act;
}

// The ordinary case: distinct names keep their short name and each names one
// operation, so every id is servable.
TEST(OperationItemIdsTest, DistinctNamesKeepTheirShortNameAndNameOneOperationEach) {
  AggregatedOperations ops;
  ops.services.push_back(service("calibrate", "/engine/calibrate"));
  ops.actions.push_back(action("long_calibration", "/engine/long_calibration"));

  const auto ids = http::operation_item_ids(ops);
  ASSERT_EQ(ids.ids.size(), 2u);
  EXPECT_EQ(ids.ids[0], "calibrate");
  EXPECT_EQ(ids.ids[1], "long_calibration");
  EXPECT_FALSE(ids.ambiguous("calibrate"));
  EXPECT_FALSE(ids.ambiguous("long_calibration"));
}

// A service and an action may legally share a name AND a ROS path: an action
// puts its own services under `<path>/_action/...`, so `<path>` itself is free
// for a service. Neither half of the scheme separates them, so the id they
// collapse to names two operations and nothing can be served under it - which
// is what a producer has to be able to see before it publishes that id.
TEST(OperationItemIdsTest, AServiceAndAnActionSharingAPathCollapseToAnUnservableId) {
  AggregatedOperations ops;
  ops.services.push_back(service("calibrate", "/engine/calibrate"));
  ops.actions.push_back(action("calibrate", "/engine/calibrate"));

  const auto ids = http::operation_item_ids(ops);
  ASSERT_EQ(ids.ids.size(), 2u);
  EXPECT_EQ(ids.ids[0], ids.ids[1]) << "the two forms of the scheme both yield the same string here";
  EXPECT_TRUE(ids.ambiguous(ids.ids[0]))
      << "an id naming two operations was reported servable, so a producer would publish it";
}

// One provider carrying a short name at two DIFFERENT paths is the case the
// path form exists for: each keeps its own path-derived id, and neither is
// ambiguous.
TEST(OperationItemIdsTest, OneProviderAtTwoPathsAddressesEachByItsPath) {
  AggregatedOperations ops;
  ops.services.push_back(service("calibrate", "/engine/left/calibrate"));
  ops.services.push_back(service("calibrate", "/engine/right/calibrate"));

  const auto ids = http::operation_item_ids(ops);
  ASSERT_EQ(ids.ids.size(), 2u);
  EXPECT_NE(ids.ids[0], ids.ids[1]);
  EXPECT_FALSE(ids.ambiguous(ids.ids[0]));
  EXPECT_FALSE(ids.ambiguous(ids.ids[1]));
}

}  // namespace
}  // namespace ros2_medkit_gateway
