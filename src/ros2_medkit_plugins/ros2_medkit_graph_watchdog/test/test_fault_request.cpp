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

#include "ros2_medkit_graph_watchdog/fault_request.hpp"

using ros2_medkit_graph_watchdog::make_fault_clear;
using ros2_medkit_graph_watchdog::make_fault_report;
using Req = ros2_medkit_msgs::srv::ReportFault::Request;

TEST(FaultRequest, ReportIsFailedEventWithAllFields) {
  const auto req = make_fault_report("/nav/planner", "GRAPH_QOS_MISMATCH", 2, "qos mismatch on /scan");
  EXPECT_EQ(req.event_type, Req::EVENT_FAILED);
  EXPECT_EQ(req.fault_code, "GRAPH_QOS_MISMATCH");
  EXPECT_EQ(req.severity, 2);
  EXPECT_EQ(req.description, "qos mismatch on /scan");
  EXPECT_EQ(req.source_id, "/nav/planner");
}

TEST(FaultRequest, ClearIsPassedEventWithSourceId) {
  const auto req = make_fault_clear("/nav/planner", "GRAPH_QOS_MISMATCH");
  EXPECT_EQ(req.event_type, Req::EVENT_PASSED);
  EXPECT_EQ(req.fault_code, "GRAPH_QOS_MISMATCH");
  EXPECT_FALSE(req.source_id.empty());  // fault_manager rejects empty source_id
  EXPECT_EQ(req.source_id, "/nav/planner");
}
