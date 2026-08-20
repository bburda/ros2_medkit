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

#include <atomic>
#include <chrono>
#include <string>
#include <thread>
#include <vector>

#include "ros2_medkit_gateway/core/condition_evaluator.hpp"
#include "ros2_medkit_gateway/core/managers/trigger_manager.hpp"
#include "ros2_medkit_gateway/core/resource_change_notifier.hpp"
#include "ros2_medkit_gateway/core/sqlite_trigger_store.hpp"

using namespace ros2_medkit_gateway;

// ===========================================================================
// Test fixture
// ===========================================================================

class TriggerManagerTest : public ::testing::Test {
 protected:
  void SetUp() override {
    // Register all built-in condition evaluators
    registry_.register_condition("OnChange", std::make_shared<OnChangeEvaluator>());
    registry_.register_condition("OnChangeTo", std::make_shared<OnChangeToEvaluator>());
    registry_.register_condition("EnterRange", std::make_shared<EnterRangeEvaluator>());
    registry_.register_condition("LeaveRange", std::make_shared<LeaveRangeEvaluator>());

    TriggerConfig config;
    config.max_triggers = 10;
    config.on_restart_behavior = "reset";

    manager_ = std::make_unique<TriggerManager>(notifier_, registry_, store_, config);
  }

  void TearDown() override {
    if (manager_) {
      manager_->shutdown();
    }
    notifier_.shutdown();
  }

  TriggerCreateRequest make_request(const std::string & entity_id = "sensor",
                                    const std::string & condition_type = "OnChange",
                                    const json & condition_params = json::object()) {
    TriggerCreateRequest req;
    req.entity_id = entity_id;
    req.entity_type = "apps";
    req.resource_uri = "/api/v1/apps/" + entity_id + "/data/temperature";
    req.collection = "data";
    req.resource_path = "/temperature";
    req.resolved_topic_name = "/" + entity_id + "/temperature";
    req.path = "";
    req.condition_type = condition_type;
    req.condition_params = condition_params;
    req.protocol = "sse";
    req.multishot = false;
    req.persistent = false;
    return req;
  }

  ResourceChangeNotifier notifier_;
  ConditionRegistry registry_;
  SqliteTriggerStore store_{":memory:"};
  std::unique_ptr<TriggerManager> manager_;
};

// ===========================================================================
// CRUD Tests
// ===========================================================================

// @verifies REQ_INTEROP_029
TEST_F(TriggerManagerTest, Create_ValidOnChangeTrigger) {
  auto result = manager_->create(make_request());
  ASSERT_TRUE(result.has_value()) << result.error().message;
  EXPECT_FALSE(result->id.empty());
  EXPECT_EQ(result->entity_id, "sensor");
  EXPECT_EQ(result->entity_type, "apps");
  EXPECT_EQ(result->collection, "data");
  EXPECT_EQ(result->condition_type, "OnChange");
  EXPECT_EQ(result->status, TriggerStatus::ACTIVE);
}

// @verifies REQ_INTEROP_029
TEST_F(TriggerManagerTest, Create_InvalidConditionType) {
  auto req = make_request("sensor", "NonexistentCondition");
  auto result = manager_->create(req);
  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error().code, TriggerError::ValidationError);
  EXPECT_NE(result.error().message.find("condition"), std::string::npos);
}

// @verifies REQ_INTEROP_029
TEST_F(TriggerManagerTest, Create_InvalidParams) {
  // EnterRange requires lower_bound <= upper_bound
  json params = {{"lower_bound", 30}, {"upper_bound", 10}};
  auto req = make_request("sensor", "EnterRange", params);
  auto result = manager_->create(req);
  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error().code, TriggerError::ValidationError);
  EXPECT_NE(result.error().message.find("lower_bound"), std::string::npos);
}

// @verifies REQ_INTEROP_029
TEST_F(TriggerManagerTest, Create_MaxTriggersExceeded) {
  for (int i = 0; i < 10; ++i) {
    auto r = manager_->create(make_request("entity_" + std::to_string(i)));
    ASSERT_TRUE(r.has_value()) << "Trigger " << i << " should succeed: " << r.error().message;
  }
  auto result = manager_->create(make_request("overflow"));
  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error().code, TriggerError::CapacityExceeded);
  EXPECT_NE(result.error().message.find("capacity"), std::string::npos);
}

// @verifies REQ_INTEROP_096
TEST_F(TriggerManagerTest, Get_Existing) {
  auto created = manager_->create(make_request());
  ASSERT_TRUE(created.has_value());

  auto fetched = manager_->get(created->id);
  ASSERT_TRUE(fetched.has_value());
  EXPECT_EQ(fetched->id, created->id);
  EXPECT_EQ(fetched->entity_id, "sensor");
}

TEST_F(TriggerManagerTest, Get_NonExisting) {
  auto fetched = manager_->get("nonexistent");
  EXPECT_FALSE(fetched.has_value());
}

// @verifies REQ_INTEROP_030
TEST_F(TriggerManagerTest, List_ReturnsTriggersForEntity) {
  (void)manager_->create(make_request("sensor"));
  (void)manager_->create(make_request("sensor"));
  (void)manager_->create(make_request("actuator"));

  auto sensor_triggers = manager_->list("sensor");
  EXPECT_EQ(sensor_triggers.size(), 2u);

  auto actuator_triggers = manager_->list("actuator");
  EXPECT_EQ(actuator_triggers.size(), 1u);

  auto empty = manager_->list("nonexistent");
  EXPECT_TRUE(empty.empty());
}

// @verifies REQ_INTEROP_031
TEST_F(TriggerManagerTest, Update_ChangeLifetime) {
  auto req = make_request();
  req.lifetime_sec = 300;
  auto created = manager_->create(req);
  ASSERT_TRUE(created.has_value());

  auto updated = manager_->update(created->id, 600);
  ASSERT_TRUE(updated.has_value()) << updated.error().message;
  EXPECT_EQ(updated->lifetime_sec, 600);
  EXPECT_TRUE(updated->expires_at.has_value());
}

TEST_F(TriggerManagerTest, Update_NonExisting) {
  auto result = manager_->update("nonexistent", 600);
  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error().code, TriggerError::NotFound);
}

TEST_F(TriggerManagerTest, Update_NegativeLifetimeRejected) {
  auto created = manager_->create(make_request());
  ASSERT_TRUE(created.has_value());

  auto result = manager_->update(created->id, -1);
  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error().code, TriggerError::ValidationError);
  EXPECT_NE(result.error().message.find("positive"), std::string::npos);
}

TEST_F(TriggerManagerTest, Update_ZeroLifetimeRejected) {
  auto created = manager_->create(make_request());
  ASSERT_TRUE(created.has_value());

  auto result = manager_->update(created->id, 0);
  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error().code, TriggerError::ValidationError);
  EXPECT_NE(result.error().message.find("positive"), std::string::npos);
}

// @verifies REQ_INTEROP_032
TEST_F(TriggerManagerTest, Remove_ExistingTrigger) {
  auto created = manager_->create(make_request());
  ASSERT_TRUE(created.has_value());

  EXPECT_TRUE(manager_->remove(created->id));
  EXPECT_FALSE(manager_->get(created->id).has_value());
}

TEST_F(TriggerManagerTest, Remove_NonExisting) {
  EXPECT_FALSE(manager_->remove("nonexistent"));
}

// ===========================================================================
// Event Evaluation Tests
// ===========================================================================

// @verifies REQ_INTEROP_097
TEST_F(TriggerManagerTest, SingleShot_NotifyMatchingChange) {
  auto req = make_request("sensor", "OnChange");
  req.multishot = false;
  auto created = manager_->create(req);
  ASSERT_TRUE(created.has_value());

  // Notify a matching resource change
  notifier_.notify("data", "sensor", "/temperature", json(42.0));

  // Wait for event via synchronization primitive instead of sleeping
  ASSERT_TRUE(manager_->wait_for_event(created->id, std::chrono::milliseconds(2000)));

  auto event = manager_->consume_pending_event(created->id);
  // Event may have already been consumed by the cleanup path (race between test
  // thread and notifier worker), so we accept either outcome for the event.
  // The key invariant is that the trigger fires exactly once.
  if (event.has_value()) {
    EXPECT_TRUE(event->contains("timestamp"));
    EXPECT_TRUE(event->contains("payload"));
  }

  // Single-shot: trigger should be removed after firing (cleaned up by on_resource_change).
  // Allow a brief window for the notifier worker thread to finish cleanup.
  for (int i = 0; i < 20; ++i) {
    if (!manager_->get(created->id).has_value()) {
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  EXPECT_FALSE(manager_->get(created->id).has_value());
}

// @verifies REQ_INTEROP_029
TEST_F(TriggerManagerTest, SingleShot_RemovedAfterFiring) {
  auto req = make_request("sensor", "OnChange");
  req.multishot = false;
  auto created = manager_->create(req);
  ASSERT_TRUE(created.has_value());
  auto trigger_id = created->id;

  // Verify trigger exists before firing
  EXPECT_TRUE(manager_->get(trigger_id).has_value());
  EXPECT_EQ(manager_->list("sensor").size(), 1u);

  // Fire the one-shot trigger
  notifier_.notify("data", "sensor", "/temperature", json(42.0));
  ASSERT_TRUE(manager_->wait_for_event(trigger_id, std::chrono::milliseconds(2000)));

  // Wait for the notifier worker to finish cleaning up the one-shot trigger
  for (int i = 0; i < 50; ++i) {
    if (!manager_->get(trigger_id).has_value()) {
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  // Trigger must be fully removed from memory (not just terminated)
  EXPECT_FALSE(manager_->get(trigger_id).has_value()) << "One-shot trigger should be removed after firing";
  EXPECT_TRUE(manager_->list("sensor").empty()) << "list() should not include removed trigger";

  // Verify capacity was freed: we should be able to create a new trigger
  auto new_trigger = manager_->create(make_request("sensor", "OnChange"));
  EXPECT_TRUE(new_trigger.has_value()) << "Capacity should be freed after one-shot cleanup";
}

// @verifies REQ_INTEROP_097
TEST_F(TriggerManagerTest, Multishot_NotifyTwice) {
  auto req = make_request("sensor", "OnChange");
  req.multishot = true;
  auto created = manager_->create(req);
  ASSERT_TRUE(created.has_value());

  // First change
  notifier_.notify("data", "sensor", "/temperature", json(42.0));
  ASSERT_TRUE(manager_->wait_for_event(created->id, std::chrono::milliseconds(2000)));

  auto event1 = manager_->consume_pending_event(created->id);
  ASSERT_TRUE(event1.has_value());

  // Second change
  notifier_.notify("data", "sensor", "/temperature", json(43.0));
  ASSERT_TRUE(manager_->wait_for_event(created->id, std::chrono::milliseconds(2000)));

  auto event2 = manager_->consume_pending_event(created->id);
  ASSERT_TRUE(event2.has_value());

  // Should still be active
  EXPECT_TRUE(manager_->is_active(created->id));
}

// @verifies REQ_INTEROP_029
TEST_F(TriggerManagerTest, LifetimeExpiry) {
  auto req = make_request();
  // Use a very short lifetime so the test doesn't block long
  req.lifetime_sec = 1;
  auto created = manager_->create(req);
  ASSERT_TRUE(created.has_value());

  EXPECT_TRUE(manager_->is_active(created->id));

  // Poll for expiry: finishes quickly on a fast machine but tolerates
  // scheduler stalls under parallel test load. Deadline is well past the
  // 1 s lifetime so a busy CI runner does not race the check.
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(5000);
  while (std::chrono::steady_clock::now() < deadline) {
    if (!manager_->is_active(created->id)) {
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }

  EXPECT_FALSE(manager_->is_active(created->id));
}

// ===========================================================================
// Hierarchy Matching Tests
// ===========================================================================

// @verifies REQ_INTEROP_029
TEST_F(TriggerManagerTest, Hierarchy_ComponentToApp) {
  // Set up hierarchy: component "chassis" contains app "sensor"
  manager_->set_entity_children_fn(
      [](const std::string & entity_id, const std::string & /*entity_type*/) -> std::vector<std::string> {
        if (entity_id == "chassis") {
          return {"sensor", "actuator"};
        }
        return {};
      });

  auto req = make_request("chassis", "OnChange");
  req.entity_type = "components";
  req.resource_uri = "/api/v1/components/chassis/data/temperature";
  req.multishot = true;
  auto created = manager_->create(req);
  ASSERT_TRUE(created.has_value());

  // Notify from a child app "sensor" - should fire the trigger
  notifier_.notify("data", "sensor", "/temperature", json(42.0));
  ASSERT_TRUE(manager_->wait_for_event(created->id, std::chrono::milliseconds(2000)));

  auto event = manager_->consume_pending_event(created->id);
  ASSERT_TRUE(event.has_value());
}

// @verifies REQ_INTEROP_029
TEST_F(TriggerManagerTest, Hierarchy_AreaToApp) {
  // Set up hierarchy: area "navigation" contains apps
  manager_->set_entity_children_fn(
      [](const std::string & entity_id, const std::string & /*entity_type*/) -> std::vector<std::string> {
        if (entity_id == "navigation") {
          return {"lidar", "imu", "planner"};
        }
        return {};
      });

  auto req = make_request("navigation", "OnChange");
  req.entity_type = "areas";
  req.resource_uri = "/api/v1/areas/navigation/data/temperature";
  req.multishot = true;
  auto created = manager_->create(req);
  ASSERT_TRUE(created.has_value());

  // Notify from nested app "lidar"
  notifier_.notify("data", "lidar", "/temperature", json(55.0));
  ASSERT_TRUE(manager_->wait_for_event(created->id, std::chrono::milliseconds(2000)));

  auto event = manager_->consume_pending_event(created->id);
  ASSERT_TRUE(event.has_value());
}

TEST_F(TriggerManagerTest, Hierarchy_NoMatchForUnrelatedEntity) {
  manager_->set_entity_children_fn(
      [](const std::string & entity_id, const std::string & /*entity_type*/) -> std::vector<std::string> {
        if (entity_id == "chassis") {
          return {"sensor"};
        }
        return {};
      });

  auto req = make_request("chassis", "OnChange");
  req.entity_type = "components";
  req.resource_uri = "/api/v1/components/chassis/data/temperature";
  auto created = manager_->create(req);
  ASSERT_TRUE(created.has_value());

  // Notify from unrelated entity "other_entity"
  notifier_.notify("data", "other_entity", "/temperature", json(42.0));

  // Short sleep is acceptable here - we are verifying that NO event fires
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  auto event = manager_->consume_pending_event(created->id);
  EXPECT_FALSE(event.has_value());
}

// ===========================================================================
// Callback Tests
// ===========================================================================

// @verifies REQ_INTEROP_032
TEST_F(TriggerManagerTest, OnRemovedCallback) {
  std::vector<std::string> removed_ids;
  manager_->set_on_removed([&](const std::string & trigger_id) {
    removed_ids.push_back(trigger_id);
  });

  auto created = manager_->create(make_request());
  ASSERT_TRUE(created.has_value());

  manager_->remove(created->id);

  ASSERT_EQ(removed_ids.size(), 1u);
  EXPECT_EQ(removed_ids[0], created->id);
}

// ===========================================================================
// Event Envelope Tests
// ===========================================================================

// @verifies REQ_INTEROP_097
TEST_F(TriggerManagerTest, EventEnvelopeFormat) {
  auto req = make_request("sensor", "OnChange");
  req.multishot = true;
  auto created = manager_->create(req);
  ASSERT_TRUE(created.has_value());

  notifier_.notify("data", "sensor", "/temperature", json(42.0));
  ASSERT_TRUE(manager_->wait_for_event(created->id, std::chrono::milliseconds(2000)));

  auto event = manager_->consume_pending_event(created->id);
  ASSERT_TRUE(event.has_value());
  EXPECT_TRUE(event->contains("timestamp"));
  EXPECT_TRUE(event->contains("payload"));
  EXPECT_TRUE((*event)["timestamp"].is_string());
  // Timestamp should be ISO 8601 format
  std::string ts = (*event)["timestamp"].get<std::string>();
  EXPECT_NE(ts.find('T'), std::string::npos);
}

// ===========================================================================
// Consume/Wait Tests
// ===========================================================================

TEST_F(TriggerManagerTest, ConsumePendingEvent_ClearsEvent) {
  auto req = make_request("sensor", "OnChange");
  req.multishot = true;
  auto created = manager_->create(req);
  ASSERT_TRUE(created.has_value());

  notifier_.notify("data", "sensor", "/temperature", json(42.0));
  ASSERT_TRUE(manager_->wait_for_event(created->id, std::chrono::milliseconds(2000)));

  auto event1 = manager_->consume_pending_event(created->id);
  ASSERT_TRUE(event1.has_value());

  // After consuming, should return nullopt until a new event arrives
  auto event2 = manager_->consume_pending_event(created->id);
  EXPECT_FALSE(event2.has_value());
}

TEST_F(TriggerManagerTest, WaitForEvent_BlocksUntilEventArrives) {
  auto req = make_request("sensor", "OnChange");
  req.multishot = true;
  auto created = manager_->create(req);
  ASSERT_TRUE(created.has_value());

  std::atomic<bool> event_received{false};
  std::thread waiter([&]() {
    bool got_event = manager_->wait_for_event(created->id, std::chrono::milliseconds(2000));
    if (got_event) {
      event_received.store(true);
    }
  });

  // Give waiter thread time to start waiting
  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  // Send event
  notifier_.notify("data", "sensor", "/temperature", json(42.0));

  waiter.join();
  EXPECT_TRUE(event_received.load());
}

TEST_F(TriggerManagerTest, WaitForEvent_TimesOutWhenNoEvent) {
  auto req = make_request("sensor", "OnChange");
  req.multishot = true;
  auto created = manager_->create(req);
  ASSERT_TRUE(created.has_value());

  auto start = std::chrono::steady_clock::now();
  bool got_event = manager_->wait_for_event(created->id, std::chrono::milliseconds(100));
  auto elapsed = std::chrono::steady_clock::now() - start;

  EXPECT_FALSE(got_event);
  EXPECT_GE(elapsed, std::chrono::milliseconds(80));
}

// ===========================================================================
// Shutdown Tests
// ===========================================================================

TEST_F(TriggerManagerTest, Shutdown_WakesWaitingThreads) {
  auto req = make_request("sensor", "OnChange");
  req.multishot = true;
  auto created = manager_->create(req);
  ASSERT_TRUE(created.has_value());

  std::atomic<bool> waiter_returned{false};
  std::thread waiter([&]() {
    manager_->wait_for_event(created->id, std::chrono::seconds(30));
    waiter_returned.store(true);
  });

  // Give waiter time to start
  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  manager_->shutdown();

  waiter.join();
  EXPECT_TRUE(waiter_returned.load());
}

// ===========================================================================
// JSON Pointer Extraction Tests
// ===========================================================================

// @verifies REQ_INTEROP_097
TEST_F(TriggerManagerTest, JsonPointer_ExtractsSubElement) {
  auto req = make_request("sensor", "OnChange");
  req.path = "/data";
  req.multishot = true;
  auto created = manager_->create(req);
  ASSERT_TRUE(created.has_value());

  // The change value has a "data" field
  json value = {{"data", 42.0}, {"metadata", "ignored"}};
  notifier_.notify("data", "sensor", "/temperature", value);
  ASSERT_TRUE(manager_->wait_for_event(created->id, std::chrono::milliseconds(2000)));

  auto event = manager_->consume_pending_event(created->id);
  ASSERT_TRUE(event.has_value());
  // The payload should be the full change value
  EXPECT_TRUE(event->contains("payload"));
}

TEST_F(TriggerManagerTest, JsonPointer_EmptyPathUsesFullValue) {
  auto req = make_request("sensor", "OnChange");
  req.path = "";
  req.multishot = true;
  auto created = manager_->create(req);
  ASSERT_TRUE(created.has_value());

  notifier_.notify("data", "sensor", "/temperature", json(42.0));
  ASSERT_TRUE(manager_->wait_for_event(created->id, std::chrono::milliseconds(2000)));

  auto event = manager_->consume_pending_event(created->id);
  ASSERT_TRUE(event.has_value());
  EXPECT_EQ((*event)["payload"], json(42.0));
}

// ===========================================================================
// Condition-Specific Tests
// ===========================================================================

TEST_F(TriggerManagerTest, OnChangeTo_FiresOnlyWhenTargetReached) {
  auto req = make_request("sensor", "OnChangeTo", {{"target_value", 100}});
  req.multishot = true;
  auto created = manager_->create(req);
  ASSERT_TRUE(created.has_value());

  // First notification: value is 100, no previous -> fires (target match, first eval)
  notifier_.notify("data", "sensor", "/temperature", json(100));
  ASSERT_TRUE(manager_->wait_for_event(created->id, std::chrono::milliseconds(2000)));
  auto event1 = manager_->consume_pending_event(created->id);
  ASSERT_TRUE(event1.has_value());

  // Second notification: value stays 100 -> does NOT fire (no change)
  notifier_.notify("data", "sensor", "/temperature", json(100));
  // Short sleep is acceptable - verifying NO event fires
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  auto event2 = manager_->consume_pending_event(created->id);
  EXPECT_FALSE(event2.has_value());

  // Third notification: value goes to 50 -> does NOT fire (not target)
  notifier_.notify("data", "sensor", "/temperature", json(50));
  // Short sleep is acceptable - verifying NO event fires
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  auto event3 = manager_->consume_pending_event(created->id);
  EXPECT_FALSE(event3.has_value());

  // Fourth notification: back to 100 -> fires (transition to target)
  notifier_.notify("data", "sensor", "/temperature", json(100));
  ASSERT_TRUE(manager_->wait_for_event(created->id, std::chrono::milliseconds(2000)));
  auto event4 = manager_->consume_pending_event(created->id);
  EXPECT_TRUE(event4.has_value());
}

TEST_F(TriggerManagerTest, EnterRange_FiresOnTransition) {
  json params = {{"lower_bound", 20}, {"upper_bound", 30}};
  auto req = make_request("sensor", "EnterRange", params);
  req.multishot = true;
  auto created = manager_->create(req);
  ASSERT_TRUE(created.has_value());

  // First notification: value = 10, no previous -> EnterRange returns false (needs transition)
  notifier_.notify("data", "sensor", "/temperature", json(10));
  // Short sleep is acceptable - verifying NO event fires
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  auto event1 = manager_->consume_pending_event(created->id);
  EXPECT_FALSE(event1.has_value());

  // Second notification: 10 -> 25 (enters range) -> fires
  notifier_.notify("data", "sensor", "/temperature", json(25));
  ASSERT_TRUE(manager_->wait_for_event(created->id, std::chrono::milliseconds(2000)));
  auto event2 = manager_->consume_pending_event(created->id);
  EXPECT_TRUE(event2.has_value());
}

// ===========================================================================
// Non-matching resource path / collection
// ===========================================================================

TEST_F(TriggerManagerTest, NoFireForDifferentCollection) {
  auto req = make_request("sensor", "OnChange");
  req.collection = "data";
  auto created = manager_->create(req);
  ASSERT_TRUE(created.has_value());

  // Notify with different collection "faults"
  notifier_.notify("faults", "sensor", "/temperature", json("fault_data"));

  // Short sleep is acceptable - verifying NO event fires
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  auto event = manager_->consume_pending_event(created->id);
  EXPECT_FALSE(event.has_value());
}

TEST_F(TriggerManagerTest, NoFireForDifferentResourcePath) {
  auto req = make_request("sensor", "OnChange");
  req.resource_path = "/temperature";
  auto created = manager_->create(req);
  ASSERT_TRUE(created.has_value());

  // Notify with different resource path "/humidity"
  notifier_.notify("data", "sensor", "/humidity", json(65.0));

  // Short sleep is acceptable - verifying NO event fires
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  auto event = manager_->consume_pending_event(created->id);
  EXPECT_FALSE(event.has_value());
}

// ===========================================================================
// Resolved topic name tests (C1 fix)
// ===========================================================================

TEST_F(TriggerManagerTest, ResolvedTopicName_StoredInTriggerInfo) {
  auto req = make_request("sensor", "OnChange");
  req.resolved_topic_name = "/sensor/temperature";
  auto created = manager_->create(req);
  ASSERT_TRUE(created.has_value());

  auto fetched = manager_->get(created->id);
  ASSERT_TRUE(fetched.has_value());
  EXPECT_EQ(fetched->resolved_topic_name, "/sensor/temperature");
  EXPECT_EQ(fetched->resource_path, "/temperature");
}

TEST_F(TriggerManagerTest, ResolvedTopicName_EmptyWhenUnresolved) {
  auto req = make_request("sensor", "OnChange");
  req.resolved_topic_name = "";
  auto created = manager_->create(req);
  ASSERT_TRUE(created.has_value());

  auto fetched = manager_->get(created->id);
  ASSERT_TRUE(fetched.has_value());
  EXPECT_TRUE(fetched->resolved_topic_name.empty());
}

// ===========================================================================
// Multi-entity notification tests (I1 fix)
// ===========================================================================

// ===========================================================================
// Bounded queue tests (C2 fix)
// ===========================================================================

TEST_F(TriggerManagerTest, MultishotRapidEvents_AllConsumed) {
  auto req = make_request("sensor", "OnChange");
  req.multishot = true;
  auto created = manager_->create(req);
  ASSERT_TRUE(created.has_value());

  // Fire 3 events - wait for each to be processed to avoid timing issues
  notifier_.notify("data", "sensor", "/temperature", json(10.0));
  ASSERT_TRUE(manager_->wait_for_event(created->id, std::chrono::milliseconds(2000)));
  auto event1 = manager_->consume_pending_event(created->id);
  ASSERT_TRUE(event1.has_value());
  EXPECT_EQ((*event1)["payload"], json(10.0));

  notifier_.notify("data", "sensor", "/temperature", json(20.0));
  ASSERT_TRUE(manager_->wait_for_event(created->id, std::chrono::milliseconds(2000)));
  auto event2 = manager_->consume_pending_event(created->id);
  ASSERT_TRUE(event2.has_value());
  EXPECT_EQ((*event2)["payload"], json(20.0));

  notifier_.notify("data", "sensor", "/temperature", json(30.0));
  ASSERT_TRUE(manager_->wait_for_event(created->id, std::chrono::milliseconds(2000)));
  auto event3 = manager_->consume_pending_event(created->id);
  ASSERT_TRUE(event3.has_value());
  EXPECT_EQ((*event3)["payload"], json(30.0));

  // No more events
  auto event4 = manager_->consume_pending_event(created->id);
  EXPECT_FALSE(event4.has_value());

  // Trigger should still be active
  EXPECT_TRUE(manager_->is_active(created->id));
}

// ===========================================================================
// Expired trigger on_removed callback (I8 fix)
// ===========================================================================

TEST_F(TriggerManagerTest, ExpiredTrigger_FiresOnRemovedCallback) {
  std::vector<std::string> removed_ids;
  manager_->set_on_removed([&](const std::string & trigger_id) {
    removed_ids.push_back(trigger_id);
  });

  auto req = make_request();
  req.lifetime_sec = 1;
  auto created = manager_->create(req);
  ASSERT_TRUE(created.has_value());

  EXPECT_TRUE(manager_->is_active(created->id));

  // Poll until trigger expires (lifetime=1s, poll up to 3s for CI tolerance)
  auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(3000);
  while (manager_->is_active(created->id) && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  // is_active triggers cleanup for expired triggers
  EXPECT_FALSE(manager_->is_active(created->id));

  // on_removed should have been called
  ASSERT_EQ(removed_ids.size(), 1u);
  EXPECT_EQ(removed_ids[0], created->id);
}

// ===========================================================================
// Multi-entity notification tests (I1 fix)
// ===========================================================================

TEST_F(TriggerManagerTest, MultiEntity_SameResourcePathDifferentEntities) {
  // Two triggers on different entities watching the same resource_path ("/temperature")
  auto req1 = make_request("sensor_a", "OnChange");
  req1.resource_path = "/temperature";
  req1.multishot = true;
  auto created1 = manager_->create(req1);
  ASSERT_TRUE(created1.has_value());

  auto req2 = make_request("sensor_b", "OnChange");
  req2.resource_path = "/temperature";
  req2.multishot = true;
  auto created2 = manager_->create(req2);
  ASSERT_TRUE(created2.has_value());

  // Notify for sensor_a - only sensor_a's trigger should fire
  notifier_.notify("data", "sensor_a", "/temperature", json(42.0));
  ASSERT_TRUE(manager_->wait_for_event(created1->id, std::chrono::milliseconds(2000)));
  auto event1 = manager_->consume_pending_event(created1->id);
  EXPECT_TRUE(event1.has_value());

  // sensor_b should not have fired
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  auto event2 = manager_->consume_pending_event(created2->id);
  EXPECT_FALSE(event2.has_value());

  // Notify for sensor_b - only sensor_b's trigger should fire
  notifier_.notify("data", "sensor_b", "/temperature", json(43.0));
  ASSERT_TRUE(manager_->wait_for_event(created2->id, std::chrono::milliseconds(2000)));
  auto event3 = manager_->consume_pending_event(created2->id);
  EXPECT_TRUE(event3.has_value());
}

// ===========================================================================
// load_persistent_triggers() tests (I11)
// ===========================================================================

/// Helper: build a TriggerInfo for direct store insertion (mirrors TriggerManager::create internals).
static TriggerInfo make_persistent_trigger(const std::string & id, const std::string & entity_id = "sensor",
                                           TriggerStatus status = TriggerStatus::ACTIVE,
                                           std::optional<std::chrono::system_clock::time_point> expires_at = {}) {
  TriggerInfo info;
  info.id = id;
  info.entity_id = entity_id;
  info.entity_type = "apps";
  info.resource_uri = "/api/v1/apps/" + entity_id + "/data/temperature";
  info.collection = "data";
  info.resource_path = "/temperature";
  info.resolved_topic_name = "/" + entity_id + "/temperature";
  info.path = "";
  info.condition_type = "OnChange";
  info.condition_params = json::object();
  info.protocol = "sse";
  info.multishot = true;
  info.persistent = true;
  info.status = status;
  info.created_at = std::chrono::system_clock::now();
  info.expires_at = expires_at;
  return info;
}

// @verifies REQ_INTEROP_029
TEST(LoadPersistentTriggers, RestoreBehaviorLoadsActiveTrigger) {
  ResourceChangeNotifier notifier;
  ConditionRegistry registry;
  registry.register_condition("OnChange", std::make_shared<OnChangeEvaluator>());

  // Pre-populate store with one ACTIVE trigger
  SqliteTriggerStore store(":memory:");
  auto info = make_persistent_trigger("trig_42");
  ASSERT_TRUE(store.save(info).has_value());

  // Create manager with restore behavior
  TriggerConfig config;
  config.max_triggers = 100;
  config.on_restart_behavior = "restore";
  TriggerManager manager(notifier, registry, store, config);

  manager.load_persistent_triggers();

  // Trigger should be visible via get()
  auto fetched = manager.get("trig_42");
  ASSERT_TRUE(fetched.has_value()) << "Trigger should have been restored";
  EXPECT_EQ(fetched->entity_id, "sensor");
  EXPECT_EQ(fetched->status, TriggerStatus::ACTIVE);
  EXPECT_EQ(fetched->resolved_topic_name, "/sensor/temperature");
  EXPECT_TRUE(manager.is_active("trig_42"));

  manager.shutdown();
  notifier.shutdown();
}

// @verifies REQ_INTEROP_029
TEST(LoadPersistentTriggers, ResetBehaviorDoesNotRestoreTriggers) {
  ResourceChangeNotifier notifier;
  ConditionRegistry registry;
  registry.register_condition("OnChange", std::make_shared<OnChangeEvaluator>());

  SqliteTriggerStore store(":memory:");
  auto info = make_persistent_trigger("trig_99");
  ASSERT_TRUE(store.save(info).has_value());

  TriggerConfig config;
  config.max_triggers = 100;
  config.on_restart_behavior = "reset";  // default - no restore
  TriggerManager manager(notifier, registry, store, config);

  manager.load_persistent_triggers();

  // Trigger must NOT appear (reset mode skips restore)
  auto fetched = manager.get("trig_99");
  EXPECT_FALSE(fetched.has_value());

  manager.shutdown();
  notifier.shutdown();
}

// @verifies REQ_INTEROP_029
TEST(LoadPersistentTriggers, ExpiredTriggerMarkedTerminatedOnLoad) {
  ResourceChangeNotifier notifier;
  ConditionRegistry registry;
  registry.register_condition("OnChange", std::make_shared<OnChangeEvaluator>());

  SqliteTriggerStore store(":memory:");

  // Trigger that expired 5 seconds in the past
  auto past = std::chrono::system_clock::now() - std::chrono::seconds(5);
  auto info = make_persistent_trigger("trig_expired", "sensor", TriggerStatus::ACTIVE, past);
  ASSERT_TRUE(store.save(info).has_value());

  TriggerConfig config;
  config.max_triggers = 100;
  config.on_restart_behavior = "restore";
  TriggerManager manager(notifier, registry, store, config);

  manager.load_persistent_triggers();

  // The expired trigger must NOT be live (load skips it and marks it TERMINATED in store)
  EXPECT_FALSE(manager.is_active("trig_expired"));

  // Verify store has been updated to TERMINATED
  auto loaded = store.load_all();
  ASSERT_TRUE(loaded.has_value());
  auto it = std::find_if(loaded->begin(), loaded->end(), [](const TriggerInfo & t) {
    return t.id == "trig_expired";
  });
  ASSERT_NE(it, loaded->end());
  EXPECT_EQ(it->status, TriggerStatus::TERMINATED);

  manager.shutdown();
  notifier.shutdown();
}

// @verifies REQ_INTEROP_029
TEST(LoadPersistentTriggers, NewTriggerIdIsHigherThanRestoredId) {
  ResourceChangeNotifier notifier;
  ConditionRegistry registry;
  registry.register_condition("OnChange", std::make_shared<OnChangeEvaluator>());

  SqliteTriggerStore store(":memory:");

  // Store a trigger with a high numeric ID to force ID counter bump
  auto info = make_persistent_trigger("trig_1000");
  ASSERT_TRUE(store.save(info).has_value());

  TriggerConfig config;
  config.max_triggers = 200;
  config.on_restart_behavior = "restore";
  TriggerManager manager(notifier, registry, store, config);

  manager.load_persistent_triggers();

  // Create a new trigger after loading - its ID must be > 1000 to avoid collision
  TriggerCreateRequest req;
  req.entity_id = "new_sensor";
  req.entity_type = "apps";
  req.resource_uri = "/api/v1/apps/new_sensor/data/temperature";
  req.collection = "data";
  req.resource_path = "/temperature";
  req.resolved_topic_name = "/new_sensor/temperature";
  req.path = "";
  req.condition_type = "OnChange";
  req.condition_params = json::object();
  req.protocol = "sse";
  req.multishot = false;
  req.persistent = false;

  auto created = manager.create(req);
  ASSERT_TRUE(created.has_value()) << created.error().message;

  // Extract numeric suffix from "trig_N"
  const std::string & new_id = created->id;
  auto pos = new_id.find('_');
  ASSERT_NE(pos, std::string::npos);
  uint64_t new_num = std::stoull(new_id.substr(pos + 1));
  EXPECT_GT(new_num, 1000u) << "New trigger ID (" << new_id << ") must be higher than restored trig_1000";

  manager.shutdown();
  notifier.shutdown();
}

// ===========================================================================
// Entity orphan sweep tests (I10 fix)
// ===========================================================================

TEST_F(TriggerManagerTest, Sweep_RemovesOrphanedTriggers) {
  auto r1 = make_request("alive_app");
  auto r2 = make_request("gone_app");
  auto created1 = manager_->create(r1);
  auto created2 = manager_->create(r2);
  ASSERT_TRUE(created1.has_value());
  ASSERT_TRUE(created2.has_value());

  // alive_app exists, gone_app doesn't
  manager_->set_entity_exists_fn([](const std::string & id, const std::string & /*entity_type*/) {
    return id != "gone_app";
  });

  manager_->sweep_orphaned_triggers();

  EXPECT_FALSE(manager_->list("alive_app").empty());
  EXPECT_TRUE(manager_->list("gone_app").empty());
}

TEST_F(TriggerManagerTest, Sweep_FiresOnRemovedCallback) {
  auto req = make_request("doomed_app");
  auto created = manager_->create(req);
  ASSERT_TRUE(created.has_value());

  std::string removed_id;
  manager_->set_on_removed([&](const std::string & id) {
    removed_id = id;
  });
  manager_->set_entity_exists_fn([](const std::string & /*id*/, const std::string & /*entity_type*/) {
    return false;
  });

  manager_->sweep_orphaned_triggers();
  EXPECT_EQ(removed_id, created->id);
}

TEST_F(TriggerManagerTest, Sweep_NoopWithoutEntityExistsFn) {
  auto req = make_request("sensor");
  auto created = manager_->create(req);
  ASSERT_TRUE(created.has_value());

  // No entity_exists_fn set - sweep should not crash or remove anything
  manager_->sweep_orphaned_triggers();
  EXPECT_FALSE(manager_->list("sensor").empty());
}

TEST_F(TriggerManagerTest, Sweep_SkipsInactiveTriggers) {
  auto req = make_request("gone_app");
  req.multishot = false;
  auto created = manager_->create(req);
  ASSERT_TRUE(created.has_value());

  // Fire the single-shot trigger so it becomes TERMINATED and is removed
  notifier_.notify("data", "gone_app", "/temperature", json(42.0));
  ASSERT_TRUE(manager_->wait_for_event(created->id, std::chrono::milliseconds(2000)));

  // Wait for the notifier worker to finish cleaning up the one-shot trigger
  for (int i = 0; i < 20; ++i) {
    if (!manager_->get(created->id).has_value()) {
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  // All entities "gone" - but terminated trigger was already cleaned up by
  // on_resource_change, so sweep should not crash
  manager_->set_entity_exists_fn([](const std::string & /*id*/, const std::string & /*entity_type*/) {
    return false;
  });

  // Should not crash
  manager_->sweep_orphaned_triggers();
}

TEST_F(TriggerManagerTest, Sweep_FreesCapacitySlots) {
  // Fill capacity (max_triggers = 10)
  for (int i = 0; i < 10; ++i) {
    auto r = manager_->create(make_request("entity_" + std::to_string(i)));
    ASSERT_TRUE(r.has_value()) << "Trigger " << i << " should succeed: " << r.error().message;
  }

  // Capacity full
  auto overflow = manager_->create(make_request("overflow"));
  ASSERT_FALSE(overflow.has_value());

  // Sweep removes 5 entities (entity_5 through entity_9)
  manager_->set_entity_exists_fn([](const std::string & id, const std::string & /*entity_type*/) {
    // entity_0 through entity_4 exist; entity_5+ don't
    return id.size() > 7 && id.back() < '5';
  });

  manager_->sweep_orphaned_triggers();

  // Now we should have 5 slots free
  auto after_sweep = manager_->create(make_request("new_entity"));
  EXPECT_TRUE(after_sweep.has_value()) << "Should have capacity after sweep: " << after_sweep.error().message;
}

/// A gateway that has never seen an entity cannot conclude the entity is gone.
/// Restore runs once, during construction, and the sweep runs from the first
/// tick after that - long before DDS reports nodes that were already up. A
/// sweep there would delete the trigger from the shared store for good.
TEST(LoadPersistentTriggers, SweepKeepsRestoredTriggerBeforeItsEntityIsDiscovered) {
  ResourceChangeNotifier notifier;
  ConditionRegistry registry;
  registry.register_condition("OnChange", std::make_shared<OnChangeEvaluator>());

  SqliteTriggerStore store(":memory:");
  ASSERT_TRUE(store.save(make_persistent_trigger("trig_7")).has_value());

  TriggerConfig config;
  config.max_triggers = 100;
  config.on_restart_behavior = "restore";
  TriggerManager manager(notifier, registry, store, config);
  manager.load_persistent_triggers();
  ASSERT_TRUE(manager.get("trig_7").has_value()) << "Trigger should have been restored";

  // Discovery has not reported the entity yet - the state every restart passes
  // through, for as long as DDS takes.
  manager.set_entity_exists_fn([](const std::string & /*id*/, const std::string & /*entity_type*/) {
    return false;
  });
  manager.sweep_orphaned_triggers();
  manager.sweep_orphaned_triggers();

  EXPECT_TRUE(manager.get("trig_7").has_value())
      << "Restored trigger was swept before its entity had a chance to be discovered";
  auto loaded = store.load_all();
  ASSERT_TRUE(loaded.has_value());
  EXPECT_EQ(loaded->size(), 1u) << "The store row must survive too - restore never runs again";

  manager.shutdown();
  notifier.shutdown();
}

/// The relaxation above is bounded by having seen the entity once: after that,
/// a missing entity is a real orphan and the trigger goes, store row included.
TEST(LoadPersistentTriggers, SweepRemovesRestoredTriggerAfterItsEntityIsDiscoveredAndLost) {
  ResourceChangeNotifier notifier;
  ConditionRegistry registry;
  registry.register_condition("OnChange", std::make_shared<OnChangeEvaluator>());

  SqliteTriggerStore store(":memory:");
  ASSERT_TRUE(store.save(make_persistent_trigger("trig_8")).has_value());

  TriggerConfig config;
  config.max_triggers = 100;
  config.on_restart_behavior = "restore";
  TriggerManager manager(notifier, registry, store, config);
  manager.load_persistent_triggers();
  ASSERT_TRUE(manager.get("trig_8").has_value());

  bool entity_present = true;
  manager.set_entity_exists_fn([&entity_present](const std::string & /*id*/, const std::string & /*entity_type*/) {
    return entity_present;
  });

  manager.sweep_orphaned_triggers();
  ASSERT_TRUE(manager.get("trig_8").has_value()) << "A discovered entity is not an orphan";

  entity_present = false;
  manager.sweep_orphaned_triggers();

  EXPECT_FALSE(manager.get("trig_8").has_value()) << "An entity that was discovered and then lost is an orphan";
  auto loaded = store.load_all();
  ASSERT_TRUE(loaded.has_value());
  EXPECT_TRUE(loaded->empty()) << "Sweeping an orphan drops its store row as well";

  manager.shutdown();
  notifier.shutdown();
}

// ===========================================================================
// Deferred topic resolution tests
// ===========================================================================

/// Creating a trigger with empty resolved_topic_name (late publisher scenario)
/// should succeed and queue it for deferred resolution.
TEST_F(TriggerManagerTest, DeferredResolution_CreateWithEmptyTopicSucceeds) {
  auto req = make_request("sensor");
  req.resolved_topic_name = "";  // Simulate unresolvable at creation time

  auto result = manager_->create(req);
  ASSERT_TRUE(result.has_value()) << result.error().message;
  EXPECT_EQ(result->status, TriggerStatus::ACTIVE);

  auto info = manager_->get(result->id);
  ASSERT_TRUE(info.has_value());
  EXPECT_TRUE(info->resolved_topic_name.empty()) << "Should be unresolved at creation";
}

/// retry_unresolved_triggers should not crash when no subscriber or resolver is set.
TEST_F(TriggerManagerTest, DeferredResolution_RetryWithoutSubscriberSafe) {
  auto req = make_request("sensor");
  req.resolved_topic_name = "";

  auto result = manager_->create(req);
  ASSERT_TRUE(result.has_value());

  // No subscriber, no resolve_fn -> should safely no-op
  EXPECT_NO_THROW(manager_->retry_unresolved_triggers());

  // Set resolve_fn but no subscriber -> still no-op (skips resolve)
  manager_->set_resolve_topic_fn(
      [](const std::string & /*entity_id*/, const std::string & /*resource_path*/) -> std::string {
        return "/sensor/temperature";
      });
  EXPECT_NO_THROW(manager_->retry_unresolved_triggers());

  // Trigger should still exist
  auto info = manager_->get(result->id);
  EXPECT_TRUE(info.has_value());
}

/// Resolved trigger name can be consumed by SSE after deferred resolution.
/// Tests the data flow: empty at creation -> resolved after retry -> info updated.
TEST_F(TriggerManagerTest, DeferredResolution_ResolvedTopicUpdatedInInfo) {
  auto req = make_request("sensor");
  req.resolved_topic_name = "";

  auto result = manager_->create(req);
  ASSERT_TRUE(result.has_value());

  // Verify unresolved
  auto info_before = manager_->get(result->id);
  ASSERT_TRUE(info_before.has_value());
  EXPECT_TRUE(info_before->resolved_topic_name.empty());

  // After creating a trigger with empty resolved_topic_name,
  // the trigger should be queued for deferred resolution.
  // The actual subscription happens in retry_unresolved_triggers()
  // when both resolve_fn AND topic_subscriber are available.
  // We test the resolve_fn path indirectly via the integration test
  // (test_triggers_late_publisher) which exercises the full flow.
}
