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

// TriggerManager routing test: links exclusively against gateway_core +
// GTest. No rclcpp/ament dependencies on the link line, proving that the
// manager body lives in the middleware-neutral build layer. The
// rclcpp::GenericSubscription / type lookup / pending retry concerns belong
// to the Ros2TopicSubscriptionTransport adapter; here we exercise the
// manager against a mock transport that records subscribe/unsubscribe
// calls and lets the test emit JSON samples directly.

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

#include "ros2_medkit_gateway/core/condition_evaluator.hpp"
#include "ros2_medkit_gateway/core/managers/trigger_manager.hpp"
#include "ros2_medkit_gateway/core/resource_change_notifier.hpp"
#include "ros2_medkit_gateway/core/sqlite_trigger_store.hpp"
#include "ros2_medkit_gateway/core/transports/topic_subscription_transport.hpp"

namespace ros2_medkit_gateway {
namespace {

// ----------------------------------------------------------------------------
// Mock transport
// ----------------------------------------------------------------------------

/// Mock TopicSubscriptionTransport recording every subscribe/unsubscribe so
/// the test can assert what the manager routes through it. Each subscribe()
/// returns a handle whose dtor flips a per-key "alive" flag, mirroring the
/// real adapter's contract.
class MockTopicSubscriptionTransport : public TopicSubscriptionTransport {
 public:
  struct Subscription {
    std::string topic_path;
    std::string msg_type;
    SampleCallback callback;
    std::shared_ptr<std::atomic<bool>> alive = std::make_shared<std::atomic<bool>>(true);
  };

  class Handle : public TopicSubscriptionHandle {
   public:
    Handle(MockTopicSubscriptionTransport * owner, std::string key) : owner_(owner), key_(std::move(key)) {
    }

    ~Handle() override {
      owner_->release(key_);
    }

    Handle(const Handle &) = delete;
    Handle & operator=(const Handle &) = delete;
    Handle(Handle &&) = delete;
    Handle & operator=(Handle &&) = delete;

    const std::string & key() const {
      return key_;
    }

   private:
    MockTopicSubscriptionTransport * owner_;
    std::string key_;
  };

  std::unique_ptr<TopicSubscriptionHandle> subscribe(const std::string & topic_path, const std::string & msg_type,
                                                     SampleCallback callback) override {
    std::lock_guard<std::mutex> lock(mutex_);
    subscribe_log_.emplace_back(std::string{}, topic_path);
    if (fail_next_subscribe_) {
      fail_next_subscribe_ = false;
      // Mirror the real adapter's contract: a null handle signals failure.
      return nullptr;
    }
    auto key = "mock_" + std::to_string(next_id_++);
    Subscription entry;
    entry.topic_path = topic_path;
    entry.msg_type = msg_type;
    entry.callback = std::move(callback);
    subs_[key] = std::move(entry);
    subscribe_log_.back().first = key;
    return std::make_unique<Handle>(this, key);
  }

  /// Test helper: force the next subscribe() call to return nullptr.
  /// Resets after one use, mirroring rclcpp's "this particular create failed".
  void fail_next_subscribe() {
    std::lock_guard<std::mutex> lock(mutex_);
    fail_next_subscribe_ = true;
  }

  /// Test helper: emit a sample as if rclcpp had just delivered one.
  /// Returns true when the key is still alive (callback was invoked).
  bool emit(const std::string & key, const nlohmann::json & sample) {
    SampleCallback cb;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      auto it = subs_.find(key);
      if (it == subs_.end() || !it->second.alive->load()) {
        return false;
      }
      cb = it->second.callback;
    }
    if (!cb) {
      return false;
    }
    cb(sample);
    return true;
  }

  /// Test helper: returns the topic path most recently registered.
  std::string last_subscribed_topic() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return subscribe_log_.empty() ? std::string() : subscribe_log_.back().second;
  }

  /// Test helper: count of currently-alive subscriptions.
  size_t alive_count() const {
    std::lock_guard<std::mutex> lock(mutex_);
    size_t n = 0;
    for (const auto & [_, sub] : subs_) {
      if (sub.alive->load()) {
        ++n;
      }
    }
    return n;
  }

  /// Test helper: total number of subscribe() calls observed.
  size_t total_subscribes() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return subscribe_log_.size();
  }

  /// Test helper: list of keys still alive.
  std::vector<std::string> alive_keys() const {
    std::lock_guard<std::mutex> lock(mutex_);
    std::vector<std::string> out;
    for (const auto & [key, sub] : subs_) {
      if (sub.alive->load()) {
        out.push_back(key);
      }
    }
    return out;
  }

 private:
  void release(const std::string & key) {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = subs_.find(key);
    if (it == subs_.end()) {
      return;
    }
    it->second.alive->store(false);
    it->second.callback = nullptr;
  }

  mutable std::mutex mutex_;
  std::unordered_map<std::string, Subscription> subs_;
  std::vector<std::pair<std::string, std::string>> subscribe_log_;  // (key, topic)
  uint64_t next_id_ = 1;
  bool fail_next_subscribe_ = false;
};

// ----------------------------------------------------------------------------
// Fixture
// ----------------------------------------------------------------------------

class TriggerManagerRoutingTest : public ::testing::Test {
 protected:
  void SetUp() override {
    registry_.register_condition("OnChange", std::make_shared<OnChangeEvaluator>());

    transport_ = std::make_shared<MockTopicSubscriptionTransport>();

    TriggerConfig config;
    config.max_triggers = 50;
    config.on_restart_behavior = "reset";
    manager_ = std::make_unique<TriggerManager>(notifier_, registry_, store_, config, transport_);
  }

  void TearDown() override {
    if (manager_) {
      manager_->shutdown();
    }
    notifier_.shutdown();
  }

  TriggerCreateRequest make_data_request(const std::string & entity_id = "sensor",
                                         const std::string & resolved_topic = "/sensor/temperature",
                                         const std::string & resource_path = "/temperature") {
    TriggerCreateRequest req;
    req.entity_id = entity_id;
    req.entity_type = "apps";
    req.resource_uri = "/api/v1/apps/" + entity_id + "/data" + resource_path;
    req.collection = "data";
    req.resource_path = resource_path;
    req.resolved_topic_name = resolved_topic;
    req.path = "";
    req.condition_type = "OnChange";
    req.condition_params = nlohmann::json::object();
    req.protocol = "sse";
    req.multishot = true;
    req.persistent = false;
    return req;
  }

  ResourceChangeNotifier notifier_;
  ConditionRegistry registry_;
  SqliteTriggerStore store_{":memory:"};
  std::shared_ptr<MockTopicSubscriptionTransport> transport_;
  std::unique_ptr<TriggerManager> manager_;
};

// ----------------------------------------------------------------------------
// Routing tests (~6 cases)
// ----------------------------------------------------------------------------

TEST_F(TriggerManagerRoutingTest, RegisteredTriggerSubscribesToTransport) {
  auto created = manager_->create(make_data_request("sensor", "/sensor/temperature"));
  ASSERT_TRUE(created.has_value()) << created.error().message;

  EXPECT_EQ(transport_->total_subscribes(), 1u);
  EXPECT_EQ(transport_->alive_count(), 1u);
  EXPECT_EQ(transport_->last_subscribed_topic(), "/sensor/temperature");
}

TEST_F(TriggerManagerRoutingTest, RemovedTriggerDestructsHandle) {
  auto created = manager_->create(make_data_request());
  ASSERT_TRUE(created.has_value());
  ASSERT_EQ(transport_->alive_count(), 1u);

  ASSERT_TRUE(manager_->remove(created->id));
  EXPECT_EQ(transport_->alive_count(), 0u)
      << "Removing a data trigger must drop its TopicSubscriptionHandle, which must unsubscribe";
}

TEST_F(TriggerManagerRoutingTest, SampleCallbackRoutesToConditionEvaluator) {
  auto created = manager_->create(make_data_request("sensor", "/sensor/temperature", "/temperature"));
  ASSERT_TRUE(created.has_value());

  auto keys = transport_->alive_keys();
  ASSERT_EQ(keys.size(), 1u);

  // Route a sample through the mock transport - this must reach the
  // resource-change notifier and fire the OnChange evaluator.
  ASSERT_TRUE(transport_->emit(keys[0], nlohmann::json(42.0)));
  ASSERT_TRUE(manager_->wait_for_event(created->id, std::chrono::milliseconds(2000)));

  auto event = manager_->consume_pending_event(created->id);
  ASSERT_TRUE(event.has_value());
  EXPECT_TRUE(event->contains("payload"));
  EXPECT_EQ((*event)["payload"], nlohmann::json(42.0));
}

TEST_F(TriggerManagerRoutingTest, OrphanedTriggerSweepUnsubscribes) {
  auto alive = manager_->create(make_data_request("alive_app", "/alive_app/temperature"));
  auto gone = manager_->create(make_data_request("gone_app", "/gone_app/temperature"));
  ASSERT_TRUE(alive.has_value());
  ASSERT_TRUE(gone.has_value());
  ASSERT_EQ(transport_->alive_count(), 2u);

  manager_->set_entity_exists_fn([](const std::string & id, const std::string & /*type*/) {
    return id != "gone_app";
  });
  manager_->sweep_orphaned_triggers();

  EXPECT_EQ(transport_->alive_count(), 1u) << "Orphan sweep must unsubscribe gone_app's transport handle";
  EXPECT_TRUE(manager_->list("gone_app").empty());
  EXPECT_EQ(manager_->list("alive_app").size(), 1u);
}

TEST_F(TriggerManagerRoutingTest, MultipleTriggersIndependentSubscriptions) {
  auto t1 = manager_->create(make_data_request("sensor_a", "/sensor_a/temperature", "/temperature"));
  auto t2 = manager_->create(make_data_request("sensor_b", "/sensor_b/temperature", "/temperature"));
  ASSERT_TRUE(t1.has_value());
  ASSERT_TRUE(t2.has_value());

  // Two distinct subscribe calls, two alive handles.
  EXPECT_EQ(transport_->total_subscribes(), 2u);
  EXPECT_EQ(transport_->alive_count(), 2u);

  // Removing only one drops only that handle; the other survives.
  ASSERT_TRUE(manager_->remove(t1->id));
  EXPECT_EQ(transport_->alive_count(), 1u);
}

TEST_F(TriggerManagerRoutingTest, ShutdownClearsAllSubscriptions) {
  ASSERT_TRUE(manager_->create(make_data_request("a", "/a/x", "/x")).has_value());
  ASSERT_TRUE(manager_->create(make_data_request("b", "/b/x", "/x")).has_value());
  ASSERT_TRUE(manager_->create(make_data_request("c", "/c/x", "/x")).has_value());
  ASSERT_EQ(transport_->alive_count(), 3u);

  manager_->shutdown();
  EXPECT_EQ(transport_->alive_count(), 0u) << "Manager shutdown must drop every transport handle";
}

TEST_F(TriggerManagerRoutingTest, CreateReturnsErrorWhenSubscribeReturnsNullHandle) {
  // Force the next subscribe() call to fail with a null handle, mirroring
  // what rclcpp does when GenericSubscription cannot be created. The manager
  // must surface this as a TriggerError::SubscribeFailed - never throw.
  transport_->fail_next_subscribe();

  auto req = make_data_request("sensor_a", "/sensor_a/temperature", "/temperature");
  auto result = manager_->create(req);

  ASSERT_FALSE(result.has_value()) << "create() must not throw and must return an error variant";
  EXPECT_EQ(result.error().code, TriggerError::SubscribeFailed);
  EXPECT_NE(result.error().message.find("Failed to subscribe"), std::string::npos);

  // Rollback contract: the trigger must not be visible to subsequent
  // queries, and no transport handle should be left alive.
  EXPECT_TRUE(manager_->list(req.entity_id).empty());
  EXPECT_EQ(transport_->alive_count(), 0u);
}

TEST_F(TriggerManagerRoutingTest, RestoreQueuesPersistentTriggerOnSubscribeFailure) {
  // Pre-populate the SQLite store with a persistent data trigger so that the
  // fresh manager's load_persistent_triggers() call has something to restore.
  TriggerInfo persisted;
  persisted.id = "trig_restore_fail_1";
  persisted.entity_id = "sensor_a";
  persisted.entity_type = "apps";
  persisted.resource_uri = "/api/v1/apps/sensor_a/data/temperature";
  persisted.collection = "data";
  persisted.resource_path = "/temperature";
  persisted.resolved_topic_name = "/sensor_a/temperature";
  persisted.path = "";
  persisted.condition_type = "OnChange";
  persisted.condition_params = nlohmann::json::object();
  persisted.protocol = "sse";
  persisted.multishot = true;
  persisted.persistent = true;
  persisted.status = TriggerStatus::ACTIVE;
  persisted.created_at = std::chrono::system_clock::now();
  ASSERT_TRUE(store_.save(persisted).has_value());

  // Force the next subscribe() call (made by load_persistent_triggers() for
  // the restored data trigger) to return a null handle, simulating the
  // "topic disappeared between shutdown and restart" race that the production
  // code path guards against by queuing the trigger onto
  // unresolved_data_triggers_ for retry.
  transport_->fail_next_subscribe();

  // Build a fresh manager configured for restore. The default fixture manager
  // uses on_restart_behavior=reset, which makes load_persistent_triggers() a
  // no-op - we need restore semantics to exercise the failure path.
  TriggerConfig restore_config;
  restore_config.max_triggers = 50;
  restore_config.on_restart_behavior = "restore";
  auto restored_mgr = std::make_unique<TriggerManager>(notifier_, registry_, store_, restore_config, transport_);
  restored_mgr->load_persistent_triggers();

  // The trigger is visible in the manager despite the subscribe failure -
  // load_persistent_triggers() must not silently drop persistent triggers.
  auto triggers = restored_mgr->list("sensor_a");
  ASSERT_EQ(triggers.size(), 1u);
  EXPECT_EQ(triggers[0].id, "trig_restore_fail_1");

  // The first failed subscribe is recorded. No alive handle yet.
  const auto subscribes_after_fail = transport_->total_subscribes();
  EXPECT_GE(subscribes_after_fail, 1u);
  EXPECT_EQ(transport_->alive_count(), 0u);

  // Now exercise the retry path. retry_unresolved_triggers() requires both a
  // resolve_topic_fn_ and a non-empty resolved topic; provide a resolver that
  // returns the persisted topic name so the manager can re-subscribe.
  restored_mgr->set_resolve_topic_fn([](const std::string & entity_id, const std::string & resource_path) {
    return "/" + entity_id + resource_path;
  });
  restored_mgr->retry_unresolved_triggers();

  // The retry must have called subscribe() again. This time it succeeds and
  // an alive handle backs the restored trigger.
  EXPECT_GT(transport_->total_subscribes(), subscribes_after_fail);
  EXPECT_EQ(transport_->alive_count(), 1u);

  restored_mgr->shutdown();
}

TEST_F(TriggerManagerRoutingTest, UnresolvedTriggerExpiryWarnsAndStopsRetrying) {
  // #584: an unresolved data trigger used to be dropped from the retry list
  // after the timeout with no trace, leaving a permanently ACTIVE trigger
  // that can never fire. The expiry must be loud.
  std::vector<std::string> warnings;
  manager_->set_warn_log_fn([&warnings](const std::string & m) {
    warnings.push_back(m);
  });
  manager_->set_resolve_topic_fn([](const std::string &, const std::string &) {
    return std::string{};
  });

  auto created = manager_->create(make_data_request("plc_app", /*resolved_topic=*/"", "/counter"));
  ASSERT_TRUE(created.has_value()) << created.error().message;

  // Within the timeout: retry ticks stay silent.
  manager_->retry_unresolved_triggers();
  EXPECT_TRUE(warnings.empty());

  // Shrink the timeout so the next tick expires the entry.
  manager_->set_unresolved_timeout(std::chrono::seconds(0));
  manager_->retry_unresolved_triggers();
  ASSERT_EQ(warnings.size(), 1u);
  EXPECT_NE(warnings[0].find(created->id), std::string::npos) << warnings[0];
  EXPECT_NE(warnings[0].find("/counter"), std::string::npos) << warnings[0];

  // The entry is gone: later ticks are silent, and the trigger itself is
  // still registered and ACTIVE - with no subscription behind it, exactly the
  // silently-dead state the warning is there to expose.
  manager_->retry_unresolved_triggers();
  EXPECT_EQ(warnings.size(), 1u);
  auto listed = manager_->list("plc_app");
  ASSERT_EQ(listed.size(), 1u);
  EXPECT_EQ(listed[0].status, TriggerStatus::ACTIVE);
  EXPECT_TRUE(listed[0].resolved_topic_name.empty());
  EXPECT_EQ(transport_->total_subscribes(), 0u) << "giving up must not leave a live subscription behind";
}

TEST_F(TriggerManagerRoutingTest, NonDataTriggerDoesNotSubscribe) {
  // Faults / configurations / etc. don't go through the topic transport.
  TriggerCreateRequest req;
  req.entity_id = "sensor";
  req.entity_type = "apps";
  req.resource_uri = "/api/v1/apps/sensor/faults/fault_001";
  req.collection = "faults";
  req.resource_path = "/fault_001";
  req.resolved_topic_name = "";
  req.path = "";
  req.condition_type = "OnChange";
  req.condition_params = nlohmann::json::object();
  req.protocol = "sse";
  req.multishot = true;
  req.persistent = false;

  ASSERT_TRUE(manager_->create(req).has_value());
  EXPECT_EQ(transport_->total_subscribes(), 0u) << "Non-data triggers must not register any topic subscription";
}

}  // namespace
}  // namespace ros2_medkit_gateway
