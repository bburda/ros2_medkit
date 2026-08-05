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

#include <chrono>
#include <set>
#include <thread>
#include <vector>

#include "ros2_medkit_gateway/core/managers/lock_manager.hpp"

using namespace ros2_medkit_gateway;

/**
 * @brief Test fixture for LockManager tests
 *
 * Sets up a ThreadSafeEntityCache with a simple hierarchy:
 * - Area "area1" contains Component "comp1"
 * - Component "comp1" contains App "app1" and App "app2"
 * - Component "comp2" (no area) contains App "app3"
 */
class LockManagerTest : public ::testing::Test {
 protected:
  void SetUp() override {
    // Build entity hierarchy
    Area area1;
    area1.id = "area1";
    area1.name = "Test Area";

    Component comp1;
    comp1.id = "comp1";
    comp1.name = "Test Component";
    comp1.area = "area1";

    Component comp2;
    comp2.id = "comp2";
    comp2.name = "Standalone Component";

    App app1;
    app1.id = "app1";
    app1.name = "Test App 1";
    app1.component_id = "comp1";

    App app2;
    app2.id = "app2";
    app2.name = "Test App 2";
    app2.component_id = "comp1";

    App app3;
    app3.id = "app3";
    app3.name = "Test App 3";
    app3.component_id = "comp2";

    cache_.update_all({area1}, {comp1, comp2}, {app1, app2, app3}, {});
  }

  ThreadSafeEntityCache cache_;

  LockConfig make_config(bool enabled = true) {
    LockConfig cfg;
    cfg.enabled = enabled;
    cfg.default_max_expiration = 3600;
    cfg.cleanup_interval = 30;
    return cfg;
  }
};

// =========================================================================
// Status surface - what the OpenAPI declarations for extend/release rest on
// =========================================================================

// The registrations for `extend{E}Lock` / `release{E}Lock` declare 403 on top
// of the blanket 400/404/500, and deliberately NOT 409. Both halves of that are
// claims about this class, so they are checked here rather than trusted: the
// verbs must be able to produce 403, and must never produce 409 (only `acquire`
// conflicts). Driving every reachable failure path is what makes the second
// half meaningful - an assertion that "no 409 was seen" is worthless if the
// paths that could produce one were never taken.
TEST_F(LockManagerTest, extend_and_release_answer_only_400_403_404) {
  std::set<int> seen;
  auto record = [&seen](int status) {
    seen.insert(status);
  };

  LockManager mgr(cache_, make_config());
  auto held = mgr.acquire("comp1", "client_a", {}, 300);
  ASSERT_TRUE(held.has_value()) << held.error().message;

  // No lock on this entity at all.
  auto missing_extend = mgr.extend("comp2", "client_a", 60);
  ASSERT_FALSE(missing_extend.has_value());
  record(missing_extend.error().status_code);
  auto missing_release = mgr.release("comp2", "client_a");
  ASSERT_FALSE(missing_release.has_value());
  record(missing_release.error().status_code);

  // Lock exists but belongs to somebody else - the 403 the routes declare.
  auto wrong_extend = mgr.extend("comp1", "client_b", 60);
  ASSERT_FALSE(wrong_extend.has_value());
  record(wrong_extend.error().status_code);
  EXPECT_EQ(wrong_extend.error().code, "lock-not-owner");
  auto wrong_release = mgr.release("comp1", "client_b");
  ASSERT_FALSE(wrong_release.has_value());
  record(wrong_release.error().status_code);
  EXPECT_EQ(wrong_release.error().code, "lock-not-owner");

  // Invalid extension durations.
  auto zero = mgr.extend("comp1", "client_a", 0);
  ASSERT_FALSE(zero.has_value());
  record(zero.error().status_code);
  auto too_long = mgr.extend("comp1", "client_a", 99999999);
  ASSERT_FALSE(too_long.has_value());
  record(too_long.error().status_code);

  // Locking switched off.
  LockManager disabled(cache_, make_config(false));
  auto off_extend = disabled.extend("comp1", "client_a", 60);
  ASSERT_FALSE(off_extend.has_value());
  record(off_extend.error().status_code);
  auto off_release = disabled.release("comp1", "client_a");
  ASSERT_FALSE(off_release.has_value());
  record(off_release.error().status_code);

  EXPECT_EQ(seen, (std::set<int>{400, 403, 404})) << "extend/release status surface changed";
}

// =========================================================================
// Acquire tests
// =========================================================================

// @verifies REQ_INTEROP_100
TEST_F(LockManagerTest, test_acquire_lock_basic) {
  LockManager mgr(cache_, make_config());

  auto result = mgr.acquire("comp1", "client_a", {}, 300);
  ASSERT_TRUE(result.has_value()) << result.error().message;

  EXPECT_FALSE(result->lock_id.empty());
  EXPECT_EQ(result->entity_id, "comp1");
  EXPECT_EQ(result->client_id, "client_a");
  EXPECT_TRUE(result->scopes.empty());
  EXPECT_TRUE(result->breakable);
  EXPECT_EQ(result->expiration_seconds, 300);
  EXPECT_GT(result->expires_at, result->created_at);
}

// @verifies REQ_INTEROP_100
TEST_F(LockManagerTest, test_acquire_lock_scoped) {
  LockManager mgr(cache_, make_config());

  auto result = mgr.acquire("comp1", "client_a", {"data", "configurations"}, 600);
  ASSERT_TRUE(result.has_value()) << result.error().message;

  ASSERT_EQ(result->scopes.size(), 2u);
  EXPECT_EQ(result->scopes[0], "data");
  EXPECT_EQ(result->scopes[1], "configurations");
}

// @verifies REQ_INTEROP_100
TEST_F(LockManagerTest, test_acquire_lock_full) {
  LockManager mgr(cache_, make_config());

  auto result = mgr.acquire("comp1", "client_a", {}, 300);
  ASSERT_TRUE(result.has_value()) << result.error().message;

  // Empty scopes means all collections locked
  EXPECT_TRUE(result->scopes.empty());
}

// @verifies REQ_INTEROP_100
TEST_F(LockManagerTest, test_acquire_already_locked) {
  LockManager mgr(cache_, make_config());

  auto first = mgr.acquire("comp1", "client_a", {}, 300);
  ASSERT_TRUE(first.has_value());

  auto second = mgr.acquire("comp1", "client_b", {}, 300);
  ASSERT_FALSE(second.has_value());
  EXPECT_EQ(second.error().code, "lock-conflict");
  EXPECT_EQ(second.error().status_code, 409);
  EXPECT_TRUE(second.error().existing_lock_id.has_value());
  EXPECT_EQ(second.error().existing_lock_id.value(), first->lock_id);
}

// @verifies REQ_INTEROP_100
TEST_F(LockManagerTest, test_acquire_break_lock) {
  LockManager mgr(cache_, make_config());

  auto first = mgr.acquire("comp1", "client_a", {}, 300);
  ASSERT_TRUE(first.has_value());

  auto second = mgr.acquire("comp1", "client_b", {"data"}, 600, true);
  ASSERT_TRUE(second.has_value()) << second.error().message;

  // New lock should have replaced the old one
  EXPECT_NE(second->lock_id, first->lock_id);
  EXPECT_EQ(second->client_id, "client_b");
  ASSERT_EQ(second->scopes.size(), 1u);
  EXPECT_EQ(second->scopes[0], "data");

  // Old lock should be gone
  EXPECT_FALSE(mgr.get_lock_by_id(first->lock_id).has_value());
}

// @verifies REQ_INTEROP_100
TEST_F(LockManagerTest, test_acquire_break_lock_not_breakable) {
  LockConfig cfg = make_config();
  cfg.entity_overrides["comp1"] = EntityLockConfig{{}, false, 0};
  LockManager mgr(cache_, cfg);

  auto first = mgr.acquire("comp1", "client_a", {}, 300);
  ASSERT_TRUE(first.has_value());

  auto second = mgr.acquire("comp1", "client_b", {}, 300, true);
  ASSERT_FALSE(second.has_value());
  EXPECT_EQ(second.error().code, "lock-not-breakable");
  EXPECT_EQ(second.error().status_code, 409);
}

// @verifies REQ_INTEROP_100
TEST_F(LockManagerTest, test_acquire_invalid_scope) {
  LockManager mgr(cache_, make_config());

  auto result = mgr.acquire("comp1", "client_a", {"data", "nonexistent_scope"}, 300);
  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error().code, "invalid-scope");
  EXPECT_EQ(result.error().status_code, 400);
}

// @verifies REQ_INTEROP_100
TEST_F(LockManagerTest, test_acquire_invalid_expiration) {
  LockManager mgr(cache_, make_config());

  // Zero expiration
  auto result1 = mgr.acquire("comp1", "client_a", {}, 0);
  ASSERT_FALSE(result1.has_value());
  EXPECT_EQ(result1.error().code, "invalid-expiration");
  EXPECT_EQ(result1.error().status_code, 400);

  // Negative expiration
  auto result2 = mgr.acquire("comp1", "client_a", {}, -10);
  ASSERT_FALSE(result2.has_value());
  EXPECT_EQ(result2.error().code, "invalid-expiration");

  // Exceeds max expiration
  auto result3 = mgr.acquire("comp1", "client_a", {}, 7200);
  ASSERT_FALSE(result3.has_value());
  EXPECT_EQ(result3.error().code, "invalid-expiration");
}

// =========================================================================
// Release tests
// =========================================================================

// @verifies REQ_INTEROP_100
TEST_F(LockManagerTest, test_release_lock) {
  LockManager mgr(cache_, make_config());

  auto acquired = mgr.acquire("comp1", "client_a", {}, 300);
  ASSERT_TRUE(acquired.has_value());

  auto result = mgr.release("comp1", "client_a");
  ASSERT_TRUE(result.has_value());

  // Lock should be gone
  EXPECT_FALSE(mgr.get_lock("comp1").has_value());
  EXPECT_FALSE(mgr.get_lock_by_id(acquired->lock_id).has_value());
}

// @verifies REQ_INTEROP_100
TEST_F(LockManagerTest, test_release_not_owner) {
  LockManager mgr(cache_, make_config());

  ASSERT_TRUE(mgr.acquire("comp1", "client_a", {}, 300).has_value());

  auto result = mgr.release("comp1", "client_b");
  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error().code, "lock-not-owner");
  EXPECT_EQ(result.error().status_code, 403);

  // Lock should still exist
  EXPECT_TRUE(mgr.get_lock("comp1").has_value());
}

// @verifies REQ_INTEROP_100
TEST_F(LockManagerTest, test_release_not_found) {
  LockManager mgr(cache_, make_config());

  auto result = mgr.release("comp1", "client_a");
  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error().code, "lock-not-found");
  EXPECT_EQ(result.error().status_code, 404);
}

// =========================================================================
// Extend tests
// =========================================================================

// @verifies REQ_INTEROP_100
TEST_F(LockManagerTest, test_extend_lock) {
  LockManager mgr(cache_, make_config());

  auto acquired = mgr.acquire("comp1", "client_a", {}, 300);
  ASSERT_TRUE(acquired.has_value());
  auto original_expires = acquired->expires_at;

  // Small delay to ensure time progresses
  std::this_thread::sleep_for(std::chrono::milliseconds(10));

  auto result = mgr.extend("comp1", "client_a", 600);
  ASSERT_TRUE(result.has_value()) << result.error().message;

  EXPECT_GT(result->expires_at, original_expires);
  EXPECT_EQ(result->expiration_seconds, 600);
  EXPECT_EQ(result->lock_id, acquired->lock_id);
}

// @verifies REQ_INTEROP_100
TEST_F(LockManagerTest, test_extend_not_owner) {
  LockManager mgr(cache_, make_config());

  ASSERT_TRUE(mgr.acquire("comp1", "client_a", {}, 300).has_value());

  auto result = mgr.extend("comp1", "client_b", 600);
  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error().code, "lock-not-owner");
  EXPECT_EQ(result.error().status_code, 403);
}

// @verifies REQ_INTEROP_100
TEST_F(LockManagerTest, test_extend_not_found) {
  LockManager mgr(cache_, make_config());

  auto result = mgr.extend("comp1", "client_a", 600);
  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error().code, "lock-not-found");
  EXPECT_EQ(result.error().status_code, 404);
}

// =========================================================================
// Get lock tests
// =========================================================================

// @verifies REQ_INTEROP_100
TEST_F(LockManagerTest, test_get_lock) {
  LockManager mgr(cache_, make_config());

  auto acquired = mgr.acquire("comp1", "client_a", {"data"}, 300);
  ASSERT_TRUE(acquired.has_value());

  auto info = mgr.get_lock("comp1");
  ASSERT_TRUE(info.has_value());
  EXPECT_EQ(info->lock_id, acquired->lock_id);
  EXPECT_EQ(info->entity_id, "comp1");
  EXPECT_EQ(info->client_id, "client_a");
  ASSERT_EQ(info->scopes.size(), 1u);
  EXPECT_EQ(info->scopes[0], "data");
}

// @verifies REQ_INTEROP_100
TEST_F(LockManagerTest, test_get_lock_by_id) {
  LockManager mgr(cache_, make_config());

  auto acquired = mgr.acquire("comp1", "client_a", {}, 300);
  ASSERT_TRUE(acquired.has_value());

  auto info = mgr.get_lock_by_id(acquired->lock_id);
  ASSERT_TRUE(info.has_value());
  EXPECT_EQ(info->entity_id, "comp1");
  EXPECT_EQ(info->client_id, "client_a");

  // Non-existent lock_id
  EXPECT_FALSE(mgr.get_lock_by_id("lock_nonexistent").has_value());
}

// =========================================================================
// Access check tests
// =========================================================================

// @verifies REQ_INTEROP_100
TEST_F(LockManagerTest, test_check_access_unlocked) {
  LockManager mgr(cache_, make_config());

  auto result = mgr.check_access("comp1", "client_a", "data");
  EXPECT_TRUE(result.allowed);
  EXPECT_TRUE(result.denied_by_lock_id.empty());
}

// @verifies REQ_INTEROP_100
TEST_F(LockManagerTest, test_check_access_owner) {
  LockManager mgr(cache_, make_config());

  ASSERT_TRUE(mgr.acquire("comp1", "client_a", {}, 300).has_value());

  auto result = mgr.check_access("comp1", "client_a", "data");
  EXPECT_TRUE(result.allowed);
}

// @verifies REQ_INTEROP_100
TEST_F(LockManagerTest, test_check_access_not_owner) {
  LockManager mgr(cache_, make_config());

  auto acquired = mgr.acquire("comp1", "client_a", {}, 300);
  ASSERT_TRUE(acquired.has_value());

  auto result = mgr.check_access("comp1", "client_b", "data");
  EXPECT_FALSE(result.allowed);
  EXPECT_EQ(result.denied_by_lock_id, acquired->lock_id);
  EXPECT_EQ(result.denied_code, "lock-conflict");
}

// @verifies REQ_INTEROP_100
TEST_F(LockManagerTest, test_check_access_scoped_match) {
  LockManager mgr(cache_, make_config());

  // Lock only "data" and "configurations"
  ASSERT_TRUE(mgr.acquire("comp1", "client_a", {"data", "configurations"}, 300).has_value());

  // Access "data" - should be denied for non-owner
  auto result = mgr.check_access("comp1", "client_b", "data");
  EXPECT_FALSE(result.allowed);

  // Access "configurations" - should also be denied
  auto result2 = mgr.check_access("comp1", "client_b", "configurations");
  EXPECT_FALSE(result2.allowed);
}

// @verifies REQ_INTEROP_100
TEST_F(LockManagerTest, test_check_access_scoped_no_match) {
  LockManager mgr(cache_, make_config());

  // Lock only "data"
  ASSERT_TRUE(mgr.acquire("comp1", "client_a", {"data"}, 300).has_value());

  // Access "operations" - should be allowed (not in locked scopes)
  auto result = mgr.check_access("comp1", "client_b", "operations");
  EXPECT_TRUE(result.allowed);
}

// @verifies REQ_INTEROP_100
TEST_F(LockManagerTest, test_check_access_parent_propagation) {
  LockManager mgr(cache_, make_config());

  // Lock the Component
  auto acquired = mgr.acquire("comp1", "client_a", {}, 300);
  ASSERT_TRUE(acquired.has_value());

  // Access check on App child - should be denied for non-owner
  auto result = mgr.check_access("app1", "client_b", "data");
  EXPECT_FALSE(result.allowed);
  EXPECT_EQ(result.denied_by_lock_id, acquired->lock_id);

  // Same for app2 (also on comp1)
  auto result2 = mgr.check_access("app2", "client_b", "data");
  EXPECT_FALSE(result2.allowed);

  // app3 is on comp2, not affected
  auto result3 = mgr.check_access("app3", "client_b", "data");
  EXPECT_TRUE(result3.allowed);
}

// @verifies REQ_INTEROP_100
TEST_F(LockManagerTest, test_check_access_area_propagation) {
  LockManager mgr(cache_, make_config());

  // Lock the Area
  auto acquired = mgr.acquire("area1", "client_a", {}, 300);
  ASSERT_TRUE(acquired.has_value());

  // Access check on Component - should be denied
  auto result_comp = mgr.check_access("comp1", "client_b", "data");
  EXPECT_FALSE(result_comp.allowed);
  EXPECT_EQ(result_comp.denied_by_lock_id, acquired->lock_id);

  // Access check on App (grandchild) - should be denied
  auto result_app = mgr.check_access("app1", "client_b", "data");
  EXPECT_FALSE(result_app.allowed);
  EXPECT_EQ(result_app.denied_by_lock_id, acquired->lock_id);

  // comp2 has no area, not affected
  auto result_comp2 = mgr.check_access("comp2", "client_b", "data");
  EXPECT_TRUE(result_comp2.allowed);
}

// @verifies REQ_INTEROP_100
TEST_F(LockManagerTest, test_check_access_partial_scope_propagation) {
  LockManager mgr(cache_, make_config());

  // Lock the Component with only "configurations" scope
  ASSERT_TRUE(mgr.acquire("comp1", "client_a", {"configurations"}, 300).has_value());

  // Check App child access for "operations" - should be allowed (scope doesn't match)
  auto result = mgr.check_access("app1", "client_b", "operations");
  EXPECT_TRUE(result.allowed);

  // Check App child access for "configurations" - should be denied
  auto result2 = mgr.check_access("app1", "client_b", "configurations");
  EXPECT_FALSE(result2.allowed);
}

// =========================================================================
// Cleanup tests
// =========================================================================

// @verifies REQ_INTEROP_100
TEST_F(LockManagerTest, test_cleanup_expired) {
  LockManager mgr(cache_, make_config());

  // Acquire a lock with very short expiration (we'll manipulate time by using a 1-second lock)
  auto result = mgr.acquire("comp1", "client_a", {}, 1);
  ASSERT_TRUE(result.has_value());
  std::string lock_id = result->lock_id;

  // Wait for it to expire
  std::this_thread::sleep_for(std::chrono::seconds(2));

  auto expired = mgr.cleanup_expired();
  ASSERT_EQ(expired.size(), 1u);
  EXPECT_EQ(expired[0].lock_id, lock_id);
  EXPECT_EQ(expired[0].entity_id, "comp1");
  EXPECT_EQ(expired[0].client_id, "client_a");

  // Lock should be gone
  EXPECT_FALSE(mgr.get_lock("comp1").has_value());
  EXPECT_FALSE(mgr.get_lock_by_id(lock_id).has_value());
}

// @verifies REQ_INTEROP_100
TEST_F(LockManagerTest, test_cleanup_not_expired) {
  LockManager mgr(cache_, make_config());

  ASSERT_TRUE(mgr.acquire("comp1", "client_a", {}, 3600).has_value());

  auto expired = mgr.cleanup_expired();
  EXPECT_TRUE(expired.empty());

  // Lock should still exist
  EXPECT_TRUE(mgr.get_lock("comp1").has_value());
}

// =========================================================================
// lock_required config tests
// =========================================================================

// @verifies REQ_INTEROP_100
TEST_F(LockManagerTest, test_lock_required_no_lock_held) {
  LockConfig cfg = make_config();
  EntityLockConfig comp_cfg;
  comp_cfg.required_scopes = {"data", "configurations"};
  cfg.type_defaults["component"] = comp_cfg;
  LockManager mgr(cache_, cfg);

  // comp1 is a component, "data" is in required_scopes -> lock required
  auto result = mgr.check_access("comp1", "client_a", "data");
  EXPECT_FALSE(result.allowed);
  EXPECT_EQ(result.denied_code, "lock-required");
}

// @verifies REQ_INTEROP_100
TEST_F(LockManagerTest, test_lock_required_has_lock) {
  LockConfig cfg = make_config();
  EntityLockConfig comp_cfg;
  comp_cfg.required_scopes = {"data", "configurations"};
  cfg.type_defaults["component"] = comp_cfg;
  LockManager mgr(cache_, cfg);

  ASSERT_TRUE(mgr.acquire("comp1", "client_a", {}, 300).has_value());

  // Client holds a lock -> access allowed
  auto result = mgr.check_access("comp1", "client_a", "data");
  EXPECT_TRUE(result.allowed);
}

// @verifies REQ_INTEROP_100
TEST_F(LockManagerTest, test_lock_required_different_collection_not_blocked) {
  LockConfig cfg = make_config();
  EntityLockConfig comp_cfg;
  comp_cfg.required_scopes = {"configurations"};
  cfg.type_defaults["component"] = comp_cfg;
  LockManager mgr(cache_, cfg);

  // "data" is NOT in required_scopes -> access allowed without lock
  auto result = mgr.check_access("comp1", "client_a", "data");
  EXPECT_TRUE(result.allowed);
}

// =========================================================================
// Disabled config tests
// =========================================================================

// @verifies REQ_INTEROP_100
TEST_F(LockManagerTest, test_config_disabled) {
  LockManager mgr(cache_, make_config(false));

  // Acquire should fail with disabled error
  auto acquire_result = mgr.acquire("comp1", "client_a", {}, 300);
  ASSERT_FALSE(acquire_result.has_value());
  EXPECT_EQ(acquire_result.error().code, "lock-disabled");

  // Release should fail with disabled error
  auto release_result = mgr.release("comp1", "client_a");
  ASSERT_FALSE(release_result.has_value());
  EXPECT_EQ(release_result.error().code, "lock-disabled");

  // Extend should fail with disabled error
  auto extend_result = mgr.extend("comp1", "client_a", 300);
  ASSERT_FALSE(extend_result.has_value());
  EXPECT_EQ(extend_result.error().code, "lock-disabled");

  // check_access should always allow when disabled
  auto access_result = mgr.check_access("comp1", "client_a", "data");
  EXPECT_TRUE(access_result.allowed);
}

// =========================================================================
// Entity lock config override tests
// =========================================================================

TEST_F(LockManagerTest, test_type_default_breakable) {
  LockConfig cfg = make_config();
  cfg.type_defaults["component"] = EntityLockConfig{{}, false, 0};
  LockManager mgr(cache_, cfg);

  auto acquired = mgr.acquire("comp1", "client_a", {}, 300);
  ASSERT_TRUE(acquired.has_value());
  EXPECT_FALSE(acquired->breakable);
}

TEST_F(LockManagerTest, test_entity_override_max_expiration) {
  LockConfig cfg = make_config();
  cfg.entity_overrides["comp1"] = EntityLockConfig{{}, true, 60};
  LockManager mgr(cache_, cfg);

  // Within entity max
  auto result1 = mgr.acquire("comp1", "client_a", {}, 60);
  ASSERT_TRUE(result1.has_value());

  ASSERT_TRUE(mgr.release("comp1", "client_a").has_value());

  // Exceeds entity max
  auto result2 = mgr.acquire("comp1", "client_a", {}, 61);
  ASSERT_FALSE(result2.has_value());
  EXPECT_EQ(result2.error().code, "invalid-expiration");

  // Different entity uses global max (3600)
  auto result3 = mgr.acquire("comp2", "client_a", {}, 3600);
  ASSERT_TRUE(result3.has_value());
}

// =========================================================================
// Lock ID uniqueness
// =========================================================================

TEST_F(LockManagerTest, test_lock_ids_unique) {
  LockManager mgr(cache_, make_config());

  auto lock1 = mgr.acquire("comp1", "client_a", {}, 300);
  ASSERT_TRUE(lock1.has_value());
  std::string id1 = lock1->lock_id;

  ASSERT_TRUE(mgr.release("comp1", "client_a").has_value());

  auto lock2 = mgr.acquire("comp1", "client_a", {}, 300);
  ASSERT_TRUE(lock2.has_value());

  EXPECT_NE(id1, lock2->lock_id);
  EXPECT_EQ(id1.substr(0, 5), "lock_");
  EXPECT_EQ(lock2->lock_id.substr(0, 5), "lock_");
}

// =========================================================================
// Multiple entities simultaneously
// =========================================================================

TEST_F(LockManagerTest, test_multiple_entity_locks) {
  LockManager mgr(cache_, make_config());

  auto lock1 = mgr.acquire("comp1", "client_a", {}, 300);
  auto lock2 = mgr.acquire("comp2", "client_b", {"data"}, 300);
  auto lock3 = mgr.acquire("app3", "client_a", {"operations"}, 300);

  ASSERT_TRUE(lock1.has_value());
  ASSERT_TRUE(lock2.has_value());
  ASSERT_TRUE(lock3.has_value());

  EXPECT_NE(lock1->lock_id, lock2->lock_id);
  EXPECT_NE(lock2->lock_id, lock3->lock_id);

  // Verify each lock is accessible
  EXPECT_TRUE(mgr.get_lock("comp1").has_value());
  EXPECT_TRUE(mgr.get_lock("comp2").has_value());
  EXPECT_TRUE(mgr.get_lock("app3").has_value());
  EXPECT_FALSE(mgr.get_lock("app1").has_value());
}

// =========================================================================
// Same client re-acquiring own lock
// =========================================================================

TEST_F(LockManagerTest, test_same_client_reacquire_without_break) {
  LockManager mgr(cache_, make_config());

  ASSERT_TRUE(mgr.acquire("comp1", "client_a", {}, 300).has_value());

  // Same client trying to acquire again without break_lock
  auto result = mgr.acquire("comp1", "client_a", {}, 300);
  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error().code, "lock-conflict");
}

// =========================================================================
// Owner access through parent lock
// =========================================================================

TEST_F(LockManagerTest, test_owner_access_through_parent) {
  LockManager mgr(cache_, make_config());

  ASSERT_TRUE(mgr.acquire("comp1", "client_a", {}, 300).has_value());

  // Owner should have access to child app
  auto result = mgr.check_access("app1", "client_a", "data");
  EXPECT_TRUE(result.allowed);
}

// =========================================================================
// Extend with invalid duration
// =========================================================================

TEST_F(LockManagerTest, test_extend_invalid_duration) {
  LockManager mgr(cache_, make_config());

  ASSERT_TRUE(mgr.acquire("comp1", "client_a", {}, 300).has_value());

  auto result = mgr.extend("comp1", "client_a", 0);
  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error().code, "invalid-expiration");

  auto result2 = mgr.extend("comp1", "client_a", -5);
  ASSERT_FALSE(result2.has_value());
  EXPECT_EQ(result2.error().code, "invalid-expiration");
}

// =========================================================================
// Valid scope values
// =========================================================================

TEST_F(LockManagerTest, test_all_valid_scopes) {
  LockManager mgr(cache_, make_config());

  std::vector<std::string> all_scopes = {"data",  "operations", "configurations", "faults",
                                         "modes", "scripts",    "bulk-data"};

  auto result = mgr.acquire("comp1", "client_a", all_scopes, 300);
  ASSERT_TRUE(result.has_value()) << result.error().message;
  EXPECT_EQ(result->scopes.size(), all_scopes.size());
}
