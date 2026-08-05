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

#include <algorithm>
#include <atomic>
#include <chrono>
#include <string>
#include <thread>
#include <vector>

#include "ros2_medkit_gateway/core/models/aggregation_service.hpp"
#include "ros2_medkit_gateway/core/models/entity_capabilities.hpp"
#include "ros2_medkit_gateway/core/models/entity_types.hpp"
#include "ros2_medkit_gateway/core/models/thread_safe_entity_cache.hpp"

using namespace ros2_medkit_gateway;
using namespace std::chrono_literals;

// ============================================================================
// Test Helper Functions - avoid C++20 designated initializers for C++17 compat
// ============================================================================

namespace {

Area make_area(const std::string & id, const std::string & name) {
  Area a;
  a.id = id;
  a.name = name;
  return a;
}

Component make_component(const std::string & id, const std::string & name, const std::string & area) {
  Component c;
  c.id = id;
  c.name = name;
  c.area = area;
  return c;
}

App make_app(const std::string & id, const std::string & name, const std::string & component_id) {
  App a;
  a.id = id;
  a.name = name;
  a.component_id = component_id;
  return a;
}

App make_app_minimal(const std::string & id, const std::string & component_id) {
  App a;
  a.id = id;
  a.component_id = component_id;
  return a;
}

ServiceInfo make_service(const std::string & name, const std::string & full_path) {
  ServiceInfo s;
  s.name = name;
  s.full_path = full_path;
  return s;
}

ActionInfo make_action(const std::string & name, const std::string & full_path) {
  ActionInfo a;
  a.name = name;
  a.full_path = full_path;
  return a;
}

}  // namespace

// ============================================================================
// EntityTypes Tests
// ============================================================================

TEST(EntityTypes, ToStringReturnsCorrectValues) {
  EXPECT_EQ(to_string(SovdEntityType::SERVER), "Server");
  EXPECT_EQ(to_string(SovdEntityType::AREA), "Area");
  EXPECT_EQ(to_string(SovdEntityType::COMPONENT), "Component");
  EXPECT_EQ(to_string(SovdEntityType::APP), "App");
  EXPECT_EQ(to_string(SovdEntityType::FUNCTION), "Function");
  EXPECT_EQ(to_string(SovdEntityType::UNKNOWN), "Unknown");
}

TEST(EntityTypes, ResourceCollectionToString) {
  EXPECT_EQ(to_string(ResourceCollection::CONFIGURATIONS), "configurations");
  EXPECT_EQ(to_string(ResourceCollection::DATA), "data");
  EXPECT_EQ(to_string(ResourceCollection::DATA_CATEGORIES), "data-categories");
  EXPECT_EQ(to_string(ResourceCollection::DATA_GROUPS), "data-groups");
  EXPECT_EQ(to_string(ResourceCollection::FAULTS), "faults");
  EXPECT_EQ(to_string(ResourceCollection::FAULT_TRIGGERS), "fault-triggers");
  EXPECT_EQ(to_string(ResourceCollection::OPERATIONS), "operations");
  EXPECT_EQ(to_string(ResourceCollection::BULK_DATA), "bulk-data");
  EXPECT_EQ(to_string(ResourceCollection::DATA_LISTS), "data-lists");
}

TEST(EntityTypes, ParseResourceCollection) {
  auto configs = parse_resource_collection("configurations");
  ASSERT_TRUE(configs.has_value());
  EXPECT_EQ(*configs, ResourceCollection::CONFIGURATIONS);

  // Recognised even though no entity lists it as a capability: parsing a path
  // segment and advertising a collection are separate questions.
  auto data_lists = parse_resource_collection("data-lists");
  ASSERT_TRUE(data_lists.has_value());
  EXPECT_EQ(*data_lists, ResourceCollection::DATA_LISTS);

  auto categories = parse_resource_collection("data-categories");
  ASSERT_TRUE(categories.has_value());
  EXPECT_EQ(*categories, ResourceCollection::DATA_CATEGORIES);

  auto groups = parse_resource_collection("data-groups");
  ASSERT_TRUE(groups.has_value());
  EXPECT_EQ(*groups, ResourceCollection::DATA_GROUPS);

  auto fault_triggers = parse_resource_collection("fault-triggers");
  ASSERT_TRUE(fault_triggers.has_value());
  EXPECT_EQ(*fault_triggers, ResourceCollection::FAULT_TRIGGERS);

  auto invalid = parse_resource_collection("invalid");
  EXPECT_FALSE(invalid.has_value());
}

TEST(EntityTypes, ParseEntityType) {
  EXPECT_EQ(parse_entity_type("Component"), SovdEntityType::COMPONENT);
  EXPECT_EQ(parse_entity_type("component"), SovdEntityType::COMPONENT);
  EXPECT_EQ(parse_entity_type("COMPONENT"), SovdEntityType::COMPONENT);
  EXPECT_EQ(parse_entity_type("App"), SovdEntityType::APP);
  EXPECT_EQ(parse_entity_type("application"), SovdEntityType::APP);
  EXPECT_EQ(parse_entity_type("invalid"), SovdEntityType::UNKNOWN);
}

// ============================================================================
// EntityCapabilities Tests
// ============================================================================

TEST(EntityCapabilities, ServerSupportsOnlyTheRootMountedCollections) {
  // `/faults` and `/updates` are the only collections mounted at the API root.
  // Everything else in the enum is entity-scoped, so listing it here would put
  // an href into a 404 on the server capability surface.
  auto caps = EntityCapabilities::for_type(SovdEntityType::SERVER);
  EXPECT_TRUE(caps.supports_collection(ResourceCollection::FAULTS));
  EXPECT_TRUE(caps.supports_collection(ResourceCollection::UPDATES));
  EXPECT_FALSE(caps.supports_collection(ResourceCollection::CONFIGURATIONS));
  EXPECT_FALSE(caps.supports_collection(ResourceCollection::DATA));
  EXPECT_FALSE(caps.supports_collection(ResourceCollection::OPERATIONS));
  EXPECT_FALSE(caps.supports_collection(ResourceCollection::LOGS));
  EXPECT_FALSE(caps.supports_collection(ResourceCollection::LOCKS));
  EXPECT_FALSE(caps.supports_resource("logs"));
  EXPECT_FALSE(caps.supports_resource("depends-on"));
}

// The three collections with no entity-scoped route anywhere. This is the
// invariant the phantom hrefs violated: `/x/{id}/data-lists`, `/x/{id}/modes`
// and `/x/{id}/updates` are registered for no entity type, so no capability
// list may name them.
TEST(EntityCapabilities, NoEntityAdvertisesACollectionWithoutARoute) {
  for (auto type : {SovdEntityType::AREA, SovdEntityType::COMPONENT, SovdEntityType::APP, SovdEntityType::FUNCTION}) {
    auto caps = EntityCapabilities::for_type(type);
    EXPECT_FALSE(caps.supports_collection(ResourceCollection::DATA_LISTS)) << to_string(type);
    EXPECT_FALSE(caps.supports_collection(ResourceCollection::MODES)) << to_string(type);
    EXPECT_FALSE(caps.supports_collection(ResourceCollection::UPDATES)) << to_string(type);
    EXPECT_FALSE(caps.supports_collection(ResourceCollection::COMMUNICATION_LOGS)) << to_string(type);
  }
}

// The four-entity-type loop in rest_server.cpp registers these for every type.
TEST(EntityCapabilities, EveryEntityTypeAdvertisesTheUnconditionalCollections) {
  for (auto type : {SovdEntityType::AREA, SovdEntityType::COMPONENT, SovdEntityType::APP, SovdEntityType::FUNCTION}) {
    auto caps = EntityCapabilities::for_type(type);
    for (auto col : {ResourceCollection::DATA, ResourceCollection::DATA_CATEGORIES, ResourceCollection::DATA_GROUPS,
                     ResourceCollection::OPERATIONS, ResourceCollection::CONFIGURATIONS, ResourceCollection::FAULTS,
                     ResourceCollection::LOGS, ResourceCollection::BULK_DATA, ResourceCollection::TRIGGERS}) {
      EXPECT_TRUE(caps.supports_collection(col)) << to_string(type) << " / " << to_string(col);
    }
  }
}

// Routes registered behind an entity-type check: locks and scripts for
// components and apps, cyclic-subscriptions for everything but areas, and
// fault-triggers for apps alone.
TEST(EntityCapabilities, TypeGatedCollectionsMatchTheirRegistrations) {
  auto area = EntityCapabilities::for_type(SovdEntityType::AREA);
  auto component = EntityCapabilities::for_type(SovdEntityType::COMPONENT);
  auto app = EntityCapabilities::for_type(SovdEntityType::APP);
  auto function = EntityCapabilities::for_type(SovdEntityType::FUNCTION);

  EXPECT_FALSE(area.supports_collection(ResourceCollection::CYCLIC_SUBSCRIPTIONS));
  EXPECT_TRUE(component.supports_collection(ResourceCollection::CYCLIC_SUBSCRIPTIONS));
  EXPECT_TRUE(app.supports_collection(ResourceCollection::CYCLIC_SUBSCRIPTIONS));
  EXPECT_TRUE(function.supports_collection(ResourceCollection::CYCLIC_SUBSCRIPTIONS));

  for (const auto & caps : {component, app}) {
    EXPECT_TRUE(caps.supports_collection(ResourceCollection::LOCKS));
    EXPECT_TRUE(caps.supports_collection(ResourceCollection::SCRIPTS));
  }
  for (const auto & caps : {area, function}) {
    EXPECT_FALSE(caps.supports_collection(ResourceCollection::LOCKS));
    EXPECT_FALSE(caps.supports_collection(ResourceCollection::SCRIPTS));
  }

  EXPECT_TRUE(app.supports_collection(ResourceCollection::FAULT_TRIGGERS));
  EXPECT_FALSE(component.supports_collection(ResourceCollection::FAULT_TRIGGERS));
  EXPECT_FALSE(area.supports_collection(ResourceCollection::FAULT_TRIGGERS));
  EXPECT_FALSE(function.supports_collection(ResourceCollection::FAULT_TRIGGERS));
}

TEST(EntityCapabilities, AreaSupportsCollectionsViaAggregation) {
  auto caps = EntityCapabilities::for_type(SovdEntityType::AREA);
  // ros2_medkit extension: areas support resource collections via aggregation
  EXPECT_TRUE(caps.supports_collection(ResourceCollection::CONFIGURATIONS));
  EXPECT_TRUE(caps.supports_collection(ResourceCollection::DATA));
  EXPECT_TRUE(caps.supports_collection(ResourceCollection::FAULTS));
  EXPECT_TRUE(caps.supports_collection(ResourceCollection::OPERATIONS));
  EXPECT_TRUE(caps.supports_collection(ResourceCollection::LOGS));
  EXPECT_TRUE(caps.supports_collection(ResourceCollection::BULK_DATA));
  EXPECT_TRUE(caps.is_aggregated(ResourceCollection::DATA));
  EXPECT_TRUE(caps.is_aggregated(ResourceCollection::OPERATIONS));
  EXPECT_TRUE(caps.is_aggregated(ResourceCollection::CONFIGURATIONS));
  EXPECT_TRUE(caps.is_aggregated(ResourceCollection::FAULTS));
  EXPECT_TRUE(caps.is_aggregated(ResourceCollection::LOGS));
  // Bulk-data uses host-scoped filtering, not aggregation
  EXPECT_FALSE(caps.is_aggregated(ResourceCollection::BULK_DATA));
}

TEST(EntityCapabilities, AreaSupportsContains) {
  auto caps = EntityCapabilities::for_type(SovdEntityType::AREA);
  EXPECT_TRUE(caps.supports_resource("contains"));
  EXPECT_TRUE(caps.supports_resource("subareas"));
  EXPECT_TRUE(caps.supports_resource("docs"));
  // The route is /areas/{area_id}/components; "related-components" named none.
  EXPECT_TRUE(caps.supports_resource("components"));
  EXPECT_FALSE(caps.supports_resource("related-components"));
}

TEST(EntityCapabilities, ComponentSupportsOperations) {
  auto caps = EntityCapabilities::for_type(SovdEntityType::COMPONENT);
  EXPECT_TRUE(caps.supports_collection(ResourceCollection::OPERATIONS));
  EXPECT_TRUE(caps.supports_collection(ResourceCollection::CONFIGURATIONS));
  EXPECT_TRUE(caps.supports_collection(ResourceCollection::DATA));
  EXPECT_TRUE(caps.supports_resource("hosts"));
}

TEST(EntityCapabilities, AppSupportsIsLocatedOn) {
  auto caps = EntityCapabilities::for_type(SovdEntityType::APP);
  EXPECT_TRUE(caps.supports_resource("is-located-on"));
  EXPECT_TRUE(caps.supports_collection(ResourceCollection::OPERATIONS));
  EXPECT_FALSE(caps.supports_resource("hosts"));  // Apps don't host anything
}

TEST(EntityCapabilities, AppSupportsBelongsTo) {
  auto caps = EntityCapabilities::for_type(SovdEntityType::APP);
  EXPECT_TRUE(caps.supports_resource("belongs-to"));
}

TEST(EntityCapabilities, FunctionHasNoDependsOnResource) {
  // /depends-on is registered for components and apps only.
  auto caps = EntityCapabilities::for_type(SovdEntityType::FUNCTION);
  EXPECT_TRUE(caps.supports_resource("hosts"));
  EXPECT_FALSE(caps.supports_resource("depends-on"));
  EXPECT_TRUE(EntityCapabilities::for_type(SovdEntityType::COMPONENT).supports_resource("depends-on"));
  EXPECT_TRUE(EntityCapabilities::for_type(SovdEntityType::APP).supports_resource("depends-on"));
}

TEST(EntityCapabilities, FunctionAggregatesCollections) {
  auto caps = EntityCapabilities::for_type(SovdEntityType::FUNCTION);
  // ros2_medkit extension: functions support additional collections via aggregation
  EXPECT_TRUE(caps.supports_collection(ResourceCollection::DATA));
  EXPECT_TRUE(caps.supports_collection(ResourceCollection::OPERATIONS));
  EXPECT_TRUE(caps.supports_collection(ResourceCollection::CONFIGURATIONS));
  EXPECT_TRUE(caps.supports_collection(ResourceCollection::FAULTS));
  EXPECT_TRUE(caps.supports_collection(ResourceCollection::LOGS));
  EXPECT_TRUE(caps.supports_collection(ResourceCollection::BULK_DATA));
  EXPECT_TRUE(caps.supports_collection(ResourceCollection::CYCLIC_SUBSCRIPTIONS));
  EXPECT_TRUE(caps.is_aggregated(ResourceCollection::DATA));
  EXPECT_TRUE(caps.is_aggregated(ResourceCollection::OPERATIONS));
  EXPECT_TRUE(caps.is_aggregated(ResourceCollection::CONFIGURATIONS));
  EXPECT_TRUE(caps.is_aggregated(ResourceCollection::FAULTS));
  EXPECT_TRUE(caps.is_aggregated(ResourceCollection::LOGS));
  // Bulk-data uses host-scoped filtering, not aggregation
  EXPECT_FALSE(caps.is_aggregated(ResourceCollection::BULK_DATA));
}

TEST(EntityCapabilities, UnknownTypeHasNoCapabilities) {
  auto caps = EntityCapabilities::for_type(SovdEntityType::UNKNOWN);
  EXPECT_TRUE(caps.collections().empty());
  EXPECT_TRUE(caps.resources().empty());
}

// ============================================================================
// ThreadSafeEntityCache Tests
// ============================================================================

class EntityCacheTest : public ::testing::Test {
 protected:
  void SetUp() override {
    // Create test data using helper functions (C++17 compatible)
    areas_ = {
        make_area("perception", "Perception"),
        make_area("control", "Control"),
    };

    components_ = {
        make_component("lidar", "LiDAR Driver", "perception"),
        make_component("nav2", "Nav2 Stack", "control"),
    };

    apps_ = {
        make_app("lidar_app", "LiDAR App", "lidar"),
        make_app("controller", "Controller", "nav2"),
        make_app("planner", "Planner", "nav2"),
    };

    // Add operations to apps
    apps_[0].services = {make_service("start_scan", "/lidar/start_scan")};
    apps_[1].services = {make_service("follow_path", "/nav2/follow_path")};
    apps_[1].actions = {make_action("navigate", "/nav2/navigate")};
    apps_[2].services = {make_service("compute_path", "/nav2/compute_path")};
  }

  ThreadSafeEntityCache cache_;
  std::vector<Area> areas_;
  std::vector<Component> components_;
  std::vector<App> apps_;
};

TEST_F(EntityCacheTest, EmptyCacheReturnsEmptyVectors) {
  EXPECT_TRUE(cache_.get_areas().empty());
  EXPECT_TRUE(cache_.get_components().empty());
  EXPECT_TRUE(cache_.get_apps().empty());
}

TEST_F(EntityCacheTest, UpdateAllStoresAllEntities) {
  cache_.update_all(areas_, components_, apps_, {});

  EXPECT_EQ(cache_.get_areas().size(), 2);
  EXPECT_EQ(cache_.get_components().size(), 2);
  EXPECT_EQ(cache_.get_apps().size(), 3);
}

TEST_F(EntityCacheTest, GetByIdReturnsCorrectEntity) {
  cache_.update_all(areas_, components_, apps_, {});

  auto area = cache_.get_area("perception");
  ASSERT_TRUE(area.has_value());
  EXPECT_EQ(area->name, "Perception");

  auto comp = cache_.get_component("nav2");
  ASSERT_TRUE(comp.has_value());
  EXPECT_EQ(comp->name, "Nav2 Stack");

  auto app = cache_.get_app("planner");
  ASSERT_TRUE(app.has_value());
  EXPECT_EQ(app->name, "Planner");
}

TEST_F(EntityCacheTest, GetByIdReturnsNulloptForUnknown) {
  cache_.update_all(areas_, components_, apps_, {});

  EXPECT_FALSE(cache_.get_area("unknown").has_value());
  EXPECT_FALSE(cache_.get_component("unknown").has_value());
  EXPECT_FALSE(cache_.get_app("unknown").has_value());
}

TEST_F(EntityCacheTest, HasEntityReturnsCorrectValue) {
  cache_.update_all(areas_, components_, apps_, {});

  EXPECT_TRUE(cache_.has_component("nav2"));
  EXPECT_FALSE(cache_.has_component("unknown"));
  EXPECT_TRUE(cache_.has_app("controller"));
  EXPECT_FALSE(cache_.has_app("unknown"));
}

TEST_F(EntityCacheTest, FindEntityReturnsCorrectType) {
  cache_.update_all(areas_, components_, apps_, {});

  auto comp_ref = cache_.find_entity("nav2");
  ASSERT_TRUE(comp_ref.has_value());
  EXPECT_EQ(comp_ref->type, SovdEntityType::COMPONENT);

  auto app_ref = cache_.find_entity("controller");
  ASSERT_TRUE(app_ref.has_value());
  EXPECT_EQ(app_ref->type, SovdEntityType::APP);

  auto area_ref = cache_.find_entity("perception");
  ASSERT_TRUE(area_ref.has_value());
  EXPECT_EQ(area_ref->type, SovdEntityType::AREA);

  EXPECT_FALSE(cache_.find_entity("unknown").has_value());
}

TEST_F(EntityCacheTest, GetEntityTypeReturnsCorrectType) {
  cache_.update_all(areas_, components_, apps_, {});

  EXPECT_EQ(cache_.get_entity_type("nav2"), SovdEntityType::COMPONENT);
  EXPECT_EQ(cache_.get_entity_type("controller"), SovdEntityType::APP);
  EXPECT_EQ(cache_.get_entity_type("perception"), SovdEntityType::AREA);
  EXPECT_EQ(cache_.get_entity_type("unknown"), SovdEntityType::UNKNOWN);
}

// ============================================================================
// Relationship Index Tests
// ============================================================================

TEST_F(EntityCacheTest, GetAppsForComponentReturnsCorrectApps) {
  cache_.update_all(areas_, components_, apps_, {});

  auto apps = cache_.get_apps_for_component("nav2");
  ASSERT_EQ(apps.size(), 2);
  EXPECT_TRUE(std::find(apps.begin(), apps.end(), "controller") != apps.end());
  EXPECT_TRUE(std::find(apps.begin(), apps.end(), "planner") != apps.end());
}

TEST_F(EntityCacheTest, GetAppsForComponentReturnsEmptyForUnknown) {
  cache_.update_all(areas_, components_, apps_, {});

  auto apps = cache_.get_apps_for_component("unknown");
  EXPECT_TRUE(apps.empty());
}

TEST_F(EntityCacheTest, GetComponentsForAreaReturnsCorrectComponents) {
  cache_.update_all(areas_, components_, apps_, {});

  auto comps = cache_.get_components_for_area("perception");
  ASSERT_EQ(comps.size(), 1);
  EXPECT_EQ(comps[0], "lidar");
}

// ============================================================================
// Aggregation Tests
// ============================================================================

TEST_F(EntityCacheTest, AppOperationsNotAggregated) {
  cache_.update_all(areas_, components_, apps_, {});

  auto ops = cache_.get_app_operations("controller");
  EXPECT_EQ(ops.aggregation_level, "app");
  EXPECT_FALSE(ops.is_aggregated);
  EXPECT_EQ(ops.services.size(), 1);
  EXPECT_EQ(ops.actions.size(), 1);
  EXPECT_EQ(ops.source_ids.size(), 1);
}

TEST_F(EntityCacheTest, ComponentOperationsAggregatesFromApps) {
  cache_.update_all(areas_, components_, apps_, {});

  auto ops = cache_.get_component_operations("nav2");
  EXPECT_EQ(ops.aggregation_level, "component");
  EXPECT_TRUE(ops.is_aggregated);

  // Should have operations from both controller and planner apps
  EXPECT_EQ(ops.services.size(), 2);  // follow_path + compute_path
  EXPECT_EQ(ops.actions.size(), 1);   // navigate

  // Source IDs should include component and both apps
  EXPECT_EQ(ops.source_ids.size(), 3);
}

TEST_F(EntityCacheTest, AreaOperationsAggregatesFromComponents) {
  cache_.update_all(areas_, components_, apps_, {});

  auto ops = cache_.get_area_operations("control");
  EXPECT_EQ(ops.aggregation_level, "area");
  EXPECT_TRUE(ops.is_aggregated);

  // Should have all operations from nav2 component and its apps
  EXPECT_GE(ops.services.size(), 2ul);
}

TEST_F(EntityCacheTest, AggregationDeduplicatesByPath) {
  // Create apps with duplicate operation paths
  apps_[1].services.push_back(make_service("shared_svc", "/shared"));
  apps_[2].services.push_back(make_service("shared_svc", "/shared"));

  cache_.update_all(areas_, components_, apps_, {});

  auto ops = cache_.get_component_operations("nav2");

  // Count occurrences of /shared path
  int shared_count = 0;
  for (const auto & svc : ops.services) {
    if (svc.full_path == "/shared") {
      shared_count++;
    }
  }
  EXPECT_EQ(shared_count, 1) << "Duplicate operation path not deduplicated";
}

TEST_F(EntityCacheTest, EmptyComponentReturnsEmptyAggregation) {
  cache_.update_all(areas_, components_, apps_, {});

  auto ops = cache_.get_component_operations("unknown");
  EXPECT_TRUE(ops.empty());
}

// ============================================================================
// Operation Index Tests
// ============================================================================

TEST_F(EntityCacheTest, FindOperationOwnerReturnsCorrectEntity) {
  cache_.update_all(areas_, components_, apps_, {});

  auto owner = cache_.find_operation_owner("/nav2/navigate");
  ASSERT_TRUE(owner.has_value());
  EXPECT_EQ(owner->type, SovdEntityType::APP);
}

TEST_F(EntityCacheTest, FindOperationOwnerReturnsNulloptForUnknown) {
  cache_.update_all(areas_, components_, apps_, {});

  EXPECT_FALSE(cache_.find_operation_owner("/unknown/op").has_value());
}

// ============================================================================
// Thread-Safety Tests
// ============================================================================

TEST_F(EntityCacheTest, ConcurrentReadsDoNotBlock) {
  cache_.update_all(areas_, components_, apps_, {});

  std::atomic<int> completed{0};
  std::vector<std::thread> readers;

  // Start multiple readers
  for (int i = 0; i < 10; ++i) {
    readers.emplace_back([this, &completed] {
      for (int j = 0; j < 100; ++j) {
        auto components = cache_.get_components();
        auto component = cache_.get_component("nav2");
        (void)components;
        (void)component;
      }
      completed++;
    });
  }

  // All readers should complete quickly
  auto start = std::chrono::high_resolution_clock::now();
  for (auto & t : readers) {
    t.join();
  }
  auto duration = std::chrono::high_resolution_clock::now() - start;

  EXPECT_EQ(completed, 10);
  EXPECT_LT(duration, 1s) << "Concurrent reads took too long - possible blocking";
}

TEST_F(EntityCacheTest, ConcurrentReadsAndWritesDoNotDeadlock) {
  cache_.update_all(areas_, components_, apps_, {});

  std::atomic<bool> keep_running{true};
  std::atomic<int> read_count{0};
  std::atomic<int> write_count{0};

  // Multiple readers
  std::vector<std::thread> readers;
  for (int i = 0; i < 4; ++i) {
    readers.emplace_back([this, &keep_running, &read_count] {
      while (keep_running) {
        auto comps = cache_.get_components();
        (void)comps;
        read_count++;
      }
    });
  }

  // Single writer
  std::thread writer([this, &keep_running, &write_count] {
    while (keep_running) {
      cache_.update_all(areas_, components_, apps_, {});
      write_count++;
    }
  });

  // Run for a short time
  std::this_thread::sleep_for(50ms);
  keep_running = false;

  for (auto & t : readers) {
    t.join();
  }
  writer.join();

  // Ensure both readers and writers made progress
  EXPECT_GT(read_count.load(), 0) << "Readers made no progress - possible deadlock";
  EXPECT_GT(write_count.load(), 0) << "Writer made no progress - possible deadlock";
}

// ============================================================================
// Validation Tests
// ============================================================================

TEST_F(EntityCacheTest, ValidateReturnsEmptyForConsistentCache) {
  cache_.update_all(areas_, components_, apps_, {});

  auto error = cache_.validate();
  EXPECT_TRUE(error.empty()) << "Validation failed: " << error;
}

TEST_F(EntityCacheTest, GetStatsReturnsCorrectCounts) {
  cache_.update_all(areas_, components_, apps_, {});

  auto stats = cache_.get_stats();
  EXPECT_EQ(stats.area_count, 2);
  EXPECT_EQ(stats.component_count, 2);
  EXPECT_EQ(stats.app_count, 3);
  EXPECT_GT(stats.total_operations, 0ul);
}

// ============================================================================
// AggregationService Tests
// ============================================================================

class AggregationServiceTest : public ::testing::Test {
 protected:
  void SetUp() override {
    // Setup test entities with operations using helper functions (C++17 compatible)
    areas_ = {make_area("navigation", "Navigation")};

    components_.push_back(make_component("nav_stack", "Nav Stack", "navigation"));
    components_[0].services = {make_service("get_state", "/nav/get_state")};

    apps_ = {
        make_app_minimal("controller", "nav_stack"),
        make_app_minimal("planner", "nav_stack"),
    };
    apps_[0].services = {make_service("follow", "/nav/follow")};
    apps_[1].services = {make_service("plan", "/nav/plan")};
    apps_[1].actions = {make_action("compute", "/nav/compute")};

    cache_.update_all(areas_, components_, apps_, {});
    service_ = std::make_unique<AggregationService>(&cache_);
  }

  ThreadSafeEntityCache cache_;
  std::unique_ptr<AggregationService> service_;
  std::vector<Area> areas_;
  std::vector<Component> components_;
  std::vector<App> apps_;
};

TEST_F(AggregationServiceTest, AppReturnsOwnOperationsOnly) {
  auto result = service_->get_operations(SovdEntityType::APP, "controller");

  EXPECT_FALSE(result.is_aggregated);
  EXPECT_EQ(result.services.size(), 1);
  EXPECT_EQ(result.services[0].name, "follow");
  EXPECT_EQ(result.source_ids.size(), 1);
}

TEST_F(AggregationServiceTest, ComponentAggregatesFromApps) {
  auto result = service_->get_operations(SovdEntityType::COMPONENT, "nav_stack");

  EXPECT_TRUE(result.is_aggregated);
  // Component's own (get_state) + controller (follow) + planner (plan)
  EXPECT_EQ(result.services.size(), 3);
  EXPECT_EQ(result.actions.size(), 1);       // compute
  EXPECT_GE(result.source_ids.size(), 3ul);  // component + 2 apps
}

TEST_F(AggregationServiceTest, AreaAggregatesFromComponents) {
  auto result = service_->get_operations(SovdEntityType::AREA, "navigation");

  EXPECT_TRUE(result.is_aggregated);
  EXPECT_GE(result.services.size(), 3ul);
  EXPECT_GE(result.source_ids.size(), 1ul);
}

TEST_F(AggregationServiceTest, XMedkitMetadataCorrect) {
  auto result = service_->get_operations(SovdEntityType::COMPONENT, "nav_stack");
  auto json = AggregationService::build_x_medkit(result);

  EXPECT_TRUE(json["aggregated"].get<bool>());
  EXPECT_FALSE(json["aggregation_sources"].empty());
  EXPECT_EQ(json["aggregation_level"], "component");
}

TEST_F(AggregationServiceTest, EmptyEntityReturnsEmptyResult) {
  auto result = service_->get_operations(SovdEntityType::COMPONENT, "unknown");

  EXPECT_TRUE(result.services.empty());
  EXPECT_TRUE(result.actions.empty());
  EXPECT_FALSE(result.is_aggregated);
}

TEST_F(AggregationServiceTest, SupportsOperationsCheckCorrect) {
  EXPECT_TRUE(AggregationService::supports_operations(SovdEntityType::COMPONENT));
  EXPECT_TRUE(AggregationService::supports_operations(SovdEntityType::APP));
  EXPECT_TRUE(AggregationService::supports_operations(SovdEntityType::FUNCTION));
  EXPECT_TRUE(AggregationService::supports_operations(SovdEntityType::AREA));
}

TEST_F(AggregationServiceTest, ShouldAggregateCheckCorrect) {
  EXPECT_TRUE(AggregationService::should_aggregate(SovdEntityType::COMPONENT));
  EXPECT_TRUE(AggregationService::should_aggregate(SovdEntityType::AREA));
  EXPECT_TRUE(AggregationService::should_aggregate(SovdEntityType::FUNCTION));
  EXPECT_FALSE(AggregationService::should_aggregate(SovdEntityType::APP));
}

TEST_F(AggregationServiceTest, GetOperationsByIdAutoDetectsType) {
  auto result = service_->get_operations_by_id("nav_stack");
  EXPECT_EQ(result.aggregation_level, "component");

  result = service_->get_operations_by_id("controller");
  EXPECT_EQ(result.aggregation_level, "app");
}

// ============================================================================
// Data Aggregation Tests
// ============================================================================

class DataAggregationTest : public ::testing::Test {
 protected:
  void SetUp() override {
    // Create a hierarchy:
    // - Area: perception
    //   - Component: sensor_stack
    //     - App: camera_driver (publishes /camera/image, subscribes /camera/enable)
    //     - App: lidar_proc (publishes /lidar/points, subscribes /camera/image)
    areas_ = {make_area("perception", "Perception Area")};

    components_.push_back(make_component("sensor_stack", "Sensor Stack", "perception"));
    components_[0].topics.publishes = {"/common/diagnostics"};
    components_[0].topics.subscribes = {"/common/clock"};

    apps_.push_back(make_app_minimal("camera_driver", "sensor_stack"));
    apps_[0].topics.publishes = {"/camera/image", "/camera/info"};
    apps_[0].topics.subscribes = {"/camera/enable"};

    apps_.push_back(make_app_minimal("lidar_proc", "sensor_stack"));
    apps_[1].topics.publishes = {"/lidar/points"};
    apps_[1].topics.subscribes = {"/camera/image"};  // Same topic that camera_driver publishes

    cache_.update_all(areas_, components_, apps_, {});
  }

  ThreadSafeEntityCache cache_;
  std::vector<Area> areas_;
  std::vector<Component> components_;
  std::vector<App> apps_;
};

TEST_F(DataAggregationTest, AppDataReturnsOwnTopicsOnly) {
  auto result = cache_.get_app_data("camera_driver");

  EXPECT_FALSE(result.is_aggregated);
  EXPECT_EQ(result.aggregation_level, "app");
  EXPECT_EQ(result.source_ids.size(), 1);
  EXPECT_EQ(result.source_ids[0], "camera_driver");

  // camera_driver: 2 publishes + 1 subscribes = 3 topics
  EXPECT_EQ(result.topics.size(), 3);

  // Check at least one publish and one subscribe
  bool has_publish = false;
  bool has_subscribe = false;
  for (const auto & topic : result.topics) {
    if (topic.direction == "publish") {
      has_publish = true;
    }
    if (topic.direction == "subscribe") {
      has_subscribe = true;
    }
  }
  EXPECT_TRUE(has_publish);
  EXPECT_TRUE(has_subscribe);
}

TEST_F(DataAggregationTest, ComponentDataAggregatesFromHostedApps) {
  auto result = cache_.get_component_data("sensor_stack");

  EXPECT_TRUE(result.is_aggregated);
  EXPECT_EQ(result.aggregation_level, "component");
  // Sources: component + 2 apps = 3
  EXPECT_EQ(result.source_ids.size(), 3);

  // Component topics: /common/diagnostics (pub), /common/clock (sub)
  // camera_driver: /camera/image (pub), /camera/info (pub), /camera/enable (sub)
  // lidar_proc: /lidar/points (pub), /camera/image (sub - already seen as pub)
  // Total unique: 6 topics (/camera/image appears twice, merged as "both")
  EXPECT_GE(result.topics.size(), 5);
}

TEST_F(DataAggregationTest, DirectionMergesToBothWhenPubAndSub) {
  auto result = cache_.get_component_data("sensor_stack");

  // /camera/image is published by camera_driver and subscribed by lidar_proc
  // Should be merged to direction="both"
  bool found_camera_image = false;
  for (const auto & topic : result.topics) {
    if (topic.name == "/camera/image") {
      found_camera_image = true;
      EXPECT_EQ(topic.direction, "both");
      break;
    }
  }
  EXPECT_TRUE(found_camera_image);
}

TEST_F(DataAggregationTest, AreaDataAggregatesFromAllComponents) {
  auto result = cache_.get_area_data("perception");

  EXPECT_TRUE(result.is_aggregated);
  EXPECT_EQ(result.aggregation_level, "area");
  EXPECT_GE(result.source_ids.size(), 1);  // At least area itself

  // Should contain topics from sensor_stack and its apps
  EXPECT_GE(result.topics.size(), 5);
}

TEST_F(DataAggregationTest, GetEntityDataAutoDetectsType) {
  auto app_result = cache_.get_entity_data("camera_driver");
  EXPECT_EQ(app_result.aggregation_level, "app");

  auto comp_result = cache_.get_entity_data("sensor_stack");
  EXPECT_EQ(comp_result.aggregation_level, "component");

  auto area_result = cache_.get_entity_data("perception");
  EXPECT_EQ(area_result.aggregation_level, "area");
}

TEST_F(DataAggregationTest, UnknownEntityReturnsEmptyData) {
  auto result = cache_.get_entity_data("nonexistent");

  EXPECT_TRUE(result.topics.empty());
  EXPECT_TRUE(result.source_ids.empty());
  EXPECT_TRUE(result.aggregation_level.empty());
}

// =============================================================================
// Member resolution
//
// A grouping entity - an Area, a Function, or a Component with children - owns
// no resources beyond what its members provide. Resolving an item to its owner
// and listing the grouping's items therefore have to walk the same graph, or
// the gateway advertises what it will not serve. These cases pin that one walk.
// =============================================================================

class MemberResolutionTest : public ::testing::Test {
 protected:
  /// Members compared as a set: the walk fixes no order and no caller needs one.
  static std::vector<std::string> sorted(std::vector<std::string> v) {
    std::sort(v.begin(), v.end());
    return v;
  }

  ThreadSafeEntityCache cache_;
};

TEST_F(MemberResolutionTest, AreaMembersAreItsComponentsAndTheirApps) {
  std::vector<Area> areas{make_area("vehicle", "Vehicle")};
  std::vector<Component> components{
      make_component("primary-ecu", "Primary", "vehicle"),
      make_component("secondary-ecu", "Secondary", "vehicle"),
  };
  std::vector<App> apps{
      make_app("temp_sensor", "Temp", "primary-ecu"),
      make_app("pressure_sensor", "Pressure", "secondary-ecu"),
  };
  cache_.update_all(areas, components, apps, {});

  EXPECT_EQ(sorted(cache_.get_members(SovdEntityType::AREA, "vehicle")),
            sorted({"primary-ecu", "secondary-ecu", "temp_sensor", "pressure_sensor"}));
}

TEST_F(MemberResolutionTest, AreaMembersDescendIntoSubareas) {
  // The operations walk that predates this one stops at the area's own
  // components, so a subarea's apps were invisible to any caller that asked the
  // parent area what it contains.
  Area parent = make_area("vehicle", "Vehicle");
  Area child = make_area("powertrain", "Powertrain");
  child.parent_area_id = "vehicle";

  std::vector<Component> components{make_component("engine-ecu", "Engine", "powertrain")};
  std::vector<App> apps{make_app("rpm_sensor", "RPM", "engine-ecu")};
  cache_.update_all({parent, child}, components, apps, {});

  EXPECT_EQ(sorted(cache_.get_members(SovdEntityType::AREA, "vehicle")), sorted({"engine-ecu", "rpm_sensor"}));
}

TEST_F(MemberResolutionTest, FunctionMembersIncludeHostsThatAreComponents) {
  // A Function host id may name an App or a Component. Only the App half was
  // indexed, and a Component host was dropped without a word, so a Function
  // hosting a Component reported none of that Component's resources.
  std::vector<Area> areas{make_area("vehicle", "Vehicle")};
  std::vector<Component> components{make_component("brake-ecu", "Brake", "vehicle")};
  std::vector<App> apps{
      make_app("actuator", "Actuator", "brake-ecu"),
      make_app("temp_sensor", "Temp", "brake-ecu"),
  };

  Function func;
  func.id = "vehicle_health";
  func.hosts = {"temp_sensor", "brake-ecu"};
  cache_.update_all(areas, components, apps, {func});

  const auto members = sorted(cache_.get_members(SovdEntityType::FUNCTION, "vehicle_health"));
  for (const char * expected : {"actuator", "brake-ecu", "temp_sensor"}) {
    EXPECT_NE(std::find(members.begin(), members.end(), expected), members.end())
        << "function member missing: " << expected;
  }
}

TEST_F(MemberResolutionTest, ComponentIsAMemberOfItselfAndCarriesItsChildren) {
  // Two things this pins. A Component is its own member because, unlike an Area
  // or a Function, it can expose services and actions directly - the operations
  // aggregation already counts those before its apps. And a parent Component's
  // children were not reachable from the cache at all before: nothing indexed
  // that direction.
  std::vector<Area> areas{make_area("vehicle", "Vehicle")};
  Component parent = make_component("vehicle-ecu", "Vehicle ECU", "vehicle");
  parent.services = {make_service("reset", "/vehicle/reset")};
  Component child = make_component("brake-ecu", "Brake ECU", "vehicle");
  child.parent_component_id = "vehicle-ecu";

  std::vector<App> apps{make_app("actuator", "Actuator", "brake-ecu")};
  cache_.update_all(areas, {parent, child}, apps, {});

  EXPECT_EQ(sorted(cache_.get_members(SovdEntityType::COMPONENT, "vehicle-ecu")),
            sorted({"vehicle-ecu", "brake-ecu", "actuator"}));
}

TEST_F(MemberResolutionTest, ComponentParentCycleTerminates) {
  // Parent-component cycles are representable: the manifest validator checks
  // only that a parent exists, not that the graph is acyclic, and the
  // aggregation classifier handles cycles explicitly because they occur. A walk
  // without a visited set would not return.
  std::vector<Area> areas{make_area("vehicle", "Vehicle")};
  Component first = make_component("ecu-a", "A", "vehicle");
  first.parent_component_id = "ecu-b";
  Component second = make_component("ecu-b", "B", "vehicle");
  second.parent_component_id = "ecu-a";
  cache_.update_all(areas, {first, second}, {}, {});

  EXPECT_EQ(sorted(cache_.get_members(SovdEntityType::COMPONENT, "ecu-a")), sorted({"ecu-a", "ecu-b"}));
}

TEST_F(MemberResolutionTest, LeafReturnsOnlyItself) {
  std::vector<Area> areas{make_area("vehicle", "Vehicle")};
  std::vector<Component> components{make_component("primary-ecu", "Primary", "vehicle")};
  std::vector<App> apps{make_app("temp_sensor", "Temp", "primary-ecu")};
  cache_.update_all(areas, components, apps, {});

  EXPECT_EQ(cache_.get_members(SovdEntityType::APP, "temp_sensor"), std::vector<std::string>{"temp_sensor"});
}

TEST_F(MemberResolutionTest, UnknownGroupingHasNoMembers) {
  cache_.update_all({}, {}, {}, {});
  EXPECT_TRUE(cache_.get_members(SovdEntityType::AREA, "nope").empty());
}

int main(int argc, char ** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
