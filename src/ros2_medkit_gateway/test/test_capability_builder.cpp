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

#include "ros2_medkit_gateway/core/http/handlers/capability_builder.hpp"

using namespace ros2_medkit_gateway::handlers;
using Cap = CapabilityBuilder::Capability;

// =============================================================================
// CapabilityBuilder Tests
// =============================================================================

TEST(CapabilityBuilderTest, BuildsCorrectCapabilities) {
  std::vector<Cap> caps = {Cap::DATA, Cap::OPERATIONS};

  auto result = CapabilityBuilder::build_capabilities("components", "test-comp", caps);

  ASSERT_EQ(result.size(), 2u);
  EXPECT_EQ(result[0].name, "data");
  EXPECT_EQ(result[0].href, "/api/v1/components/test-comp/data");
  EXPECT_EQ(result[1].name, "operations");
  EXPECT_EQ(result[1].href, "/api/v1/components/test-comp/operations");
}

TEST(CapabilityBuilderTest, BuildsEmptyArray) {
  std::vector<Cap> caps = {};

  auto result = CapabilityBuilder::build_capabilities("areas", "test-area", caps);

  EXPECT_TRUE(result.empty());
}

TEST(CapabilityBuilderTest, BuildsAllCapabilities) {
  std::vector<Cap> caps = {Cap::DATA,          Cap::OPERATIONS, Cap::CONFIGURATIONS, Cap::FAULTS, Cap::SUBAREAS,
                           Cap::SUBCOMPONENTS, Cap::COMPONENTS, Cap::CONTAINS,       Cap::HOSTS};

  auto result = CapabilityBuilder::build_capabilities("entities", "test-id", caps);

  EXPECT_EQ(result.size(), 9u);
}

TEST(CapabilityBuilderTest, CapabilityToNameReturnsCorrectStrings) {
  EXPECT_EQ(CapabilityBuilder::capability_to_name(Cap::DATA), "data");
  EXPECT_EQ(CapabilityBuilder::capability_to_name(Cap::DATA_CATEGORIES), "data-categories");
  EXPECT_EQ(CapabilityBuilder::capability_to_name(Cap::DATA_GROUPS), "data-groups");
  EXPECT_EQ(CapabilityBuilder::capability_to_name(Cap::FAULT_TRIGGERS), "fault-triggers");
  EXPECT_EQ(CapabilityBuilder::capability_to_name(Cap::OPERATIONS), "operations");
  EXPECT_EQ(CapabilityBuilder::capability_to_name(Cap::CONFIGURATIONS), "configurations");
  EXPECT_EQ(CapabilityBuilder::capability_to_name(Cap::FAULTS), "faults");
  EXPECT_EQ(CapabilityBuilder::capability_to_name(Cap::SUBAREAS), "subareas");
  EXPECT_EQ(CapabilityBuilder::capability_to_name(Cap::SUBCOMPONENTS), "subcomponents");
  // `/areas/{area_id}/components`, the route an area really serves. The
  // `related-components` / `related-apps` enumerators this replaced named
  // segments no entity type registers.
  EXPECT_EQ(CapabilityBuilder::capability_to_name(Cap::COMPONENTS), "components");
  EXPECT_EQ(CapabilityBuilder::capability_to_name(Cap::CONTAINS), "contains");
  EXPECT_EQ(CapabilityBuilder::capability_to_name(Cap::HOSTS), "hosts");
  EXPECT_EQ(CapabilityBuilder::capability_to_name(Cap::LOGS), "logs");
  EXPECT_EQ(CapabilityBuilder::capability_to_name(Cap::STATUS), "status");
}

// `-Werror=switch-enum` already refuses an enumerator with no arm, so what this
// adds is the case the compiler cannot see: an arm that *is* present but
// resolves to the `"unknown"` placeholder the `default:` returns, which would
// render an href of `/api/v1/{type}/{id}/unknown`.
//
// Scope: the name table only. An enumerator that no handler list uses is dead
// and invisible to this and every other check - see the header comment for why
// pinning that would cost a duplicated fact.
TEST(CapabilityBuilderTest, NamesEveryEnumerator) {
  // Range end is the last enumerator; extend it when the enum grows - the
  // static_assert below is what makes forgetting that fail to compile.
  constexpr auto kLast = Cap::STATUS;
  static_assert(static_cast<int>(kLast) == 21, "Capability gained or lost an enumerator - update kLast");

  for (int i = 0; i <= static_cast<int>(kLast); ++i) {
    const auto cap = static_cast<Cap>(i);
    EXPECT_NE(CapabilityBuilder::capability_to_name(cap), "unknown") << "enumerator " << i << " has no name arm";
    EXPECT_FALSE(CapabilityBuilder::capability_to_name(cap).empty()) << "enumerator " << i;
  }
}

TEST(CapabilityBuilderTest, CapabilityToPathMatchesName) {
  // For all capabilities, the path segment matches the name
  EXPECT_EQ(CapabilityBuilder::capability_to_path(Cap::DATA), CapabilityBuilder::capability_to_name(Cap::DATA));
  EXPECT_EQ(CapabilityBuilder::capability_to_path(Cap::COMPONENTS),
            CapabilityBuilder::capability_to_name(Cap::COMPONENTS));
  EXPECT_EQ(CapabilityBuilder::capability_to_path(Cap::LOGS), CapabilityBuilder::capability_to_name(Cap::LOGS));
}

TEST(CapabilityBuilderTest, BuildsForDifferentEntityTypes) {
  std::vector<Cap> caps = {Cap::DATA};

  auto areas_result = CapabilityBuilder::build_capabilities("areas", "a1", caps);
  auto components_result = CapabilityBuilder::build_capabilities("components", "c1", caps);
  auto apps_result = CapabilityBuilder::build_capabilities("apps", "app1", caps);
  auto functions_result = CapabilityBuilder::build_capabilities("functions", "f1", caps);

  EXPECT_EQ(areas_result[0].href, "/api/v1/areas/a1/data");
  EXPECT_EQ(components_result[0].href, "/api/v1/components/c1/data");
  EXPECT_EQ(apps_result[0].href, "/api/v1/apps/app1/data");
  EXPECT_EQ(functions_result[0].href, "/api/v1/functions/f1/data");
}

// =============================================================================
// LinksBuilder Tests
// =============================================================================

TEST(LinksBuilderTest, BuildsLinks) {
  LinksBuilder builder;
  auto links = builder.self("/areas/test").parent("/areas").add("custom", "/custom/link").build();

  EXPECT_EQ(links["self"], "/areas/test");
  EXPECT_EQ(links["parent"], "/areas");
  EXPECT_EQ(links["custom"], "/custom/link");
}

TEST(LinksBuilderTest, BuildsEmptyLinks) {
  LinksBuilder builder;
  auto links = builder.build();

  EXPECT_TRUE(links.is_object());
  EXPECT_EQ(links.size(), 0);
}

TEST(LinksBuilderTest, SelfLinkOnly) {
  LinksBuilder builder;
  auto links = builder.self("/components/my-comp").build();

  EXPECT_EQ(links.size(), 1);
  EXPECT_EQ(links["self"], "/components/my-comp");
}

TEST(LinksBuilderTest, CollectionLink) {
  LinksBuilder builder;
  auto links = builder.self("/apps/app1").collection("/apps").build();

  EXPECT_EQ(links["self"], "/apps/app1");
  EXPECT_EQ(links["collection"], "/apps");
}

TEST(LinksBuilderTest, FluentChaining) {
  LinksBuilder builder;
  auto links = builder.self("/functions/f1")
                   .collection("/functions")
                   .parent("/functions")
                   .add("hosts", "/functions/f1/hosts")
                   .add("depends-on", "/functions/f2")
                   .build();

  EXPECT_EQ(links.size(), 5);
  EXPECT_EQ(links["self"], "/functions/f1");
  EXPECT_EQ(links["collection"], "/functions");
  EXPECT_EQ(links["parent"], "/functions");
  EXPECT_EQ(links["hosts"], "/functions/f1/hosts");
  EXPECT_EQ(links["depends-on"], "/functions/f2");
}

TEST(LinksBuilderTest, OverwriteLink) {
  LinksBuilder builder;
  auto links = builder.self("/old").self("/new").build();

  EXPECT_EQ(links["self"], "/new");
}

TEST(LinksBuilderTest, MultipleCustomLinks) {
  LinksBuilder builder;
  auto links = builder.self("/areas/area1")
                   .add("subareas", "/areas/area1/subareas")
                   .add("related-components", "/areas/area1/related-components")
                   .add("area", "/areas/parent")
                   .build();

  EXPECT_EQ(links.size(), 4);
  EXPECT_EQ(links["subareas"], "/areas/area1/subareas");
  EXPECT_EQ(links["related-components"], "/areas/area1/related-components");
  EXPECT_EQ(links["area"], "/areas/parent");
}
