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
#include <condition_variable>
#include <mutex>
#include <thread>
#include <vector>

#include "ros2_medkit_gateway/core/resource_sampler.hpp"

using namespace ros2_medkit_gateway;

TEST(ResourceSamplerRegistryTest, RegisterAndLookupBuiltinSampler) {
  ResourceSamplerRegistry registry;
  registry.register_sampler(
      "data",
      [](const std::string &, const std::string &) -> tl::expected<nlohmann::json, std::string> {
        return nlohmann::json{{"test", true}};
      },
      true);

  EXPECT_TRUE(registry.has_sampler("data"));
  auto sampler = registry.get_sampler("data");
  ASSERT_TRUE(sampler.has_value());
  auto result = (*sampler)("entity1", "/topic");
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ((*result)["test"], true);
}

TEST(ResourceSamplerRegistryTest, RegisterAndLookupVendorExtensionSampler) {
  ResourceSamplerRegistry registry;
  registry.register_sampler("x-medkit-metrics",
                            [](const std::string &, const std::string &) -> tl::expected<nlohmann::json, std::string> {
                              return nlohmann::json{{"cpu", 42}};
                            });

  EXPECT_TRUE(registry.has_sampler("x-medkit-metrics"));
}

TEST(ResourceSamplerRegistryTest, HasSamplerReturnsFalseForUnregistered) {
  ResourceSamplerRegistry registry;
  EXPECT_FALSE(registry.has_sampler("nonexistent"));
  EXPECT_FALSE(registry.get_sampler("nonexistent").has_value());
}

TEST(ResourceSamplerRegistryTest, BuiltinOverwritesExisting) {
  ResourceSamplerRegistry registry;
  registry.register_sampler(
      "data",
      [](const std::string &, const std::string &) -> tl::expected<nlohmann::json, std::string> {
        return nlohmann::json{{"version", 1}};
      },
      true);
  registry.register_sampler(
      "data",
      [](const std::string &, const std::string &) -> tl::expected<nlohmann::json, std::string> {
        return nlohmann::json{{"version", 2}};
      },
      true);

  auto sampler = registry.get_sampler("data");
  ASSERT_TRUE(sampler.has_value());
  auto result = (*sampler)("e", "");
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ((*result)["version"], 2);
}

TEST(ResourceSamplerRegistryTest, RejectDuplicatePluginRegistration) {
  ResourceSamplerRegistry registry;
  registry.register_sampler("x-medkit-metrics",
                            [](const std::string &, const std::string &) -> tl::expected<nlohmann::json, std::string> {
                              return nlohmann::json{};
                            });

  EXPECT_THROW(registry.register_sampler(
                   "x-medkit-metrics",
                   [](const std::string &, const std::string &) -> tl::expected<nlohmann::json, std::string> {
                     return nlohmann::json{};
                   }),
               std::runtime_error);
}

TEST(ResourceSamplerRegistryTest, RejectPluginRegistrationWithoutXPrefix) {
  ResourceSamplerRegistry registry;
  EXPECT_THROW(registry.register_sampler(
                   "data",
                   [](const std::string &, const std::string &) -> tl::expected<nlohmann::json, std::string> {
                     return nlohmann::json{};
                   }),
               std::runtime_error);
}

TEST(ResourceSamplerRegistryTest, ConcurrentRegisterAndLookup) {
  ResourceSamplerRegistry registry;
  std::vector<std::thread> threads;
  threads.reserve(10);

  for (int i = 0; i < 10; ++i) {
    threads.emplace_back([&registry, i]() {
      registry.register_sampler(
          "collection_" + std::to_string(i),
          [i](const std::string &, const std::string &) -> tl::expected<nlohmann::json, std::string> {
            return nlohmann::json{{"i", i}};
          },
          true);
    });
  }
  for (auto & t : threads) {
    t.join();
  }

  for (int i = 0; i < 10; ++i) {
    EXPECT_TRUE(registry.has_sampler("collection_" + std::to_string(i)));
  }
}

TEST(ResourceSamplerRegistryTest, RemoveSamplerMakesItUnavailable) {
  ResourceSamplerRegistry registry;
  registry.register_sampler("x-medkit-metrics",
                            [](const std::string &, const std::string &) -> tl::expected<nlohmann::json, std::string> {
                              return nlohmann::json{{"cpu", 42}};
                            });
  ASSERT_TRUE(registry.has_sampler("x-medkit-metrics"));

  registry.remove_sampler("x-medkit-metrics");

  EXPECT_FALSE(registry.has_sampler("x-medkit-metrics"));
  EXPECT_FALSE(registry.get_sampler("x-medkit-metrics").has_value());
}

TEST(ResourceSamplerRegistryTest, RemoveSamplerLeavesBuiltinInPlace) {
  ResourceSamplerRegistry registry;
  registry.register_sampler(
      "data",
      [](const std::string &, const std::string &) -> tl::expected<nlohmann::json, std::string> {
        return nlohmann::json{{"builtin", true}};
      },
      true);
  ASSERT_TRUE(registry.has_sampler("data"));

  registry.remove_sampler("data");

  // Built-ins are owned by the gateway node, not by any plugin - removal
  // must not silently take out a core feature.
  EXPECT_TRUE(registry.has_sampler("data"));
  auto sampler = registry.get_sampler("data");
  ASSERT_TRUE(sampler.has_value());
  auto result = (*sampler)("e", "");
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ((*result)["builtin"], true);
}

// A stronger version of the no-op check: removing an unregistered name must
// not disturb an *unrelated*, already-registered collection - the collection
// must remain present, and callable, afterward. Asserting only that the
// never-registered name stays absent (as the original version of this test
// did) would pass even if remove_sampler() were a complete no-op, or even if
// it erased the wrong entry - it exercises nothing about remove_sampler()'s
// actual behavior.
TEST(ResourceSamplerRegistryTest, RemoveSamplerOnUnregisteredCollectionLeavesUnrelatedSamplerIntact) {
  ResourceSamplerRegistry registry;
  registry.register_sampler("x-medkit-metrics",
                            [](const std::string &, const std::string &) -> tl::expected<nlohmann::json, std::string> {
                              return nlohmann::json{{"cpu", 42}};
                            });
  ASSERT_TRUE(registry.has_sampler("x-medkit-metrics"));

  EXPECT_NO_THROW(registry.remove_sampler("x-never-registered"));

  EXPECT_FALSE(registry.has_sampler("x-never-registered"));
  ASSERT_TRUE(registry.has_sampler("x-medkit-metrics"));
  auto sampler = registry.get_sampler("x-medkit-metrics");
  ASSERT_TRUE(sampler.has_value());
  auto result = (*sampler)("e", "");
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ((*result)["cpu"], 42);
}

// Reproduces the actual use-after-free gap left by cb9ae513: get_sampler()
// hands out a copy of the callable (as SubscriptionTransportProvider::start()
// does, holding it for the life of a cyclic subscription). Removing the
// collection from the registry only stops *new* lookups - it must also make
// every *already-copied* callable refuse to call through to the (possibly
// now-destroyed) plugin instance.
TEST(ResourceSamplerRegistryTest, CopyObtainedBeforeRemovalRefusesToCallAfterRemoval) {
  ResourceSamplerRegistry registry;
  bool underlying_called = false;
  registry.register_sampler(
      "x-medkit-metrics",
      [&underlying_called](const std::string &, const std::string &) -> tl::expected<nlohmann::json, std::string> {
        underlying_called = true;
        return nlohmann::json{{"cpu", 42}};
      });

  // Simulate a cyclic subscription created before teardown: it obtains its
  // own copy of the sampler and holds onto it independently of the registry.
  auto copy = registry.get_sampler("x-medkit-metrics");
  ASSERT_TRUE(copy.has_value());

  // Simulate plugin teardown.
  registry.remove_sampler("x-medkit-metrics");
  ASSERT_FALSE(registry.has_sampler("x-medkit-metrics"));

  // The copy the "subscription" already holds must NOT call into the
  // (now-torn-down) plugin's callable - it must fail safely instead.
  auto result = (*copy)("entity1", "/topic");
  EXPECT_FALSE(result.has_value());
  EXPECT_FALSE(underlying_called) << "removed sampler copy must not invoke the underlying plugin callable";
  if (!result.has_value()) {
    EXPECT_FALSE(result.error().empty());
  }
}

// A copy of a *built-in* sampler must keep working even after other plugin
// samplers are torn down elsewhere in the registry - built-ins are never
// removed, so their copies never observe a "removed" state.
TEST(ResourceSamplerRegistryTest, CopyOfBuiltinSamplerKeepsWorkingAfterUnrelatedRemoval) {
  ResourceSamplerRegistry registry;
  registry.register_sampler(
      "data",
      [](const std::string &, const std::string &) -> tl::expected<nlohmann::json, std::string> {
        return nlohmann::json{{"builtin", true}};
      },
      true);
  registry.register_sampler("x-medkit-metrics",
                            [](const std::string &, const std::string &) -> tl::expected<nlohmann::json, std::string> {
                              return nlohmann::json{{"cpu", 42}};
                            });

  auto builtin_copy = registry.get_sampler("data");
  ASSERT_TRUE(builtin_copy.has_value());

  registry.remove_sampler("x-medkit-metrics");
  // remove_sampler on the builtin itself must also be a no-op (existing
  // behavior), so a copy taken beforehand keeps working regardless.
  registry.remove_sampler("data");

  auto result = (*builtin_copy)("e", "");
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ((*result)["builtin"], true);
}

TEST(ResourceSamplerRegistryTest, UpdatesSamplerRegisteredAsBuiltin) {
  ResourceSamplerRegistry registry;
  registry.register_sampler(
      "updates",
      [](const std::string & /*entity_id*/,
         const std::string & resource_path) -> tl::expected<nlohmann::json, std::string> {
        if (resource_path == "known-pkg") {
          return nlohmann::json{{"status", "inProgress"}, {"progress", 42}};
        }
        return tl::make_unexpected("Update not found: " + resource_path);
      },
      true);

  EXPECT_TRUE(registry.has_sampler("updates"));
  auto sampler = registry.get_sampler("updates");
  ASSERT_TRUE(sampler.has_value());

  auto result = (*sampler)("any-entity", "known-pkg");
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ((*result)["status"], "inProgress");
  EXPECT_EQ((*result)["progress"], 42);

  auto err = (*sampler)("any-entity", "unknown-pkg");
  ASSERT_FALSE(err.has_value());
}

// Closes the gap the liveness flag alone left open: a call already inside the
// underlying callable (the check-then-call race - see the class comment on
// ResourceSamplerRegistry) must finish before remove_sampler() returns, not
// merely be told "you're removed" on its next attempt. A regression here
// would mean PluginManager::disable_plugin's subsequent plugin.reset() could
// free the plugin object while this thread is still executing inside it.
TEST(ResourceSamplerRegistryTest, RemoveSamplerBlocksUntilInFlightCallCompletes) {
  ResourceSamplerRegistry registry;

  std::mutex state_mutex;
  std::condition_variable call_started_cv;
  bool call_started = false;
  std::atomic<bool> call_finished{false};

  registry.register_sampler("x-medkit-metrics",
                            [&](const std::string &, const std::string &) -> tl::expected<nlohmann::json, std::string> {
                              {
                                std::lock_guard<std::mutex> lock(state_mutex);
                                call_started = true;
                              }
                              call_started_cv.notify_one();
                              // Hold the call open long enough that the test thread's
                              // remove_sampler() call below is guaranteed to observe it in flight.
                              std::this_thread::sleep_for(std::chrono::milliseconds(200));
                              call_finished.store(true, std::memory_order_release);
                              return nlohmann::json{{"cpu", 42}};
                            });

  // Simulate a cyclic subscription's transport holding its own copy of the
  // callable, obtained before teardown starts.
  auto copy = registry.get_sampler("x-medkit-metrics");
  ASSERT_TRUE(copy.has_value());

  bool invocation_succeeded = false;
  std::thread invoker([&]() {
    auto invocation_result = (*copy)("entity1", "/topic");
    invocation_succeeded = invocation_result.has_value();
  });

  {
    std::unique_lock<std::mutex> lock(state_mutex);
    call_started_cv.wait(lock, [&] {
      return call_started;
    });
  }

  // The invocation is now inside the underlying callable (sleeping).
  // remove_sampler() must not return until that call has completed.
  registry.remove_sampler("x-medkit-metrics");
  EXPECT_TRUE(call_finished.load(std::memory_order_acquire))
      << "remove_sampler() returned before the in-flight call finished draining - "
         "the caller (PluginManager::disable_plugin) could have freed the plugin "
         "while this call was still executing inside it";

  invoker.join();
  EXPECT_TRUE(invocation_succeeded);
}

// --- resource-path support declaration ---

TEST(ResourceSamplerRegistryTest, UndeclaredSamplerDoesNotHonourResourcePath) {
  ResourceSamplerRegistry registry;
  registry.register_sampler("x-medkit-metrics",
                            [](const std::string &, const std::string &) -> tl::expected<nlohmann::json, std::string> {
                              return nlohmann::json{{"cpu", 42}};
                            });

  EXPECT_FALSE(registry.honours_resource_path("x-medkit-metrics"));
}

TEST(ResourceSamplerRegistryTest, DeclaredSamplerHonoursResourcePath) {
  ResourceSamplerRegistry registry;
  registry.register_sampler(
      "x-medkit-metrics",
      [](const std::string &, const std::string &) -> tl::expected<nlohmann::json, std::string> {
        return nlohmann::json{{"cpu", 42}};
      },
      /*is_builtin=*/false, /*honours_resource_path=*/true);

  EXPECT_TRUE(registry.honours_resource_path("x-medkit-metrics"));
}

TEST(ResourceSamplerRegistryTest, BuiltinSamplerCanHonourResourcePath) {
  ResourceSamplerRegistry registry;
  registry.register_sampler(
      "data",
      [](const std::string &, const std::string &) -> tl::expected<nlohmann::json, std::string> {
        return nlohmann::json{{"value", 1}};
      },
      /*is_builtin=*/true, /*honours_resource_path=*/true);
  registry.register_sampler(
      "configurations",
      [](const std::string &, const std::string &) -> tl::expected<nlohmann::json, std::string> {
        return nlohmann::json{{"items", nlohmann::json::array()}};
      },
      /*is_builtin=*/true);

  EXPECT_TRUE(registry.honours_resource_path("data"));
  EXPECT_FALSE(registry.honours_resource_path("configurations"));
}

TEST(ResourceSamplerRegistryTest, UnregisteredCollectionDoesNotHonourResourcePath) {
  ResourceSamplerRegistry registry;
  EXPECT_FALSE(registry.honours_resource_path("nonexistent"));
}

TEST(ResourceSamplerRegistryTest, ReregisteringBuiltinReplacesResourcePathDeclaration) {
  ResourceSamplerRegistry registry;
  registry.register_sampler(
      "data",
      [](const std::string &, const std::string &) -> tl::expected<nlohmann::json, std::string> {
        return nlohmann::json{{"version", 1}};
      },
      /*is_builtin=*/true, /*honours_resource_path=*/true);
  ASSERT_TRUE(registry.honours_resource_path("data"));

  registry.register_sampler(
      "data",
      [](const std::string &, const std::string &) -> tl::expected<nlohmann::json, std::string> {
        return nlohmann::json{{"version", 2}};
      },
      /*is_builtin=*/true);

  EXPECT_FALSE(registry.honours_resource_path("data"));
}

TEST(ResourceSamplerRegistryTest, RemovedSamplerNoLongerHonoursResourcePath) {
  ResourceSamplerRegistry registry;
  registry.register_sampler(
      "x-medkit-metrics",
      [](const std::string &, const std::string &) -> tl::expected<nlohmann::json, std::string> {
        return nlohmann::json{{"cpu", 42}};
      },
      /*is_builtin=*/false, /*honours_resource_path=*/true);
  ASSERT_TRUE(registry.honours_resource_path("x-medkit-metrics"));

  registry.remove_sampler("x-medkit-metrics");

  EXPECT_FALSE(registry.honours_resource_path("x-medkit-metrics"));
}
