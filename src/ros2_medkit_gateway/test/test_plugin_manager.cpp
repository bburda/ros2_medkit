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
#include <httplib.h>

#include <atomic>
#include <chrono>
#include <thread>

#include "ros2_medkit_gateway/core/plugins/plugin_manager.hpp"
#include "ros2_medkit_gateway/core/providers/introspection_provider.hpp"
#include "ros2_medkit_gateway/plugins/ros_plugin_context.hpp"

using namespace ros2_medkit_gateway;
// json alias is already imported by the `using namespace` above (defined in
// discovery/models/common.hpp). A local `using json = nlohmann::json;` would
// shadow it and trip clang-diagnostic-shadow under clang-tidy.

/// Minimal mock plugin for compile-time testing (no .so needed)
class MockPlugin : public GatewayPlugin, public UpdateProvider, public IntrospectionProvider {
 public:
  std::string name() const override {
    return "mock";
  }
  void configure(const json & config) override {
    configured_ = true;
    config_ = config;
  }
  void shutdown() override {
    shutdown_called_ = true;
  }

  // UpdateProvider
  tl::expected<std::vector<std::string>, UpdateBackendErrorInfo>
  list_updates(const UpdateFilter & /*filter*/) override {
    return std::vector<std::string>{};
  }
  tl::expected<dto::UpdateDetail, UpdateBackendErrorInfo> get_update(const std::string & /*id*/) override {
    return tl::make_unexpected(UpdateBackendErrorInfo{UpdateBackendError::NotFound, "mock"});
  }
  tl::expected<void, UpdateBackendErrorInfo> register_update(const json & /*metadata*/) override {
    return {};
  }
  tl::expected<void, UpdateBackendErrorInfo> delete_update(const std::string & /*id*/) override {
    return {};
  }
  tl::expected<void, UpdateBackendErrorInfo> prepare(const std::string & /*id*/,
                                                     UpdateProgressReporter & /*reporter*/) override {
    return {};
  }
  tl::expected<void, UpdateBackendErrorInfo> execute(const std::string & /*id*/,
                                                     UpdateProgressReporter & /*reporter*/) override {
    return {};
  }
  tl::expected<bool, UpdateBackendErrorInfo> supports_automated(const std::string & /*id*/) override {
    return false;
  }

  // IntrospectionProvider
  IntrospectionResult introspect(const IntrospectionInput & /*input*/) override {
    return {};
  }

  bool configured_ = false;
  bool shutdown_called_ = false;
  json config_;
};

/// Plugin that only provides IntrospectionProvider
class MockIntrospectionOnly : public GatewayPlugin, public IntrospectionProvider {
 public:
  std::string name() const override {
    return "introspection_only";
  }
  void configure(const json & /*cfg*/) override {
  }
  IntrospectionResult introspect(const IntrospectionInput & /*input*/) override {
    return {};
  }
};

/// Plugin that throws during configure
class MockThrowingPlugin : public GatewayPlugin {
 public:
  std::string name() const override {
    return "throwing";
  }
  void configure(const json & /*cfg*/) override {
    throw std::runtime_error("configure failed");
  }
};

/// Plugin that throws during set_context
class MockThrowOnSetContext : public GatewayPlugin, public UpdateProvider {
 public:
  std::string name() const override {
    return "throw_set_context";
  }
  void configure(const json & /*cfg*/) override {
  }
  void set_context(PluginContext & /*ctx*/) override {
    throw std::runtime_error("set_context failed");
  }

  tl::expected<std::vector<std::string>, UpdateBackendErrorInfo>
  list_updates(const UpdateFilter & /*filter*/) override {
    return std::vector<std::string>{};
  }
  tl::expected<dto::UpdateDetail, UpdateBackendErrorInfo> get_update(const std::string & /*id*/) override {
    return dto::UpdateDetail{json::object()};
  }
  tl::expected<void, UpdateBackendErrorInfo> register_update(const json & /*metadata*/) override {
    return {};
  }
  tl::expected<void, UpdateBackendErrorInfo> delete_update(const std::string & /*id*/) override {
    return {};
  }
  tl::expected<void, UpdateBackendErrorInfo> prepare(const std::string & /*id*/,
                                                     UpdateProgressReporter & /*reporter*/) override {
    return {};
  }
  tl::expected<void, UpdateBackendErrorInfo> execute(const std::string & /*id*/,
                                                     UpdateProgressReporter & /*reporter*/) override {
    return {};
  }
  tl::expected<bool, UpdateBackendErrorInfo> supports_automated(const std::string & /*id*/) override {
    return false;
  }
};

/// Plugin that returns a test route for verifying wrapping logic
class MockRoutePlugin : public GatewayPlugin {
 public:
  std::string name() const override {
    return "mock_route";
  }
  void configure(const json & /*cfg*/) override {
  }
  std::vector<PluginRoute> get_routes() override {
    return {
        {"GET", R"(apps/([^/]+)/x-test-route)",
         [this](const PluginRequest & req, PluginResponse & res) {
           last_path_param_ = req.path_param(1);
           res.send_json({{"handled", true}, {"entity", last_path_param_}});
         }},
    };
  }
  std::string last_path_param_;
};

/// Commercial-bridge-shaped plugin: serves live values only through a
/// registered x-plc-data route (no DataProvider), like the S7comm/Modbus/ADS
/// bridges. Exercises PluginManager::fetch_entity_data_via_route.
class MockPlcRoutePlugin : public GatewayPlugin {
 public:
  std::string name() const override {
    return "mock_plc";
  }
  void configure(const json & /*cfg*/) override {
  }
  std::vector<PluginRoute> get_routes() override {
    ++get_routes_calls_;
    return {
        {"GET", R"(apps/([^/]+)/x-plc-data)",
         [this](const PluginRequest & req, PluginResponse & res) {
           last_path_ = req.path();
           auto entity = req.path_param(1);
           if (entity != "plc_app") {
             res.send_error(404, "resource-not-found", "No PLC data mapped for entity: " + entity);
             return;
           }
           res.send_json({{"connected", connected_},
                          {"items", json::array({{{"name", "level"}, {"value", 87.5}, {"unit", "mm"}},
                                                 {{"name", "alarm"}, {"value", true}}})}});
         }},
        // Longer sibling pattern must not shadow the exact match above.
        {"GET", R"(apps/([^/]+)/x-plc-data/([^/]+))",
         [this](const PluginRequest & req, PluginResponse & res) {
           last_path_ = req.path();
           res.send_json({{"single", true}});
         }},
    };
  }
  bool connected_ = true;
  int get_routes_calls_ = 0;
  std::string last_path_;
};

/// Plugin that throws during get_routes
class MockThrowOnGetRoutes : public GatewayPlugin, public IntrospectionProvider {
 public:
  std::string name() const override {
    return "throw_get_routes";
  }
  void configure(const json & /*cfg*/) override {
  }
  std::vector<PluginRoute> get_routes() override {
    throw std::runtime_error("get_routes failed");
  }

  IntrospectionResult introspect(const IntrospectionInput & /*input*/) override {
    return {};
  }
};

/// Plugin that throws during shutdown
class MockThrowOnShutdown : public GatewayPlugin {
 public:
  std::string name() const override {
    return "throw_shutdown";
  }
  void configure(const json & /*cfg*/) override {
  }
  void shutdown() override {
    throw std::runtime_error("shutdown failed");
  }
};

/// Plugin that registers a cyclic-subscription sampler during set_context(),
/// mirroring GraphProviderPlugin::set_context(). The callable captures `this`
/// so a dangling registration after teardown would be a use-after-free.
class MockSamplerPlugin : public GatewayPlugin {
 public:
  explicit MockSamplerPlugin(std::string collection, bool honours_resource_path = false)
    : collection_(std::move(collection)), honours_resource_path_(honours_resource_path) {
  }
  std::string name() const override {
    return "sampler_plugin";
  }
  void configure(const json & /*cfg*/) override {
  }
  void set_context(PluginContext & context) override {
    context.register_sampler(
        collection_,
        [this](const std::string &, const std::string &) -> tl::expected<json, std::string> {
          return json{{"plugin", name()}};
        },
        honours_resource_path_);
  }

 private:
  std::string collection_;
  bool honours_resource_path_;
};

/// Same sampler-registering behavior as MockSamplerPlugin, but also throws
/// from get_routes() - the reachable ordering the design doc calls out:
/// set_context() has already registered the sampler by the time a later
/// get_routes() failure disables the plugin.
class MockSamplerThrowOnGetRoutes : public GatewayPlugin {
 public:
  explicit MockSamplerThrowOnGetRoutes(std::string collection) : collection_(std::move(collection)) {
  }
  std::string name() const override {
    return "sampler_throw_get_routes";
  }
  void configure(const json & /*cfg*/) override {
  }
  void set_context(PluginContext & context) override {
    context.register_sampler(collection_,
                             [](const std::string &, const std::string &) -> tl::expected<json, std::string> {
                               return json{{"ok", true}};
                             });
  }
  std::vector<PluginRoute> get_routes() override {
    throw std::runtime_error("get_routes failed");
  }

 private:
  std::string collection_;
};

TEST(PluginManagerTest, EmptyManagerHasNoPlugins) {
  PluginManager mgr;
  EXPECT_FALSE(mgr.has_plugins());
  EXPECT_TRUE(mgr.plugin_names().empty());
  EXPECT_EQ(mgr.get_update_provider(), nullptr);
  EXPECT_TRUE(mgr.get_introspection_providers().empty());
}

TEST(PluginManagerTest, AddPluginAndDispatch) {
  PluginManager mgr;
  auto plugin = std::make_unique<MockPlugin>();
  auto * raw = plugin.get();
  mgr.add_plugin(std::move(plugin));

  EXPECT_TRUE(mgr.has_plugins());
  EXPECT_EQ(mgr.plugin_names().size(), 1u);
  EXPECT_EQ(mgr.plugin_names()[0], "mock");

  EXPECT_EQ(mgr.get_update_provider(), static_cast<UpdateProvider *>(raw));
  EXPECT_EQ(mgr.get_introspection_providers().size(), 1u);
}

TEST(PluginManagerTest, ConfigurePassesConfig) {
  PluginManager mgr;
  auto plugin = std::make_unique<MockPlugin>();
  auto * raw = plugin.get();
  mgr.add_plugin(std::move(plugin));

  mgr.configure_plugins();
  EXPECT_TRUE(raw->configured_);
  // add_plugin() uses empty config by default
  EXPECT_TRUE(raw->config_.is_object());
  EXPECT_TRUE(raw->config_.empty());
}

TEST(PluginManagerTest, LoadPluginsForwardsConfig) {
  // load_plugins() should forward the PluginConfig.config to configure()
  PluginManager mgr;
  json cfg = {{"server_url", "https://example.com"}, {"timeout_ms", 5000}};
  std::vector<PluginConfig> configs = {{"test", "/nonexistent/path.so", cfg}};

  // Plugin won't load (bad path), but verify PluginConfig struct holds config
  EXPECT_EQ(configs[0].config["server_url"], "https://example.com");
  EXPECT_EQ(configs[0].config["timeout_ms"], 5000);
}

TEST(PluginManagerTest, ShutdownCallsAllPlugins) {
  PluginManager mgr;
  auto plugin = std::make_unique<MockPlugin>();
  auto * raw = plugin.get();
  mgr.add_plugin(std::move(plugin));

  mgr.shutdown_all();
  EXPECT_TRUE(raw->shutdown_called_);
}

TEST(PluginManagerTest, MultiCapabilityPluginDispatchedToBoth) {
  PluginManager mgr;
  mgr.add_plugin(std::make_unique<MockPlugin>());

  EXPECT_NE(mgr.get_update_provider(), nullptr);
  EXPECT_EQ(mgr.get_introspection_providers().size(), 1u);
}

TEST(PluginManagerTest, IntrospectionOnlyPluginNotUpdateProvider) {
  PluginManager mgr;
  mgr.add_plugin(std::make_unique<MockIntrospectionOnly>());

  EXPECT_EQ(mgr.get_update_provider(), nullptr);
  EXPECT_EQ(mgr.get_introspection_providers().size(), 1u);
}

TEST(PluginManagerTest, MultipleIntrospectionProviders) {
  PluginManager mgr;
  mgr.add_plugin(std::make_unique<MockPlugin>());
  mgr.add_plugin(std::make_unique<MockIntrospectionOnly>());

  EXPECT_EQ(mgr.get_introspection_providers().size(), 2u);
}

TEST(PluginManagerTest, DuplicateUpdateProviderFirstWins) {
  PluginManager mgr;
  auto first = std::make_unique<MockPlugin>();
  auto * first_raw = first.get();
  mgr.add_plugin(std::move(first));
  mgr.add_plugin(std::make_unique<MockPlugin>());

  // First UpdateProvider wins
  EXPECT_EQ(mgr.get_update_provider(), static_cast<UpdateProvider *>(first_raw));
}

TEST(PluginManagerTest, ThrowingPluginDisabledDuringConfigure) {
  PluginManager mgr;
  mgr.add_plugin(std::make_unique<MockThrowingPlugin>());
  auto good = std::make_unique<MockPlugin>();
  auto * good_raw = good.get();
  mgr.add_plugin(std::move(good));

  // Should not throw
  mgr.configure_plugins();

  // Good plugin still works
  EXPECT_TRUE(good_raw->configured_);
  EXPECT_NE(mgr.get_update_provider(), nullptr);
}

TEST(PluginManagerTest, LoadNonexistentPluginReturnsZero) {
  PluginManager mgr;
  std::vector<PluginConfig> configs = {{"nonexistent", "/nonexistent/path.so", json::object()}};
  auto loaded = mgr.load_plugins(configs);
  EXPECT_EQ(loaded, 0u);
  EXPECT_FALSE(mgr.has_plugins());
}

TEST(PluginManagerTest, ThrowOnSetContextDisablesPlugin) {
  PluginManager mgr;
  mgr.add_plugin(std::make_unique<MockThrowOnSetContext>());
  auto good = std::make_unique<MockPlugin>();
  auto * good_raw = good.get();
  mgr.add_plugin(std::move(good));

  mgr.configure_plugins();

  // Create a minimal PluginContext for the test
  auto ctx = make_gateway_plugin_context(nullptr, nullptr);
  mgr.set_context(*ctx);

  // Throwing plugin disabled, good plugin's UpdateProvider still works
  EXPECT_EQ(mgr.get_update_provider(), static_cast<UpdateProvider *>(good_raw));
  // Only good plugin remains active
  EXPECT_EQ(mgr.plugin_names().size(), 1u);
  EXPECT_EQ(mgr.plugin_names()[0], "mock");
}

TEST(PluginManagerTest, DisablePluginRemovesItsSampler) {
  ResourceSamplerRegistry sampler_registry;
  TransportRegistry transport_registry;
  PluginManager mgr;
  mgr.set_registries(sampler_registry, transport_registry);
  mgr.add_plugin(std::make_unique<MockSamplerThrowOnGetRoutes>("x-mock-disable-sampler"));
  mgr.configure_plugins();

  auto ctx = make_gateway_plugin_context(nullptr, nullptr, &sampler_registry);
  mgr.set_context(*ctx);
  ASSERT_TRUE(sampler_registry.has_sampler("x-mock-disable-sampler"));

  httplib::Server srv;
  mgr.register_routes(srv, "/api/v1");  // get_routes() throws -> disable_plugin

  EXPECT_FALSE(sampler_registry.has_sampler("x-mock-disable-sampler"));
  EXPECT_FALSE(sampler_registry.get_sampler("x-mock-disable-sampler").has_value());
  EXPECT_FALSE(mgr.has_plugins());
}

TEST(PluginManagerTest, ShutdownAllRemovesPluginSamplers) {
  ResourceSamplerRegistry sampler_registry;
  TransportRegistry transport_registry;
  PluginManager mgr;
  mgr.set_registries(sampler_registry, transport_registry);
  mgr.add_plugin(std::make_unique<MockSamplerPlugin>("x-mock-shutdown-sampler"));
  mgr.configure_plugins();

  auto ctx = make_gateway_plugin_context(nullptr, nullptr, &sampler_registry);
  mgr.set_context(*ctx);
  ASSERT_TRUE(sampler_registry.has_sampler("x-mock-shutdown-sampler"));

  mgr.shutdown_all();

  EXPECT_FALSE(sampler_registry.has_sampler("x-mock-shutdown-sampler"));
}

TEST(PluginManagerTest, PluginResourcePathDeclarationReachesRegistry) {
  ResourceSamplerRegistry sampler_registry;
  TransportRegistry transport_registry;
  PluginManager mgr;
  mgr.set_registries(sampler_registry, transport_registry);
  mgr.add_plugin(std::make_unique<MockSamplerPlugin>("x-mock-whole-collection"));
  mgr.add_plugin(std::make_unique<MockSamplerPlugin>("x-mock-per-item", /*honours_resource_path=*/true));
  mgr.configure_plugins();

  auto ctx = make_gateway_plugin_context(nullptr, nullptr, &sampler_registry);
  mgr.set_context(*ctx);

  ASSERT_TRUE(sampler_registry.has_sampler("x-mock-whole-collection"));
  ASSERT_TRUE(sampler_registry.has_sampler("x-mock-per-item"));
  EXPECT_FALSE(sampler_registry.honours_resource_path("x-mock-whole-collection"));
  EXPECT_TRUE(sampler_registry.honours_resource_path("x-mock-per-item"));
}

TEST(PluginManagerTest, BuiltinSamplerSurvivesPluginDisable) {
  ResourceSamplerRegistry sampler_registry;
  TransportRegistry transport_registry;
  sampler_registry.register_sampler(
      "data",
      [](const std::string &, const std::string &) -> tl::expected<json, std::string> {
        return json{{"builtin", true}};
      },
      true);

  PluginManager mgr;
  mgr.set_registries(sampler_registry, transport_registry);
  mgr.add_plugin(std::make_unique<MockSamplerThrowOnGetRoutes>("x-mock-alongside-builtin"));
  mgr.configure_plugins();

  auto ctx = make_gateway_plugin_context(nullptr, nullptr, &sampler_registry);
  mgr.set_context(*ctx);

  httplib::Server srv;
  mgr.register_routes(srv, "/api/v1");  // disables the plugin

  EXPECT_FALSE(sampler_registry.has_sampler("x-mock-alongside-builtin"));
  EXPECT_TRUE(sampler_registry.has_sampler("data"));
}

TEST(PluginManagerTest, ThrowOnGetRoutesDisablesPlugin) {
  PluginManager mgr;
  mgr.add_plugin(std::make_unique<MockThrowOnGetRoutes>());
  auto good = std::make_unique<MockIntrospectionOnly>();
  mgr.add_plugin(std::move(good));

  mgr.configure_plugins();
  httplib::Server srv;
  mgr.register_routes(srv, "/api/v1");

  // Throwing plugin disabled, good plugin's IntrospectionProvider still works
  EXPECT_EQ(mgr.get_introspection_providers().size(), 1u);
  EXPECT_EQ(mgr.plugin_names().size(), 1u);
  EXPECT_EQ(mgr.plugin_names()[0], "introspection_only");
}

TEST(PluginManagerTest, RegisterRoutesWrapsPluginHandlers) {
  PluginManager mgr;
  auto plugin = std::make_unique<MockRoutePlugin>();
  auto * raw = plugin.get();
  mgr.add_plugin(std::move(plugin));
  mgr.configure_plugins();

  httplib::Server srv;
  mgr.register_routes(srv, "/api/v1");

  // Bind to ephemeral port to avoid conflicts in parallel CTest runs
  auto port = srv.bind_to_any_port("127.0.0.1");
  std::thread server_thread([&srv]() {
    srv.listen_after_bind();
  });
  srv.wait_until_ready();

  httplib::Client cli("127.0.0.1", port);
  auto res = cli.Get("/api/v1/apps/test_entity/x-test-route");

  srv.stop();
  server_thread.join();

  ASSERT_TRUE(res);
  EXPECT_EQ(res->status, 200);
  auto body = json::parse(res->body);
  EXPECT_EQ(body["handled"], true);
  EXPECT_EQ(body["entity"], "test_entity");
  EXPECT_EQ(raw->last_path_param_, "test_entity");
}

TEST(PluginManagerTest, FetchEntityDataViaRouteDispatchesOwningPluginHandler) {
  PluginManager mgr;
  auto plugin = std::make_unique<MockPlcRoutePlugin>();
  auto * raw = plugin.get();
  mgr.add_plugin(std::move(plugin));
  mgr.configure_plugins();
  mgr.register_entity_ownership("mock_plc", {"plc_app"});

  auto body = mgr.fetch_entity_data_via_route("plc_app");
  ASSERT_TRUE(body.has_value());
  EXPECT_EQ((*body)["connected"], true);
  ASSERT_EQ((*body)["items"].size(), 2u);
  EXPECT_EQ((*body)["items"][0]["name"], "level");
  EXPECT_EQ((*body)["items"][0]["value"], 87.5);
  // The handler saw a real-looking request: full API path with regex captures.
  EXPECT_EQ(raw->last_path_, "/api/v1/apps/plc_app/x-plc-data");
}

TEST(PluginManagerTest, FetchEntityDataViaRouteSingleItemDispatchesItemRoute) {
  PluginManager mgr;
  auto plugin = std::make_unique<MockPlcRoutePlugin>();
  auto * raw = plugin.get();
  mgr.add_plugin(std::move(plugin));
  mgr.configure_plugins();
  mgr.register_entity_ownership("mock_plc", {"plc_app"});

  auto body = mgr.fetch_entity_data_via_route("plc_app", "level");
  ASSERT_TRUE(body.has_value());
  EXPECT_EQ((*body)["single"], true);
  // The item must actually reach the dispatched path - a regression to the
  // list route would still answer 200 here.
  EXPECT_EQ(raw->last_path_, "/api/v1/apps/plc_app/x-plc-data/level");
}

TEST(PluginManagerTest, FetchEntityDataViaRouteUnownedEntityReturnsNullopt) {
  PluginManager mgr;
  mgr.add_plugin(std::make_unique<MockPlcRoutePlugin>());
  mgr.configure_plugins();

  EXPECT_FALSE(mgr.fetch_entity_data_via_route("plc_app").has_value());
}

TEST(PluginManagerTest, FetchEntityDataViaRouteErrorResponseReturnsNullopt) {
  PluginManager mgr;
  mgr.add_plugin(std::make_unique<MockPlcRoutePlugin>());
  mgr.configure_plugins();
  // Owned by the plugin, but its handler 404s for this entity.
  mgr.register_entity_ownership("mock_plc", {"unmapped_app"});

  EXPECT_FALSE(mgr.fetch_entity_data_via_route("unmapped_app").has_value());
}

TEST(PluginManagerTest, FetchEntityDataViaRouteNoMatchingRouteReturnsNullopt) {
  PluginManager mgr;
  mgr.add_plugin(std::make_unique<MockRoutePlugin>());  // registers x-test-route only
  mgr.configure_plugins();
  mgr.register_entity_ownership("mock_route", {"plc_app"});

  EXPECT_FALSE(mgr.fetch_entity_data_via_route("plc_app").has_value());
}

TEST(PluginManagerTest, GetRoutesCalledOnceAcrossRegisterAndFetch) {
  PluginManager mgr;
  auto plugin = std::make_unique<MockPlcRoutePlugin>();
  auto * raw = plugin.get();
  mgr.add_plugin(std::move(plugin));
  mgr.configure_plugins();
  mgr.register_entity_ownership("mock_plc", {"plc_app"});

  // Fetch before server setup (lazy cache), then register, then fetch again:
  // the "called once during REST server setup" plugin contract must hold.
  EXPECT_TRUE(mgr.fetch_entity_data_via_route("plc_app").has_value());
  httplib::Server srv;
  mgr.register_routes(srv, "/api/v1");
  EXPECT_TRUE(mgr.fetch_entity_data_via_route("plc_app").has_value());
  EXPECT_EQ(raw->get_routes_calls_, 1);
}

TEST(PluginManagerTest, ShutdownAllIdempotent) {
  PluginManager mgr;
  auto plugin = std::make_unique<MockPlugin>();
  auto * raw = plugin.get();
  mgr.add_plugin(std::move(plugin));

  mgr.shutdown_all();
  EXPECT_TRUE(raw->shutdown_called_);

  // Reset flag, call again - should be a no-op
  raw->shutdown_called_ = false;
  mgr.shutdown_all();
  EXPECT_FALSE(raw->shutdown_called_);
}

TEST(PluginManagerTest, ShutdownSwallowsExceptions) {
  PluginManager mgr;
  mgr.add_plugin(std::make_unique<MockThrowOnShutdown>());
  auto good = std::make_unique<MockPlugin>();
  auto * good_raw = good.get();
  mgr.add_plugin(std::move(good));

  // Should not throw, even though first plugin throws during shutdown
  EXPECT_NO_THROW(mgr.shutdown_all());
  EXPECT_TRUE(good_raw->shutdown_called_);
}

using namespace std::chrono_literals;

// Test 1: Concurrent readers don't block each other
TEST(PluginManagerConcurrencyTest, ConcurrentReadsDoNotBlock) {
  PluginManager mgr;
  mgr.add_plugin(std::make_unique<MockPlugin>());
  mgr.add_plugin(std::make_unique<MockIntrospectionOnly>());

  std::atomic<int> completed{0};
  std::vector<std::thread> readers;
  readers.reserve(8);

  for (int i = 0; i < 8; ++i) {
    readers.emplace_back([&mgr, &completed] {
      for (int j = 0; j < 200; ++j) {
        auto up = mgr.get_update_provider();
        auto ips = mgr.get_introspection_providers();
        auto lp = mgr.get_log_provider();
        auto obs = mgr.get_log_observers();
        auto has = mgr.has_plugins();
        auto names = mgr.plugin_names();
        (void)up;
        (void)ips;
        (void)lp;
        (void)obs;
        (void)has;
        (void)names;
      }
      completed++;
    });
  }

  auto start = std::chrono::high_resolution_clock::now();
  for (auto & t : readers) {
    t.join();
  }
  auto duration = std::chrono::high_resolution_clock::now() - start;

  EXPECT_EQ(completed.load(), 8);
  EXPECT_LT(duration, 2s) << "Concurrent reads took too long - possible blocking";
}

// Test 2: Concurrent reads and writes (lifecycle/disable) don't deadlock
TEST(PluginManagerConcurrencyTest, ConcurrentReadsAndLifecycleDoNotDeadlock) {
  PluginManager mgr;
  mgr.add_plugin(std::make_unique<MockPlugin>());
  mgr.add_plugin(std::make_unique<MockIntrospectionOnly>());

  std::atomic<bool> keep_running{true};
  std::atomic<int> read_count{0};
  std::atomic<int> write_count{0};

  // Multiple reader threads (simulating ROS 2 executor calling get_log_observers)
  std::vector<std::thread> readers;
  readers.reserve(4);
  for (int i = 0; i < 4; ++i) {
    readers.emplace_back([&mgr, &keep_running, &read_count] {
      while (keep_running) {
        auto obs = mgr.get_log_observers();
        auto ips = mgr.get_introspection_providers();
        auto up = mgr.get_update_provider();
        (void)obs;
        (void)ips;
        (void)up;
        read_count++;
      }
    });
  }

  // Writer thread (simulating lifecycle operations that take unique_lock)
  std::thread writer([&mgr, &keep_running, &write_count] {
    while (keep_running) {
      mgr.configure_plugins();
      write_count++;
    }
  });

  std::this_thread::sleep_for(50ms);
  keep_running = false;

  for (auto & t : readers) {
    t.join();
  }
  writer.join();

  EXPECT_GT(read_count.load(), 0) << "Readers made no progress - possible deadlock";
  EXPECT_GT(write_count.load(), 0) << "Writer made no progress - possible deadlock";
}

// Test 3: Shutdown while readers are active doesn't deadlock
TEST(PluginManagerConcurrencyTest, ShutdownWhileReadersActiveDoesNotDeadlock) {
  auto mgr = std::make_unique<PluginManager>();
  mgr->add_plugin(std::make_unique<MockPlugin>());

  std::atomic<bool> keep_running{true};
  std::atomic<int> read_count{0};

  std::vector<std::thread> readers;
  readers.reserve(4);
  for (int i = 0; i < 4; ++i) {
    readers.emplace_back([&mgr, &keep_running, &read_count] {
      while (keep_running) {
        if (mgr) {
          auto obs = mgr->get_log_observers();
          (void)obs;
        }
        read_count++;
      }
    });
  }

  std::this_thread::sleep_for(10ms);
  mgr->shutdown_all();

  keep_running = false;
  for (auto & t : readers) {
    t.join();
  }

  EXPECT_GT(read_count.load(), 0) << "Readers made no progress before shutdown";
}
