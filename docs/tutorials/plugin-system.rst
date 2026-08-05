Plugin System
=============

The gateway supports a plugin system for extending functionality with shared libraries (``.so`` files).
Plugins can provide software update backends, platform-specific introspection, and custom REST endpoints.

Overview
--------

Plugins implement the ``GatewayPlugin`` C++ base class plus one or more typed provider interfaces:

- **UpdateProvider** - software update backend (CRUD, prepare/execute, automated, status)
- **IntrospectionProvider** - provides platform-specific metadata and can introduce new
  entities into the entity cache. Called during each discovery cycle by the merge pipeline's
  PluginLayer. Plugin-provided metadata is accessible via the plugin API, not automatically
  merged into entity responses. See :doc:`/config/discovery-options` for merge pipeline configuration.
- **LogProvider** - replaces or augments the default ``/rosout`` log backend.
  Can operate in observer mode (receives log entries) or full-ingestion mode
  (owns the entire log pipeline). See the ``/logs`` endpoints in :doc:`/api/rest`.
- **ScriptProvider** - replaces or augments the default filesystem-based script backend.
  Plugins can provide script listings, create custom scripts, and execute them using
  alternative runtimes. See the ``/scripts`` endpoints in :doc:`/api/rest`.
- **DataProvider** - per-entity data resource backend (list, read, write data). Plugins
  that create entities via IntrospectionProvider can serve data for those entities.
  Entity requests are routed to the owning plugin automatically.
- **OperationProvider** - per-entity operation backend (list operations, execute). Uses
  the same per-entity routing model as DataProvider.
- **FaultProvider** - per-entity fault backend (list faults, get fault details, clear
  faults). Uses the same per-entity routing model as DataProvider.

A single plugin can implement multiple provider interfaces. For example, a "systemd" plugin
could provide both introspection (discover systemd units) and updates (manage service restarts).

Configuration
-------------

Add plugins to ``gateway_params.yaml``:

.. code-block:: yaml

   ros2_medkit_gateway:
     ros__parameters:
       plugins: ["my_ota_plugin"]
       plugins.my_ota_plugin.path: "/opt/ros2_medkit/lib/libmy_ota_plugin.so"
       plugins.my_ota_plugin.server_url: "https://updates.example.com"
       plugins.my_ota_plugin.api_key: "secret123"
       plugins.my_ota_plugin.timeout_ms: 5000

       # Enable updates if your plugin implements UpdateProvider
       updates:
         enabled: true

Each plugin name in the ``plugins`` array requires a corresponding ``plugins.<name>.path``
parameter with the absolute path to the ``.so`` file. Any additional ``plugins.<name>.<key>``
parameters are collected into a JSON object and passed to the plugin's ``configure()`` method.

For the example above, ``configure()`` receives:

.. code-block:: json

   {"server_url": "https://updates.example.com", "api_key": "secret123", "timeout_ms": 5000}

Plugins are loaded in the order listed. An empty list (default) means no plugins are loaded,
with zero overhead.

Plugin names must contain only alphanumeric characters, underscores, and hyphens (max 256 chars).

All standard ROS 2 parameter types are supported: strings, integers, doubles, booleans,
and their array variants. They are automatically converted to their JSON equivalents.

Writing a Plugin
----------------

1. Create a C++ shared library implementing ``GatewayPlugin`` and optionally one or more providers:

.. code-block:: cpp

   #include "ros2_medkit_gateway/core/plugins/gateway_plugin.hpp"
   #include "ros2_medkit_gateway/core/plugins/plugin_types.hpp"
   #include "ros2_medkit_gateway/core/providers/update_provider.hpp"

   using namespace ros2_medkit_gateway;

   class MyPlugin : public GatewayPlugin, public UpdateProvider {
    public:
     std::string name() const override { return "my_plugin"; }

     void configure(const nlohmann::json& config) override {
       // Read plugin-specific configuration (env vars, files, etc.)
     }

     void shutdown() override {
       // Clean up resources
     }

     // UpdateProvider methods - all 7 must be implemented:

     tl::expected<std::vector<std::string>, UpdateBackendErrorInfo>
       list_updates(const UpdateFilter& filter) override { /* ... */ }

     tl::expected<dto::UpdateDetail, UpdateBackendErrorInfo>
       get_update(const std::string& id) override { /* ... */ }

     tl::expected<void, UpdateBackendErrorInfo>
       register_update(const nlohmann::json& metadata) override { /* ... */ }

     tl::expected<void, UpdateBackendErrorInfo>
       delete_update(const std::string& id) override { /* ... */ }

     tl::expected<void, UpdateBackendErrorInfo>
       prepare(const std::string& id, UpdateProgressReporter& reporter) override { /* ... */ }

     tl::expected<void, UpdateBackendErrorInfo>
       execute(const std::string& id, UpdateProgressReporter& reporter) override { /* ... */ }

     tl::expected<bool, UpdateBackendErrorInfo>
       supports_automated(const std::string& id) override { /* ... */ }
   };

2. Export the required ``extern "C"`` symbols:

.. code-block:: cpp

   // Required: API version check
   extern "C" GATEWAY_PLUGIN_EXPORT int plugin_api_version() {
     return ros2_medkit_gateway::PLUGIN_API_VERSION;
   }

   // Required: factory function
   extern "C" GATEWAY_PLUGIN_EXPORT GatewayPlugin* create_plugin() {
     return new MyPlugin();
   }

   // Required if your plugin implements UpdateProvider:
   extern "C" GATEWAY_PLUGIN_EXPORT UpdateProvider* get_update_provider(GatewayPlugin* p) {
     return static_cast<MyPlugin*>(p);
   }

The ``get_update_provider`` (and ``get_introspection_provider``, ``get_log_provider``, ``get_script_provider``, ``get_data_provider``, ``get_operation_provider``, ``get_fault_provider``) functions use ``extern "C"``
to avoid RTTI issues across shared library boundaries. The ``static_cast`` is safe because
these functions execute inside the plugin's own ``.so`` where the type hierarchy is known.

Without the corresponding ``get_*_provider`` export, the gateway cannot detect that your plugin
implements the provider interface, even if the class inherits from it.

3. Build as a MODULE library:

.. code-block:: cmake

   add_library(my_plugin MODULE src/my_plugin.cpp)

   # Pull in include directories for the gateway plugin headers. Plugins
   # do NOT link the gateway static libraries directly - PluginManager
   # loads the plugin via dlopen and symbols from gateway_ros2 (and
   # gateway_core transitively) resolve from the host gateway process at
   # runtime.
   medkit_target_dependencies(my_plugin
     ros2_medkit_gateway
   )

   # Allow gateway-provided symbols to remain unresolved at link time;
   # they bind to the host process when the plugin is loaded.
   target_link_options(my_plugin PRIVATE
     -Wl,--unresolved-symbols=ignore-all
   )

   # Link only third-party libraries the plugin needs at link time
   # (e.g. nlohmann_json). Avoid target_link_libraries(my_plugin
   # gateway_ros2) - it would embed gateway objects into the plugin .so
   # and risk ODR / static-state duplication.

4. Install the ``.so`` and add its path to ``gateway_params.yaml``.

Complete Minimal Plugin
-----------------------

A self-contained plugin implementing UpdateProvider (copy-paste starting point):

.. code-block:: cpp

   // my_ota_plugin.cpp
   #include "ros2_medkit_gateway/core/plugins/gateway_plugin.hpp"
   #include "ros2_medkit_gateway/core/plugins/plugin_types.hpp"
   #include "ros2_medkit_gateway/core/providers/update_provider.hpp"

   #include <nlohmann/json.hpp>

   using namespace ros2_medkit_gateway;

   class MyOtaPlugin : public GatewayPlugin, public UpdateProvider {
    public:
     std::string name() const override { return "my_ota"; }

     void configure(const nlohmann::json& /*config*/) override {}

     void shutdown() override {}

     // UpdateProvider CRUD
     tl::expected<std::vector<std::string>, UpdateBackendErrorInfo>
     list_updates(const UpdateFilter& /*filter*/) override {
       return std::vector<std::string>{};
     }

     tl::expected<dto::UpdateDetail, UpdateBackendErrorInfo>
     get_update(const std::string& id) override {
       return tl::make_unexpected(
         UpdateBackendErrorInfo{UpdateBackendError::NotFound, "not found: " + id});
     }

     tl::expected<void, UpdateBackendErrorInfo>
     register_update(const nlohmann::json& /*metadata*/) override { return {}; }

     tl::expected<void, UpdateBackendErrorInfo>
     delete_update(const std::string& /*id*/) override { return {}; }

     // UpdateProvider async operations
     tl::expected<void, UpdateBackendErrorInfo>
     prepare(const std::string& /*id*/, UpdateProgressReporter& reporter) override {
       reporter.set_progress(100);
       return {};
     }

     tl::expected<void, UpdateBackendErrorInfo>
     execute(const std::string& /*id*/, UpdateProgressReporter& reporter) override {
       reporter.set_progress(100);
       return {};
     }

     tl::expected<bool, UpdateBackendErrorInfo>
     supports_automated(const std::string& /*id*/) override { return false; }
   };

   // Required exports
   extern "C" GATEWAY_PLUGIN_EXPORT int plugin_api_version() {
     return PLUGIN_API_VERSION;
   }

   extern "C" GATEWAY_PLUGIN_EXPORT GatewayPlugin* create_plugin() {
     return new MyOtaPlugin();
   }

   // Required for UpdateProvider detection
   extern "C" GATEWAY_PLUGIN_EXPORT UpdateProvider* get_update_provider(GatewayPlugin* p) {
     return static_cast<MyOtaPlugin*>(p);
   }

Plugin Lifecycle
----------------

1. ``dlopen`` loads the ``.so`` with ``RTLD_NOW | RTLD_LOCAL``
2. ``plugin_api_version()`` is checked against the gateway's ``PLUGIN_API_VERSION``
3. ``create_plugin()`` factory function creates the plugin instance
4. Provider interfaces are queried via ``get_update_provider()`` / ``get_introspection_provider()`` / ``get_log_provider()`` / ``get_script_provider()`` / ``get_data_provider()`` / ``get_operation_provider()`` / ``get_fault_provider()``
5. ``configure()`` is called with per-plugin JSON config
6. ``set_context()`` provides ``PluginContext`` with ROS 2 node, entity cache, faults, and HTTP utilities
7. ``get_routes()`` returns custom REST endpoint definitions as ``vector<PluginRoute>``
8. Runtime: subsystem managers call provider methods as needed
9. ``shutdown()`` is called before the plugin is destroyed

PluginContext
-------------

After ``configure()``, the gateway calls ``set_context()`` with a ``PluginContext`` reference
providing access to gateway data and utilities:

- ``node()`` - ROS 2 node pointer for service clients, timers, and parameter access.

  .. warning::

     Do **not** call ``node()->create_subscription<T>()``,
     ``create_generic_subscription()``, or ``create_callback_group()``
     directly on this node. Issue #375 showed that concurrent rcl
     mutations on the gateway node race its internal hash map and
     SIGSEGV under load on Rolling/Lyrical. The regression gate in
     ``scripts/check_no_naked_subscriptions.sh`` is run in CI and will
     fail PRs that add such calls outside the allowlist. Use
     ``ros2_medkit_gateway::ros2_common::Ros2SubscriptionSlot::create_typed``
     or ``create_generic`` via the executor that already serializes all
     subscription lifecycle calls. See
     :doc:`/design/ros2_medkit_gateway/ros2_subscription_architecture`
     for the allowed pattern and the rationale.

- ``get_entity(id)`` - look up any entity (area, component, app, function) from the discovery cache
- ``list_entity_faults(entity_id)`` - query faults for an entity
- ``validate_entity_for_route(req, res, entity_id)`` - validate entity exists and matches the route type, auto-sending SOVD errors on failure
- ``register_capability()`` / ``register_entity_capability()`` - register custom capabilities on entities

.. note::

   SOVD-compliant HTTP response helpers (``send_json()``, ``send_error()``) are instance
   methods on ``PluginResponse``, not static methods on ``PluginContext``. Use
   ``res.send_json(data)`` and ``res.send_error(status, code, msg)`` inside route handlers.

- ``check_lock(entity_id, client_id, collection)`` - verify lock access before mutating operations; returns ``LockAccessResult`` with ``allowed`` flag and denial details
- ``acquire_lock()`` / ``release_lock()`` - acquire and release entity locks with optional scope and TTL
- ``get_entity_snapshot()`` - returns an ``IntrospectionInput`` populated from the current entity cache
- ``list_all_faults()`` - returns JSON object with a ``"faults"`` array containing all active faults across all entities
- ``register_sampler(collection, fn)`` - registers a cyclic subscription sampler for a custom collection name

.. code-block:: cpp

   void set_context(PluginContext& ctx) override {
     ctx_ = &ctx;

     // Register a custom capability for all apps
     ctx.register_capability(SovdEntityType::APP, "x-medkit-traces");

     // Register a capability for a specific entity
     ctx.register_entity_capability("sensor1", "x-medkit-calibration");

     // Get a snapshot of all currently discovered entities
     IntrospectionInput snapshot = ctx.get_entity_snapshot();

     // Query all active faults (returns {"faults": [...]})
     nlohmann::json all_faults = ctx.list_all_faults();

     // Register a sampler so clients can subscribe to "x-medkit-metrics" cyclically
     ctx.register_sampler("x-medkit-metrics",
       [this](const std::string& entity_id, const std::string& /*resource_path*/)
           -> tl::expected<nlohmann::json, std::string> {
         auto data = collect_metrics(entity_id);
         if (!data) return tl::make_unexpected("no data for: " + entity_id);
         return *data;
       });
   }

   PluginContext* ctx_ = nullptr;

``get_entity_snapshot()`` returns an ``IntrospectionInput`` with vectors for all discovered
areas, components, apps, and functions at the moment of the call. The snapshot is read-only
and reflects the state of the gateway's thread-safe entity cache.

``list_all_faults()`` is useful for plugins that need cross-entity fault visibility (e.g.
mapping fault codes to topics). Returns ``{}`` if the fault manager is unavailable.

``register_sampler(collection, fn)`` wires a sampler into the ``ResourceSamplerRegistry``
so that cyclic subscriptions created for ``collection`` (e.g. ``"x-medkit-metrics"``)
call ``fn(entity_id, resource_path)`` on each tick. The function must return
``tl::expected<nlohmann::json, std::string>``. See `Cyclic Subscription Extensions`_
for the lower-level registry API.

.. note::

   The ``PluginContext`` interface is versioned alongside ``PLUGIN_API_VERSION``.
   Breaking changes to existing methods or removal of methods increment the version.
   New non-breaking methods (like ``check_lock``, ``get_entity_snapshot``,
   ``list_all_faults``, and ``register_sampler``) provide default no-op implementations
   so plugins that do not use these methods need no code changes. However, a rebuild is
   still required because ``plugin_api_version()`` must return the current version
   (exact-match check).

PluginContext API (v5)
----------------------

Version 5 of the plugin API replaced ``register_routes()`` with ``get_routes()``
and moved ``send_json``/``send_error`` from ``PluginContext`` static methods to
``PluginResponse`` instance methods. Plugins that implement custom REST routes
require source changes to adapt to the new API. Plugins that do not implement
routes only need a rebuild to match the new ``PLUGIN_API_VERSION``.

**check_lock(entity_id, client_id, collection)**

Verify whether a lock blocks access to a resource collection on an entity. Plugins
that perform mutating operations (writing configurations, executing scripts, etc.)
should call this before proceeding:

.. code-block:: cpp

   auto result = ctx_->check_lock(entity_id, client_id, "configurations");
   if (!result.allowed) {
     res.send_error(409, result.denied_code, result.denied_reason);
     return;
   }

The returned ``LockAccessResult`` contains an ``allowed`` flag and, when denied,
``denied_by_lock_id``, ``denied_code``, and ``denied_reason`` fields. Companion
methods ``acquire_lock()`` and ``release_lock()`` let plugins manage locks directly.

**get_entity_snapshot()**

Returns an ``IntrospectionInput`` populated from the current entity cache. The
snapshot contains read-only vectors for all discovered areas, components, apps, and
functions at the moment of the call:

.. code-block:: cpp

   IntrospectionInput snapshot = ctx_->get_entity_snapshot();
   for (const auto& app : snapshot.apps) {
     RCLCPP_INFO(ctx_->node()->get_logger(), "App: %s", app.id.c_str());
   }

This is useful for plugins that need a consistent view of all entities without
subscribing to discovery events.

**list_all_faults()**

Returns a JSON object with a ``"faults"`` array containing all active faults across
all entities. Returns an empty object if the fault manager is unavailable:

.. code-block:: cpp

   nlohmann::json faults = ctx_->list_all_faults();
   for (const auto& fault : faults.value("faults", nlohmann::json::array())) {
     // Process each fault
   }

**register_sampler(collection, fn)**

Registers a cyclic subscription sampler for a custom collection name. Once
registered, clients can create cyclic subscriptions on that collection for any
entity:

.. code-block:: cpp

   ctx_->register_sampler("x-medkit-metrics",
     [this](const std::string& entity_id, const std::string& /*resource_path*/)
         -> tl::expected<nlohmann::json, std::string> {
       auto data = collect_metrics(entity_id);
       if (!data) return tl::make_unexpected("no data for: " + entity_id);
       return *data;
     });

This is a convenience wrapper around the lower-level ``ResourceSamplerRegistry``
API described in `Cyclic Subscription Extensions`_.

Custom REST Endpoints
---------------------

Any plugin can expose vendor-specific endpoints by overriding ``get_routes()``, which
returns a ``vector<PluginRoute>``. Each route specifies an HTTP method, a URL pattern
relative to the API prefix (no leading slash), and a handler. Use ``PluginRequest`` and
``PluginResponse`` for path parameters and SOVD-compliant responses:

.. code-block:: cpp

   std::vector<PluginRoute> get_routes() override {
     return {
         // Global vendor endpoint
         {"GET", "x-myvendor/status",
          [this](const PluginRequest& /*req*/, PluginResponse& res) {
            res.send_json(get_status_json());
          }},

         // Entity-scoped endpoint (matches a registered capability)
         {"GET", R"(apps/([^/]+)/x-medkit-traces)",
          [this](const PluginRequest& req, PluginResponse& res) {
            auto entity_id = req.path_param(1);
            auto entity = ctx_->validate_entity_for_route(req, res, entity_id);
            if (!entity) return;  // Error already sent

            auto faults = ctx_->list_entity_faults(entity->id);
            res.send_json({{"entity", entity->id}, {"faults", faults}});
          }},
     };
   }

Use the ``x-`` prefix for vendor-specific endpoints per SOVD convention. Patterns are
relative to the API prefix and must not include a leading slash.

For entity-scoped endpoints, register a matching capability via ``register_capability()``
or ``register_entity_capability()`` in ``set_context()`` so the endpoint appears in the
entity's capabilities array in discovery responses.

Cyclic Subscription Extensions
-------------------------------

Plugins can extend cyclic subscriptions by registering custom resource samplers
and transport providers during ``set_context()``.

**Resource Samplers** provide the data for a collection when sampled by a subscription.
Built-in samplers (``data``, ``faults``, ``configurations``, ``updates``) are registered
by the gateway during startup. Custom samplers are registered via ``ResourceSamplerRegistry``
on the ``GatewayNode``:

.. code-block:: cpp

   auto* sampler_registry = node->get_sampler_registry();
   sampler_registry->register_sampler("x-medkit-metrics",
     [this](const std::string& entity_id, const std::string& resource_path)
       -> tl::expected<nlohmann::json, std::string> {
       return get_metrics(entity_id, resource_path);
     });

Once registered, clients can create cyclic subscriptions on the ``x-medkit-metrics``
collection for any entity.

**Transport Providers** deliver subscription data via alternative protocols (beyond
the built-in SSE transport). Register via ``TransportRegistry`` on the ``GatewayNode``:

.. code-block:: cpp

   auto* transport_registry = node->get_transport_registry();
   transport_registry->register_transport(
     std::make_unique<MqttTransportProvider>(mqtt_client_));

The transport must implement ``SubscriptionTransportProvider`` (``start``, ``stop``,
``notify_update``, ``protocol()``). Clients specify the protocol in the subscription
creation request.

IntrospectionProvider Example
-----------------------------

An ``IntrospectionProvider`` enriches entities in the merge pipeline. The plugin
receives all entities from earlier layers (manifest + runtime) and returns new or
enriched entities with ``ENRICHMENT`` merge policy.

The ``ros2_medkit_topic_beacon`` (push-based) and ``ros2_medkit_param_beacon``
(pull-based) plugins are the reference implementations. ``ros2_medkit_topic_beacon``
subscribes to ``/ros2_medkit/discovery``, stores incoming
``MedkitDiscoveryHint`` messages, and injects their fields during ``introspect()``.
``ros2_medkit_param_beacon`` polls ROS 2 parameters instead:

.. code-block:: cpp

   #include "ros2_medkit_gateway/core/plugins/gateway_plugin.hpp"
   #include "ros2_medkit_gateway/core/providers/introspection_provider.hpp"

   using namespace ros2_medkit_gateway;

   class MyIntrospectionPlugin : public GatewayPlugin, public IntrospectionProvider {
    public:
     std::string name() const override { return "my_introspection"; }

     void configure(const nlohmann::json& config) override {
       // Read plugin configuration
     }

     void shutdown() override {}

     // IntrospectionProvider: called on every merge pipeline refresh
     IntrospectionResult introspect(const IntrospectionInput& input) override {
       IntrospectionResult result;
       for (const auto& app : input.apps) {
         // Shadow the existing app with enriched metadata
         App shadow;
         shadow.id = app.id;
         result.new_entities.apps.push_back(shadow);
         result.metadata[app.id] = {
           {"x-platform-version", get_platform_version()}
         };
       }
       return result;
     }
   };

   // Required exports
   extern "C" GATEWAY_PLUGIN_EXPORT int plugin_api_version() {
     return PLUGIN_API_VERSION;
   }

   extern "C" GATEWAY_PLUGIN_EXPORT GatewayPlugin* create_plugin() {
     return new MyIntrospectionPlugin();
   }

   extern "C" GATEWAY_PLUGIN_EXPORT IntrospectionProvider* get_introspection_provider(GatewayPlugin* p) {
     return static_cast<MyIntrospectionPlugin*>(p);
   }

The ``IntrospectionInput`` contains entity vectors from previous layers:

- ``input.areas``, ``input.components``, ``input.apps``, ``input.functions`` -
  read-only vectors of discovered entities to use as context

The returned ``IntrospectionResult`` wraps two fields:

- ``new_entities`` - a ``NewEntities`` struct with vectors for ``areas``,
  ``components``, ``apps``, and ``functions`` that the plugin introduces
- ``metadata`` - a map from entity ID to a ``nlohmann::json`` object with
  plugin-specific enrichment data (e.g. vendor extension fields)

New entities in ``new_entities`` only appear in responses when
``allow_new_entities`` is true in the plugin configuration (or an equivalent
policy is set).

If the plugin publishes ROS 2 topics that back an entity's data points (the
PLC-bridge pattern: values mirrored to ``/plc/...`` topics), declare them on
the entity via ``App::topics`` / ``Component::topics``:

.. code-block:: cpp

   App app;
   app.id = "my_device";
   app.topics.publishes.push_back("/plc/main/counter");

Runtime graph discovery attributes a topic to the ROS node that publishes it -
for plugin-published topics that is the gateway's own node, not the plugin
entity. Without the declaration, entity-scoped topic lookups on the plugin
entity return nothing, and in particular data triggers
(``POST /apps/<id>/triggers`` with a ``data/...`` resource) can never resolve
and never fire. Declare only topics that actually get a publisher.

ScriptProvider Example
----------------------

A ``ScriptProvider`` replaces the built-in filesystem-based script backend with a
custom implementation. This is useful for plugins that store scripts in a database,
fetch them from a remote service, or execute them in a sandboxed runtime.

The interface mirrors the ``/scripts`` REST endpoints - list, get, upload, delete,
execute, and control executions:

.. code-block:: cpp

   #include "ros2_medkit_gateway/core/plugins/gateway_plugin.hpp"
   #include "ros2_medkit_gateway/core/providers/script_provider.hpp"

   using namespace ros2_medkit_gateway;

   class MyScriptPlugin : public GatewayPlugin, public ScriptProvider {
    public:
     std::string name() const override { return "my_scripts"; }

     void configure(const nlohmann::json& /*config*/) override {}

     void shutdown() override {}

     // ScriptProvider: list scripts available for an entity
     tl::expected<std::vector<ScriptInfo>, ScriptBackendErrorInfo>
     list_scripts(const std::string& /*entity_id*/) override {
       return std::vector<ScriptInfo>{};
     }

     // ScriptProvider: get metadata for a specific script
     tl::expected<ScriptInfo, ScriptBackendErrorInfo>
     get_script(const std::string& /*entity_id*/, const std::string& script_id) override {
       return tl::make_unexpected(
         ScriptBackendErrorInfo{ScriptBackendError::NotFound, "not found: " + script_id});
     }

     // ScriptProvider: upload a new script
     tl::expected<ScriptUploadResult, ScriptBackendErrorInfo>
     upload_script(const std::string& /*entity_id*/, const std::string& /*filename*/,
                   const std::string& /*content*/,
                   const std::optional<nlohmann::json>& /*metadata*/) override {
       return tl::make_unexpected(
         ScriptBackendErrorInfo{ScriptBackendError::UnsupportedType, "uploads not supported"});
     }

     // ScriptProvider: delete a script
     tl::expected<void, ScriptBackendErrorInfo>
     delete_script(const std::string& /*entity_id*/, const std::string& /*script_id*/) override {
       return {};
     }

     // ScriptProvider: start executing a script
     tl::expected<ExecutionInfo, ScriptBackendErrorInfo>
     start_execution(const std::string& /*entity_id*/, const std::string& /*script_id*/,
                     const ExecutionRequest& /*request*/) override {
       return tl::make_unexpected(
         ScriptBackendErrorInfo{ScriptBackendError::NotFound, "no scripts available"});
     }

     // ScriptProvider: query execution status
     tl::expected<ExecutionInfo, ScriptBackendErrorInfo>
     get_execution(const std::string& /*entity_id*/, const std::string& /*script_id*/,
                   const std::string& /*execution_id*/) override {
       return tl::make_unexpected(
         ScriptBackendErrorInfo{ScriptBackendError::NotFound, "no executions"});
     }

     // ScriptProvider: control a running execution (stop or force-terminate)
     tl::expected<ExecutionInfo, ScriptBackendErrorInfo>
     control_execution(const std::string& /*entity_id*/, const std::string& /*script_id*/,
                       const std::string& /*execution_id*/,
                       const std::string& /*action*/) override {
       return tl::make_unexpected(
         ScriptBackendErrorInfo{ScriptBackendError::NotRunning, "no running execution"});
     }

     // ScriptProvider: delete a completed execution record
     tl::expected<void, ScriptBackendErrorInfo>
     delete_execution(const std::string& /*entity_id*/, const std::string& /*script_id*/,
                      const std::string& /*execution_id*/) override {
       return {};
     }
   };

   // Required exports
   extern "C" GATEWAY_PLUGIN_EXPORT int plugin_api_version() {
     return PLUGIN_API_VERSION;
   }

   extern "C" GATEWAY_PLUGIN_EXPORT GatewayPlugin* create_plugin() {
     return new MyScriptPlugin();
   }

   // Required for ScriptProvider detection
   extern "C" GATEWAY_PLUGIN_EXPORT ScriptProvider* get_script_provider(GatewayPlugin* p) {
     return static_cast<MyScriptPlugin*>(p);
   }

When a plugin ScriptProvider is detected, it replaces the built-in
``DefaultScriptProvider``. Only the first ScriptProvider plugin is used
(same semantics as UpdateProvider). All 8 methods must be implemented -
the ``ScriptManager`` wraps calls with null-safety and exception isolation.

**Configuration** - enable scripts and load the plugin:

.. code-block:: yaml

   ros2_medkit_gateway:
     ros__parameters:
       plugins: ["my_scripts"]
       plugins.my_scripts.path: "/opt/ros2_medkit/lib/libmy_scripts.so"

       scripts:
         scripts_dir: "/var/lib/ros2_medkit/scripts"

The ``scripts.scripts_dir`` parameter must be set for the scripts subsystem to
initialize, even when using a plugin backend. The plugin replaces how scripts are
stored and executed, but the subsystem must be enabled first.

Per-Entity Provider Example (DataProvider)
-------------------------------------------

A plugin that creates entities via ``IntrospectionProvider`` and serves data for them
via ``DataProvider``. OperationProvider and FaultProvider follow the same pattern.

.. code-block:: cpp

   #include "ros2_medkit_gateway/core/plugins/gateway_plugin.hpp"
   #include "ros2_medkit_gateway/core/plugins/plugin_types.hpp"
   #include "ros2_medkit_gateway/core/providers/introspection_provider.hpp"
   #include "ros2_medkit_gateway/core/providers/data_provider.hpp"

   using namespace ros2_medkit_gateway;

   class MyDataPlugin : public GatewayPlugin, public IntrospectionProvider, public DataProvider {
    public:
     std::string name() const override { return "my_data_plugin"; }
     void configure(const nlohmann::json &) override {}
     void shutdown() override {}

     // IntrospectionProvider: create entities owned by this plugin
     IntrospectionResult introspect(const IntrospectionInput &) override {
       IntrospectionResult result;
       App ecu;
       ecu.id = "my_ecu";
       ecu.name = "My ECU";
       result.new_entities.apps.push_back(std::move(ecu));
       return result;
     }

     // DataProvider: serve data for owned entities
     tl::expected<nlohmann::json, DataProviderErrorInfo> list_data(const std::string & entity_id) override {
       return nlohmann::json{{"items", {{{"id", "temperature"}, {"name", "Temperature"}}}}};
     }
     tl::expected<nlohmann::json, DataProviderErrorInfo> read_data(
         const std::string &, const std::string & resource_name) override {
       if (resource_name == "temperature") {
         return nlohmann::json{{"id", resource_name}, {"data", {{"temperature", 42.5}}}};
       }
       return tl::make_unexpected(DataProviderErrorInfo{DataProviderError::ResourceNotFound, "Unknown resource", 404});
     }
     tl::expected<nlohmann::json, DataProviderErrorInfo> write_data(
         const std::string &, const std::string &, const nlohmann::json &) override {
       return tl::make_unexpected(DataProviderErrorInfo{DataProviderError::ReadOnly, "Read-only data", 403});
     }
   };

   // Plugin exports - both providers must be exported via extern "C"
   extern "C" GATEWAY_PLUGIN_EXPORT GatewayPlugin* create_plugin() { return new MyDataPlugin(); }
   extern "C" GATEWAY_PLUGIN_EXPORT int plugin_api_version() { return PLUGIN_API_VERSION; }
   extern "C" GATEWAY_PLUGIN_EXPORT IntrospectionProvider* get_introspection_provider(GatewayPlugin* p) {
     return static_cast<MyDataPlugin*>(p);
   }
   extern "C" GATEWAY_PLUGIN_EXPORT DataProvider* get_data_provider(GatewayPlugin* p) {
     return static_cast<MyDataPlugin*>(p);
   }

The gateway automatically adds ``data`` capability to entities owned by a plugin that
registers a ``DataProvider``. Similarly for ``operations`` (OperationProvider) and ``faults``
(FaultProvider). There is no need to call ``register_entity_capability()`` for these
standard resource collections.

.. warning::

   All provider methods may be called concurrently from multiple HTTP handler
   threads. Implementations that access shared state must provide their own
   synchronization (e.g., ``std::mutex``). Plugin responses are passed through
   verbatim to the API consumer - match the SOVD response format for consistency
   with native entities.

Multiple Plugins
----------------

Multiple plugins can be loaded simultaneously:

- **UpdateProvider**: Only one plugin's UpdateProvider is used (first in config order)
- **IntrospectionProvider**: All plugins' ``IntrospectionResult`` values are merged
  via the PluginLayer in the discovery pipeline - ``new_entities`` vectors are
  concatenated and ``metadata`` maps are merged. Both ``ros2_medkit_topic_beacon``
  and ``ros2_medkit_param_beacon`` can be active at the same time, each contributing
  their own discovered entities and metadata.
- **LogProvider**: Only the first plugin's LogProvider is used for queries (same as UpdateProvider).
  All LogProvider plugins receive ``on_log_entry()`` calls as observers.
- **ScriptProvider**: Only the first plugin's ScriptProvider is used (same as UpdateProvider).
- **DataProvider / OperationProvider / FaultProvider**: These use per-entity routing based
  on entity ownership. Entities created by a plugin's IntrospectionProvider are automatically
  routed to that same plugin's DataProvider, OperationProvider, and FaultProvider. Multiple
  plugins can each serve different entities concurrently - there is no "first wins" conflict
  because each plugin only handles requests for its own entities.
- **Custom routes**: All plugins can register endpoints (use unique path prefixes)

Entity Ownership
~~~~~~~~~~~~~~~~

DataProvider, OperationProvider, and FaultProvider use an entity ownership model to
route requests to the correct plugin.

- ``IntrospectionProvider::introspect()`` determines ownership: entities returned in a
  plugin's ``IntrospectionResult::new_entities`` are owned by that plugin.
- Entity ownership is refreshed periodically during cache updates (each discovery cycle
  re-evaluates ``introspect()`` results from all plugins).
- The gateway maintains an internal map from entity ID to the plugin that created it.
- When a data, operation, or fault request arrives for an entity, the handler looks up
  the owning plugin and delegates to its corresponding provider. Entities not owned by
  any plugin fall through to the default gateway behavior.

This model allows multiple plugins to coexist without conflict - each plugin manages
its own entities independently.

Graph Provider Plugin (ros2_medkit_graph_provider)
---------------------------------------------------

The gateway ships with an optional first-party plugin that exposes a ROS 2 topic graph for
each SOVD ``Function`` entity. It lives in a separate colcon package,
``ros2_medkit_graph_provider``, under ``src/ros2_medkit_plugins/``.

**What it does**

- Registers the ``x-medkit-graph`` vendor capability on all ``Function`` entities.
- Exposes ``GET /api/v1/functions/{id}/x-medkit-graph`` returning a graph document
  with nodes (apps), edges (topic connections), per-edge frequency/latency/drop-rate
  metrics (sourced from the ``/diagnostics`` topic), and an overall ``pipeline_status``
  (``healthy``, ``degraded``, or ``broken``).
- Supports cyclic subscriptions on the ``x-medkit-graph`` collection so clients can
  stream live graph updates.

**Package layout**

.. code-block::

   src/ros2_medkit_plugins/
   └── ros2_medkit_graph_provider/
       ├── CMakeLists.txt
       ├── package.xml
       ├── include/ros2_medkit_graph_provider/graph_provider_plugin.hpp
       └── src/
           ├── graph_provider_plugin.cpp
           └── graph_provider_plugin_exports.cpp

**Loading the plugin**

The plugin is loaded via the ``gateway_params.yaml`` plugin list. The ``.so`` path is
resolved at launch time by ``gateway.launch.py`` using ``get_package_prefix()``; if the
package is not installed the gateway starts normally without graph functionality:

.. code-block:: python

   # Excerpt from gateway.launch.py
   try:
       graph_provider_prefix = get_package_prefix('ros2_medkit_graph_provider')
       graph_provider_path = os.path.join(
           graph_provider_prefix, 'lib', 'ros2_medkit_graph_provider',
           'libros2_medkit_graph_provider_plugin.so')
   except PackageNotFoundError:
       pass  # Plugin not installed - gateway runs without graph provider

The path is then injected into the node parameters as ``plugins.graph_provider.path``.

**YAML configuration**

.. code-block:: yaml

   ros2_medkit_gateway:
     ros__parameters:
       plugins: ["graph_provider"]

       # Absolute path to the .so - set automatically by gateway.launch.py
       plugins.graph_provider.path: "/opt/ros/jazzy/lib/ros2_medkit_graph_provider/libros2_medkit_graph_provider_plugin.so"

       # Default expected publish frequency for topics without per-topic overrides.
       # An edge whose measured frequency is below
       # expected_frequency_hz_default * degraded_frequency_ratio is marked degraded.
       plugins.graph_provider.expected_frequency_hz_default: 30.0

       # Fraction of expected frequency below which an edge is "degraded" (0.0-1.0).
       plugins.graph_provider.degraded_frequency_ratio: 0.5

       # Drop-rate percentage above which an edge is marked degraded.
       plugins.graph_provider.drop_rate_percent_threshold: 5.0

Per-function overrides are also supported:

.. code-block:: yaml

       # Override thresholds for a specific function
       plugins.graph_provider.function_overrides.my_pipeline.expected_frequency_hz: 10.0
       plugins.graph_provider.function_overrides.my_pipeline.degraded_frequency_ratio: 0.3
       plugins.graph_provider.function_overrides.my_pipeline.drop_rate_percent_threshold: 2.0

.. note::

   The keys above are not the full set. There are also two freshness-window keys
   (``freshness_floor_sec``, ``freshness_headroom_factor``), a stale-grace
   debounce key (``stale_grace_sec``), and a ``multi_publisher_rate`` policy key
   that control when and how an edge with no recent ``/diagnostics`` sample
   flips from ``active`` to ``error``/``metrics_stale``, and how a topic with
   more than one live publisher is reported. See :doc:`/config/graph-provider`
   for the complete reference, and :doc:`graph-provider` for the ``/diagnostics``
   producer contract this plugin depends on - without a matching producer,
   every edge stays ``pending`` forever.

**Disabling the plugin**

To disable graph functionality without uninstalling the package, remove ``"graph_provider"``
from the ``plugins`` list in your params file:

.. code-block:: yaml

   plugins: []

Alternatively, simply do not install the ``ros2_medkit_graph_provider`` package -
``gateway.launch.py`` will skip the plugin automatically.

Error Handling
--------------

If a plugin throws during any lifecycle method (``configure``, ``set_context``, ``get_routes``,
``shutdown``), the exception is caught and logged. The plugin is disabled but the gateway continues
operating. A failing plugin never crashes the gateway.

API Versioning
--------------

Plugins export ``plugin_api_version()`` which must return the gateway's ``PLUGIN_API_VERSION``.
If the version does not match, the plugin is rejected with a clear error message suggesting
a rebuild against matching gateway headers.

The current API version is **5**. It is incremented when the ``PluginContext`` vtable changes
or breaking changes are made to ``GatewayPlugin`` or provider interfaces.

Build Requirements
------------------

- **Same compiler and ABI** as the gateway executable
- **RTTI must be enabled for in-process plugins** - the ``add_plugin()`` path uses ``dynamic_cast``
  to query provider interfaces, so ``-fno-rtti`` will break it. Shared-library plugins loaded via
  ``PluginLoader::load()`` use ``extern "C"`` query functions and do not require RTTI.
- The ``GATEWAY_PLUGIN_EXPORT`` macro ensures correct symbol visibility
