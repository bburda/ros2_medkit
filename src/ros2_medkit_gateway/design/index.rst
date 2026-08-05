ros2_medkit_gateway
===================

This section contains design documentation for the ros2_medkit_gateway project.

Architecture
------------

Build Layers
~~~~~~~~~~~~

The package compiles into two layered static libraries with a strict
dependency direction:

* ``gateway_core`` - middleware-neutral business logic. Sources live under
  ``src/core/`` and headers under ``include/ros2_medkit_gateway/core/``.
  Links only header-only and C-level externals (cpp-httplib,
  nlohmann/json, yaml-cpp, tl::expected, jwt-cpp, OpenSSL, SQLite, dl).
  Carries no rclcpp / rcl_interfaces / message-package dependency.
  Hosts the neutral HTTP request handlers, JWT authentication,
  fault model (debounce, storage, cache, correlation), peer aggregation,
  manifest parsing, the entity cache, the neutral managers (lock,
  bulk-data, subscription, script, update, plugin), and every provider
  interface contract.
* ``gateway_ros2`` - ROS adapter layer. Compiles the remaining sources
  under ``src/`` and links ``gateway_core`` publicly via
  ``medkit_target_dependencies``. Hosts ``GatewayNode`` (the
  ``rclcpp::Node`` entry point), the ROS-coupled managers (data access,
  operation, configuration, fault facade, log, trigger), runtime
  discovery, the native topic sampler, and the ROS-specific provider
  default implementations.

The ``gateway_node`` executable and existing test targets link
``gateway_ros2``, so they transitively get both layers from a single
dependency. The neutral contract is enforced by two CTest checks:

* ``gateway_core_purity`` (linter label) - greps ``core/`` for any
  ROS-package include and fails on any match.
* ``test_gateway_core_smoke`` (unit label) - compiles a translation unit
  including a sampling of ``core/`` headers and links exclusively against
  ``gateway_core`` + GTest with no ``ament_target_dependencies``. Build
  failure indicates a transitive ROS coupling that the grep guard might
  miss when an include is reached through a third-party header.

Class Diagram
~~~~~~~~~~~~~

The following diagram shows the relationships between the main components of the gateway.

.. plantuml::
   :caption: ROS 2 Medkit Gateway Class Architecture

   @startuml ros2_medkit_gateway_architecture

   skinparam linetype ortho
   skinparam classAttributeIconSize 0

   title ROS 2 Medkit Gateway - Class Architecture

   package "ROS 2 Framework" {
       class "rclcpp::Node" {
           +get_node_names()
           +get_topic_names_and_types()
           +get_service_names_and_types()
       }
   }

   package "ros2_medkit_gateway" {

   note as N_layer_split
   This package compiles into two layered static libraries:
   * gateway_core - middleware-neutral business logic (handlers,
     auth, fault model, aggregation, all SOVD managers including
     DataAccessManager, OperationManager, ConfigurationManager,
     FaultManager, LogManager, TriggerManager, providers, entity
     model). Does not link rclcpp. Managers consume neutral
     Transport interfaces (TopicTransport, ServiceTransport,
     ActionTransport, ParameterTransport, FaultServiceTransport,
     LogSource, TopicSubscriptionTransport).
   * gateway_ros2 - ROS-binding adapters (GatewayNode plus the
     Ros2*Transport implementations, TriggerTopicSubscriber,
     Ros2RuntimeIntrospection, NativeTopicSampler). Publicly links
     gateway_core. Class associations on this diagram remain valid;
     the boundary affects build-time only.
   end note

       class GatewayNode {
           + get_entity_cache(): EntityCache
           + get_data_access_manager(): DataAccessManager*
           + get_operation_manager(): OperationManager*
           + get_discovery_manager(): DiscoveryManager*
           + get_configuration_manager(): ConfigurationManager*
           + get_lock_manager(): LockManager*
           + get_script_manager(): ScriptManager*
           + get_trigger_manager(): TriggerManager*
       }

       class DiscoveryManager {
           + discover_areas(): vector<Area>
           + discover_components(): vector<Component>
           + discover_apps(): vector<App>
           + discover_services(): vector<ServiceInfo>
           + discover_actions(): vector<ActionInfo>
           + find_service_for_component(): optional<ServiceInfo>
           + find_action_for_component(): optional<ActionInfo>
       }

       interface DiscoveryStrategy <<interface>> {
           + discover_areas(): vector<Area>
           + discover_components(): vector<Component>
           + discover_apps(): vector<App>
           + discover_functions(): vector<Function>
           + get_name(): string
       }

       class Ros2RuntimeIntrospection {
           + introspect(IntrospectionInput): IntrospectionResult
           + discover_apps(): vector<App>
           + discover_functions(): vector<Function>
           - config_: RuntimeConfig
       }

       class MergePipeline {
           + add_layer(): void
           + execute(): MergeResult
           + set_linker(): void
           + get_last_report(): MergeReport
           - merge_entities<T>(): vector<T>
       }

       interface DiscoveryLayer <<interface>> {
           + name(): string
           + discover(): LayerOutput
           + policy_for(FieldGroup): MergePolicy
       }

       class ManifestLayer {
           - manifest_mgr_: ManifestManager*
       }

       class RuntimeLayer {
           - runtime_introspection_: Ros2RuntimeIntrospection*
           - gap_fill_config_: GapFillConfig
       }

       class PluginLayer {
           - provider_: IntrospectionProvider*
       }

       class RuntimeLinker {
           + link(): LinkingResult
       }

       class OperationManager {
           + call_service(): ServiceCallResult
           + call_component_service(): ServiceCallResult
           + send_action_goal(): ActionSendGoalResult
           + send_component_action_goal(): ActionSendGoalResult
           + cancel_action_goal(): ActionCancelResult
           + get_tracked_goal(): optional<ActionGoalInfo>
           + get_latest_goal_for_action(): optional<ActionGoalInfo>
           + cleanup_old_goals(): void
           + is_valid_uuid_hex(): bool {static}
           + is_service_type(): bool {static}
           + is_action_type(): bool {static}
       }

       class RESTServer {
           + start(): void
           + stop(): void
       }

       class DataAccessManager {
           + get_topic_sample_with_fallback(): json
           + get_component_data_with_fallback(): json
           + publish_to_topic(): json
           + get_topic_sample_native(): json
           + get_component_data_native(): json
       }

       class ConfigurationManager {
           + list_parameters(): ParameterResult
           + get_parameter(): ParameterResult
           + set_parameter(): ParameterResult
       }

       class LockManager {
           + acquire(): expected<LockInfo, LockError>
           + release(): expected<void, LockError>
           + extend(): expected<LockInfo, LockError>
           + check_access(): LockAccessResult
           + cleanup_expired(): vector<ExpiredLockInfo>
           + get_lock(): optional<LockInfo>
       }

       class ScriptManager {
           + set_backend(): void
           + has_backend(): bool
           + list_scripts(): expected<vector<ScriptInfo>, Error>
           + get_script(): expected<ScriptInfo, Error>
           + upload_script(): expected<ScriptUploadResult, Error>
           + delete_script(): expected<void, Error>
           + start_execution(): expected<ExecutionInfo, Error>
           + get_execution(): expected<ExecutionInfo, Error>
           + control_execution(): expected<ExecutionInfo, Error>
           + delete_execution(): expected<void, Error>
       }

       interface TopicDataProvider <<interface>> {
           + sample(): expected<TopicSampleResult, ErrorInfo>
           + sample_parallel(): expected<vector<TopicSampleResult>, ErrorInfo>
           + discover_all(): vector<TopicInfo>
           + has_publishers(): bool
       }

       class Ros2TopicDataProvider {
           + sample(): expected<TopicSampleResult, ErrorInfo>
           + sample_parallel(): expected<vector<TopicSampleResult>, ErrorInfo>
           + discover_all(): vector<TopicInfo>
           + x_medkit_stats(): json
       }

       class Ros2SubscriptionExecutor {
           + run_sync(): expected<R, string>
           + post(): bool
           + on_graph_change(): void
       }

       class JsonSerializer {
           + serialize(): SerializedMessage
           + deserialize(): json
           + to_json(): json
           + from_json(): RosMessage_Cpp
       }

       class Area {
           + id: string
           + namespace_path: string
           + type: string
           + to_json(): json
       }

       class Component {
           + id: string
           + namespace_path: string
           + fqn: string
           + type: string
           + area: string
           + source: string
           + services: vector<ServiceInfo>
           + actions: vector<ActionInfo>
           + to_json(): json
       }

       class App {
           + id: string
           + name: string
           + namespace_path: string
           + area: string
           + component: string
           + source: string
           + to_json(): json
       }

       class ServiceInfo {
           + full_path: string
           + name: string
           + type: string
       }

       class ActionInfo {
           + full_path: string
           + name: string
           + type: string
       }

       class TriggerManager {
           + create(): expected<TriggerInfo, string>
           + get(): optional<TriggerInfo>
           + list(): vector<TriggerInfo>
           + update(): expected<TriggerInfo, string>
           + remove(): bool
           + wait_for_event(): bool
           + consume_pending_event(): optional<json>
           + load_persistent_triggers(): void
           + shutdown(): void
       }

       class ResourceChangeNotifier {
           + subscribe(): NotifierSubscriptionId
           + unsubscribe(): void
           + notify(): void
           + shutdown(): void
       }

       class ConditionRegistry {
           + register_condition(): void
           + get(): shared_ptr<ConditionEvaluator>
           + has(): bool
       }

       interface ConditionEvaluator <<interface>> {
           + evaluate(): bool
           + validate_params(): expected<void, string>
       }

       interface TriggerStore <<interface>> {
           + save(): expected<void, string>
           + update(): expected<void, string>
           + remove(): expected<void, string>
           + load_all(): expected<vector<TriggerInfo>, string>
       }

       class SqliteTriggerStore {
           - db_path_: string
       }

       class TriggerTopicSubscriber {
           + subscribe(topic, type, handle_key, cb): void
           + unsubscribe(handle_key): void
           + set_retry_callback(cb): void
       }

       class EntityCache {
           + areas: vector<Area>
           + components: vector<Component>
           + apps: vector<App>
           + last_update: time_point
       }
   }

   package "External Libraries" {
       class "httplib::Server" as HTTPLibServer
       class "nlohmann::json" as JSON
   }

   ' Relationships

   ' Inheritance
   GatewayNode -up-|> "rclcpp::Node" : extends

   ' Composition (Gateway owns these)
   GatewayNode *-down-> DiscoveryManager : owns
   GatewayNode *-down-> RESTServer : owns
   GatewayNode *-down-> DataAccessManager : owns
   GatewayNode *-down-> OperationManager : owns
   GatewayNode *-down-> ConfigurationManager : owns
   GatewayNode *-down-> LockManager : owns
   GatewayNode *-down-> ScriptManager : owns
   GatewayNode *-down-> EntityCache : owns

   ' Discovery Manager uses Node interface
   DiscoveryManager --> "rclcpp::Node" : uses

   ' REST Server references Gateway, DataAccessManager, OperationManager, and ConfigurationManager
   RESTServer --> GatewayNode : uses
   RESTServer --> DataAccessManager : uses
   RESTServer --> OperationManager : uses
   RESTServer --> ConfigurationManager : uses
   RESTServer --> ScriptManager : uses

   ' OperationManager routes via Service/Action transports
   OperationManager --> DiscoveryManager : uses
   OperationManager --> ServiceTransport : uses (via transport port)
   OperationManager --> ActionTransport : uses (via transport port)

   ' DataAccessManager routes via TopicTransport; serialization moves to the transport
   DataAccessManager --> TopicTransport : uses (via transport port)
   DataAccessManager ..> TopicDataProvider : uses (non-owning)

   ' Ros2TopicDataProvider is the default implementation of TopicDataProvider
   Ros2TopicDataProvider ..|> TopicDataProvider
   Ros2TopicDataProvider *--> Ros2SubscriptionExecutor : uses
   Ros2SubscriptionExecutor --> "rclcpp::Node" : uses (subscription node)

   ' ConfigurationManager routes parameter CRUD via ParameterTransport
   ConfigurationManager --> ParameterTransport : uses (via transport port)

   ' Entity Cache aggregates entities
   EntityCache o-right-> Area : contains many
   EntityCache o-right-> Component : contains many
   EntityCache o-right-> App : contains many

   ' Component contains operations
   Component o--> ServiceInfo : contains many
   Component o--> ActionInfo : contains many

   ' Discovery produces entities
   DiscoveryManager ..> Area : creates
   DiscoveryManager ..> Component : creates
   DiscoveryManager ..> App : creates
   DiscoveryManager ..> ServiceInfo : creates
   DiscoveryManager ..> ActionInfo : creates

   ' Discovery strategy hierarchy
   DiscoveryManager *--> Ros2RuntimeIntrospection : owns
   DiscoveryManager *--> MergePipeline : owns (hybrid mode)
   Ros2RuntimeIntrospection .up.|> IntrospectionProvider : implements

   ' MergePipeline layer architecture
   MergePipeline o--> DiscoveryLayer : ordered layers
   MergePipeline --> RuntimeLinker : post-merge linking
   ManifestLayer .up.|> DiscoveryLayer : implements
   RuntimeLayer .up.|> DiscoveryLayer : implements
   PluginLayer .up.|> DiscoveryLayer : implements
   RuntimeLayer --> Ros2RuntimeIntrospection : delegates

   ' REST Server uses HTTP library
   RESTServer *--> HTTPLibServer : owns

   ' Models use JSON for serialization
   Area ..> JSON : serializes to
   Component ..> JSON : serializes to
   App ..> JSON : serializes to

   ' Trigger subsystem
   GatewayNode *-down-> TriggerManager : owns
   GatewayNode *-down-> ResourceChangeNotifier : owns
   GatewayNode *-down-> ConditionRegistry : owns
   GatewayNode *-down-> TriggerTopicSubscriber : owns
   TriggerManager --> ResourceChangeNotifier : subscribes to
   TriggerManager --> ConditionRegistry : evaluates with
   TriggerManager --> TriggerStore : persists via
   TriggerManager --> "TopicSubscriptionTransport" : data subscriptions via transport
   SqliteTriggerStore .up.|> TriggerStore : implements
   ConditionRegistry o--> ConditionEvaluator : contains many
   RESTServer --> TriggerManager : uses
   TriggerTopicSubscriber --> "rclcpp::Node" : uses

   @enduml

Testing with Mock Transports
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The Transport ports under ``core/transports/`` are abstract interfaces. A
manager unit test injects a mock implementation and links only against
``gateway_core`` + GTest, with no ``rclcpp`` on the link line. This is the
pattern used by the ``test_*_manager_routing.cpp`` tests:

.. code-block:: cpp

   class MockTopicTransport : public TopicTransport {
    public:
     nlohmann::json publish(const std::string &, const std::string &,
                            const nlohmann::json &,
                            std::chrono::duration<double>) override {
       return nlohmann::json::object();
     }
     TopicSample sample(const std::string & topic,
                        std::chrono::duration<double>) override {
       TopicSample s;
       s.success = true;
       s.status = "data";
       s.topic = topic;
       s.data = next_sample_;
       return s;
     }
     std::pair<uint64_t, uint64_t>
     count_publishers_subscribers(const std::string &) const override {
       return {1, 0};
     }
     ros2_medkit_serialization::TypeIntrospection *
     get_type_introspection() const override { return nullptr; }

     nlohmann::json next_sample_ = nlohmann::json::object();
   };

   TEST(DataAccessManagerRouting, ReadRoutesThroughTransport) {
     MockTopicTransport transport;
     transport.next_sample_["value"] = 42;
     DataAccessManager mgr(&transport, /* other neutral deps */);

     auto result = mgr.get_topic_sample_with_fallback("/sensor", "");
     ASSERT_EQ(result["data"]["value"], 42);
   }

The mock is purely C++; no ROS context, executor, or domain ID is needed.

Main Components
---------------

Each entry below is tagged with the static library it compiles into:
``[gateway_core]`` (middleware-neutral, no ROS dependency) or
``[gateway_ros2]`` (links rclcpp / message packages).

1. **GatewayNode** ``[gateway_ros2]`` - The main ROS 2 node that orchestrates the system
   - Extends ``rclcpp::Node``
   - Manages periodic discovery and cache refresh
   - Runs the REST server in a separate thread
   - Provides thread-safe access to the entity cache
   - Manages periodic cleanup of old action goals (60s interval)

2. **DiscoveryManager** ``[gateway_ros2]`` - Discovers ROS 2 entities and maps them to the SOVD hierarchy
   - Discovers Areas from node namespaces or manifest definitions
   - Discovers Components (synthetic groups from runtime, or explicit from manifest)
   - Discovers Apps from ROS 2 nodes (individual running processes)
   - Discovers Services and Actions using native rclcpp APIs
   - Attaches operations (services/actions) to their parent Apps and Components
   - Routes built-in graph queries through the same ``IntrospectionProvider`` chain used by plugins
   - Uses O(n+m) algorithm with hash maps for efficient service/action attachment

   **Discovery providers and layers:**

   - **Ros2RuntimeIntrospection** ``[gateway_ros2]`` - IntrospectionProvider wrapping ROS 2 graph queries
     - Maps nodes to Apps with ``source: "heuristic"``
     - Creates Functions from namespace grouping
     - Never creates Areas or Components (those come from manifest/HostInfoProvider)
     - Same interface as plugin-provided IntrospectionProviders, so the merge pipeline treats built-in graph queries identically to plugin contributions
   - **ManifestLayer** ``[gateway_core]`` - Static layer fed by ``ManifestManager``
     - Provides stable, semantic entity IDs from declarative YAML
     - Supports offline detection of failed components
     - ``manifest_only`` mode bypasses the merge pipeline and routes directly through ``ManifestManager``
   - **Hybrid mode (DiscoveryManager + MergePipeline)** - Combines manifest + runtime + plugins
     - DiscoveryManager constructs a ``MergePipeline`` with the configured layers
     - Supports dynamic plugin layers added at runtime via ``add_plugin_layer()``
     - Thread-safe: a mutex protects the cached merged result, all reads return by value

   **Merge Pipeline:**

   The ``MergePipeline`` is the core engine for hybrid discovery. It:

   - Maintains an ordered list of ``DiscoveryLayer`` instances (first = highest priority)
   - Executes all layers, collects entities by ID, and merges them per-field-group
   - Each layer declares a ``MergePolicy`` per ``FieldGroup``: AUTHORITATIVE (wins), ENRICHMENT (fills empty), FALLBACK (last resort)
   - Runs ``RuntimeLinker`` post-merge to bind manifest apps to live ROS 2 nodes
   - Suppresses runtime-origin entities that duplicate manifest entities: components/areas
     by namespace match, apps by ID match (gap-fill apps in uncovered namespaces survive)
   - Produces a ``MergeReport`` with conflict diagnostics, enrichment counts, and ID collision detection

   **Built-in Layers:**

   - ``ManifestLayer`` - Wraps ManifestManager; IDENTITY/HIERARCHY/METADATA are AUTHORITATIVE,
     LIVE_DATA is ENRICHMENT (runtime wins for topics/services), STATUS is FALLBACK
   - ``RuntimeLayer`` - Wraps Ros2RuntimeIntrospection; LIVE_DATA/STATUS are AUTHORITATIVE,
     METADATA is ENRICHMENT, IDENTITY/HIERARCHY are FALLBACK.
     Supports ``GapFillConfig`` to control which heuristic entities are allowed when manifest is present
   - ``PluginLayer`` - Wraps IntrospectionProvider; all fields ENRICHMENT (plugins enrich, they don't override).
     Before each layer's ``discover()`` call, the pipeline populates ``IntrospectionInput`` with entities
     from all previous layers, so plugins see the current manifest + runtime entity set

3. **OperationManager** ``[gateway_core]`` - Routes SOVD operation execution through ``ServiceTransport`` and ``ActionTransport`` ports
   - Manager body is middleware-neutral; ``Ros2ServiceTransport`` and ``Ros2ActionTransport`` adapters perform the actual ``rclcpp::GenericClient`` / action-client calls
   - Tracks active action goals with status, feedback, and timestamps
   - Subscribes to ``/_action/status`` topics via the action transport for real-time goal status updates
   - Supports goal cancellation via the action transport's cancel path
   - Supports SOVD capability-based control (stop maps to ROS 2 cancel)
   - Automatically cleans up completed goals older than 5 minutes
   - JSON ↔ ROS 2 message conversion is performed inside the ROS-side transport adapter via ``ros2_medkit_serialization``

4. **RESTServer** ``[gateway_ros2]`` - Provides the HTTP/REST API (route table couples to gateway lifecycle; the individual handlers it dispatches to live in ``gateway_core``)
   - Discovery endpoints: ``/health``, ``/areas``, ``/components``
   - Data endpoints: ``/components/{id}/data``, ``/components/{id}/data/{topic}``
   - Operations endpoints: ``/apps/{id}/operations``, ``/apps/{id}/operations/{op}/executions``
   - Configurations endpoints: ``/apps/{id}/configurations``, ``/apps/{id}/configurations/{param}``
   - Scripts endpoints: ``/{entity_type}/{id}/scripts``, ``/{entity_type}/{id}/scripts/{script_id}/executions``
   - Retrieves cached entities from the GatewayNode
   - Uses DataAccessManager for runtime topic data access
   - Uses OperationManager for service/action execution
   - Uses ConfigurationManager for parameter CRUD operations
   - Uses ScriptManager for script upload and execution
   - Runs on configurable host and port with CORS support

5. **ConfigurationManager** ``[gateway_core]`` - Routes SOVD configuration CRUD through ``ParameterTransport``
   - Manager body is middleware-neutral; ``Ros2ParameterTransport`` adapter performs the ``rclcpp::AsyncParametersClient`` calls (explicit ``spin_until_future_complete`` so a service TIMEOUT is distinguished from a successful-but-empty response)
   - Lists / gets / sets individual parameter values with type conversion
   - Provides parameter descriptors (description, constraints, read-only flag)
   - The transport caches parameter clients per node for efficiency
   - JSON ↔ ROS 2 parameter conversion happens inside the transport adapter

6. **DataAccessManager** ``[gateway_core]`` - Routes SOVD data read/write through ``TopicTransport``
   - Manager body is middleware-neutral; ``Ros2TopicTransport`` adapter performs ``rclcpp::GenericPublisher``-backed publishing and CDR serialization
   - Delegates topic sampling to the attached ``TopicDataProvider`` (pool-backed, race-free)
   - Checks publisher counts before sampling to skip idle topics instantly
   - Returns metadata (type, schema) for topics without publishers
   - Returns topic data as JSON with metadata (topic name, timestamp, type info)
   - Parallel topic sampling with configurable concurrency limit (``data_provider.max_parallel_samples`` on the TopicDataProvider, default: 8)

7. **TopicDataProvider** / **Ros2TopicDataProvider** ``[gateway_core / gateway_ros2]`` - The interface lives in the neutral layer (``core/providers/data_provider.hpp``); the pool-backed ROS 2 default implementation lives in the adapter layer
   - ``TopicDataProvider`` is a pure C++ interface consumed by HTTP handlers and managers, with no rclcpp headers required on the consumer side
   - ``Ros2TopicDataProvider`` keeps one shared subscription per topic and serves many sample calls from the cached latest message, avoiding the rcl hash-map race that short-lived per-sample subscriptions used to trigger
   - All subscription / callback-group creation and destruction runs on ``Ros2SubscriptionExecutor``'s single worker thread
   - LRU cap, idle safety-net sweep, graph-change eviction, cold-wait cap for cpp-httplib liveness, and publisher-matching QoS (reliable / transient_local)
   - Exposes pool + executor stats on ``GET /health`` under the ``x-medkit-subscription-executor`` and ``x-medkit-data-provider`` vendor-extension keys
   - See :doc:`ros2_subscription_architecture` for the full design

8. **JsonSerializer** (ros2_medkit_serialization) - Converts between JSON and ROS 2 messages (separate package, not part of the gateway layer split)
   - Uses ``dynmsg`` library for dynamic type introspection
   - Serializes JSON to CDR format for publishing via ``serialize()``
   - Deserializes CDR to JSON for subscriptions via ``deserialize()``
   - Converts between deserialized ROS 2 messages and JSON via ``to_json()`` / ``from_json()``
   - Provides static ``yaml_to_json()`` utility for YAML to JSON conversion
   - Thread-safe and stateless design

9. **LockManager** ``[gateway_core]`` - SOVD resource locking (ISO 17978-3, Section 7.17)
    - Transport-agnostic lock store with ``shared_mutex`` for thread safety
    - Acquire, release, extend locks on components and apps
    - Scoped locks restrict protection to specific resource collections
    - Lazy parent propagation (lock on component protects child apps)
    - Lock-required enforcement via per-entity ``required_scopes`` config
    - Automatic expiry with cyclic subscription cleanup
    - Plugin access via ``PluginContext::check_lock/acquire_lock/release_lock``

10. **Data Models** ``[gateway_core]`` - Entity representations
    - ``Area`` - Physical or logical domain (namespace grouping)
    - ``Component`` - Logical grouping of Apps; can be ``synthetic`` (auto-created from namespace), ``topic`` (from topic-only namespace), or ``manifest`` (explicitly defined)
    - ``App`` - Software application (ROS 2 node); individual running process linked to parent Component
    - ``ServiceInfo`` - Service metadata (path, name, type)
    - ``ActionInfo`` - Action metadata (path, name, type)
    - ``EntityCache`` - Thread-safe cache of discovered entities (areas, components, apps)

11. **ScriptManager** ``[gateway_ros2]`` - Manages diagnostic script upload, storage, and execution (SOVD 7.15)
    - Delegates to a pluggable ``ScriptProvider`` backend (set via ``set_backend()``)
    - Lists, uploads, and deletes scripts per entity
    - Starts script executions as POSIX subprocesses with timeout support
    - Tracks execution status (``prepared``, ``running``, ``completed``, ``failed``, ``terminated``)
    - Supports termination via ``stop`` (SIGTERM) and ``forced_termination`` (SIGKILL)
    - Built-in ``DefaultScriptProvider`` handles filesystem storage and manifest-defined scripts
    - Supports concurrent execution limits and per-script timeout configuration

Triggers
--------

The trigger subsystem implements SOVD condition-based resource change notifications.
It consists of five main components:

1. **TriggerManager** ``[gateway_core]`` - Central coordinator for trigger lifecycle (CRUD), condition
   evaluation, and event dispatch.

   - Subscribes to ``ResourceChangeNotifier`` for resource change events
   - Evaluates conditions using registered ``ConditionEvaluator`` instances via the ``ConditionRegistry``
   - Uses O(1) dispatch indexing by ``{collection, entity_id}`` for efficient notification matching
   - Supports entity hierarchy matching (area-level triggers catch descendant changes)
   - Manages pending events for SSE stream pickup with per-trigger mutexes
   - Persists triggers via the ``TriggerStore`` interface
   - Data-trigger ROS subscriptions are routed through ``TopicSubscriptionTransport`` (``Ros2TopicSubscriptionTransport`` adapter wraps ``TriggerTopicSubscriber``)

2. **ResourceChangeNotifier** ``[gateway_core]`` - Async notification hub for resource changes.

   - Producers (FaultManager, DataAccessManager, UpdateManager, OperationManager, LogManager) call ``notify()``
   - Observers (TriggerManager) register callbacks with filters
   - ``notify()`` is non-blocking - pushes to an internal queue processed by a dedicated worker thread
   - Filters support collection, entity_id, and resource_path matching

3. **ConditionRegistry** ``[gateway_core]`` - Thread-safe registry for condition evaluators.

   - Built-in SOVD types: ``OnChange``, ``OnChangeTo``, ``EnterRange``, ``LeaveRange``
   - Plugins register custom evaluators with ``x-`` prefixed names
   - Uses ``shared_mutex`` for concurrent read access during evaluation

4. **TriggerStore** / **SqliteTriggerStore** ``[gateway_core]`` - Persistence backend for triggers.

   - Abstract ``TriggerStore`` interface allows plugin-provided backends
   - Default ``SqliteTriggerStore`` uses SQLite for persistent triggers
   - Stores trigger metadata, condition parameters, and evaluator state (previous values)
   - Supports partial updates for status changes and lifetime extensions

5. **TriggerTopicSubscriber** ``[gateway_ros2]`` - Generic per-handle subscription executor for data triggers.

   - Creates one ``rclcpp::GenericSubscription`` per ``handle_key`` provided by the caller
   - Each trigger owns its own subscription handle - no reference-counting across triggers
   - Wrapped by ``Ros2TopicSubscriptionTransport``, which exposes the neutral
     ``TopicSubscriptionTransport`` interface to ``TriggerManager``
   - Subscription lifetime is tied to the trigger entry: removing the trigger drops the
     handle and tears down the underlying subscription
   - Per-handle callbacks publish samples back to ``ResourceChangeNotifier`` for
     condition evaluation

Additional Design Documents
----------------------------

.. toctree::
   :maxdepth: 1

   aggregation
   dto_contract
   entity_cache_architecture
   hardening
   lifecycle
   openapi_derivation
   plugin_entity_notifications
   ros2_subscription_architecture
