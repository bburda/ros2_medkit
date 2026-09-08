Discovery Options Reference
===========================

.. contents::
   :local:
   :depth: 2

This document describes configuration options for the gateway's discovery system.
The discovery system maps ROS 2 graph entities (nodes, topics, services, actions)
to SOVD entities (areas, components, apps, functions).

Discovery Modes
---------------

The gateway supports three discovery modes, controlled by the ``discovery.mode`` parameter:

.. list-table:: Discovery Modes
   :header-rows: 1
   :widths: 20 80

   * - Mode
     - Description
   * - ``runtime_only``
     - Use ROS 2 graph introspection only. Nodes are discovered at runtime
       and mapped to SOVD entities using heuristic rules. (default)
   * - ``manifest_only``
     - Only expose entities declared in the manifest file. Runtime discovery
       is disabled.
   * - ``hybrid``
     - Manifest defines the structure, runtime links to discovered nodes.

Runtime Discovery Options
-------------------------

When using ``runtime_only`` or ``hybrid`` mode, the following options control
how ROS 2 nodes are mapped to SOVD entities.

Entity Model
^^^^^^^^^^^^

.. important::

   The entity model changed in this release. Synthetic Areas and per-namespace
   Components are no longer created. If you are upgrading from a previous
   version, see :ref:`Breaking Changes <aggregation-breaking-changes>` for
   details, removed parameters, cross-layer impact, and migration guidance.

In runtime mode, the gateway maps the ROS 2 graph to SOVD entities as follows:

- **Areas** - not created by runtime discovery. Areas come from manifest only.
- **Components** - a single host-level Component is created from
  ``HostInfoProvider`` (see Default Component below). No synthetic/heuristic
  Components are created from namespaces.
- **Apps** - each ROS 2 node becomes an App with ``source: "heuristic"``.
- **Functions** - namespace grouping creates Function entities (see below).

Default Component
^^^^^^^^^^^^^^^^^

.. code-block:: yaml

   discovery:
     runtime:
       default_component:
         enabled: true

When ``default_component.enabled`` is true (the default), the gateway creates
a single host-level Component from the local system info (hostname, OS,
architecture) via ``HostInfoProvider``. All discovered Apps are linked to this
Component.

Internal Node Filtering
^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: yaml

   discovery:
     runtime:
       filter_internal_nodes: true

When ``filter_internal_nodes`` is true (the default), ROS 2 nodes whose names
start with an underscore (``_``) are excluded from the entity tree. This
filters out ROS 2 internal infrastructure nodes such as ``_ros2cli_*``,
``_param_client_node``, and similar system nodes that should not appear as
SOVD entities. The filter applies to both locally discovered Apps and
peer-discovered Apps (after stripping the peer prefix).

Set to ``false`` if you need to expose all ROS 2 nodes regardless of naming
convention.

Function Entities from Namespaces
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: yaml

   discovery:
     runtime:
       create_functions_from_namespaces: true

When ``create_functions_from_namespaces`` is true (the default), each ROS 2
namespace becomes a Function entity that groups the Apps in that namespace.
This is the SOVD-correct mapping where Functions represent logical capabilities
(what the software does) rather than deployment topology.

Merge Pipeline (Hybrid Mode)
-----------------------------

In hybrid mode, the gateway uses a layered merge pipeline to combine entities
from multiple sources. Three layers contribute entities independently:

- **ManifestLayer** - entities declared in the YAML manifest (highest priority for identity/hierarchy)
- **RuntimeLayer** - entities discovered from the ROS 2 graph (highest priority for live data/status)
- **PluginLayer** - entities contributed by ``IntrospectionProvider`` plugins

Each layer's contribution is merged per **field-group** with configurable policies.

Layer Enable/Disable
^^^^^^^^^^^^^^^^^^^^

Individual layers can be disabled without changing the discovery mode.
This is useful when running beacon plugins as the sole enrichment source
(disable manifest and/or runtime layers, keep only plugins):

.. code-block:: yaml

   discovery:
     manifest:
       enabled: true    # Default: true
     runtime:
       enabled: true    # Default: true

When a layer is disabled, its entities are not discovered and its merge
contributions are skipped. Plugins always run regardless of these flags.

Manifest Loading
^^^^^^^^^^^^^^^^

.. list-table::
   :header-rows: 1
   :widths: 34 8 14 44

   * - Parameter
     - Type
     - Default
     - Description
   * - ``discovery.manifest_strict_validation``
     - bool
     - ``true``
     - Treat manifest validation WARNINGS as fatal. Validation errors (broken
       references, circular dependencies, duplicate bindings) always abort the
       load whatever this is set to, and a manifest is only ever applied whole,
       so turning this off never loads a partly-valid manifest - it only lets
       warnings through. Turn it off to bring up a manifest that is still being
       written.
   * - ``discovery.manifest.fragments_dir``
     - string
     - ``""``
     - Directory of manifest fragment YAML files. Every load and reload of the
       manifest scans it and merges the apps, components and functions it finds
       on top of the base manifest, so a fleet can ship one base file plus
       per-robot fragments. Empty disables fragment loading.

Field Groups
^^^^^^^^^^^^

.. list-table::
   :header-rows: 1
   :widths: 20 80

   * - Field Group
     - Contents
   * - ``identity``
     - id, name, translation_id, description, tags
   * - ``hierarchy``
     - area, component_id, parent references, depends_on, hosts
   * - ``live_data``
     - topics, services, actions
   * - ``status``
     - is_online, bound_fqn
   * - ``metadata``
     - ros_binding, external, x-medkit extensions, custom metadata fields.
       **Not** ``source``: an entity's provenance decides whether the orphan
       filter may delete it, so it is never negotiated between layers and
       setting ``layers.manifest.metadata`` does not move it.

Merge Policies
^^^^^^^^^^^^^^

Each layer declares a policy per field-group:

.. list-table::
   :header-rows: 1
   :widths: 20 80

   * - Policy
     - Behavior
   * - ``authoritative``
     - This layer's value wins over lower-priority layers.
   * - ``enrichment``
     - Fill empty fields only; don't override existing values.
   * - ``fallback``
     - Use only if no other layer provides the value.

**Defaults:**

- Manifest: ``authoritative`` for identity/hierarchy/metadata, ``enrichment`` for live_data, ``fallback`` for status
- Runtime: ``authoritative`` for live_data/status, ``enrichment`` for metadata, ``fallback`` for identity/hierarchy

.. note::

   The app ``external`` classification is tri-state (unset / ``true`` /
   ``false``), so it follows the policy table with one refinement: a layer that
   does not classify the app (a manifest entry that omits ``external:``) never
   clears or shadows a classification another layer contributed. A manifest stub
   that omits ``external:`` therefore keeps a protocol plugin's introspected
   ``external: true`` (so the app still owns its faults via bare-id fault scope
   in hybrid mode). An *explicit* value is authoritative and resolves by normal
   layer priority: an authoritative manifest ``external: false`` overrides a
   plugin's ``external: true``. Only omission is a no-op; an explicit value is
   never silently discarded.

   The same tri-state field and merge priority apply to Components: an
   external Component with no bound child Apps owns its fault-scope by entity
   id, so an Area or Function hosting it rolls up its faults without a
   synthetic child App. Components have no ``ros_binding``, so there is no
   equivalent to the App's R013 contradiction check.

Override per-layer policies in ``gateway_params.yaml``. Empty string means
"use layer default". Policy values are **case-sensitive** and must be lowercase
(``authoritative``, ``enrichment``, ``fallback``):

.. code-block:: yaml

   discovery:
     merge_pipeline:
       layers:
         manifest:
           identity: ""          # authoritative (default)
           hierarchy: ""         # authoritative (default)
           live_data: ""         # enrichment (default)
           status: ""            # fallback (default)
           metadata: ""          # authoritative (default)
         runtime:
           identity: ""          # fallback (default)
           hierarchy: ""         # fallback (default)
           live_data: ""         # authoritative (default)
           status: ""            # authoritative (default)
           metadata: ""          # enrichment (default)

Gap-Fill
^^^^^^^^

In hybrid mode, the runtime layer can create heuristic entities for namespaces
not covered by the manifest. Gap-fill controls what the runtime layer is allowed
to create:

.. code-block:: yaml

   discovery:
     merge_pipeline:
       gap_fill:
         allow_heuristic_apps: true
         allow_heuristic_functions: false
         # namespace_blacklist: ["/rosout"]
         # namespace_whitelist: []

.. note::

   Areas and Components are never created by runtime discovery. Areas come
   from manifest only. Components come from ``HostInfoProvider`` or manifest.

.. list-table::
   :header-rows: 1
   :widths: 35 15 50

   * - Parameter
     - Default
     - Description
   * - ``allow_heuristic_apps``
     - ``true``
     - Create apps for nodes without manifest ``ros_binding``.
   * - ``allow_heuristic_functions``
     - ``false``
     - Create heuristic functions (not recommended).
   * - ``namespace_blacklist``
     - ``[]``
     - Namespaces excluded from gap-fill (e.g., ``["/rosout"]``).
   * - ``namespace_whitelist``
     - ``[]``
     - If non-empty, only these namespaces are eligible for gap-fill.

Health Endpoint
^^^^^^^^^^^^^^^

``GET /api/v1/health`` includes a ``discovery`` section in hybrid mode with
pipeline stats and linking results. The ``warnings`` array and
``warning_schema_version`` are unconditional - they are served in every mode,
and an empty array means the gateway has no anomaly to report (see
:doc:`/api/warning_codes`). ``discovery.linking`` is the conditional part: it
appears only when a manifest is loaded, and ``unmanifested_policy`` carries the
configured :ref:`unmanifested_nodes <manifest-unmanifested-nodes>` value so a
client can evaluate ``orphan_count > 0 && unmanifested_policy == "error"``
without parsing prose.

.. code-block:: json

   {
     "status": "healthy",
     "warnings": [],
     "warning_schema_version": 2,
     "discovery": {
       "mode": "hybrid",
       "strategy": "hybrid",
       "pipeline": {
         "layers": ["manifest", "runtime"],
         "total_entities": 9,
         "enriched_count": 1,
         "conflict_count": 0,
         "conflicts": [],
         "id_collisions": 0,
         "filtered_by_gap_fill": 0
       },
       "linking": {
         "linked_count": 2,
         "orphan_count": 5,
         "binding_conflicts": 0,
         "unmanifested_policy": "warn"
       }
     }
   }

The response also carries the ``x-medkit-data-provider``,
``x-medkit-entity-cache`` and ``x-medkit-subscription-executor`` statistics
objects, omitted here. ``layers`` lists one entry per active layer, and a
discovery plugin contributes under its own registered name (for example
``"opcua"``) rather than a literal ``"plugin"``.

Configuration Example
---------------------

Complete YAML configuration for runtime discovery:

.. code-block:: yaml

   ros2_medkit_gateway:
     ros__parameters:
       discovery:
         mode: "runtime_only"

         runtime:
           # Function entities from namespace grouping (default: true)
           create_functions_from_namespaces: true

           # Single host-level Component (default: true)
           default_component:
             enabled: true

         # Merge pipeline (hybrid mode only)
         merge_pipeline:
           gap_fill:
             allow_heuristic_apps: true
             namespace_blacklist: ["/rosout"]

Command Line Override
---------------------

Override discovery options via command line:

.. code-block:: bash

   ros2 launch ros2_medkit_gateway gateway.launch.py \
     discovery.runtime.create_functions_from_namespaces:=false

Discovery Mechanism Selection
-----------------------------

.. list-table::
   :header-rows: 1
   :widths: 40 60

   * - Scenario
     - Recommended mechanism
   * - Quick start, no configuration needed
     - ``runtime_only`` (default)
   * - Stable system with known topology
     - ``manifest_only`` or ``hybrid``
   * - Runtime metadata from nodes (fast updates, custom topics)
     - TopicBeaconPlugin (push-based)
   * - Runtime metadata from nodes (no custom code, uses parameters)
     - ParameterBeaconPlugin (pull-based)
   * - Structured topology with runtime enrichment
     - ``hybrid`` + beacon plugin(s)

**Topic beacon vs parameter beacon:**

- Use **TopicBeaconPlugin** when nodes can publish ``MedkitDiscoveryHint`` messages
  to a shared topic. Best for low-latency updates and nodes that already have
  custom publishing logic.
- Use **ParameterBeaconPlugin** when nodes declare metadata as standard ROS 2
  parameters. No custom publishing code needed - the plugin polls parameters
  automatically.
- Both can run simultaneously. Each maintains a separate store, and both inject
  into the merge pipeline as enrichment layers.

Beacon Discovery Plugin (TopicBeaconPlugin)
--------------------------------------------

The ``ros2_medkit_topic_beacon`` plugin enriches discovered entities with
push-based metadata from ROS 2 nodes. Nodes publish
``ros2_medkit_msgs/msg/MedkitDiscoveryHint`` messages to
``/ros2_medkit/discovery`` and the plugin injects the data into the merge
pipeline as an ``ENRICHMENT`` layer.

Configuration
^^^^^^^^^^^^^

.. code-block:: yaml

   ros2_medkit_gateway:
     ros__parameters:
       plugins: ["topic_beacon"]
       plugins.topic_beacon.path: "/path/to/libtopic_beacon_plugin.so"

       # ROS 2 topic to subscribe for beacon hints
       # Default: "/ros2_medkit/discovery"
       plugins.topic_beacon.topic: "/ros2_medkit/discovery"

       # Soft TTL: hints older than this are marked STALE (seconds)
       # Default: 10.0
       plugins.topic_beacon.beacon_ttl_sec: 10.0

       # Hard expiry: hints older than this are removed (seconds)
       # Default: 300.0
       plugins.topic_beacon.beacon_expiry_sec: 300.0

       # Allow plugin to introduce entirely new entities not seen by other layers
       # When false (recommended), only known entities are enriched
       # Default: false
       plugins.topic_beacon.allow_new_entities: false

       # Maximum number of hints to keep in memory
       # Default: 10000
       plugins.topic_beacon.max_hints: 10000

       # Rate limit for incoming beacon messages
       # Default: 100
       plugins.topic_beacon.max_messages_per_second: 100

Beacon Lifecycle
^^^^^^^^^^^^^^^^

Each hint follows a soft TTL lifecycle based on the ``stamp`` field when
provided, otherwise based on reception time:

.. list-table::
   :header-rows: 1
   :widths: 15 85

   * - State
     - Description
   * - **ACTIVE**
     - Hint is within ``beacon_ttl_sec``. Enrichment is applied normally.
   * - **STALE**
     - Hint is past TTL but within ``beacon_expiry_sec``.
       Enrichment is still applied; the ``x-medkit-topic-beacon`` endpoint
       returns ``"status": "stale"`` in its response.
   * - **EXPIRED**
     - Hint is past ``beacon_expiry_sec``. It is removed from the store
       and no longer contributes to enrichment.

The ``x-medkit-topic-beacon`` vendor endpoint exposes current beacon state:

.. code-block:: bash

   curl -H "Authorization: Bearer $TOKEN" http://localhost:8080/api/v1/apps/engine_temp_sensor/x-medkit-topic-beacon

**Example Response:**

.. code-block:: json

   {
     "entity_id": "engine_temp_sensor",
     "status": "active",
     "age_sec": 1.234,
     "stable_id": "",
     "display_name": "Engine Temperature Sensor",
     "transport_type": "shared_memory",
     "negotiated_format": "",
     "process_id": 12345,
     "process_name": "sensor_node",
     "hostname": "robot-1",
     "component_id": "powertrain",
     "function_ids": ["monitoring"],
     "depends_on": [],
     "metadata": {"custom_key": "custom_value"}
   }

Beacon Metadata Keys
^^^^^^^^^^^^^^^^^^^^

When a beacon hint is applied to an entity, the following metadata keys are
injected into the entity's response (e.g., ``GET /api/v1/apps/{id}``).
These keys appear in the entity's ``metadata`` field, NOT in the vendor
extension endpoint response.

The keys are built by ``build_metadata()`` in ``beacon_entity_mapper.cpp``:

.. list-table::
   :header-rows: 1
   :widths: 40 15 45

   * - Key
     - Type
     - Description
   * - ``x-medkit-beacon-status``
     - string
     - Beacon state: ``"active"`` or ``"stale"``
   * - ``x-medkit-beacon-age-sec``
     - number
     - Seconds since the hint was last seen
   * - ``x-medkit-beacon-transport-type``
     - string
     - DDS transport type (e.g., ``"shared_memory"``, ``"nitros_zero_copy"``)
   * - ``x-medkit-beacon-negotiated-format``
     - string
     - Negotiated data format (e.g., ``"nitros_image_bgr8"``)
   * - ``x-medkit-beacon-process-id``
     - integer
     - OS process ID (PID)
   * - ``x-medkit-beacon-process-name``
     - string
     - Process name
   * - ``x-medkit-beacon-hostname``
     - string
     - Host identifier
   * - ``x-medkit-beacon-stable-id``
     - string
     - Stable identity alias
   * - ``x-medkit-beacon-depends-on``
     - array
     - Entity IDs this entity depends on
   * - ``x-medkit-beacon-functions``
     - array
     - Function IDs this entity belongs to
   * - ``x-medkit-beacon-<key>``
     - string
     - Freeform metadata from ``MedkitDiscoveryHint.metadata[]``

Parameter Beacon Plugin (ParameterBeaconPlugin)
------------------------------------------------

The ``ros2_medkit_param_beacon`` plugin enriches discovered entities with
pull-based metadata by polling ROS 2 node parameters. Unlike the topic beacon
which relies on nodes publishing to a shared topic, the parameter beacon
actively discovers and reads parameters matching a configurable prefix from
each known node.

This approach works well for nodes that already declare their metadata as
ROS 2 parameters (e.g., via ``declare_parameter()``) without needing any
custom publishing code.

Configuration
^^^^^^^^^^^^^

.. code-block:: yaml

   ros2_medkit_gateway:
     ros__parameters:
       plugins: ["parameter_beacon"]
       plugins.parameter_beacon.path: "/path/to/libparam_beacon_plugin.so"

       # Parameter name prefix to scan
       # Default: "ros2_medkit.discovery"
       plugins.parameter_beacon.parameter_prefix: "ros2_medkit.discovery"

       # How often to poll all nodes (seconds)
       # Default: 5.0
       plugins.parameter_beacon.poll_interval_sec: 5.0

       # Maximum time per poll cycle (seconds)
       # Default: 10.0
       plugins.parameter_beacon.poll_budget_sec: 10.0

       # Timeout for each node's parameter service call (seconds)
       # Default: 2.0
       plugins.parameter_beacon.param_timeout_sec: 2.0

       # Soft TTL: hints older than this are marked STALE (seconds)
       # Default: 15.0
       plugins.parameter_beacon.beacon_ttl_sec: 15.0

       # Hard expiry: hints older than this are removed (seconds)
       # Default: 300.0
       plugins.parameter_beacon.beacon_expiry_sec: 300.0

       # Allow plugin to introduce entirely new entities not seen by other layers
       # Default: false
       plugins.parameter_beacon.allow_new_entities: false

       # Maximum number of hints to keep in memory
       # Default: 10000
       plugins.parameter_beacon.max_hints: 10000

Parameter Naming
^^^^^^^^^^^^^^^^

Nodes declare parameters under the configured prefix (default:
``ros2_medkit.discovery``). Each parameter maps to a ``BeaconHint`` field:

.. list-table::
   :header-rows: 1
   :widths: 40 60

   * - Parameter
     - BeaconHint field
   * - ``ros2_medkit.discovery.entity_id`` (string, required)
     - ``entity_id``
   * - ``ros2_medkit.discovery.stable_id`` (string)
     - ``stable_id``
   * - ``ros2_medkit.discovery.display_name`` (string)
     - ``display_name``
   * - ``ros2_medkit.discovery.component_id`` (string)
     - ``component_id``
   * - ``ros2_medkit.discovery.transport_type`` (string)
     - ``transport_type``
   * - ``ros2_medkit.discovery.negotiated_format`` (string)
     - ``negotiated_format``
   * - ``ros2_medkit.discovery.process_name`` (string)
     - ``process_name``
   * - ``ros2_medkit.discovery.hostname`` (string)
     - ``hostname``
   * - ``ros2_medkit.discovery.process_id`` (integer)
     - ``process_id``
   * - ``ros2_medkit.discovery.function_ids`` (string array)
     - ``function_ids``
   * - ``ros2_medkit.discovery.depends_on`` (string array)
     - ``depends_on``
   * - ``ros2_medkit.discovery.metadata.<key>`` (string)
     - ``metadata[<key>]``

**Example node (C++):**

.. code-block:: cpp

   node->declare_parameter("ros2_medkit.discovery.entity_id", "my_sensor");
   node->declare_parameter("ros2_medkit.discovery.display_name", "Temperature Sensor");
   node->declare_parameter("ros2_medkit.discovery.function_ids",
       std::vector<std::string>{"thermal_monitoring"});
   node->declare_parameter("ros2_medkit.discovery.process_id",
       static_cast<int64_t>(getpid()));

Running Both Plugins
^^^^^^^^^^^^^^^^^^^^

The topic and parameter beacon plugins can be active simultaneously. Each
maintains its own ``BeaconHintStore`` and contributes independently to the
merge pipeline.

When both are active, align TTL and expiry values to avoid inconsistent
staleness behavior:

.. code-block:: yaml

   ros2_medkit_gateway:
     ros__parameters:
       plugins: ["topic_beacon", "parameter_beacon"]
       plugins.topic_beacon.path: "/path/to/libtopic_beacon_plugin.so"
       plugins.topic_beacon.beacon_ttl_sec: 10.0
       plugins.topic_beacon.beacon_expiry_sec: 300.0

       plugins.parameter_beacon.path: "/path/to/libparam_beacon_plugin.so"
       plugins.parameter_beacon.poll_interval_sec: 5.0
       plugins.parameter_beacon.beacon_ttl_sec: 15.0
       plugins.parameter_beacon.beacon_expiry_sec: 300.0

See Also
--------

- :doc:`manifest-schema` - Manifest-based configuration
- :doc:`/tutorials/heuristic-apps` - Tutorial on runtime discovery
- :doc:`/tutorials/manifest-discovery` - Hybrid mode tutorial
- :doc:`/tutorials/plugin-system` - Plugin layer integration
