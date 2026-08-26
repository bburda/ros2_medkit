Beacon Discovery Plugins
========================

This tutorial explains how to use the beacon discovery plugins to enrich
SOVD entities with runtime metadata from your ROS 2 nodes. Beacons let nodes
self-report identity, topology, transport, and process information that the
gateway cannot infer from the ROS 2 graph alone.

.. contents:: Table of Contents
   :local:
   :depth: 2

Overview
--------

The gateway's built-in runtime discovery maps ROS 2 nodes to SOVD entities
automatically, but the information it can extract is limited to what the
ROS 2 graph API exposes - node names, namespaces, topics, services, and
actions. Details like transport type (shared memory, zero-copy), process IDs,
host identifiers, logical function membership, and human-friendly display
names are invisible to the graph API.

**Beacon discovery** fills this gap. It provides two complementary plugins
that collect runtime metadata from nodes and inject it into the gateway's
entity model via the merge pipeline:

.. list-table::
   :header-rows: 1
   :widths: 25 25 50

   * - Plugin
     - Model
     - How it works
   * - **TopicBeaconPlugin**
     - Push-based
     - Nodes publish ``MedkitDiscoveryHint`` messages to a shared topic.
       The plugin subscribes and processes hints in real time.
   * - **ParameterBeaconPlugin**
     - Pull-based
     - Nodes declare standard ROS 2 parameters under a known prefix. The
       plugin polls each node's parameter service periodically.

Both plugins feed into the merge pipeline's **PluginLayer** with
``ENRICHMENT`` policy - they add metadata to entities already discovered
by the manifest and runtime layers, without overriding authoritative fields.
Both can run simultaneously, each maintaining a separate hint store.

The MedkitDiscoveryHint Message
-------------------------------

Both beacon plugins work with the same data model. The topic beacon uses it
directly as a ROS 2 message; the parameter beacon maps individual ROS 2
parameters to the same fields.

``ros2_medkit_msgs/msg/MedkitDiscoveryHint``:

.. code-block:: text

   # === Required ===
   string entity_id                          # Target entity (App or Component ID)

   # === Identity hints ===
   string stable_id                          # Stable ID alias
   string display_name                       # Human-friendly name

   # === Topology hints ===
   string[] function_ids                     # Function membership
   string[] depends_on                       # Entity dependencies
   string component_id                       # Parent Component binding

   # === Transport hints ===
   string transport_type                     # "nitros_zero_copy", "shared_memory", etc.
   string negotiated_format                  # "nitros_image_bgr8", etc.

   # === Process diagnostics ===
   uint32 process_id                         # OS process ID (PID), 0 = not provided
   string process_name                       # Process name
   string hostname                           # Host identifier

   # === Freeform ===
   diagnostic_msgs/KeyValue[] metadata       # Arbitrary key-value pairs

   # === Timing ===
   builtin_interfaces/Time stamp             # Timestamp for TTL calculation

All fields except ``entity_id`` are optional. Empty strings and empty arrays
are treated as "not provided" and the plugin ignores them.

Hint Lifecycle
^^^^^^^^^^^^^^

Every hint stored by a beacon plugin transitions through three states based
on its age:

.. list-table::
   :header-rows: 1
   :widths: 15 85

   * - State
     - Description
   * - **ACTIVE**
     - Within ``beacon_ttl_sec``. Enrichment is applied normally.
   * - **STALE**
     - Past TTL but within ``beacon_expiry_sec``. Enrichment is still applied,
       but the vendor endpoint marks the hint as ``"status": "stale"``.
   * - **EXPIRED**
     - Past ``beacon_expiry_sec``. The hint is removed from the store and no
       longer contributes to enrichment.

This two-tier lifecycle lets consumers distinguish between fresh data and
data that may be outdated, without immediately losing all enrichment when a
node stops publishing.

Topic Beacon
------------

The **TopicBeaconPlugin** (``ros2_medkit_topic_beacon`` package) subscribes
to a ROS 2 topic and processes incoming ``MedkitDiscoveryHint`` messages in
real time. This is the push-based approach - nodes actively publish their
metadata at a chosen frequency.

How It Works
^^^^^^^^^^^^

1. Your node creates a publisher on ``/ros2_medkit/discovery`` (configurable).
2. The node publishes ``MedkitDiscoveryHint`` messages periodically (e.g., 1 Hz).
3. The plugin receives each message, validates it, converts it to an internal
   ``BeaconHint``, and stores it in a thread-safe ``BeaconHintStore``.
4. On each discovery cycle, the plugin's ``introspect()`` method snapshots the
   store and maps hints to SOVD entities via the ``BeaconEntityMapper``.
5. A token bucket rate limiter (configurable via ``max_messages_per_second``)
   prevents overload from high-frequency publishers.

Publisher Example (C++)
^^^^^^^^^^^^^^^^^^^^^^^

A minimal node that publishes beacon hints:

.. code-block:: cpp

   #include <rclcpp/rclcpp.hpp>
   #include <ros2_medkit_msgs/msg/medkit_discovery_hint.hpp>
   #include <unistd.h>

   class MyBeaconNode : public rclcpp::Node {
    public:
     MyBeaconNode() : Node("my_sensor_node") {
       publisher_ = create_publisher<ros2_medkit_msgs::msg::MedkitDiscoveryHint>(
           "/ros2_medkit/discovery", 10);

       timer_ = create_wall_timer(std::chrono::seconds(1), [this]() {
         publish_beacon();
       });
     }

    private:
     void publish_beacon() {
       auto msg = ros2_medkit_msgs::msg::MedkitDiscoveryHint();

       // Required - must match an entity known to the gateway
       msg.entity_id = "engine_temp_sensor";

       // Identity
       msg.display_name = "Engine Temperature Sensor";

       // Topology - assign to a function and declare dependencies
       msg.function_ids = {"thermal_monitoring"};
       msg.component_id = "powertrain";

       // Transport
       msg.transport_type = "shared_memory";

       // Process diagnostics
       msg.process_id = static_cast<uint32_t>(getpid());
       msg.process_name = "sensor_node";
       char hostname[256];
       if (gethostname(hostname, sizeof(hostname)) == 0) {
         msg.hostname = hostname;
       }

       // Freeform metadata
       diagnostic_msgs::msg::KeyValue kv;
       kv.key = "firmware_version";
       kv.value = "2.1.0";
       msg.metadata.push_back(kv);

       // Timestamp for TTL calculation
       msg.stamp = now();

       publisher_->publish(msg);
     }

     rclcpp::Publisher<ros2_medkit_msgs::msg::MedkitDiscoveryHint>::SharedPtr publisher_;
     rclcpp::TimerBase::SharedPtr timer_;
   };

The ``stamp`` field is used for TTL calculation. If the stamp is non-zero,
the plugin computes the message age by comparing the stamp against the
system clock and back-projects into ``steady_clock`` domain. If the stamp
is zero, the plugin falls back to using reception time.

CMakeLists.txt Dependencies
^^^^^^^^^^^^^^^^^^^^^^^^^^^

Your node package needs a dependency on ``ros2_medkit_msgs``:

.. code-block:: cmake

   find_package(ros2_medkit_msgs REQUIRED)

   add_executable(my_sensor_node src/my_sensor_node.cpp)
   ament_target_dependencies(my_sensor_node rclcpp ros2_medkit_msgs diagnostic_msgs)

And in ``package.xml``:

.. code-block:: xml

   <depend>ros2_medkit_msgs</depend>
   <depend>diagnostic_msgs</depend>

Configuration
^^^^^^^^^^^^^

Enable the topic beacon in ``gateway_params.yaml``:

.. code-block:: yaml

   ros2_medkit_gateway:
     ros__parameters:
       plugins: ["topic_beacon"]
       plugins.topic_beacon.path: "/path/to/libtopic_beacon_plugin.so"

       # ROS 2 topic to subscribe (default: "/ros2_medkit/discovery")
       plugins.topic_beacon.topic: "/ros2_medkit/discovery"

       # Soft TTL - hints older than this are marked STALE (default: 10.0)
       plugins.topic_beacon.beacon_ttl_sec: 10.0

       # Hard expiry - hints older than this are removed (default: 300.0)
       plugins.topic_beacon.beacon_expiry_sec: 300.0

       # Allow plugin to introduce new entities not seen by other layers
       # When false (recommended), only existing entities are enriched
       plugins.topic_beacon.allow_new_entities: false

       # Maximum number of hints in memory (default: 10000)
       plugins.topic_beacon.max_hints: 10000

       # Rate limit for incoming messages (default: 100)
       plugins.topic_beacon.max_messages_per_second: 100

The ``.so`` path is typically resolved at launch time. If you build the
gateway with colcon, the plugin is installed alongside the gateway packages.

Querying Topic Beacon Data
^^^^^^^^^^^^^^^^^^^^^^^^^^

The plugin registers a vendor extension endpoint on all apps and components:

.. code-block:: bash

   curl -H "Authorization: Bearer $TOKEN" http://localhost:8080/api/v1/apps/engine_temp_sensor/x-medkit-topic-beacon

Example response:

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
     "function_ids": ["thermal_monitoring"],
     "depends_on": [],
     "metadata": {"firmware_version": "2.1.0"}
   }

Parameter Beacon
----------------

The **ParameterBeaconPlugin** (``ros2_medkit_param_beacon`` package) takes
the opposite approach - instead of nodes pushing data, the plugin actively
polls each node's parameter service for parameters matching a configurable
prefix.

How It Works
^^^^^^^^^^^^

1. Your node declares ROS 2 parameters under the ``ros2_medkit.discovery``
   prefix (configurable) using standard ``declare_parameter()`` calls.
2. The plugin runs a background polling thread that periodically lists and
   reads these parameters from each known node.
3. In hybrid mode, poll targets come from the introspection input (apps with
   a bound FQN). In runtime-only mode, the plugin discovers nodes directly
   from the ROS 2 graph.
4. Parameters are mapped to ``BeaconHint`` fields, validated, and stored.
5. Exponential backoff is applied to nodes whose parameter service is
   unreachable, preventing wasted cycles on crashed or unavailable nodes.

Parameter Naming Convention
^^^^^^^^^^^^^^^^^^^^^^^^^^^

Nodes declare parameters under the configured prefix (default:
``ros2_medkit.discovery``). Each parameter maps to a specific ``BeaconHint``
field:

.. list-table::
   :header-rows: 1
   :widths: 45 20 35

   * - Parameter name
     - Type
     - BeaconHint field
   * - ``ros2_medkit.discovery.entity_id``
     - string (required)
     - ``entity_id``
   * - ``ros2_medkit.discovery.stable_id``
     - string
     - ``stable_id``
   * - ``ros2_medkit.discovery.display_name``
     - string
     - ``display_name``
   * - ``ros2_medkit.discovery.component_id``
     - string
     - ``component_id``
   * - ``ros2_medkit.discovery.transport_type``
     - string
     - ``transport_type``
   * - ``ros2_medkit.discovery.negotiated_format``
     - string
     - ``negotiated_format``
   * - ``ros2_medkit.discovery.process_name``
     - string
     - ``process_name``
   * - ``ros2_medkit.discovery.hostname``
     - string
     - ``hostname``
   * - ``ros2_medkit.discovery.process_id``
     - integer
     - ``process_id``
   * - ``ros2_medkit.discovery.function_ids``
     - string array
     - ``function_ids``
   * - ``ros2_medkit.discovery.depends_on``
     - string array
     - ``depends_on``
   * - ``ros2_medkit.discovery.metadata.<key>``
     - string
     - ``metadata[<key>]``

Freeform metadata uses sub-parameters under ``ros2_medkit.discovery.metadata.``.
For example, ``ros2_medkit.discovery.metadata.firmware_version`` maps to
``metadata["firmware_version"]`` in the hint.

Node Example (C++)
^^^^^^^^^^^^^^^^^^

A minimal node that declares beacon parameters:

.. code-block:: cpp

   #include <rclcpp/rclcpp.hpp>
   #include <unistd.h>

   class MyParamBeaconNode : public rclcpp::Node {
    public:
     MyParamBeaconNode() : Node("my_sensor_node") {
       // Required - must match an entity known to the gateway
       declare_parameter("ros2_medkit.discovery.entity_id", "engine_temp_sensor");

       // Identity
       declare_parameter("ros2_medkit.discovery.display_name", "Engine Temperature Sensor");

       // Topology
       declare_parameter("ros2_medkit.discovery.function_ids",
           std::vector<std::string>{"thermal_monitoring"});
       declare_parameter("ros2_medkit.discovery.component_id", "powertrain");

       // Transport
       declare_parameter("ros2_medkit.discovery.transport_type", "shared_memory");

       // Process diagnostics
       declare_parameter("ros2_medkit.discovery.process_id",
           static_cast<int64_t>(getpid()));
       declare_parameter("ros2_medkit.discovery.process_name", "sensor_node");

       // Freeform metadata
       declare_parameter("ros2_medkit.discovery.metadata.firmware_version", "2.1.0");
     }
   };

No custom publishing code is needed. The plugin handles all polling
automatically. Parameters can also be set at launch time via command line
or YAML files:

.. code-block:: bash

   ros2 run my_package my_sensor_node \
     --ros-args \
     -p ros2_medkit.discovery.entity_id:=engine_temp_sensor \
     -p ros2_medkit.discovery.display_name:="Engine Temperature Sensor" \
     -p ros2_medkit.discovery.transport_type:=shared_memory

Configuration
^^^^^^^^^^^^^

Enable the parameter beacon in ``gateway_params.yaml``:

.. code-block:: yaml

   ros2_medkit_gateway:
     ros__parameters:
       plugins: ["parameter_beacon"]
       plugins.parameter_beacon.path: "/path/to/libparam_beacon_plugin.so"

       # Parameter name prefix to scan (default: "ros2_medkit.discovery")
       plugins.parameter_beacon.parameter_prefix: "ros2_medkit.discovery"

       # How often to poll all nodes in seconds (default: 5.0)
       plugins.parameter_beacon.poll_interval_sec: 5.0

       # Maximum time per poll cycle in seconds (default: 10.0)
       plugins.parameter_beacon.poll_budget_sec: 10.0

       # Timeout for each node's parameter service call in seconds (default: 2.0)
       plugins.parameter_beacon.param_timeout_sec: 2.0

       # Soft TTL - hints older than this are marked STALE (default: 15.0)
       plugins.parameter_beacon.beacon_ttl_sec: 15.0

       # Hard expiry - hints older than this are removed (default: 300.0)
       plugins.parameter_beacon.beacon_expiry_sec: 300.0

       # Allow plugin to introduce new entities (default: false)
       plugins.parameter_beacon.allow_new_entities: false

       # Maximum hints in memory (default: 10000)
       plugins.parameter_beacon.max_hints: 10000

.. note::

   The plugin automatically validates timing relationships. If
   ``beacon_ttl_sec`` is less than or equal to ``poll_interval_sec``, hints
   would go stale between polls - the plugin auto-corrects to
   ``ttl = 3 * poll_interval``. Similarly, if ``beacon_expiry_sec`` is less
   than or equal to ``beacon_ttl_sec``, it auto-corrects to
   ``expiry = 10 * ttl``.

Querying Parameter Beacon Data
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The plugin registers its own vendor extension endpoint:

.. code-block:: bash

   curl -H "Authorization: Bearer $TOKEN" http://localhost:8080/api/v1/apps/engine_temp_sensor/x-medkit-param-beacon

The response format is identical to the topic beacon endpoint.

Running Both Plugins
--------------------

The topic and parameter beacon plugins can be active simultaneously. Each
maintains its own ``BeaconHintStore`` and contributes independently to the
merge pipeline. A node could use the topic beacon for frequently changing
metadata (transport metrics, load data) and the parameter beacon for stable
identity metadata (display name, function membership).

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

When both plugins provide a hint for the same entity, both sets of metadata
are merged into the entity response. Align TTL and expiry values across
plugins to avoid inconsistent staleness behavior.

Entity Metadata Injection
-------------------------

When a beacon hint is applied to an entity, the following metadata keys
appear in the entity's JSON response (e.g., ``GET /api/v1/apps/{id}``).
These keys are in the entity's ``metadata`` field, separate from the vendor
extension endpoint:

.. list-table::
   :header-rows: 1
   :widths: 40 15 45

   * - Key
     - Type
     - Description
   * - ``x-medkit-beacon-status``
     - string
     - ``"active"`` or ``"stale"``
   * - ``x-medkit-beacon-age-sec``
     - number
     - Seconds since the hint was last seen
   * - ``x-medkit-beacon-transport-type``
     - string
     - DDS transport type (e.g., ``"shared_memory"``)
   * - ``x-medkit-beacon-negotiated-format``
     - string
     - Negotiated data format
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
     - Freeform metadata from the hint's ``metadata`` field

When to Use Which
-----------------

**Use the Topic Beacon when:**

- Metadata changes in real time - transport type, load metrics, connection
  quality, process health.
- You want low-latency updates. As soon as a node publishes, the plugin
  processes the hint within the same DDS callback.
- Multiple entities need to be updated from a single publisher (a manager
  node publishing hints for its child nodes).
- You need precise control over the publish rate and timing via the
  ``stamp`` field.

**Use the Parameter Beacon when:**

- Metadata is stable and infrequently updated - hardware descriptions,
  firmware versions, display names, function membership.
- You want zero custom code. Nodes only need to call ``declare_parameter()``
  with the right prefix - no publisher setup, no timer, no message
  construction.
- Parameters can be set externally at launch time via YAML or command line,
  without modifying node source code.
- You prefer a pull-based model where the gateway controls the polling
  frequency.

**Use both when:**

- Some metadata is real-time (topic beacon) and some is static (parameter
  beacon). For example, a sensor node might publish live transport metrics
  via the topic beacon while declaring its display name and function
  membership as parameters.

.. list-table::
   :header-rows: 1
   :widths: 30 35 35

   * - Consideration
     - Topic Beacon
     - Parameter Beacon
   * - Latency
     - Sub-second (push)
     - ``poll_interval_sec`` (pull)
   * - Node code required
     - Publisher + timer + message
     - ``declare_parameter()`` only
   * - Bandwidth
     - Proportional to publish rate
     - Proportional to poll rate
   * - External configurability
     - Limited (node must publish)
     - Full (``--ros-args -p``)
   * - Multiple entities per node
     - Yes (publish different entity_ids)
     - One entity_id per node
   * - Works without custom code
     - No
     - Yes (set params at launch)

See Also
--------

- :doc:`/config/discovery-options` - Full configuration reference for both plugins
- :doc:`plugin-system` - General plugin system architecture
- :doc:`heuristic-apps` - Runtime discovery without beacons
- :doc:`manifest-discovery` - Hybrid mode with manifest + beacons
- :doc:`/api/messages` - Message definitions including ``MedkitDiscoveryHint``
