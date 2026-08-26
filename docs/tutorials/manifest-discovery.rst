Manifest-Based Discovery
========================

This tutorial explains how to use manifest files to define your ROS 2 system
structure for SOVD discovery.

Introduction
------------

The ros2_medkit gateway supports three discovery modes:

1. **Runtime-Only** (default): Traditional ROS 2 graph introspection
2. **Hybrid** (recommended): Manifest as source of truth + runtime linking
3. **Manifest-Only**: Only expose manifest-declared entities

Hybrid mode is recommended because it:

- Provides **stable entity IDs** that don't change across restarts
- Enables **semantic groupings** (areas, functions) for better organization
- Preserves **runtime data** (topic values, service calls)
- Allows **documentation and metadata** in entity definitions
- Supports **offline detection** for apps not currently running

Discovery Modes Comparison
--------------------------

.. list-table::
   :header-rows: 1
   :widths: 20 30 30 20

   * - Feature
     - Runtime-Only
     - Hybrid
     - Manifest-Only
   * - Entity IDs
     - Derived from ROS graph
     - Stable (from manifest)
     - Stable (from manifest)
   * - Live topics/services
     - ✅ Yes
     - ✅ Yes
     - ❌ No
   * - Custom areas
     - No (Areas come from manifest only)
     - Yes
     - Yes
   * - Apps entity type
     - Yes (ROS nodes -> Apps)
     - Yes
     - Yes
   * - Functions entity type
     - Yes (from namespace segments)
     - Yes
     - Yes
   * - Offline detection
     - ❌ No
     - ✅ Yes
     - N/A

Writing Your First Manifest
---------------------------

Create a file named ``system_manifest.yaml``:

.. code-block:: yaml

   # SOVD System Manifest
   manifest_version: "1.0"

   metadata:
     name: "my-robot"
     version: "1.0.0"
     description: "My Robot System"

   # Define logical areas (subsystems)
   areas:
     - id: perception
       name: "Perception"
       category: "sensor-processing"
       description: "Sensor data processing and fusion"

   # Define hardware/virtual components
   components:
     - id: lidar-sensor
       name: "LiDAR Sensor"
       type: "sensor"
       area: perception

   # Define apps (map to ROS 2 nodes)
   apps:
     - id: lidar-driver
       name: "LiDAR Driver"
       is_located_on: lidar-sensor
       ros_binding:
         node_name: velodyne_driver
         namespace: /sensors

   # Define high-level functions
   functions:
     - id: environment-sensing
       name: "Environment Sensing"
       hosted_by:
         - lidar-driver

Enabling Manifest Mode
----------------------

Option 1: Using Launch File
~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: python

   from launch import LaunchDescription
   from launch_ros.actions import Node

   def generate_launch_description():
       return LaunchDescription([
           Node(
               package='ros2_medkit_gateway',
               executable='gateway_node',
               parameters=[{
                   'discovery.mode': 'hybrid',
                   'discovery.manifest_path': '/path/to/system_manifest.yaml',
                   'discovery.manifest_strict_validation': True,
               }]
           )
       ])

Option 2: Using Parameter File
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Add to your ``gateway_params.yaml``:

.. code-block:: yaml

   ros2_medkit_gateway:
     ros__parameters:
       # ... existing parameters ...

       discovery:
         # Discovery mode: "runtime_only", "hybrid", or "manifest_only"
         mode: "hybrid"

         # Path to manifest YAML file (required for hybrid and manifest_only)
         manifest_path: "/path/to/system_manifest.yaml"

         # Strict validation: fail on any validation error
         manifest_strict_validation: true

Then launch with:

.. code-block:: bash

   ros2 run ros2_medkit_gateway gateway_node \
       --ros-args --params-file /path/to/gateway_params.yaml

Option 3: Command Line Parameters
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: bash

   ros2 run ros2_medkit_gateway gateway_node --ros-args \
       -p discovery.mode:=hybrid \
       -p discovery.manifest_path:=/path/to/system_manifest.yaml

Verifying the Configuration
---------------------------

Once the gateway is running with a manifest, you can verify the configuration
via the REST API.

Check manifest status:

.. code-block:: bash

   curl -H "Authorization: Bearer $TOKEN" http://localhost:8080/api/v1/manifest/status

Expected response:

.. code-block:: json

   {
     "status": "active",
     "discovery_mode": "hybrid",
     "manifest_path": "/path/to/system_manifest.yaml",
     "statistics": {
       "areas_count": 1,
       "components_count": 1,
       "apps_count": 1,
       "functions_count": 1
     }
   }

List apps:

.. code-block:: bash

   curl -H "Authorization: Bearer $TOKEN" http://localhost:8080/api/v1/apps

List functions:

.. code-block:: bash

   curl -H "Authorization: Bearer $TOKEN" http://localhost:8080/api/v1/functions

Understanding Hybrid Mode
-------------------------

In hybrid mode, discovery uses a **merge pipeline** that combines entities from
multiple discovery layers:

1. **ManifestLayer** (highest priority) - entities from the YAML manifest
2. **RuntimeLayer** - entities discovered via ROS 2 graph introspection
3. **PluginLayers** (optional) - entities from gateway plugins

The pipeline merges entities by ID. When the same entity appears in multiple layers,
per-field-group merge policies determine which values win. See
:doc:`/config/discovery-options` for details on merge policies and gap-fill configuration.

After merging, the **RuntimeLinker** binds manifest apps to running ROS 2 nodes:

1. **Discovery**: All layers produce entities
2. **Merging**: Pipeline merges entities by ID, applying field-group policies
3. **Linking**: For each manifest app, checks ``ros_binding`` configuration
4. **Binding**: If match found, copies runtime resources (topics, services, actions)
5. **Status**: Apps with matched nodes are marked ``is_online: true``
6. **Suppression**: Heuristic entities that duplicate manifest-covered namespaces
   (components/areas) or linked app IDs (apps) are removed. Gap-fill apps in
   uncovered namespaces survive.

Merge Report
~~~~~~~~~~~~

After each pipeline execution, the gateway produces a ``MergeReport`` available
via the health endpoint (``GET /health``). The report includes:

- Layer names and ordering
- Total entity count, enrichment count
- Conflict details (which layers disagreed on which field groups)
- Cross-type ID collision warnings
- Gap-fill filtering statistics

In hybrid mode, the ``GET /health`` response includes full discovery diagnostics:

.. code-block:: json

   {
     "discovery": {
       "mode": "hybrid",
       "strategy": "hybrid",
       "pipeline": {
         "layers": ["manifest", "runtime"],
         "total_entities": 12,
         "enriched_count": 8,
         "conflict_count": 0,
         "id_collisions": 0
       },
       "linking": {
         "linked_count": 5,
         "orphan_count": 1,
         "binding_conflicts": 0,
         "warnings": ["Orphan node: /unmanifested_node"]
       }
     }
   }


Runtime Linking
~~~~~~~~~~~~~~~

.. note::

   In hybrid mode, the gateway uses a layered merge pipeline: manifest entities,
   runtime-discovered entities, and plugin-contributed entities are merged per
   field-group before linking. See :doc:`/config/discovery-options` for merge
   pipeline configuration.

.. note::

   **Trigger entity resolution in hybrid mode**

   In hybrid mode, fault and log triggers are delivered using the manifest entity
   ID rather than the raw ROS node FQN. The runtime linker builds a node-to-app
   mapping during the linking phase (step 4 above), and the fault manager and log
   manager use this mapping to resolve incoming trigger events to the correct
   manifest entity. This means fault codes, log subscriptions, and SSE events
   reference stable manifest IDs (e.g., ``lidar-driver``) instead of the
   ephemeral node FQN (e.g., ``/sensors/velodyne_driver``), even when nodes
   restart and re-bind.

ROS Binding Configuration
~~~~~~~~~~~~~~~~~~~~~~~~~

The ``ros_binding`` section specifies how to match an app to a ROS 2 node:

.. code-block:: yaml

   ros_binding:
     # Match by node name
     node_name: "velodyne_driver"

     # Match within a specific namespace (optional)
     namespace: "/sensors"

     # Or match by topic namespace prefix (alternative)
     topic_namespace: "/sensors/lidar"

**Match Types:**

- **Name and namespace match** (default): Node name must match exactly. Namespace uses
  path-segment-boundary matching (``/nav`` matches ``/nav`` and ``/nav/sub`` but NOT ``/navigation``)
- **Wildcard namespace**: Use ``namespace: "*"`` to match any namespace
- **Topic namespace**: Match nodes by their topic prefix

.. code-block:: yaml

   # Example: Match node in any namespace
   apps:
     - id: camera-driver
       name: "Camera Driver"
       ros_binding:
         node_name: usb_cam
         namespace: "*"

   # Example: Match by topic namespace
   apps:
     - id: lidar-processing
       name: "LiDAR Processing"
       ros_binding:
         topic_namespace: "/perception/lidar"

Checking Linking Status
~~~~~~~~~~~~~~~~~~~~~~~

Check which apps are online:

.. code-block:: bash

   curl -H "Authorization: Bearer $TOKEN" http://localhost:8080/api/v1/apps | jq '.items[] | {id, name, is_online}'

Example response:

.. code-block:: json

   {"id": "lidar-driver", "name": "LiDAR Driver", "is_online": true}
   {"id": "camera-driver", "name": "Camera Driver", "is_online": false}

Entity Hierarchy
----------------

The manifest supports two hierarchy patterns depending on your robot's complexity.

With Areas (Complex Robots)
~~~~~~~~~~~~~~~~~~~~~~~~~~~

For robots with multiple subsystems (e.g., autonomous vehicles, industrial platforms),
use areas to group components by function or location:

.. code-block:: text

   Areas (logical/physical groupings)
   +-- Components (hardware/virtual units)
       +-- Apps (software applications)
           +-- Data (topics)
           +-- Operations (services/actions)
           +-- Configurations (parameters)

   Functions (cross-cutting capabilities)
   +-- Apps (hosted by)

**Areas** group related components by function or location:

.. code-block:: yaml

   areas:
     - id: perception
       name: "Perception Subsystem"
       subareas:
         - id: lidar-processing
           name: "LiDAR Processing"

**Components** are assigned to areas:

.. code-block:: yaml

   components:
     - id: main-computer
       name: "Main Computer"
       type: "controller"
       area: perception
       subcomponents:
         - id: gpu-unit
           name: "GPU Processing Unit"

Without Areas (Simple Robots)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

For simple robots where the entire system is a single unit (e.g., TurtleBot3, small
mobile platforms), you can omit areas entirely and use a flat component tree. The
robot itself is the top-level component, with subcomponents for hardware modules:

.. code-block:: text

   Components (top-level)
   +-- Subcomponents (hardware modules)
       +-- Apps (software applications)

   Functions (cross-cutting capabilities)
   +-- Apps (hosted by)

.. code-block:: yaml

   # No areas section needed
   components:
     - id: turtlebot3
       name: "TurtleBot3 Burger"
       type: "mobile-robot"

     - id: raspberry-pi
       name: "Raspberry Pi 4"
       type: "controller"
       parent_component_id: turtlebot3

   apps:
     - id: nav2-controller
       name: "Nav2 Controller"
       is_located_on: raspberry-pi
       ros_binding:
         node_name: controller_server
         namespace: /

For runtime-only mode, Areas are never created from namespaces - they come
from manifest only. See :ref:`manifest-flat-entity-tree` in the manifest
schema reference and ``config/examples/flat_robot_manifest.yaml`` for a
complete example.

**When to use each pattern:**

- **With areas**: Multiple subsystems, deep namespace hierarchy, large teams
  working on separate domains (perception, navigation, control)
- **Without areas**: Single robot with a handful of nodes, flat or shallow
  namespace structure, quick prototypes

Common Elements
~~~~~~~~~~~~~~~

Both patterns use **Apps** and **Functions** the same way.

**Apps** are software applications (typically ROS 2 nodes):

.. code-block:: yaml

   apps:
     - id: slam-node
       name: "SLAM Node"
       is_located_on: main-computer
       depends_on:
         - lidar-driver
         - imu-driver
       ros_binding:
         node_name: slam_toolbox

**Functions** are high-level capabilities spanning multiple apps:

.. code-block:: yaml

   functions:
     - id: autonomous-navigation
       name: "Autonomous Navigation"
       hosted_by:
         - planner-node
         - controller-node
         - localization-node

Controlling Gap-Fill in Hybrid Mode
------------------------------------

In hybrid mode, the runtime layer can create heuristic entities for namespaces
not covered by the manifest. The ``merge_pipeline.gap_fill`` parameters control
this behavior:

.. code-block:: yaml

   discovery:
     merge_pipeline:
       gap_fill:
         allow_heuristic_apps: true          # Create apps from unbound nodes
         allow_heuristic_functions: false    # Don't create heuristic functions
         # namespace_blacklist: ["/rosout"]  # Exclude specific namespaces
         # namespace_whitelist: []           # If set, only allow these namespaces

.. note::

   Areas and Components are never created by runtime discovery. Areas come
   from manifest only. Components come from ``HostInfoProvider`` or manifest.

When all ``allow_heuristic_*`` options are ``false``, only manifest-declared
entities appear. Runtime nodes are still discovered for linking, but no
heuristic entities (apps, functions) are created from unmatched namespaces
or nodes.

See :doc:`/config/discovery-options` for the full merge pipeline reference.

Hot Reloading
-------------

Reload the manifest without restarting the gateway:

.. code-block:: bash

   curl -X POST http://localhost:8080/api/v1/manifest/reload

This re-parses the manifest file and re-links apps to nodes.

REST API Endpoints
------------------

Manifest mode adds the following endpoints:

.. list-table::
   :header-rows: 1
   :widths: 30 50 20

   * - Endpoint
     - Description
     - Since
   * - ``GET /apps``
     - List all apps
     - 0.1.0
   * - ``GET /apps/{id}``
     - Get app capabilities
     - 0.1.0
   * - ``GET /apps/{id}/data``
     - Get app topic data
     - 0.1.0
   * - ``GET /apps/{id}/operations``
     - List app operations
     - 0.1.0
   * - ``GET /apps/{id}/configurations``
     - List app configurations
     - 0.1.0
   * - ``GET /functions``
     - List all functions
     - 0.1.0
   * - ``GET /functions/{id}``
     - Get function capabilities
     - 0.1.0
   * - ``GET /functions/{id}/hosts``
     - List function host apps
     - 0.1.0
   * - ``GET /functions/{id}/data``
     - Aggregated data from hosts
     - 0.1.0
   * - ``GET /functions/{id}/operations``
     - Aggregated operations
     - 0.1.0

Next Steps
----------

- :doc:`/config/manifest-schema` - Complete YAML schema reference
- :doc:`migration-to-manifest` - Migrate from runtime-only mode
- :doc:`/getting_started` - Basic gateway setup
