Migration Guide: Runtime to Hybrid Mode
========================================

This guide helps you migrate from runtime-only discovery to hybrid mode,
enabling stable entity IDs, semantic groupings, and offline detection.

.. contents:: Table of Contents
   :local:
   :depth: 2

Overview
--------

**Runtime-only mode** (default) automatically discovers:

- ROS 2 nodes -> Apps
- A single host-derived Component groups all Apps
- Namespaces create Functions (not Areas or Components)
- Areas come from manifest only
- Topics, services, actions -> Data, operations

**Hybrid mode** adds:

- Stable, meaningful entity IDs
- Apps entity type (software applications)
- Functions entity type (high-level capabilities)
- Offline detection for apps
- Custom metadata and descriptions

The migration process creates a manifest that maps your existing ROS 2 nodes
to the SOVD entity model while preserving runtime data access.

Step 1: Audit Your System
-------------------------

First, understand your current ROS 2 system structure.

List all running nodes:

.. code-block:: bash

   ros2 node list

Example output:

.. code-block:: text

   /amcl
   /bt_navigator
   /controller_server
   /ld08_driver
   /map_server
   /planner_server
   /robot_state_publisher
   /turtlebot3_node

For each node, get detailed info:

.. code-block:: bash

   ros2 node info /amcl

Note down:

- Node name and namespace
- Published/subscribed topics
- Provided services and actions
- Purpose and category

Create a spreadsheet or document with columns:

.. list-table::
   :header-rows: 1
   :widths: 20 20 20 40

   * - Node Name
     - Namespace
     - Category
     - Purpose
   * - amcl
     - /
     - localization
     - Adaptive Monte Carlo Localization
   * - planner_server
     - /
     - navigation
     - Global path planning
   * - controller_server
     - /
     - navigation
     - Local trajectory tracking
   * - ...
     - ...
     - ...
     - ...

Step 2: Define Areas
--------------------

Group your nodes into logical subsystems. Common groupings:

- **By function**: perception, localization, navigation, control
- **By hardware**: sensors, actuators, compute
- **By namespace**: ROS 2 namespace structure

Example area mapping:

.. code-block:: yaml

   areas:
     - id: perception
       name: "Perception"
       category: "sensor-processing"
       description: "Sensor data acquisition and processing"

     - id: localization
       name: "Localization"
       category: "state-estimation"
       description: "Robot pose estimation"

     - id: navigation
       name: "Navigation"
       category: "motion-planning"
       description: "Path planning and execution"

     - id: control
       name: "Control"
       category: "motion-control"
       description: "Low-level motor control"

.. tip::

   Start with broad categories and refine later. You can nest areas using
   ``subareas`` if needed.

Step 3: Define Components
-------------------------

Map physical and virtual hardware. Components represent:

- Physical sensors (LiDAR, cameras, IMU)
- Actuators (motors, grippers)
- Compute units (main computer, edge devices)
- Virtual units (containers, processes)

Example component mapping:

.. code-block:: yaml

   components:
     - id: lidar-sensor
       name: "LiDAR Sensor"
       type: "sensor"
       area: perception
       description: "360° laser range finder"

     - id: main-computer
       name: "Main Computer"
       type: "controller"
       area: control
       description: "Raspberry Pi 4 running ROS 2"

     - id: opencr-board
       name: "OpenCR Board"
       type: "controller"
       area: control
       description: "Motor controller board"

Step 4: Define Apps
-------------------

Create an app entry for each ROS 2 node. Key decisions:

1. **Choose a stable ID**: Use lowercase with hyphens (e.g., ``lidar-driver``)
2. **Set human-readable name**: Clear, descriptive name
3. **Configure ros_binding**: How to match the ROS 2 node
4. **Set component**: Which component hosts this app
5. **Set depends_on**: Dependencies on other apps

Example app mapping:

.. code-block:: yaml

   apps:
     # Node: /ld08_driver
     - id: lidar-driver
       name: "LiDAR Driver"
       category: "driver"
       is_located_on: lidar-sensor
       ros_binding:
         node_name: ld08_driver

     # Node: /amcl
     - id: amcl-node
       name: "AMCL Localization"
       category: "localization"
       is_located_on: main-computer
       depends_on:
         - lidar-driver
       ros_binding:
         node_name: amcl

     # Node: /planner_server
     - id: planner-server
       name: "Planner Server"
       category: "navigation"
       is_located_on: main-computer
       depends_on:
         - amcl-node
       ros_binding:
         node_name: planner_server

**Handling namespaced nodes:**

.. code-block:: yaml

   # Node: /robot1/lidar_driver
   - id: robot1-lidar-driver
     name: "Robot 1 LiDAR Driver"
     ros_binding:
       node_name: lidar_driver
       namespace: /robot1

   # Match node in any namespace
   - id: generic-teleop
     name: "Teleop Controller"
     ros_binding:
       node_name: teleop_keyboard
       namespace: "*"

Step 5: Define Functions
------------------------

Identify high-level capabilities your robot provides:

- What can your robot do as a user?
- Which apps work together for each capability?

Example function definitions:

.. code-block:: yaml

   functions:
     - id: autonomous-navigation
       name: "Autonomous Navigation"
       category: "mobility"
       description: "Navigate autonomously to goals"
       hosted_by:
         - amcl-node
         - planner-server
         - controller-server
         - bt-navigator

     - id: localization
       name: "Localization"
       category: "state-estimation"
       description: "Determine robot position on map"
       hosted_by:
         - amcl-node
         - map-server

     - id: perception
       name: "Environment Perception"
       category: "sensing"
       hosted_by:
         - lidar-driver

Step 6: Create the Manifest File
--------------------------------

Combine all sections into a complete manifest:

.. code-block:: yaml

   manifest_version: "1.0"

   metadata:
     name: "my-robot"
     version: "1.0.0"
     description: "My robot system manifest"

   config:
     unmanifested_nodes: warn
     inherit_runtime_resources: true

   areas:
     # ... your area definitions

   components:
     # ... your component definitions

   apps:
     # ... your app definitions

   functions:
     # ... your function definitions

Save as ``system_manifest.yaml`` in your config directory.

.. note::

   The discovery-configuration block is spelled ``config:``. Older manifests
   written against the gateway source rather than this documentation may spell
   it ``discovery:``; that key still works as a deprecated alias and the
   gateway logs a warning naming it, but it will be removed in a future
   release - rename it to ``config:``. A manifest that declares both keys uses
   ``config:`` and ignores ``discovery:``.

Step 7: Test in Hybrid Mode
---------------------------

1. **Start your ROS 2 system** as normal

2. **Start the gateway with manifest**:

   .. code-block:: bash

      ros2 run ros2_medkit_gateway gateway_node --ros-args \
          -p discovery.mode:=hybrid \
          -p discovery.manifest_path:=/path/to/system_manifest.yaml

3. **Check manifest status**:

   .. code-block:: bash

      curl -H "Authorization: Bearer $TOKEN" http://localhost:8080/api/v1/manifest/status | jq

4. **Verify apps are linked**:

   .. code-block:: bash

      curl -H "Authorization: Bearer $TOKEN" http://localhost:8080/api/v1/apps | jq '.items[] | {id, name, is_online}'

5. **Check for orphan nodes** (warnings in gateway logs):

   .. code-block:: bash

      ros2 run ros2_medkit_gateway gateway_node ... 2>&1 | grep -i orphan

Step 8: Iterate and Refine
--------------------------

Based on testing results:

**App not linking to node:**

- Check ``ros_binding.node_name`` spelling
- Verify namespace matches (use ``ros2 node list`` to confirm)
- Try wildcard namespace: ``namespace: "*"``
- Check gateway logs for matching attempts

**Orphan nodes appearing:**

- Add app entries for missing nodes
- Or set ``config.unmanifested_nodes: ignore`` to hide them. Note how wide
  that setting reaches: it drops every app that did not come from the
  manifest, the asset inventory or a discovery plugin, so nodes you have not
  declared yet disappear from the entity tree along with the ones you meant to
  hide. "From a plugin" means the plugin's layer *declared* the entity - a
  plugin that only adds detail to an app another layer discovered does not
  shield it, whatever ``x-medkit.source`` ends up saying.
  While you are still filling the manifest in, ``warn`` keeps them visible.
- ``config.unmanifested_nodes: error`` does not stop the gateway. It logs at
  error level and lists the orphan nodes on ``GET /api/v1/health`` under the
  ``unmanifested_nodes`` warning code, which is the form to use if you want CI
  or a monitor to fail on an incomplete manifest.

**Missing data/operations:**

- Ensure ``config.inherit_runtime_resources: true``
- Check that the bound node has the expected topics/services

**Refining the manifest:**

- Add descriptions to improve documentation
- Add tags for filtering
- Adjust categories for better organization
- Define dependencies between apps

Validation Checklist
--------------------

Before finalizing your manifest:

.. code-block:: text

   [ ] All required fields present (manifest_version, entity ids and names)
   [ ] All area references are valid (component.area, subarea.parent)
   [ ] All component references are valid (app.component)
   [ ] All app references are valid (depends_on, function.hosted_by)
   [ ] IDs are unique within each entity type
   [ ] ros_binding configured for all apps that map to ROS nodes
   [ ] Functions have at least one app in hosted_by

Run validation:

.. code-block:: bash

   # Start gateway with strict validation
   ros2 run ros2_medkit_gateway gateway_node --ros-args \
       -p discovery.mode:=hybrid \
       -p discovery.manifest_path:=/path/to/system_manifest.yaml \
       -p discovery.manifest_strict_validation:=true

Common Issues
-------------

**"App X not found for function Y"**

The ``hosted_by`` list references an app ID that doesn't exist.

.. code-block:: yaml

   # Wrong - typo in app ID
   functions:
     - id: nav-function
       hosted_by:
         - amcl_node  # Should be "amcl-node"

**"Component X not found for app Y"**

The ``component`` references a component that doesn't exist.

.. code-block:: yaml

   # Wrong - component doesn't exist
   apps:
     - id: my-app
       component: nonexistent-component

**"Duplicate ID: X"**

Two entities have the same ID.

.. code-block:: yaml

   # Wrong - duplicate IDs
   components:
     - id: sensor
       name: "LiDAR"
     - id: sensor  # Duplicate!
       name: "Camera"

**Node not linking (is_online: false)**

Common causes:

1. Node name mismatch (check exact spelling)
2. Namespace mismatch (try wildcard)
3. Node not running when gateway starts
4. Node in different ROS domain

Debug with:

.. code-block:: bash

   # List actual nodes
   ros2 node list

   # Check gateway logs
   ros2 run ros2_medkit_gateway gateway_node ... 2>&1 | grep -i link

Best Practices
--------------

1. **Start simple**: Begin with just apps and expand to functions later

2. **Use consistent naming**: Follow a pattern for IDs (e.g., ``{component}-{function}``)

3. **Document as you go**: Add descriptions while the context is fresh

4. **Version your manifest**: Use semantic versioning in metadata

5. **Keep manifest in source control**: Track changes alongside code

6. **Test incrementally**: Add a few entities, test, repeat

7. **Use validation**: Always run with ``strict_validation: true`` initially

Next Steps
----------

- :doc:`manifest-discovery` - Complete user guide
- :doc:`/config/manifest-schema` - Full schema reference
