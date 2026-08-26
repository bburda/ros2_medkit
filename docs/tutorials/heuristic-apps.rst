Heuristic Runtime Discovery
===========================

This tutorial explains how to use heuristic runtime discovery to expose
ROS 2 nodes as SOVD entities without any configuration.

.. contents:: Table of Contents
   :local:
   :depth: 2

Overview
--------

By default, ros2_medkit gateway uses **heuristic discovery** to map ROS 2 graph
entities to SOVD entities:

- **ROS 2 nodes** -> **Apps** (software applications)
- **First namespace segment** -> **Functions** (capability groupings)
- **Host system info** -> **Component** (single host-level Component)
- **Topics, services, actions** -> **Data, Operations**

Areas are not created in runtime mode - they come from manifest only.

This approach requires no configuration and works out of the box with any
ROS 2 system.

When to Use Heuristic Discovery
-------------------------------

Heuristic discovery is ideal when:

✅ You want to explore a ROS 2 system without prior setup
✅ Entity IDs can be derived from node names
✅ Your system doesn't require stable IDs across restarts
✅ You're prototyping or debugging

Consider using :doc:`manifest-discovery` when:

❌ You need stable, semantic IDs (e.g., ``front-lidar`` instead of ``scan_node``)
❌ You need to define entities that don't exist at runtime
❌ You need offline detection of failed components

Quick Start
-----------

Launch the gateway with default settings:

.. code-block:: bash

   ros2 launch ros2_medkit_gateway gateway.launch.py

The gateway automatically discovers all nodes and maps them to SOVD entities.

Query available Apps:

.. code-block:: bash

   curl -H "Authorization: Bearer $TOKEN" http://localhost:8080/api/v1/apps | jq

Example response:

.. code-block:: json

   {
     "items": [
       {
         "id": "lidar_driver",
         "name": "lidar_driver",
         "source": "heuristic"
       },
       {
         "id": "camera_node",
         "name": "camera_node",
         "source": "heuristic"
       }
     ]
   }

Understanding the Entity Hierarchy
----------------------------------

With heuristic discovery, the SOVD hierarchy is built as follows:

.. code-block:: text

   Component: "my-hostname"              <- single host-level Component
   Function: "perception"                <- from first namespace segment /perception
       App: "lidar_driver"               <- from node /perception/lidar_driver
           Data: "scan"                  <- published topics
           Operations: ...               <- services/actions
       App: "camera_node"                <- from node /perception/camera_node
           Data: "image_raw"

Configuration Options
---------------------

All options are under ``discovery.runtime`` in the gateway parameters:

.. code-block:: yaml

   ros2_medkit_gateway:
     ros__parameters:
       discovery:
         mode: "runtime_only"  # or "hybrid"

         runtime:
           create_functions_from_namespaces: true
           default_component:
             enabled: true

Entity Model
^^^^^^^^^^^^^

In runtime mode, the gateway maps the ROS 2 graph as follows:

- **Apps** - each ROS 2 node becomes an App (``source: "heuristic"``)
- **Functions** - namespace grouping creates Function entities
- **Components** - a single host-level Component from ``HostInfoProvider``
- **Areas** - not created (Areas come from manifest only)

.. code-block:: bash

   curl -H "Authorization: Bearer $TOKEN" http://localhost:8080/api/v1/apps
   # Returns: [{"id": "lidar_driver"}, {"id": "camera_node"}]

   curl -H "Authorization: Bearer $TOKEN" http://localhost:8080/api/v1/components
   # Returns: [{"id": "my-hostname", "source": "runtime", ...}]

   curl -H "Authorization: Bearer $TOKEN" http://localhost:8080/api/v1/functions
   # Returns: [{"id": "perception"}, {"id": "navigation"}]

   curl -H "Authorization: Bearer $TOKEN" http://localhost:8080/api/v1/areas
   # Returns: {"items": []}  (empty - Areas come from manifest only)

API Endpoints
-------------

Apps Endpoints
^^^^^^^^^^^^^^

.. list-table::
   :header-rows: 1
   :widths: 40 60

   * - Endpoint
     - Description
   * - ``GET /apps``
     - List all discovered Apps
   * - ``GET /apps/{app_id}``
     - Get specific App details
   * - ``GET /components/{id}/apps``
     - List Apps in a Component

Components Endpoints
^^^^^^^^^^^^^^^^^^^^

In runtime mode, the ``/components`` endpoint returns a single host-level
Component derived from system information:

.. code-block:: bash

   curl -H "Authorization: Bearer $TOKEN" http://localhost:8080/api/v1/components | jq '.items[] | {id, source}'

.. code-block:: json

   {"id": "my-hostname", "source": "runtime"}

The ``source`` field indicates how the component was discovered:

- ``runtime``: Auto-created from host system information
- ``manifest``: Explicitly defined in manifest file (see :doc:`manifest-discovery`)

Functions Endpoints
^^^^^^^^^^^^^^^^^^^

In runtime mode, Functions are created from the first namespace segment:

.. code-block:: bash

   curl -H "Authorization: Bearer $TOKEN" http://localhost:8080/api/v1/functions | jq '.items[] | {id}'

.. code-block:: json

   {"id": "perception"}
   {"id": "navigation"}

Migrating to Manifest Discovery
-------------------------------

When you need more control, consider migrating to hybrid mode.
See :doc:`migration-to-manifest` for a step-by-step guide.

Key differences:

.. list-table::
   :header-rows: 1
   :widths: 30 35 35

   * - Feature
     - Heuristic
     - Manifest
   * - Setup required
     - None
     - YAML manifest file
   * - Entity IDs
     - Derived from node names
     - Custom, semantic IDs
   * - Offline detection
     - No
     - Yes (failed components)
   * - Stable across restarts
     - No (depends on node names)
     - Yes (defined in manifest)
   * - Functions
     - Yes (from namespaces)
     - Yes (from manifest)
   * - Areas
     - No
     - Yes (from manifest)

See Also
--------

- :doc:`/config/discovery-options` - Full configuration reference
- :doc:`manifest-discovery` - Manifest-based discovery
- :doc:`migration-to-manifest` - Migration guide
