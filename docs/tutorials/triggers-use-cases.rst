Triggers Use Cases
==================

This tutorial walks through three practical multi-trigger scenarios that
demonstrate how to combine triggers with different condition types, resource
collections, and entity scopes. Each scenario includes step-by-step curl
commands you can run against a live gateway.

.. contents:: Table of Contents
   :local:
   :depth: 2

Prerequisites
-------------

Start the gateway with demo nodes and a fault manager:

.. code-block:: bash

   # Terminal 1: Gateway with demo nodes
   ros2 launch ros2_medkit_gateway gateway.launch.py server_host:=0.0.0.0 &
   ros2 launch ros2_medkit_integration_tests demo_nodes.launch.py

Verify the gateway is running:

.. code-block:: bash

   curl http://localhost:8080/api/v1/health

Scenario 1: OTA Update Monitoring
----------------------------------

A maintenance engineer monitors an OTA update process using three triggers
on the same entity, each watching a different collection or condition:

1. **Update status tracker** - OnChange on the ``updates`` collection to
   track any status transition during the OTA process
2. **Failure alert** - OnChangeTo ``"failed"`` on the ``updates`` collection
   with ``log_settings`` to capture extra context if the update fails
3. **Fault monitor** - OnChange on the ``faults`` collection to catch any
   fault reported during the update

Step 1: Create the update status trigger
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: bash

   curl -X POST http://localhost:8080/api/v1/apps/temp_sensor/triggers \
     -H "Content-Type: application/json" \
     -d '{
       "resource": "/api/v1/apps/temp_sensor/updates",
       "trigger_condition": {
         "condition_type": "OnChange"
       },
       "multishot": true,
       "lifetime": 120
     }'

**Response (201 Created):**

.. code-block:: json

   {
     "id": "trig_1",
     "status": "active",
     "observed_resource": "/api/v1/apps/temp_sensor/updates",
     "event_source": "/api/v1/apps/temp_sensor/triggers/trig_1/events",
     "protocol": "sse",
     "trigger_condition": {
       "condition_type": "OnChange"
     },
     "multishot": true,
     "persistent": false,
     "lifetime": 120
   }

Step 2: Create the failure alert trigger with log_settings
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The ``log_settings`` field configures temporary log capture when the trigger
fires, useful for recording context around a failure event.

.. code-block:: bash

   curl -X POST http://localhost:8080/api/v1/apps/temp_sensor/triggers \
     -H "Content-Type: application/json" \
     -d '{
       "resource": "/api/v1/apps/temp_sensor/updates",
       "trigger_condition": {
         "condition_type": "OnChangeTo",
         "target_value": "failed"
       },
       "multishot": true,
       "lifetime": 120,
       "log_settings": {
         "severity": "warning",
         "marker": "OTA update failure detected"
       }
     }'

Step 3: Create the fault monitor trigger
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: bash

   curl -X POST http://localhost:8080/api/v1/apps/temp_sensor/triggers \
     -H "Content-Type: application/json" \
     -d '{
       "resource": "/api/v1/apps/temp_sensor/faults",
       "trigger_condition": {
         "condition_type": "OnChange"
       },
       "multishot": true,
       "lifetime": 120
     }'

Step 4: List all triggers
~~~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: bash

   curl -H "Authorization: Bearer $TOKEN" http://localhost:8080/api/v1/apps/temp_sensor/triggers

**Response:**

.. code-block:: json

   {
     "items": [
       {
         "id": "trig_1",
         "status": "active",
         "observed_resource": "/api/v1/apps/temp_sensor/updates",
         "event_source": "/api/v1/apps/temp_sensor/triggers/trig_1/events",
         "protocol": "sse",
         "trigger_condition": {"condition_type": "OnChange"},
         "multishot": true,
         "persistent": false,
         "lifetime": 120
       },
       {
         "id": "trig_2",
         "status": "active",
         "observed_resource": "/api/v1/apps/temp_sensor/updates",
         "event_source": "/api/v1/apps/temp_sensor/triggers/trig_2/events",
         "protocol": "sse",
         "trigger_condition": {"condition_type": "OnChangeTo", "target_value": "failed"},
         "multishot": true,
         "persistent": false,
         "lifetime": 120
       },
       {
         "id": "trig_3",
         "status": "active",
         "observed_resource": "/api/v1/apps/temp_sensor/faults",
         "event_source": "/api/v1/apps/temp_sensor/triggers/trig_3/events",
         "protocol": "sse",
         "trigger_condition": {"condition_type": "OnChange"},
         "multishot": true,
         "persistent": false,
         "lifetime": 120
       }
     ]
   }

Step 5: Connect to SSE event streams
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Open a separate terminal for each trigger's event stream:

.. code-block:: bash

   # Terminal 2: Update status events
   curl -N http://localhost:8080/api/v1/apps/temp_sensor/triggers/trig_1/events

   # Terminal 3: Failure alert events
   curl -N http://localhost:8080/api/v1/apps/temp_sensor/triggers/trig_2/events

   # Terminal 4: Fault events
   curl -N http://localhost:8080/api/v1/apps/temp_sensor/triggers/trig_3/events

Events arrive as SSE frames:

.. code-block:: text

   data: {"timestamp":"2026-03-19T10:30:00.250Z","payload":{"status":"inProgress"}}

Step 6: Clean up
~~~~~~~~~~~~~~~~

.. code-block:: bash

   curl -X DELETE http://localhost:8080/api/v1/apps/temp_sensor/triggers/trig_1
   curl -X DELETE http://localhost:8080/api/v1/apps/temp_sensor/triggers/trig_2
   curl -X DELETE http://localhost:8080/api/v1/apps/temp_sensor/triggers/trig_3

Scenario 2: Thermal Protection
-------------------------------

A safety system monitors engine temperature using three triggers with
different condition types to implement a layered alert cascade:

1. **Range warning** - LeaveRange [20.0, 80.0] on ``data`` to warn when
   temperature leaves the normal operating range
2. **Critical alert** - OnChangeTo 95.0 on ``data`` with ``log_settings``
   to fire when temperature hits a dangerous threshold
3. **Component fault monitor** - OnChange on the component's ``faults``
   collection to catch any fault during thermal events

Step 1: Create the range warning trigger
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: bash

   curl -X POST http://localhost:8080/api/v1/apps/temp_sensor/triggers \
     -H "Content-Type: application/json" \
     -d '{
       "resource": "/api/v1/apps/temp_sensor/data/powertrain%2Fengine%2Ftemperature",
       "trigger_condition": {
         "condition_type": "LeaveRange",
         "lower_bound": 20.0,
         "upper_bound": 80.0
       },
       "multishot": true,
       "lifetime": 300
     }'

**Response (201 Created):**

.. code-block:: json

   {
     "id": "trig_1",
     "status": "active",
     "observed_resource": "/api/v1/apps/temp_sensor/data/powertrain%2Fengine%2Ftemperature",
     "event_source": "/api/v1/apps/temp_sensor/triggers/trig_1/events",
     "protocol": "sse",
     "trigger_condition": {
       "condition_type": "LeaveRange",
       "lower_bound": 20.0,
       "upper_bound": 80.0
     },
     "multishot": true,
     "persistent": false,
     "lifetime": 300
   }

Step 2: Create the critical temperature alert
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: bash

   curl -X POST http://localhost:8080/api/v1/apps/temp_sensor/triggers \
     -H "Content-Type: application/json" \
     -d '{
       "resource": "/api/v1/apps/temp_sensor/data/powertrain%2Fengine%2Ftemperature",
       "trigger_condition": {
         "condition_type": "OnChangeTo",
         "target_value": 95.0
       },
       "multishot": true,
       "lifetime": 300,
       "log_settings": {
         "severity": "error",
         "marker": "Critical temperature threshold exceeded"
       }
     }'

Step 3: Create the component fault trigger
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Note the use of ``/components/`` instead of ``/apps/`` - this monitors faults
at the component level, which aggregates faults from all apps within the
component.

.. note::

   In runtime-only discovery mode (the default), components are synthesized
   from ROS 2 namespaces. The ``engine`` component is auto-created from nodes
   running under the ``/powertrain/engine`` namespace. Verify it exists first:

   .. code-block:: bash

      curl -H "Authorization: Bearer $TOKEN" http://localhost:8080/api/v1/components | jq '.items[].id'

   If ``engine`` does not appear, the demo nodes may not be running or the
   namespace grouping may differ. Adjust ``engine`` to match the actual
   component ID shown.

.. code-block:: bash

   curl -X POST http://localhost:8080/api/v1/components/engine/triggers \
     -H "Content-Type: application/json" \
     -d '{
       "resource": "/api/v1/components/engine/faults",
       "trigger_condition": {
         "condition_type": "OnChange"
       },
       "multishot": true,
       "lifetime": 300
     }'

Step 4: Verify all triggers
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: bash

   # App-level triggers
   curl -H "Authorization: Bearer $TOKEN" http://localhost:8080/api/v1/apps/temp_sensor/triggers

   # Component-level triggers
   curl -H "Authorization: Bearer $TOKEN" http://localhost:8080/api/v1/components/engine/triggers

Step 5: Connect SSE streams and observe cascade
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: bash

   # Range warning stream
   curl -N http://localhost:8080/api/v1/apps/temp_sensor/triggers/trig_1/events

   # Critical alert stream
   curl -N http://localhost:8080/api/v1/apps/temp_sensor/triggers/trig_2/events

   # Component fault stream
   curl -N http://localhost:8080/api/v1/components/engine/triggers/trig_3/events

When temperature exceeds 80.0, the range warning trigger fires first. If
temperature reaches exactly 95.0, the critical alert fires as well. Any faults
reported by nodes in the ``engine`` component are independently caught by the
fault monitor.

Step 6: Clean up
~~~~~~~~~~~~~~~~

.. code-block:: bash

   curl -X DELETE http://localhost:8080/api/v1/apps/temp_sensor/triggers/trig_1
   curl -X DELETE http://localhost:8080/api/v1/apps/temp_sensor/triggers/trig_2
   curl -X DELETE http://localhost:8080/api/v1/components/engine/triggers/trig_3

Scenario 3: Fleet Diagnostics
------------------------------

A fleet monitoring dashboard uses triggers at different levels of the entity
hierarchy for layered diagnostic visibility:

1. **Area-level fault trigger** - OnChange on an area's ``faults`` collection
   to catch any fault from any component or app within the area
2. **App-level data trigger** - LeaveRange on a specific app's sensor data
   to monitor for out-of-range conditions

This scenario requires manifest discovery mode to establish a proper
area-component-app hierarchy.

Step 1: Start gateway in manifest mode
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: bash

   ros2 launch ros2_medkit_gateway gateway.launch.py \
     server_host:=0.0.0.0 \
     discovery_mode:=manifest_only \
     manifest_path:=$(ros2 pkg prefix ros2_medkit_gateway)/share/ros2_medkit_gateway/config/examples/demo_nodes_manifest.yaml

Step 2: Create the area-level fault trigger
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The ``/areas/powertrain/faults`` trigger monitors faults across the entire
powertrain area, including all components and apps within it.

.. code-block:: bash

   curl -X POST http://localhost:8080/api/v1/areas/powertrain/triggers \
     -H "Content-Type: application/json" \
     -d '{
       "resource": "/api/v1/areas/powertrain/faults",
       "trigger_condition": {
         "condition_type": "OnChange"
       },
       "multishot": true,
       "lifetime": 600
     }'

**Response (201 Created):**

.. code-block:: json

   {
     "id": "trig_1",
     "status": "active",
     "observed_resource": "/api/v1/areas/powertrain/faults",
     "event_source": "/api/v1/areas/powertrain/triggers/trig_1/events",
     "protocol": "sse",
     "trigger_condition": {
       "condition_type": "OnChange"
     },
     "multishot": true,
     "persistent": false,
     "lifetime": 600
   }

Step 3: Create the app-level data trigger
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: bash

   curl -X POST http://localhost:8080/api/v1/apps/engine-temp-sensor/triggers \
     -H "Content-Type: application/json" \
     -d '{
       "resource": "/api/v1/apps/engine-temp-sensor/data/powertrain%2Fengine%2Ftemperature",
       "trigger_condition": {
         "condition_type": "LeaveRange",
         "lower_bound": 15.0,
         "upper_bound": 90.0
       },
       "multishot": true,
       "lifetime": 600
     }'

Step 4: Verify triggers on different entity types
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: bash

   # Area-level triggers
   curl -H "Authorization: Bearer $TOKEN" http://localhost:8080/api/v1/areas/powertrain/triggers

   # App-level triggers
   curl -H "Authorization: Bearer $TOKEN" http://localhost:8080/api/v1/apps/engine-temp-sensor/triggers

Step 5: Connect SSE streams
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: bash

   # Area fault stream - catches faults from any entity in powertrain
   curl -N http://localhost:8080/api/v1/areas/powertrain/triggers/trig_1/events

   # App data stream - fires when temperature leaves [15, 90]
   curl -N http://localhost:8080/api/v1/apps/engine-temp-sensor/triggers/trig_2/events

When a fault is reported by any node in the ``powertrain`` area, the
area-level trigger fires. Meanwhile, the app-level trigger independently
monitors the temperature sensor for out-of-range values.

Step 6: Clean up
~~~~~~~~~~~~~~~~

.. code-block:: bash

   curl -X DELETE http://localhost:8080/api/v1/areas/powertrain/triggers/trig_1
   curl -X DELETE http://localhost:8080/api/v1/apps/engine-temp-sensor/triggers/trig_2

Key Concepts
------------

**Condition types** determine when a trigger fires:

- **OnChange** - fires whenever the monitored value changes
- **OnChangeTo** - fires when the value transitions to a specific target
- **LeaveRange** - fires when a numeric value exits a defined range
- **EnterRange** - fires when a numeric value enters a defined range

**Multishot vs one-shot**: set ``"multishot": true`` for continuous monitoring
or ``"multishot": false`` (default) to auto-terminate after the first event.

**Persistence**: set ``"persistent": true`` to survive gateway restarts (requires
``on_restart_behavior: "restore"`` in the gateway configuration).

**Hierarchy scoping**: area-level triggers catch changes from all entities
within the area, while app-level triggers only catch changes for that
specific app. This allows layered monitoring at different granularity levels.

.. note::

   **SOVD compliance**: The SOVD specification defines ``/triggers`` for
   apps and components only. ros2_medkit extends this to areas and functions.
   Script executions and software update locks are not yet observable as
   trigger resources - triggers currently support the ``data``, ``faults``,
   and ``updates`` collections.

See Also
--------

- :doc:`/api/rest` - Full REST API reference with trigger endpoint details
- :doc:`/config/server` - Gateway configuration including trigger settings
