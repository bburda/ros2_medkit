Configuring Snapshot Capture
============================

This tutorial shows how to configure snapshot capture to automatically
preserve topic data when faults are confirmed, enabling post-mortem debugging.

.. contents:: Table of Contents
   :local:
   :depth: 2

Overview
--------

When a fault transitions to CONFIRMED status, the system automatically
captures data from ROS 2 topics. This snapshot preserves the system state
at the moment of fault occurrence, similar to:

- **AUTOSAR DEM freeze frames** - diagnostic data captured at fault detection
- **SOVD environment data** - system context for fault analysis

Snapshots are useful for:

- Debugging intermittent faults that are hard to reproduce
- Understanding system state when a fault occurred
- Post-mortem analysis without real-time access to the robot

Capture works out of the box with **zero configuration**: when no explicit
snapshot config matches a fault code, the faulting entity's own data is
captured by default (see :ref:`entity-default-freeze-frames`). Explicit
configuration always overrides the zero-config fallback when present.

.. note::

   Snapshots are automatically deleted when a fault is cleared via the
   ``DELETE /api/v1/faults/{code}`` endpoint or ``~/clear_fault`` service.

Quick Start
-----------

1. **Start the fault manager with snapshot capture enabled:**

   .. code-block:: bash

      ros2 run ros2_medkit_fault_manager fault_manager_node --ros-args \
        -p snapshots.enabled:=true \
        -p snapshots.default_topics:="['/odom', '/battery_state']"

2. **Start the gateway:**

   .. code-block:: bash

      ros2 launch ros2_medkit_gateway gateway.launch.py

3. **When a fault is confirmed, query its snapshots:**

   .. code-block:: bash

      curl http://localhost:8080/api/v1/faults/MOTOR_OVERHEAT/snapshots

Configuration Options
---------------------

Configure snapshot capture via fault manager parameters:

.. list-table::
   :widths: 30 15 55
   :header-rows: 1

   * - Parameter
     - Default
     - Description
   * - ``snapshots.enabled``
     - ``true``
     - Enable/disable snapshot capture
   * - ``snapshots.default_topics``
     - ``[]``
     - Topics to capture for all faults (empty entries are ignored)
   * - ``snapshots.entity_default``
     - ``true``
     - Zero-config fallback: when no explicit config matches a fault code,
       capture the reporting source node's own published topics. Set to
       ``false`` to opt out (unconfigured faults then get no freeze-frame).
   * - ``snapshots.config_file``
     - ``""``
     - Path to YAML config for fault-specific topics
   * - ``snapshots.timeout_sec``
     - ``1.0``
     - Timeout waiting for topic message
   * - ``snapshots.max_message_size``
     - ``65536``
     - Maximum message size in bytes (larger messages skipped)
   * - ``snapshots.background_capture``
     - ``false``
     - Use background subscriptions (caches latest message)
   * - ``snapshots.recapture_cooldown_sec``
     - ``60.0``
     - Minimum seconds between snapshot captures for the same fault code.
       Prevents snapshot storms when a fault is reported repeatedly. Set to 0 to disable.
   * - ``snapshots.max_per_fault``
     - ``10``
     - Maximum number of snapshots stored per fault code. When the limit is reached,
       new snapshots for that fault are rejected. Set to 0 for unlimited.
   * - ``snapshots.capture_pool_size``
     - ``2``
     - Max concurrent capture threads under a fault storm (>= 1). This parallelizes
       freeze-frame snapshot capture only; rosbag capture stays single-writer
       regardless of this value - correlated faults confirming inside one
       post-roll window share a single recording.
   * - ``snapshots.capture_queue_depth``
     - ``16``
     - Max pending captures before the full-queue policy applies (>= 1).
   * - ``snapshots.capture_queue_full_policy``
     - ``reject_newest``
     - Policy when the queue is full: ``reject_newest`` or ``drop_oldest``.

Advanced Configuration
----------------------

For fault-specific topic capture, create a YAML configuration file:

.. code-block:: yaml

   # snapshots.yaml
   fault_specific:
     MOTOR_OVERHEAT:
       - /joint_states
       - /motor/temperature
     BATTERY_LOW:
       - /battery_state
       - /power_management/status

   patterns:
     "MOTOR_.*":
       - /joint_states
       - /cmd_vel
     "SENSOR_.*":
       - /diagnostics

**Topic Resolution Priority:**

1. ``fault_specific`` - Exact match for fault code
2. ``patterns`` - Regex pattern match (first matching pattern wins)
3. ``default_topics`` - Fallback for all faults
4. Entity-default - zero-config fallback when nothing above matches
   (``snapshots.entity_default``, on by default)

**Launch with config file:**

.. code-block:: bash

   ros2 run ros2_medkit_fault_manager fault_manager_node --ros-args \
     -p snapshots.enabled:=true \
     -p snapshots.config_file:=/path/to/snapshots.yaml \
     -p snapshots.default_topics:="['/diagnostics']"

.. _entity-default-freeze-frames:

Zero-Config Entity Freeze-Frames
--------------------------------

With no snapshot configuration at all, every confirmed fault still carries
at-fault-time context: the faulting entity's own current data. Two paths
cover the two kinds of entities, both on by default with an opt-out flag,
and explicit config (``fault_specific`` / ``patterns`` / ``default_topics``)
always wins when it matches.

**ROS-backed entities** (``snapshots.entity_default``, fault manager): when
no explicit config matches the fault code, the fault manager captures the
topics *published by the fault's reporting source node(s)* (resolved from
``source_id``), excluding per-node noise (``/rosout``,
``/parameter_events``) and capped at 16 topics. These captures always sample
on demand, even when ``snapshots.background_capture`` is enabled, because
the topics are not known until the fault confirms. If the source is not a
live node (e.g. a plugin entity id) or publishes nothing, no freeze-frame
row is written.

**Plugin-backed entities** (``entity_freeze_frame.enabled``, gateway): PLC
apps bridged by protocol plugins report faults under their bare SOVD entity
id and their live values are not ROS topics, so the fault manager cannot
capture them. Instead, the gateway snapshots the entity's data values as
served by the owning plugin (its ``DataProvider``, i.e. the latest polled
values) when the fault confirms, and merges them into the fault detail's
``environment_data.snapshots`` as a standard ``freeze_frame`` entry named
after the entity - unless the fault manager already captured a freeze-frame
for that fault (explicit config wins). When the entity reports its link down
(the loss-of-comms case), the frozen values are the plugin's last known ones
and may predate the confirmation by the length of the outage; the entry's
``x-medkit`` block then carries ``connected: false`` and, when the plugin's
payload includes one, ``source_timestamp`` (the payload's own timestamp)
alongside ``captured_at``.

Faults that are already confirmed when the gateway starts are caught up at
startup: the gateway lists the confirmed faults and captures a frame for each
plugin-backed one, so a device standing in fault across a gateway restart
still gets a frame. Catch-up frames carry ``"capture_origin": "startup"`` in
their ``x-medkit`` block because their values were read at gateway start, not
when the fault confirmed (which may be long before, since the fault manager
persists faults); ``captured_at`` always stamps the moment the values were
read. Frames without the marker were captured on the confirm edge. Disable
with:

.. code-block:: bash

   ros2 run ros2_medkit_gateway gateway_node --ros-args \
     -p entity_freeze_frame.enabled:=false

Example plugin-entity freeze-frame in the fault response:

.. code-block:: json

   {
     "type": "freeze_frame",
     "name": "beckhoff_plc_app",
     "data": {"tank_level": 87.5, "pump_running": true},
     "x-medkit": {
       "topic": "",
       "message_type": "",
       "full_data": {"tank_level": 87.5, "pump_running": true},
       "captured_at": "2026-07-14T12:00:00.000Z"
     }
   }

Querying Snapshots
------------------

Snapshots are included inline in the fault response as ``environment_data``:

**Get fault details with snapshots:**

.. code-block:: bash

   curl http://localhost:8080/api/v1/apps/motor_controller/faults/MOTOR_OVERHEAT

**Response:**

.. code-block:: json

   {
     "item": {
       "code": "MOTOR_OVERHEAT",
       "fault_name": "Motor temperature exceeded threshold",
       "severity": 2,
       "status": {
         "aggregatedStatus": "active",
         "testFailed": "1",
         "confirmedDTC": "1"
       }
     },
     "environment_data": {
       "extended_data_records": {
         "first_occurrence": "2026-02-04T10:30:00.000Z",
         "last_occurrence": "2026-02-04T10:35:00.000Z"
       },
       "snapshots": [
         {
           "type": "freeze_frame",
           "name": "motor_temperature",
           "data": 85.5,
           "x-medkit": {
             "topic": "/motor/temperature",
             "message_type": "sensor_msgs/msg/Temperature",
             "full_data": {"temperature": 85.5, "variance": 0.1},
             "captured_at": "2026-02-04T10:30:00.123Z"
           }
         },
         {
           "type": "rosbag",
           "name": "fault_recording",
           "bulk_data_uri": "/apps/motor_controller/bulk-data/rosbags/550e8400-e29b-41d4-a716-446655440000",
           "size_bytes": 1234567,
           "duration_sec": 6.0,
           "format": "mcap"
         }
       ]
     },
     "x-medkit": {
       "occurrence_count": 3,
       "reporting_sources": ["/powertrain/motor_controller"]
     }
   }

**Snapshot Types:**

- ``freeze_frame``: Data captured at fault confirmation (JSON format). Entity
  frames caught up for faults that predate the gateway are captured at
  gateway start instead, marked ``x-medkit.capture_origin: startup``; a
  plugin entity that reports its link down contributes its last known values,
  marked ``connected: false`` in ``x-medkit``
- ``rosbag``: Recording file available via bulk-data endpoint (binary format)

**Get snapshots from fault response using jq:**

.. code-block:: bash

   curl http://localhost:8080/api/v1/apps/motor_controller/faults/MOTOR_OVERHEAT | \
     jq '.environment_data.snapshots'

Example Workflow
----------------

This example demonstrates the complete snapshot capture workflow.

**1. Configure and start the fault manager:**

.. code-block:: bash

   ros2 run ros2_medkit_fault_manager fault_manager_node --ros-args \
     -p snapshots.enabled:=true \
     -p snapshots.default_topics:="['/odom']"

**2. Start a node that publishes odometry:**

.. code-block:: bash

   ros2 topic pub /odom nav_msgs/msg/Odometry \
     "{pose: {pose: {position: {x: 1.5, y: 2.0}}}}" -r 10

**3. Report a fault (it will be confirmed immediately by default):**

.. code-block:: bash

   ros2 service call /fault_manager/report_fault ros2_medkit_msgs/srv/ReportFault \
     "{fault_code: 'NAV_ERROR', event_type: 0, severity: 2, \
       description: 'Navigation failed', source_id: '/nav_node'}"

**4. Query the captured snapshot:**

.. code-block:: bash

   curl http://localhost:8080/api/v1/apps/nav_node/faults/NAV_ERROR | \
     jq '.environment_data.snapshots'

The response will contain the odometry data that was captured at the
moment the fault was confirmed.

Troubleshooting
---------------

**No snapshots captured**

- Verify ``snapshots.enabled`` is ``true``
- Check that configured topics exist and are publishing
- Increase ``snapshots.timeout_sec`` for slow-publishing topics
- Check fault manager logs for capture errors

**Empty topics object in response**

- The fault may have been cleared (snapshots are deleted on clear)
- No topics were configured for this fault code
- All configured topics timed out or exceeded size limit

**Snapshot data truncated**

- Message exceeded ``snapshots.max_message_size``
- Increase the limit or filter to smaller topics

**Wrong topics captured**

- Check topic resolution priority (fault_specific > patterns > default)
- Verify regex patterns in config file are correct

Rosbag Capture (Time-Window Recording)
--------------------------------------

In addition to JSON snapshots, you can enable **rosbag capture** for "black box"
style recording. This continuously buffers messages in memory and flushes them
to a bag file when a fault is confirmed.

**Key differences from JSON snapshots:**

.. list-table::
   :widths: 25 35 40
   :header-rows: 1

   * - Feature
     - JSON Snapshots
     - Rosbag Capture
   * - Data format
     - JSON (human-readable)
     - Binary (native ROS 2)
   * - Time coverage
     - Point-in-time (at confirmation)
     - Time window (before + after fault)
   * - Message fidelity
     - Converted to JSON
     - Original serialization preserved
   * - Playback
     - N/A
     - ``ros2 bag play``
   * - Default
     - Enabled
     - Disabled

Enabling Rosbag Capture
^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: bash

   ros2 run ros2_medkit_fault_manager fault_manager_node --ros-args \
     -p snapshots.rosbag.enabled:=true \
     -p snapshots.rosbag.duration_sec:=5.0 \
     -p snapshots.rosbag.duration_after_sec:=1.0

This captures 5 seconds of data **before** the fault and 1 second **after**.

Rosbag Configuration Options
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. list-table::
   :widths: 35 15 50
   :header-rows: 1

   * - Parameter
     - Default
     - Description
   * - ``snapshots.rosbag.enabled``
     - ``false``
     - Enable rosbag capture. When enabled, the system continuously buffers
       messages in memory and writes them to a bag file when faults are confirmed.
   * - ``snapshots.rosbag.duration_sec``
     - ``5.0``
     - Ring buffer duration in seconds. This determines how much history is
       preserved before the fault confirmation. Larger values provide more
       context but consume more memory.
   * - ``snapshots.rosbag.duration_after_sec``
     - ``1.0``
     - Post-fault recording duration. After a fault is confirmed, recording
       continues for this many seconds to capture immediate system response.
       A fault confirming while this window is still running attaches to the
       in-flight recording and shares its bag, with one metadata entry per
       fault. At most 32 faults attach on top of the first; any beyond that
       are recorded in the bag's data but get no per-fault entry (logged as
       a warning). A fault confirming just *after* the window closes gets a
       post-fault-only bag - see `Bursts and the Window Boundary`_ below.
   * - ``snapshots.rosbag.topics``
     - ``"entity"``
     - Topic selection mode:

       - ``"entity"`` - Default. Subscribe broadly for pre-roll, but on fault
         confirmation write only the faulting source node's topics (resolved from
         the fault's reporting source) plus ``/tf`` and ``/tf_static``. Falls back
         to the full buffer if the source is not a live node. A fault attaching
         to an in-flight recording adds its own node's topics from that point
         on (or widens the recording to all topics if its scope is unresolved).
       - ``"config"`` - Use same topics as JSON snapshots (from config file)
       - ``"all"`` or ``"auto"`` - Auto-discover and record all available topics
       - ``"explicit"`` - Use only topics from ``include_topics`` list

       .. note::

          The default changed from ``"config"`` to ``"entity"`` so an enabled black
          box produces a useful, fault-scoped bag with no per-topic configuration.
          Set ``topics: "config"`` to restore the previous behavior.

          Faults reported by the ``diagnostic_bridge`` carry the bridge's own node
          as the source, so in ``entity`` mode the bag is scoped to the bridge
          (e.g. ``/diagnostics``) rather than the node that actually failed. Use a
          manual mode if you need a different scope for bridge-sourced faults.
   * - ``snapshots.rosbag.include_topics``
     - ``[]``
     - Explicit list of topics to record (only used when ``topics: "explicit"``).
       Example: ``["/odom", "/joint_states", "/cmd_vel"]``
   * - ``snapshots.rosbag.exclude_topics``
     - ``[]``
     - Topics to exclude from recording (applies to all modes). Useful for
       filtering high-bandwidth topics like camera images.
   * - ``snapshots.rosbag.exclude_sensor_topics``
     - ``true``
     - In broad modes (``all``/``entity``), auto-exclude high-bandwidth sensor
       topics (image/points/depth/compressed) to bound memory. Dropped silently;
       ``include_topics`` re-adds any you specifically need.
   * - ``snapshots.rosbag.qos_match``
     - ``true``
     - Subscribe with each topic's publisher-offered QoS (reliable/transient-local
       where offered) for faithful capture, instead of forcing best-effort.
   * - ``snapshots.rosbag.max_buffer_mb``
     - ``256``
     - In-memory ring-buffer cap; oldest buffered messages drop once exceeded, so a
       broad subscribe set cannot grow memory without bound.
   * - ``snapshots.rosbag.format``
     - ``"sqlite3"``
     - Bag storage format: ``"sqlite3"`` (default, widely compatible) or
       ``"mcap"`` (more efficient compression, requires plugin).
   * - ``snapshots.rosbag.storage_path``
     - ``""``
     - Directory for bag files. Empty string uses system temp directory
       (``/tmp``). Bags are named ``fault_{code}_{timestamp}/`` after the first
       fault of the recording; faults that attached during its post-roll window
       are served from that same directory.
   * - ``snapshots.rosbag.auto_cleanup``
     - ``true``
     - Automatically delete a fault's bag when it is cleared. A recording
       shared by a burst of faults is deleted when the last fault referencing
       it clears. Set to ``false`` to retain bags for manual analysis.
   * - ``snapshots.rosbag.lazy_start``
     - ``false``
     - Controls when the ring buffer starts recording. See diagram below.
   * - ``snapshots.rosbag.max_bag_size_mb``
     - ``50``
     - Maximum size per bag file in MB. When exceeded, rosbag2 creates
       additional segment files.
   * - ``snapshots.rosbag.max_total_storage_mb``
     - ``500``
     - Total storage limit for all bag files. Oldest bags are automatically
       deleted when this limit is exceeded. A recording shared by a burst of
       faults counts once towards the total, and eviction removes a whole
       burst's bag at a time.

Bursts and the Window Boundary
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Faults rarely arrive alone. One root cause produces a burst, and where each fault
of that burst lands relative to the previous ``duration_after_sec`` window decides
what it gets. There is only ever one recording open (one ring buffer, one writer),
so the three cases are:

.. list-table::
   :widths: 30 70
   :header-rows: 1

   * - When the fault confirms
     - What it gets
   * - No recording open
     - A **full bag**: the buffered pre-fault history plus the post-fault window.
   * - Inside an open window
     - **Attached** to the in-flight recording - the same bag, with its own
       metadata entry. Up to 32 faults attach on top of the first.
   * - Right after a window closed
     - A **post-fault-only bag**: its own recording holding just its
       ``duration_after_sec`` window.

The third case is not an edge case, it is the normal shape of a burst. Nothing is
buffered while a window is open (messages go straight into the open bag), and the
flush that opened that bag had already emptied the buffer, so the next fault to
confirm genuinely has no pre-fault history available. It still gets a black box
covering how the system behaved *after* it failed, which is what the second fault
of a burst is usually investigated for. The log names the cause:

.. code-block:: text

   No pre-fault data buffered for fault 'BRAKE_PRESSURE_LOW' - recording post-fault window only

Two consequences worth knowing:

- **With** ``duration_after_sec: 0`` **there is no window**, so a fault landing on
  an empty buffer gets no bag at all - there is neither history to write nor a
  window to record into. If bursts matter to you, keep a non-zero post-fault
  window.
- **A post-fault-only bag may contain zero messages** if nothing publishes during
  the window (a quiet system, or a narrow ``topics`` filter). It still finalises
  and downloads normally on both storage formats; only the payload is empty.

The stored ``duration_sec`` is the span the recording was open, not the configured
windows, so the two kinds of bag are easy to tell apart:

.. code-block:: bash

   ros2 service call /fault_manager/get_rosbag ros2_medkit_msgs/srv/GetRosbag \
     "{fault_code: 'BRAKE_PRESSURE_LOW'}"
   # duration_sec ~= duration_after_sec -> post-fault-only bag
   # anything longer                    -> the recording also holds pre-fault history

Note the second rule has no upper bound, deliberately. The span can exceed
``duration_sec + duration_after_sec``: the ring buffer is pruned only when a
message arrives, so a topic that stops publishing keeps its last window buffered
until the next confirmation flushes it. That is intentional - the final messages
before a topic died are exactly what a black box is for - and the reported
duration reflects it.

It is a *recording* span and not a *content* span: a post-fault-only bag whose
window stayed quiet still reports its window rather than zero, because the useful
statement is "the black box covered these seconds and nothing was published"
rather than a bare ``0.0`` that would be indistinguishable from a broken artifact.

Understanding lazy_start Mode
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The ``lazy_start`` parameter controls when the ring buffer starts recording:

**lazy_start: false (default)** - Recording starts immediately at node startup.
Best for development and when you need maximum context for any fault.

.. uml::

   @startuml
   skinparam backgroundColor transparent
   participant "Ring Buffer" as RB
   participant "Fault Manager" as FM

   == Node Startup ==
   RB -> RB : Start recording
   note right of RB #LightGreen : Buffer active\n(continuous)

   ... time passes (messages buffered) ...

   == Fault Detected ==
   FM -> FM : PREFAILED
   note right of RB #LightGreen : duration_sec\nof data buffered

   FM -> FM : CONFIRMED
   FM -> RB : Flush buffer
   RB -> RB : Write pre-fault data
   note right of RB #LightBlue : Post-fault\nrecording

   ... duration_after_sec ...

   RB -> RB : Save bag file
   note right of RB #LightGreen : Resume\nbuffering
   @enduml

**lazy_start: true** - Recording only starts when a fault enters PREFAILED state.
Saves resources but may miss context if fault confirms before buffer fills.

.. uml::

   @startuml
   skinparam backgroundColor transparent
   participant "Ring Buffer" as RB
   participant "Fault Manager" as FM

   == Node Startup ==
   note right of RB #LightGray : Buffer inactive\n(saving resources)

   ... time passes (no recording) ...

   == Fault Detected ==
   FM -> FM : PREFAILED
   FM -> RB : Start buffer
   note right of RB #LightGreen : Recording\nstarts now

   ... buffer filling (may be < duration_sec) ...

   FM -> FM : CONFIRMED
   FM -> RB : Flush buffer
   RB -> RB : Write pre-fault data
   note right of RB #LightBlue : Post-fault\nrecording

   ... duration_after_sec ...

   RB -> RB : Save bag file
   note right of RB #LightGray : Buffer\ninactive
   @enduml

**When to use lazy_start: true:**

- Production systems with limited resources
- When faults have reliable PREFAILED → CONFIRMED progression
- Systems where most faults are debounced (enter PREFAILED first)

**When to use lazy_start: false:**

- Development and debugging
- When faults may skip PREFAILED state (severity 3 = CRITICAL)
- When maximum fault context is more important than resource usage

.. note::

   The ``"mcap"`` format requires ``rosbag2_storage_mcap`` to be installed.
   ``"sqlite3"`` (the default) is always shipped with rosbag2 and needs no extra
   package.

   You do not have to switch formats manually: if a configured backend's plugin
   is unavailable at startup, the FaultManager logs a warning naming the missing
   package and automatically falls back to ``"sqlite3"`` for black-box capture.
   If no storage backend is usable at all, rosbag capture self-disables (the node
   keeps running and freeze-frame snapshots are unaffected) instead of crashing.

   To use mcap (e.g. for Foxglove), install the plugin:

   .. code-block:: bash

      # Install MCAP support (optional)
      sudo apt install ros-${ROS_DISTRO}-rosbag2-storage-mcap

Downloading Rosbag Files
^^^^^^^^^^^^^^^^^^^^^^^^

Rosbag files are downloaded via SOVD bulk-data endpoints.

**1. List available rosbags for an entity:**

.. code-block:: bash

   curl http://localhost:8080/api/v1/apps/motor_controller/bulk-data/rosbags

**Response:**

.. code-block:: json

   {
     "items": [
       {
         "id": "550e8400-e29b-41d4-a716-446655440000",
         "name": "MOTOR_OVERHEAT recording",
         "mimetype": "application/x-mcap",
         "size": 1234567,
         "creation_date": "2026-02-04T10:30:00.000Z",
         "x-medkit": {
           "fault_code": "MOTOR_OVERHEAT",
           "duration_sec": 6.0,
           "format": "mcap"
         }
       }
     ]
   }

**2. Download a specific rosbag:**

Use the ``bulk_data_uri`` from the fault response, or construct from listing:

.. code-block:: bash

   # Using bulk_data_uri from fault response
   curl -O -J http://localhost:8080/api/v1/apps/motor_controller/bulk-data/rosbags/550e8400-e29b-41d4-a716-446655440000

The ``-J`` flag uses the server-provided filename from ``Content-Disposition`` header.

**3. Play back the rosbag:**

.. code-block:: bash

   ros2 bag play MOTOR_OVERHEAT.mcap

**Via ROS 2 service (alternative):**

.. code-block:: bash

   ros2 service call /fault_manager/get_rosbag ros2_medkit_msgs/srv/GetRosbag \
     "{fault_code: 'MOTOR_OVERHEAT'}"

Example: Production Configuration
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

For production use with conservative resource usage:

.. code-block:: yaml

   # config/snapshots.yaml
   rosbag:
     enabled: true
     duration_sec: 3.0
     duration_after_sec: 0.5
     topics: "config"           # Use same topics as JSON snapshots
     lazy_start: true           # Save resources until fault detected
     format: "sqlite3"
     max_bag_size_mb: 25
     max_total_storage_mb: 200
     auto_cleanup: true

   # Exclude high-bandwidth topics
   # exclude_topics:
   #   - /camera/image_raw
   #   - /pointcloud

Example: Debugging Configuration
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

For development with maximum context:

.. code-block:: yaml

   rosbag:
     enabled: true
     duration_sec: 10.0         # 10 seconds before fault
     duration_after_sec: 2.0    # 2 seconds after
     topics: "config"
     lazy_start: false          # Always recording
     format: "sqlite3"
     storage_path: "/var/log/ros2_medkit/rosbags"
     max_bag_size_mb: 100
     max_total_storage_mb: 1000
     auto_cleanup: false        # Keep bags for analysis

See Also
--------

- :doc:`../api/rest` - REST API reference (Bulk Data section)
- :doc:`../requirements/specs/faults` - Fault API requirements
- `Gateway README <https://github.com/selfpatch/ros2_medkit/blob/main/src/ros2_medkit_gateway/README.md>`_ - REST API reference
- `config/snapshots.yaml <https://github.com/selfpatch/ros2_medkit/blob/main/src/ros2_medkit_fault_manager/config/snapshots.yaml>`_ - Full configuration reference

Migration from Legacy Endpoints
-------------------------------

If you were using the legacy snapshot endpoints, migrate to the new SOVD-compliant API:

**Snapshots:**

.. list-table::
   :widths: 45 55
   :header-rows: 1

   * - Previous (removed)
     - Current
   * - ``GET /faults/{code}/snapshots``
     - ``GET /apps/{app}/faults/{code}`` → ``environment_data.snapshots[]``
   * - ``GET /components/{id}/faults/{code}/snapshots``
     - ``GET /components/{id}/faults/{code}`` → ``environment_data.snapshots[]``

**Rosbag Downloads:**

.. list-table::
   :widths: 45 55
   :header-rows: 1

   * - Previous (removed)
     - Current
   * - ``GET /faults/{code}/snapshots/bag``
     - ``GET /apps/{app}/bulk-data/rosbags/{id}``
   * - ``GET /components/{id}/faults/{code}/snapshots/bag``
     - ``GET /components/{id}/bulk-data/rosbags/{id}``

**Key Changes:**

1. **Snapshots inline**: No separate snapshot endpoint; data is in fault response
2. **Bulk-data pattern**: Rosbags use SOVD bulk-data with UUID identifiers
3. **Entity-scoped**: Bulk-data endpoints require entity path (e.g., ``/apps/motor``)
4. **SOVD status**: Fault response includes SOVD-compliant ``status`` object
