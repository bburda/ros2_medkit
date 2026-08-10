Fault Manager Configuration
===========================

The ``ros2_medkit_fault_manager`` node aggregates and manages faults from multiple sources.
This page documents all configuration parameters.

.. contents:: Table of Contents
   :local:
   :depth: 2

Basic Configuration
-------------------

Storage
~~~~~~~

.. code-block:: yaml

   fault_manager:
     ros__parameters:
       storage_type: "sqlite"              # Storage backend: "sqlite" or "memory"
       database_path: "/var/lib/ros2_medkit/faults.db"  # Path for sqlite storage

.. list-table::
   :header-rows: 1
   :widths: 25 15 60

   * - Parameter
     - Default
     - Description
   * - ``storage_type``
     - ``sqlite``
     - Storage backend. ``sqlite`` persists faults to disk, ``memory`` keeps in RAM only.
   * - ``database_path``
     - ``/var/lib/ros2_medkit/faults.db``
     - File path for SQLite database. Directory must exist and be writable.

Debounce Settings
~~~~~~~~~~~~~~~~~

The fault manager uses AUTOSAR DEM-style debounce filtering to prevent fault flapping.

.. code-block:: yaml

   fault_manager:
     ros__parameters:
       confirmation_threshold: -1          # Counter threshold to confirm fault
       healing_enabled: false              # Enable auto-healing via PASSED events
       healing_threshold: 3                # Counter threshold to heal fault
       auto_confirm_after_sec: 0.0         # Auto-confirm timeout (0 = disabled)

.. list-table::
   :header-rows: 1
   :widths: 30 12 58

   * - Parameter
     - Default
     - Description
   * - ``confirmation_threshold``
     - ``-1``
     - Number of FAILED events to confirm fault. Negative values mean more events needed.
       Use ``-3`` to require 3 FAILED events before confirmation.
   * - ``healing_enabled``
     - ``false``
     - When true, PASSED events can heal confirmed faults.
   * - ``healing_threshold``
     - ``3``
     - Number of PASSED events to transition from CONFIRMED to HEALED.
   * - ``auto_confirm_after_sec``
     - ``0.0``
     - Auto-confirm prefailed faults after this duration. Set to 0 to disable.

.. tip::

   For immediate fault confirmation (no debounce), set ``confirmation_threshold: 0``.
   Faults with ``SEVERITY_CRITICAL`` always bypass debounce regardless of this setting.

Per-Entity Thresholds
~~~~~~~~~~~~~~~~~~~~~

Different subsystems often have different failure characteristics. For example, a lidar
sensor is binary (instant confirmation), while a motor controller may produce transient
errors that need debouncing. Per-entity thresholds let you configure different debounce
policies per reporting entity using longest-prefix matching on ``source_id``.

.. code-block:: yaml

   fault_manager:
     ros__parameters:
       # Global defaults (used when no entity-specific match)
       confirmation_threshold: -1
       healing_enabled: false
       healing_threshold: 3

       # Path to YAML file with per-entity overrides
       entity_thresholds:
         config_file: "/etc/ros2_medkit/entity_thresholds.yaml"

The entity thresholds config file uses a simple map of entity path prefixes to
threshold overrides:

.. code-block:: yaml

   # entity_thresholds.yaml
   /sensors/lidar:
     confirmation_threshold: -1    # instant - lidar is binary
     healing_threshold: 1

   /powertrain/motor_left:
     confirmation_threshold: -5    # motor has transients, need 5 events
     healing_threshold: 10

   /safety:
     confirmation_threshold: -1    # instant, never auto-heal
     healing_enabled: false

.. list-table::
   :header-rows: 1
   :widths: 35 15 50

   * - Parameter
     - Default
     - Description
   * - ``entity_thresholds.config_file``
     - ``""``
     - Path to YAML file with per-entity threshold overrides. Empty = disabled.

**How matching works:**

- The ``source_id`` is the identifier passed in ``ReportFault`` service requests, typically the
  fully qualified name of the reporting ROS 2 node (e.g., ``/sensors/lidar/front_node``).
  You can inspect actual ``source_id`` values in the ``reporting_sources`` field of existing
  faults via ``GET /api/v1/faults``.
- The ``source_id`` from ``ReportFault`` requests is matched against configured prefixes.
- The **longest matching prefix** wins. For example, ``/sensors/lidar/front`` matches
  ``/sensors/lidar`` over ``/sensors``.
- Unspecified fields in an entity override inherit from the global defaults.
- If no prefix matches, the global defaults apply.
- The config file is loaded once at node startup. Changes require a node restart.

.. note::

   When multiple entities report the same ``fault_code``, each event applies the
   thresholds resolved from that event's ``source_id``. This means the debounce
   behavior follows the reporting entity, not the fault.

   ``auto_confirm_after_sec`` and ``critical_immediate_confirm`` are global-only and
   cannot be overridden per-entity.

Snapshot Configuration
----------------------

Snapshots capture diagnostic data when faults occur.

Basic Snapshot Settings
~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: yaml

   fault_manager:
     ros__parameters:
       snapshots:
         enabled: true                     # Enable snapshot capture
         background_capture: false         # Capture in background thread
         timeout_sec: 1.0                  # Timeout for topic sampling
         max_message_size: 65536           # Max message size in bytes (64KB)
         default_topics: []                # Topics to capture for all faults
         config_file: ""                   # Path to YAML config file
         recapture_cooldown_sec: 60.0      # Min seconds between snapshot captures per fault
         max_per_fault: 10                 # Max snapshots stored per fault code (0 = unlimited)
         capture_pool_size: 2              # Max concurrent capture threads (>= 1)
         capture_queue_depth: 16           # Max pending captures before policy applies (>= 1)
         capture_queue_full_policy: reject_newest  # reject_newest | drop_oldest

.. list-table::
   :header-rows: 1
   :widths: 30 15 55

   * - Parameter
     - Default
     - Description
   * - ``snapshots.enabled``
     - ``true``
     - Master switch to enable/disable snapshot capture.
   * - ``snapshots.background_capture``
     - ``false``
     - Capture snapshots in background thread (non-blocking).
   * - ``snapshots.timeout_sec``
     - ``1.0``
     - Timeout for sampling each topic.
   * - ``snapshots.max_message_size``
     - ``65536``
     - Maximum message size to capture (bytes). Larger messages are truncated.
   * - ``snapshots.default_topics``
     - ``[]``
     - List of topics to capture for all faults. Empty entries are ignored.
   * - ``snapshots.entity_default``
     - ``true``
     - Zero-config fallback: when no explicit config matches a fault code,
       capture the reporting source node's own published topics. Set to
       ``false`` to opt out.
   * - ``snapshots.config_file``
     - ``""``
     - Path to YAML file with fault-specific snapshot configurations.
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
     - Max concurrent capture threads under a fault storm (>= 1). The capture pool is
       shared and created when snapshots **or** rosbag is enabled, so these parameters
       bound both. ``capture_pool_size`` parallelizes freeze-frame snapshot capture
       only; rosbag stays single-writer regardless - correlated faults confirming
       inside one post-roll window share a single recording.
   * - ``snapshots.capture_queue_depth``
     - ``16``
     - Max pending captures before the full-queue policy applies (>= 1).
   * - ``snapshots.capture_queue_full_policy``
     - ``reject_newest``
     - Policy when the queue is full: ``reject_newest`` or ``drop_oldest``.

Rosbag Recording
~~~~~~~~~~~~~~~~

Capture continuous rosbag recordings around fault events.

.. code-block:: yaml

   fault_manager:
     ros__parameters:
       snapshots:
         rosbag:
           enabled: false                  # Enable rosbag recording
           duration_sec: 5.0               # Pre-fault buffer duration
           duration_after_sec: 1.0         # Post-fault recording duration
           topics: "entity"                # Topic selection: "entity" (default), "config", "all", "explicit"
           include_topics: []              # Additional topics to include
           exclude_topics: []              # Topics to exclude
           exclude_sensor_topics: true     # Auto-exclude image/points/depth/compressed in broad modes
           lazy_start: false               # Start recording on first fault
           format: "sqlite3"               # Storage format
           qos_match: true                 # Match each topic's publisher QoS
           storage_path: ""                # Custom storage path
           max_buffer_mb: 256              # Ring-buffer RAM cap
           max_bag_size_mb: 50             # Max size per bag file
           max_total_storage_mb: 500       # Max total storage
           auto_cleanup: true              # Auto-delete old bags

.. list-table::
   :header-rows: 1
   :widths: 35 15 50

   * - Parameter
     - Default
     - Description
   * - ``rosbag.enabled``
     - ``false``
     - Enable rosbag recording for snapshots.
   * - ``rosbag.duration_sec``
     - ``5.0``
     - Duration of pre-fault circular buffer.
   * - ``rosbag.duration_after_sec``
     - ``1.0``
     - How long to record after fault. A fault confirming while this window is
       still running attaches to the in-flight recording and shares its bag,
       with one metadata entry per fault. At most 32 faults attach on top of
       the first; any beyond that are recorded in the bag's data but get no
       per-fault entry (logged as a warning). A fault confirming just *after*
       the window closes gets a post-fault-only bag of its own - see
       :ref:`rosbag-recording-lifecycle` below. Setting this to ``0`` disables
       the post-fault window entirely, and with it post-fault-only recordings.
   * - ``rosbag.topics``
     - ``entity``
     - Topic selection mode: ``entity`` (default; write only the faulting node's
       topics + ``/tf`` - a fault attaching to an in-flight recording adds its
       own node's topics from that point on), ``config`` (per-fault), ``all``,
       or ``explicit``.
   * - ``rosbag.exclude_sensor_topics``
     - ``true``
     - In broad modes (``all``/``entity``), auto-exclude high-bandwidth sensor
       topics (image/points/depth/compressed) to bound memory. Excluded topics are
       dropped silently; ``include_topics`` re-adds any you need.
   * - ``rosbag.qos_match``
     - ``true``
     - Subscribe with each topic's publisher-offered QoS for faithful capture
       instead of forcing best-effort.
   * - ``rosbag.max_buffer_mb``
     - ``256``
     - Ring-buffer RAM cap; oldest buffered messages drop past it.
   * - ``rosbag.lazy_start``
     - ``false``
     - Start recording only when first fault occurs.
   * - ``rosbag.max_bag_size_mb``
     - ``50``
     - Maximum size per rosbag file (MB).
   * - ``rosbag.max_total_storage_mb``
     - ``500``
     - Maximum total storage for all rosbags (MB). A recording shared by a
       burst of faults counts once towards the total, and eviction removes a
       whole burst's bag at a time (oldest first).
   * - ``rosbag.auto_cleanup``
     - ``true``
     - Delete a fault's bag when the fault is cleared. A recording shared by a
       burst survives until the last fault referencing it clears.

.. _rosbag-recording-lifecycle:

Recording Lifecycle and the Window Boundary
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Only one recording is open at a time: there is one ring buffer, one bag writer and
one post-fault window per fault manager. That single-writer design decides what a
fault gets depending on *when* it confirms relative to the previous fault's
``duration_after_sec`` window.

**While no recording is open.** Messages accumulate in the ring buffer. A
confirmation flushes the whole buffer into a new bag and, if
``duration_after_sec > 0``, keeps that bag open for the post-fault window. The
result is a full bag: pre-fault history plus post-fault response.

**Inside an open window.** The confirming fault attaches to the in-flight
recording and shares its bag, getting its own metadata entry (see
``duration_after_sec`` above). Nothing is buffered while a window is open -
incoming messages are written straight to the open bag.

**Right after a window closes.** The ring buffer is empty *by construction*: the
flush that opened the recording moved the whole buffer out, and everything
published during the window went into that bag rather than back into the buffer.
A fault confirming in this moment - typically the second fault of a burst - has no
pre-fault history to write. It gets a **post-fault-only bag**: a recording of its
own containing just its ``duration_after_sec`` window. It is a normal recording in
every other respect, so a further fault of the burst attaches to it as usual. The
fault manager logs the cause:

.. code-block:: text

   No pre-fault data buffered for fault 'BRAKE_PRESSURE_LOW' - recording post-fault window only

With ``duration_after_sec: 0`` there is no window to record into and no history to
write, so such a fault gets no bag at all (also logged). If the bag cannot be
written (unwritable ``storage_path``, missing storage backend), no recording is
opened and no metadata entry is stored - a failed capture never leaves a row
pointing at a bag that is not there.

A post-fault-only bag on a quiet or heavily filtered system can contain zero
messages. It still finalises normally on both ``sqlite3`` and ``mcap``, is listed
by the bulk-data endpoints and can be downloaded; only its payload is empty.

**What ``duration_sec`` on a stored bag means.** The value returned by
``~/get_rosbag`` and the gateway's bulk-data listing is the span the recording was
open, not the configured windows. It is a *recording* span, not a *content* span:
the zero-message bag above still reports its window rather than ``0.0``, because
"the black box covered these seconds and nothing was published" is the useful
statement and a bare zero would be indistinguishable from a broken artifact. A
post-fault-only bag therefore reports roughly ``duration_after_sec``, and a bag
flushed from a buffer that never filled reports the history it holds plus its
window. The span can also *exceed* ``duration_sec + duration_after_sec``:
the ring buffer is pruned only when a message arrives, so a topic that goes quiet
keeps its last window buffered until the next confirmation flushes it. That is
deliberate - a black box should keep the final messages of a topic that stopped
publishing - and the reported duration says so honestly.

.. seealso::

   :doc:`/tutorials/snapshots` for detailed snapshot configuration examples.

Correlation Configuration
-------------------------

Fault correlation identifies root causes and filters symptom faults.

.. code-block:: yaml

   fault_manager:
     ros__parameters:
       correlation:
         config_file: "/path/to/correlation_rules.yaml"
         cleanup_interval_sec: 5.0         # Interval for cleanup tasks

.. list-table::
   :header-rows: 1
   :widths: 35 15 50

   * - Parameter
     - Default
     - Description
   * - ``correlation.config_file``
     - ``""``
     - Path to YAML file defining correlation rules.
   * - ``correlation.cleanup_interval_sec``
     - ``5.0``
     - Interval for running correlation cleanup tasks.

.. seealso::

   :doc:`/tutorials/fault-correlation` for correlation rule syntax and examples.

Complete Example
----------------

.. code-block:: yaml

   fault_manager:
     ros__parameters:
       # Storage
       storage_type: "sqlite"
       database_path: "/var/lib/ros2_medkit/faults.db"

       # Debounce (require 3 FAILED events to confirm)
       confirmation_threshold: -3
       healing_enabled: true
       healing_threshold: 3
       auto_confirm_after_sec: 30.0

       # Per-entity debounce overrides
       entity_thresholds:
         config_file: "/etc/ros2_medkit/entity_thresholds.yaml"

       # Snapshots
       snapshots:
         enabled: true
         background_capture: true
         timeout_sec: 2.0
         max_message_size: 131072
         recapture_cooldown_sec: 60.0
         max_per_fault: 10
         default_topics:
           - /diagnostics
           - /rosout
         config_file: "/etc/ros2_medkit/snapshot_config.yaml"
         rosbag:
           enabled: true
           duration_sec: 10.0
           duration_after_sec: 2.0
           topics: "config"
           max_bag_size_mb: 100
           max_total_storage_mb: 1000
           auto_cleanup: true

       # Correlation
       correlation:
         config_file: "/etc/ros2_medkit/correlation_rules.yaml"
         cleanup_interval_sec: 10.0

See Also
--------

- :doc:`/tutorials/snapshots` - Diagnostic snapshot configuration
- :doc:`/tutorials/fault-correlation` - Fault correlation rules
- :doc:`/api/messages` - Message definitions (Fault.msg, FaultEvent.msg)
- :doc:`/design/ros2_medkit_fault_manager/index` - FaultManager architecture
