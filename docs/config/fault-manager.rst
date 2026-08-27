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

.. important::

   The counter only moves when an event arrives, so ``confirmation_threshold`` and
   ``healing_threshold`` work only for a reporter that keeps sending FAILED while the condition
   is still there.

   A reporter that sends one FAILED when a condition appears and one clear when it goes away
   never sends the second event. ``confirmation_threshold: -3`` then leaves the fault in
   PREFAILED forever, and the default fault list returns CONFIRMED only, so the fault is never
   seen. ``healing_threshold: 3`` has the same problem: healing needs
   ``healing_threshold - confirmation_threshold`` consecutive PASSED events, and only one is
   sent, so the fault stays CONFIRMED until someone calls ``~/clear_fault``.

   For such a reporter, filter by time instead of by count:

   .. code-block:: yaml

      confirmation_threshold: -2      # first FAILED stays PREFAILED
      auto_confirm_after_sec: 3.0     # confirm it if it is still there after 3 s
      healing_enabled: true
      healing_threshold: 0            # heal on the single PASSED

   Choose ``auto_confirm_after_sec`` from how often the reporter samples, so a condition has to
   survive a few sampling cycles before it confirms. A glitch that clears in time never reaches
   CONFIRMED, because the clear takes the fault out of PREFAILED before the timer fires.

Near-Miss Retention
~~~~~~~~~~~~~~~~~~~

A **near miss** is a FAILED report that moved the debounce counter without the fault ending up
CONFIRMED - the fault nearly happened. PASSED reports move the counter in the healing direction
(the fault receding) and are not near misses.

The fault manager appends one entry per near miss to a per-fault-code series, holding the
timestamp, the counter value after the report, the confirmation threshold, the severity, the
reporting source and the fault status the report left behind.

The status matters when reading the series. The HEALED latch holds the status all the way from the
healing threshold down to the confirmation threshold, so reports on the way back into a fault that
does confirm are also near misses by the definition above. Entries recording ``PREFAILED`` are
approaches from a resting state; entries recording ``HEALED`` are a counter walking back down under
the latch. The recorded confirmation threshold belongs to the reporting source, while the counter
is shared by all sources of that fault code, so with per-entity thresholds it is not on its own the
distance to confirmation. The series is **retained when the fault is cleared**, because acknowledging one
fault cycle must not erase how often that code approached confirmation across cycles.

.. code-block:: yaml

   fault_manager:
     ros__parameters:
       near_miss:
         max_per_fault: 200                # Entries kept per fault code (0 = unlimited)

.. list-table::
   :header-rows: 1
   :widths: 30 12 58

   * - Parameter
     - Default
     - Description
   * - ``near_miss.max_per_fault``
     - ``200``
     - Near-miss entries retained per fault code. When the bound is reached the **oldest**
       entries are evicted, the same direction as ``snapshots.max_per_fault`` and the rosbag cap:
       a series frozen at boot says nothing about whether the rate of near misses is changing.
       Set to 0 for unlimited, accepting growth with the reporting rate.

.. note::

   The series lives in the ``near_misses`` table of the fault database and is read through the
   storage API (``FaultStorage::get_near_misses``). A database written by an earlier build gains
   the table on first open. There is no service or REST surface for it yet.

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

   ``auto_confirm_after_sec`` is global-only and cannot be overridden per-entity.
   Critical faults skip debounce and confirm on their first occurrence; that is
   built in, not a parameter, so it can be neither disabled nor set per entity.

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
     - Maximum number of snapshot rows stored per fault code. One confirmation
       writes one row per configured topic, and those rows are evicted together:
       past the limit the OLDEST capture set is dropped whole. A capture larger
       than the cap is kept anyway rather than torn, since half a freeze frame is
       indistinguishable from topics that were silent. Set to 0 for unlimited.
   * - ``snapshots.retain_on_clear``
     - ``false``
     - Keep a fault's value snapshots when it is acknowledged. ``false`` is the
       historical behaviour: clearing a fault deletes them. Turn it on together
       with ``rosbag.max_bags_per_fault``, or acknowledging leaves the fault
       holding recordings whose matching readings are gone. Independent of
       ``max_per_fault``, which still bounds growth either way.
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
           format: "mcap"                  # Storage format (default: mcap)
           qos_match: true                 # Match each topic's publisher QoS
           storage_path: ""                # Custom storage path
           max_buffer_mb: 256              # Ring-buffer RAM cap
           max_bag_size_mb: 50             # Max size per bag file
           max_total_storage_mb: 500       # Max total storage
           max_bags_per_fault: 1           # Recordings kept per fault code
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
   * - ``rosbag.include_topics``
     - ``[]``
     - Topics to record on top of whatever the selection mode picked. This is
       how a topic dropped by ``exclude_sensor_topics`` is brought back.
   * - ``rosbag.exclude_topics``
     - ``[]``
     - Topics to drop from whatever the selection mode picked.
   * - ``rosbag.storage_path``
     - ``""``
     - Directory the bag files are written to. Empty falls back to the system
       temporary directory, which on most systems is cleared on reboot - set
       this if the bags have to survive one.
   * - ``rosbag.qos_match``
     - ``true``
     - Subscribe with each topic's publisher-offered QoS for faithful capture
       instead of forcing best-effort.
   * - ``rosbag.format``
     - ``mcap``
     - Bag storage format: ``mcap`` (default; opens directly in Foxglove and
       Lichtblick) or ``sqlite3`` (also what an unknown format string lands
       on). Neither is privileged: whichever is configured, an unavailable
       plugin falls back automatically to the other one, and capture disables
       itself only if neither loads. Both plugins are runtime dependencies of
       ``ros2_medkit_fault_manager``.
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
   * - ``rosbag.max_bags_per_fault``
     - ``1``
     - How many recordings one fault code keeps. Past the cap the oldest is
       unlinked, so the default reproduces the historical behaviour exactly: a
       new recording replaces the previous one. ``0`` means unlimited, bounded
       only by ``max_total_storage_mb``. ``3`` is a reasonable value for a fault
       that flaps - see the note below before raising it.
   * - ``rosbag.auto_cleanup``
     - ``true``
     - Delete a fault's bags when the fault is cleared. A recording shared by a
       burst survives until the last fault referencing it clears. Has no effect
       once ``max_bags_per_fault`` is anything other than ``1``: a history someone
       configured must not be what an acknowledgement takes away, so the cap
       governs retention there instead.

.. note::

   ``max_bags_per_fault`` is a **fairness** knob, not a depth knob.
   ``max_total_storage_mb`` is the real disk bound and eviction across it is
   global and oldest-first, so a fault that flaps often enough will consume the
   budget and push out every other fault's black box. Raise the per-fault cap
   when you need the history of a specific intermittent fault; raise the total
   budget with it if other faults still need theirs.

   The cap keeps the newest recordings and evicts the oldest, the same direction
   as ``snapshots.max_per_fault``. Refusing a NEW recording instead would mean a
   technician standing next to a machine faulting right now downloads a bag from
   three days ago.

   ``snapshots.recapture_cooldown_sec`` (default 60 s) gates the capture job as a
   whole, rosbags included, so it puts a floor under how fast a history can grow:
   a fault that returns sooner than the cooldown keeps ONE recording however high
   this cap is. That is the fast-flapping fault the cap exists for, so lower the
   cooldown when you raise the cap. The fault manager logs a warning at startup
   when the two are configured against each other.

.. _rosbag-recording-lifecycle:

Recording Lifecycle and the Window Boundary
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Only one recording is open at a time: there is one ring buffer, one bag writer and
one post-fault window per fault manager. That single-writer design decides what a
fault gets depending on *when* it confirms relative to the previous fault's
``duration_after_sec`` window.

**While no recording is open, with history buffered.** Messages accumulate in the
ring buffer. A confirmation flushes the whole buffer into a new bag and, if
``duration_after_sec > 0``, keeps that bag open for the post-fault window. The
result is a full bag: pre-fault history plus post-fault response.

**Inside an open window.** The confirming fault attaches to the in-flight
recording and shares its bag, getting its own metadata entry (see
``duration_after_sec`` above). Nothing is buffered while a window is open -
incoming messages are written straight to the open bag.

**While no recording is open, with the buffer empty.** A fault confirming here has
no pre-fault history to write. It gets a **post-fault-only bag**: a recording of its
own containing just its ``duration_after_sec`` window. It is a normal recording in
every other respect, so a further fault of the burst attaches to it as usual.

The usual way to reach it is right after a window closes. The flush that opened the
previous recording moved the whole buffer out, and everything published during its
window went into that bag rather than back into the buffer, so the fault confirming
in that moment - typically the second fault of a burst - has nothing to write. A
fault confirming before any captured topic has published, just after startup or with
``lazy_start``, finds the same empty buffer and gets the same bag. Either way the
fault manager logs the cause:

.. code-block:: text

   No pre-fault data buffered for fault 'BRAKE_PRESSURE_LOW' - recording post-fault window only

**How often this case actually arises depends on** ``topics``. The broad modes -
``all``, ``auto`` and ``entity``, which is the default - subscribe to everything on
the graph, and that includes ``/fault_manager/events``. Reporting a fault publishes
an event there, so the buffer is refilled by the act of reporting and the next fault
of the burst usually finds history rather than an empty buffer. The empty-buffer case
belongs mainly to a narrowed capture: ``explicit``, a topic list, ``config``, or a
broad mode with ``/fault_manager/events`` in ``exclude_topics``.

With ``duration_after_sec: 0`` there is no window to record into and no history to
write, so such a fault gets no bag at all (also logged). If the bag cannot be
written (unwritable ``storage_path``, missing storage backend), no recording is
opened and no metadata entry is stored.

A capture never leaves a row pointing at a bag that is not there, and never leaves a
bag that no row points at. If the metadata cannot be stored, the recording is
discarded with it: nothing could reach that bag anyway, since retrieval is keyed by
fault code and the quota is computed from rows, so it would sit on disk unreachable
and uncounted. If the quota sweep that follows fails instead, the bag and its rows
both stay - the sweep is about the whole store, not about this recording, and it
runs again on the next capture.

A post-fault-only bag on a quiet or heavily filtered system can contain zero
messages. It still finalises normally on both ``mcap`` and ``sqlite3``, is listed
by the bulk-data endpoints and can be downloaded; only its payload is empty.

**What ``duration_sec`` on a stored bag means.** The value returned by
``~/get_rosbag`` and the gateway's bulk-data listing is the span the recording was
open, not the configured windows. It is a *recording* span, not a *content* span:
the zero-message bag above still reports its window rather than ``0.0``, because
"the black box covered these seconds and nothing was published" is the useful
statement and a bare zero would be indistinguishable from a broken artifact. A
post-fault-only bag therefore reports roughly ``duration_after_sec``, and a bag
flushed from a buffer that never filled reports the history it holds plus its
window. The span can also *exceed* ``duration_sec + duration_after_sec``, because pruning is
driven by message arrival rather than by a timer: when **every** captured topic goes
quiet nothing prunes, and the buffer keeps its last window until the next
confirmation flushes it. That is deliberate - a black box should keep the final
messages before everything stopped. Note the condition: a single captured topic that
goes quiet while others keep publishing is pruned like anything else, because each
arrival prunes the whole buffer by age.

.. seealso::

   :doc:`/tutorials/snapshots` for detailed snapshot configuration examples.

Audit Log
---------

An append-only, hash-chained record of fault state transitions, for deployments
that have to show afterwards what the fault state was and when it changed. Off
by default: with it off there is no table, no file and no write cost.

.. code-block:: yaml

   fault_manager:
     ros__parameters:
       audit_log:
         enabled: false
         transitions: "all"
         retention_max_records: 0
         fail_closed: false
         database_path: ""

.. list-table::
   :header-rows: 1
   :widths: 34 14 52

   * - Parameter
     - Default
     - Description
   * - ``audit_log.enabled``
     - ``false``
     - Turn the audit log on.
   * - ``audit_log.transitions``
     - ``"all"``
     - Which transitions are recorded: ``all`` (occurred, confirmed, cleared) or
       ``confirmed_only``. Any other value falls back to ``all`` with a warning.
   * - ``audit_log.retention_max_records``
     - ``0``
     - Seal and prune the oldest segment once the log passes this many records.
       ``0`` disables rotation, and a negative value is refused with a warning
       and treated as ``0``.
   * - ``audit_log.fail_closed``
     - ``false``
     - Re-raise a failed audit append as a hard error instead of carrying on.
       Note what this does and does not do: it does NOT roll back the fault
       state change that triggered the transition, because that change has
       already committed to the separate fault-store database and the two
       databases cannot be made atomic. It signals a broken audit chain so an
       operator has to act. Off by default so the audit can never block fault
       processing.
   * - ``audit_log.database_path``
     - ``""``
     - Where the audit database lives. Empty puts it beside the fault database,
       or in memory when the fault store is itself in memory or not SQLite.

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
