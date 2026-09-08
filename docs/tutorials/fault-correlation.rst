Configuring Fault Correlation
=============================

This tutorial shows how to configure fault correlation to automatically
identify root causes and reduce fault noise in complex systems.

.. contents:: Table of Contents
   :local:
   :depth: 2

Overview
--------

When multiple faults occur in rapid succession, they often share a common
root cause. Fault correlation helps by:

- **Hierarchical mode**: Identifying root cause → symptom relationships.
  Symptoms are muted while the root cause is displayed.
- **Auto-cluster mode**: Grouping similar faults that occur within a time
  window into a single cluster.

This is similar to:

- **AUTOSAR DEM event combination** - grouping related diagnostic events
- **AIOps alert correlation** - reducing alert storms in monitoring systems

Benefits:

- Reduced fault noise for operators
- Faster root cause identification
- Cleaner fault lists in dashboards

How It Works
~~~~~~~~~~~~

**Hierarchical Mode Flow:**

.. uml::

   @startuml
   skinparam backgroundColor #FEFEFE
   skinparam sequenceMessageAlign center

   participant "Fault Source" as src
   participant "FaultManager" as fm
   participant "CorrelationEngine" as ce
   database "Fault Storage" as db

   == Root Cause Detected ==
   src -> fm: ReportFault(ESTOP_001, CRITICAL)
   fm -> ce: process_fault("ESTOP_001")
   ce -> ce: Register as root cause\n(rule: estop_cascade, window: 2000ms)
   ce --> fm: is_root_cause=true
   fm -> db: Store fault
   fm --> src: accepted

   == Symptoms Arrive (within window) ==
   src -> fm: ReportFault(MOTOR_COMM_FL, ERROR)
   fm -> ce: process_fault("MOTOR_COMM_FL")
   ce -> ce: Matches MOTOR_* pattern\nWithin 2000ms window
   ce --> fm: should_mute=true, root_cause="ESTOP_001"
   fm -> db: Store fault (muted)
   note right: Fault stored but\nnot published to SSE

   src -> fm: ReportFault(DRIVE_FAULT, ERROR)
   fm -> ce: process_fault("DRIVE_FAULT")
   ce --> fm: should_mute=true
   fm -> db: Store fault (muted)

   == Query Faults ==
   src -> fm: ListFaults()
   fm --> src: [ESTOP_001], muted_count=2

   == Clear Root Cause ==
   src -> fm: ClearFault(ESTOP_001)
   fm -> ce: process_clear("ESTOP_001")
   ce --> fm: auto_cleared=[MOTOR_COMM_FL, DRIVE_FAULT]
   fm -> db: Clear all 3 faults
   @enduml

**Auto-Cluster Mode Flow:**

.. uml::

   @startuml
   skinparam backgroundColor #FEFEFE

   participant "Fault Source" as src
   participant "FaultManager" as fm
   participant "CorrelationEngine" as ce

   note over ce: min_count=3, window=500ms\nrepresentative=highest_severity

   == Faults Accumulate ==
   src -> fm: ReportFault(SENSOR_001, ERROR)
   fm -> ce: process_fault("SENSOR_001", "ERROR")
   ce -> ce: Start pending cluster\n[SENSOR_001]
   ce --> fm: cluster_id="sensor_storm_1"
   note right: Not yet active\n(count=1 < min_count=3)

   src -> fm: ReportFault(SENSOR_002, WARN)
   fm -> ce: process_fault("SENSOR_002", "WARN")
   ce -> ce: Add to pending\n[SENSOR_001, SENSOR_002]
   ce --> fm: cluster_id="sensor_storm_1"

   == Cluster Activates ==
   src -> fm: ReportFault(SENSOR_003, CRITICAL)
   fm -> ce: process_fault("SENSOR_003", "CRITICAL")
   ce -> ce: count=3 >= min_count\n**CLUSTER ACTIVE**
   ce -> ce: Select representative:\nSENSOR_003 (highest severity)
   ce --> fm: should_mute=false (is representative)\nretroactive_mute=[SENSOR_001, SENSOR_002]
   note right #LightGreen: SENSOR_003 shown\nothers muted

   src -> fm: ReportFault(SENSOR_004, ERROR)
   fm -> ce: process_fault("SENSOR_004", "ERROR")
   ce --> fm: should_mute=true (not representative)

   == Query Result ==
   src -> fm: ListFaults(include_clusters=true)
   fm --> src: [SENSOR_003]\nclusters: [{id: "sensor_storm_1",\n  fault_codes: [001,002,003,004],\n  representative: "SENSOR_003"}]
   @enduml

Quick Start
-----------

1. **Create a correlation configuration file:**

   .. code-block:: yaml

      # correlation.yaml
      correlation:
        enabled: true
        default_window_ms: 500

        patterns:
          motor_errors:
            codes: ["MOTOR_*"]
          sensor_errors:
            codes: ["SENSOR_*"]

        rules:
          - id: estop_cascade
            name: "E-Stop Cascade"
            mode: hierarchical
            root_cause:
              codes: ["ESTOP_001"]
            symptoms:
              - pattern: motor_errors
            window_ms: 2000
            mute_symptoms: true
            auto_clear_with_root: true

2. **Start the fault manager with correlation enabled:**

   .. code-block:: bash

      ros2 run ros2_medkit_fault_manager fault_manager_node --ros-args \
        -p correlation.config_file:=/path/to/correlation.yaml

3. **Query faults with correlation data:**

   .. code-block:: bash

      curl "http://localhost:8080/api/v1/faults?include_muted=true"

Configuration Reference
-----------------------

Top-Level Settings
~~~~~~~~~~~~~~~~~~

.. list-table::
   :widths: 25 15 60
   :header-rows: 1

   * - Parameter
     - Default
     - Description
   * - ``enabled``
     - ``false``
     - Enable/disable fault correlation
   * - ``default_window_ms``
     - ``500``
     - Default time window for correlation rules (ms)

Patterns Section
~~~~~~~~~~~~~~~~

Patterns define reusable groups of fault codes with wildcard support:

.. code-block:: yaml

   patterns:
     motor_errors:
       codes: ["MOTOR_COMM_*", "MOTOR_TIMEOUT_*"]
     drive_faults:
       codes: ["DRIVE_*", "INVERTER_*"]
     sensor_errors:
       codes: ["SENSOR_*"]

**Wildcard syntax:**

- ``*`` matches any sequence of characters
- ``MOTOR_*`` matches ``MOTOR_001``, ``MOTOR_OVERHEAT``, etc.
- Exact codes (without ``*``) use fast string comparison

Hierarchical Mode
-----------------

Hierarchical mode identifies root cause → symptom relationships.

.. code-block:: yaml

   rules:
     - id: estop_cascade
       name: "E-Stop Cascade"
       mode: hierarchical
       root_cause:
         codes: ["ESTOP_001", "ESTOP_002"]
       symptoms:
         - pattern: motor_errors
         - pattern: drive_faults
       window_ms: 2000
       mute_symptoms: true
       auto_clear_with_root: true

.. list-table::
   :widths: 25 15 60
   :header-rows: 1

   * - Parameter
     - Default
     - Description
   * - ``root_cause.codes``
     - (required)
     - Fault codes that trigger this rule
   * - ``symptoms``
     - (required)
     - List of pattern references for symptom faults
   * - ``window_ms``
     - ``default_window_ms``
     - Time window to look for symptoms after root cause
   * - ``mute_symptoms``
     - ``true``
     - Hide symptom faults from normal queries
   * - ``auto_clear_with_root``
     - ``true``
     - Auto-clear symptoms when root cause is cleared

**How it works:**

1. When ``ESTOP_001`` is reported, it's marked as a root cause
2. Any ``MOTOR_*`` or ``DRIVE_*`` faults within 2000ms are marked as symptoms
3. Symptoms are muted (not shown in default fault list)
4. Clearing ``ESTOP_001`` auto-clears all its symptoms

Auto-Cluster Mode
-----------------

Auto-cluster mode groups similar faults into clusters.

.. code-block:: yaml

   rules:
     - id: sensor_storm
       name: "Sensor Storm"
       mode: auto_cluster
       match:
         - pattern: sensor_errors
       min_count: 3
       window_ms: 2000
       show_as_single: true
       representative: highest_severity

.. list-table::
   :widths: 25 15 60
   :header-rows: 1

   * - Parameter
     - Default
     - Description
   * - ``match``
     - (required)
     - List of pattern references for faults to cluster
   * - ``min_count``
     - ``3``
     - Minimum faults needed to form a cluster
   * - ``window_ms``
     - ``default_window_ms``
     - Time window for clustering faults
   * - ``show_as_single``
     - ``true``
     - Show cluster as single fault (mute non-representatives)
   * - ``representative``
     - ``highest_severity``
     - How to select the cluster representative:
       ``first``, ``most_recent``, or ``highest_severity``

**How it works:**

1. First ``SENSOR_*`` fault starts a potential cluster
2. Additional ``SENSOR_*`` faults within 2000ms join the cluster
3. When 3+ faults accumulate, cluster becomes active
4. Only the representative fault is shown; others are muted

Querying Correlation Data
-------------------------

**Basic query (includes counts):**

.. code-block:: bash

   curl -H "Authorization: Bearer $TOKEN" http://localhost:8080/api/v1/faults

Response always includes:

.. code-block:: javascript

   {
     "faults": [...],  // fault objects
     "count": 5,
     "muted_count": 12,
     "cluster_count": 2
   }

**With muted fault details:**

.. code-block:: bash

   curl "http://localhost:8080/api/v1/faults?include_muted=true"

.. code-block:: javascript

   {
     "faults": [...],  // fault objects
     "muted_count": 12,
     "muted_faults": [
       {
         "fault_code": "MOTOR_COMM_001",
         "root_cause_code": "ESTOP_001",
         "rule_id": "estop_cascade",
         "delay_ms": 150
       }
     ]
   }

**With cluster details:**

.. code-block:: bash

   curl "http://localhost:8080/api/v1/faults?include_clusters=true"

.. code-block:: javascript

   {
     "faults": [...],  // fault objects
     "cluster_count": 2,
     "clusters": [
       {
         "cluster_id": "sensor_storm_1",
         "rule_id": "sensor_storm",
         "rule_name": "Sensor Storm",
         "representative_code": "SENSOR_CRITICAL_001",
         "representative_severity": "CRITICAL",
         "fault_codes": ["SENSOR_001", "SENSOR_002", "SENSOR_CRITICAL_001"],
         "first_at": "2026-01-19T10:00:00Z",
         "last_at": "2026-01-19T10:00:01Z"
       }
     ]
   }

**Auto-clear on root cause resolution:**

.. code-block:: bash

   curl -X DELETE http://localhost:8080/api/v1/faults/ESTOP_001

.. code-block:: json

   {
     "success": true,
     "fault_code": "ESTOP_001",
     "auto_cleared_codes": ["MOTOR_COMM_001", "MOTOR_TIMEOUT_002", "DRIVE_001"]
   }

.. note::

   **Per-entity DELETE opts out of the cascade.** The same fault cleared
   via ``DELETE /api/v1/{entity-path}/faults/ESTOP_001`` clears only
   ``ESTOP_001`` itself - ``auto_cleared_codes`` will be empty in the
   response. The gateway sets ``ClearFault.srv``'s
   ``skip_correlation_auto_clear`` to ``true`` on per-entity routes so
   that an operator with access to one entity cannot cascade-clear
   correlated symptom faults reported by apps in other entities. Use
   the global ``DELETE /api/v1/faults/{fault_code}`` route when you do
   want the correlation cascade. Direct ``ros2 service call`` clients
   can choose explicitly by setting ``skip_correlation_auto_clear`` in
   the request body (see ``ros2_medkit_msgs`` ``ClearFault.srv``).

Example: Complete Configuration
-------------------------------

.. code-block:: yaml

   # /etc/ros2_medkit/correlation.yaml
   correlation:
     enabled: true
     default_window_ms: 500

     patterns:
       motor_errors:
         codes: ["MOTOR_COMM_*", "MOTOR_TIMEOUT_*", "MOTOR_OVERHEAT_*"]
       drive_faults:
         codes: ["DRIVE_*", "INVERTER_*"]
       sensor_errors:
         codes: ["SENSOR_*"]
       battery_warnings:
         codes: ["BATTERY_LOW", "BATTERY_CRITICAL"]

     rules:
       # E-Stop causes motor and drive shutdowns
       - id: estop_cascade
         name: "E-Stop Cascade"
         mode: hierarchical
         root_cause:
           codes: ["ESTOP_001", "ESTOP_002"]
         symptoms:
           - pattern: motor_errors
           - pattern: drive_faults
         window_ms: 2000
         mute_symptoms: true
         auto_clear_with_root: true

       # Battery critical causes low battery warnings
       - id: battery_cascade
         name: "Battery Cascade"
         mode: hierarchical
         root_cause:
           codes: ["BATTERY_CRITICAL"]
         symptoms:
           - pattern: battery_warnings
         window_ms: 1000

       # Group sensor storms
       - id: sensor_storm
         name: "Sensor Storm"
         mode: auto_cluster
         match:
           - pattern: sensor_errors
         min_count: 3
         window_ms: 2000
         show_as_single: true
         representative: highest_severity

Troubleshooting
---------------

**Symptoms not being muted**

- Check that ``mute_symptoms: true`` is set
- Verify the symptom fault code matches a pattern in ``symptoms``
- Ensure the symptom occurs within ``window_ms`` of the root cause
- Check fault manager logs for correlation matches

**Cluster not forming**

- Verify ``min_count`` faults have occurred within ``window_ms``
- Check that fault codes match patterns in ``match``
- Clusters only become "active" after reaching ``min_count``

**Root cause not detected**

- Verify the fault code exactly matches one in ``root_cause.codes``
- Wildcards in ``root_cause.codes`` are supported

**Configuration validation**

The fault manager validates configuration on startup. Check logs for:

.. code-block:: text

   [WARN] Rule 'my_rule' references unknown pattern: missing_pattern
   [ERROR] Hierarchical rule 'my_rule' has no root_cause codes

See Also
--------

- :doc:`snapshots` - Capture topic data when faults are confirmed
- :doc:`../requirements/specs/faults` - Fault API requirements
- `FaultManager README <https://github.com/selfpatch/ros2_medkit/blob/main/src/ros2_medkit_fault_manager/README.md>`_ - Detailed configuration reference
