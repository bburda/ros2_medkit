Action Status Bridge Configuration
==================================

The ``ros2_medkit_action_status_bridge`` node turns ROS 2 action outcomes into
faults. An action that aborts is a discrete, authoritative failure, and without
this bridge it is visible only to whoever was watching that action's result.

.. contents:: Table of Contents
   :local:
   :depth: 2

Overview
--------

The bridge rescans the graph for action status topics, subscribes to them, and
reports a fault when a goal reaches a terminal state it is configured to treat
as a failure. Terminal action states do not flap - the state model only moves on
a net change - so the reports are not debounced the way log messages are.

If the FaultManager service is not discovered yet, a report is held and retried
on a short timer, so the freeze-frame snapshot the FaultManager takes lands as
close to the event as discovery allows.

Parameters
----------

.. code-block:: yaml

   action_status_bridge:
     ros__parameters:
       aborted_severity: 2         # SEVERITY_ERROR
       canceled_is_fault: false
       heal_on_succeeded: true
       rescan_period_sec: 2.0
       retry_period_sec: 0.05
       code_prefix: "ACTION"
       exclude_actions: []
       include_only_actions: []
       dedup_capacity: 4096

.. list-table::
   :header-rows: 1
   :widths: 32 12 56

   * - Parameter
     - Default
     - Description
   * - ``aborted_severity``
     - ``2`` (ERROR)
     - Severity reported when a goal aborts: 0 INFO, 1 WARN, 2 ERROR,
       3 CRITICAL. Out of range is refused with a warning naming the value, and
       ERROR is used.
   * - ``canceled_is_fault``
     - ``false``
     - Report a fault when a goal is canceled. Off by default, because a cancel
       is usually a decision rather than a failure.
   * - ``heal_on_succeeded``
     - ``true``
     - Report a healing event when a goal succeeds, so an earlier fault for the
       same action clears on its own.
   * - ``rescan_period_sec``
     - ``2.0``
     - How often the graph is rescanned for action status topics. A
       non-positive value is refused with a warning and ``2.0`` is used.
   * - ``retry_period_sec``
     - ``0.05``
     - Retry cadence for a report deferred because the FaultManager service was
       not discovered. The timer is armed only while a delivery is pending. Kept
       short so the freeze-frame lands near the event. A non-positive value is
       refused with a warning and ``0.05`` is used.
   * - ``code_prefix``
     - ``ACTION``
     - Prefix of every generated fault code. Normalized to upper snake case so
       it cannot produce a code outside medkit's ``[A-Z0-9_]`` charset; an empty
       result falls back to ``ACTION``.
   * - ``exclude_actions``
     - ``[]``
     - Action names the bridge ignores.
   * - ``include_only_actions``
     - ``[]``
     - When non-empty, only these actions are watched.
   * - ``dedup_capacity``
     - ``4096``
     - How many goal IDs are remembered to avoid reporting the same terminal
       outcome twice. A non-positive value, or one past what the counter can
       hold, is refused with a warning naming the value, and ``4096`` is used.

The bridge also declares ``fault_reporter.local_filtering.enabled`` (default
``false``) on its own node. See :doc:`fault-manager` for what the reporter-side
filter does.

See Also
--------

- :doc:`fault-manager` - where the reported faults land
- :doc:`log-bridge` - the same idea for ``/rosout``
