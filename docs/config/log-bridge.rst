Log Bridge Configuration
========================

The ``ros2_medkit_log_bridge`` node turns ``/rosout`` messages into faults, so a
node that only ever logged a problem still shows up in the fault list without
being changed.

.. contents:: Table of Contents
   :local:
   :depth: 2

Overview
--------

The bridge subscribes to ``/rosout``, keeps the messages at or above a severity
floor, derives a fault code from the logging node and the message, and reports
it through the same ``ReportFault`` path any node would use.

Two things bound the volume. Messages at ``WARN`` pass through each node's
``FaultReporter`` local filter, which debounces on a threshold and a window;
``ERROR`` and ``FATAL`` bypass that filter. On top of that, the bridge itself
holds a per-node cooldown so one node in a loop cannot flood the fault store.

Parameters
----------

.. code-block:: yaml

   log_bridge:
     ros__parameters:
       rosout_topic: "/rosout"
       severity_floor: 30          # WARN
       code_prefix: "LOG"
       exclude_nodes: []
       include_only_nodes: []
       max_tracked_nodes: 512
       report_cooldown_sec: 5.0
       exclude_medkit_stack: true

.. list-table::
   :header-rows: 1
   :widths: 30 12 58

   * - Parameter
     - Default
     - Description
   * - ``rosout_topic``
     - ``/rosout``
     - Topic the bridge subscribes to.
   * - ``severity_floor``
     - ``30`` (WARN)
     - Lowest rcutils log level that becomes a fault: 10 DEBUG, 20 INFO,
       30 WARN, 40 ERROR, 50 FATAL. Raise it to ``40`` on chatty or constrained
       targets to cut volume. Out of range is clamped to ``[0, 50]`` with a
       warning, because the value is narrowed to a byte and would otherwise wrap
       into "let everything through".
   * - ``code_prefix``
     - ``LOG``
     - Prefix of every generated fault code. Normalized to upper snake case and
       truncated to 32 characters, so it cannot produce a code outside the
       ``[A-Z0-9_]`` charset medkit requires. An empty result falls back to
       ``LOG``.
   * - ``exclude_nodes``
     - ``[]``
     - Node names whose logs are ignored.
   * - ``include_only_nodes``
     - ``[]``
     - When non-empty, only these nodes are considered.
   * - ``max_tracked_nodes``
     - ``512``
     - How many distinct nodes the bridge keeps cooldown state for. Bounds
       memory when node names are generated. Clamped to at least ``1``.
   * - ``report_cooldown_sec``
     - ``5.0``
     - Minimum gap between two reports for the same node. A negative value is
       treated as ``0``.
   * - ``exclude_medkit_stack``
     - ``true``
     - Ignore logs from medkit's own nodes, so the diagnostics stack does not
       report on itself.

See Also
--------

- :doc:`fault-manager` - where the reported faults land
- :doc:`diagnostic-bridge` - the same idea for ``/diagnostics``
