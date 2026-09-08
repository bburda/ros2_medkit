Graph Provider Plugin
======================

The ``ros2_medkit_graph_provider`` plugin serves ``x-medkit-graph``: a per-Function
dataflow graph of ROS 2 apps (nodes) and topic connections (edges), enriched with
frequency, latency, and drop-rate metrics. It answers the question an entity tree
alone cannot: *is data actually flowing through this pipeline, and if not, where is
it stuck?*

.. contents:: Table of Contents
   :local:
   :depth: 2

Overview
--------

Each ``x-medkit-graph`` document is scoped to a single SOVD Function. It models:

- **nodes** - the Function's hosted Apps, each with a ``node_status``
  (``reachable`` / ``unreachable``).
- **edges** - publisher/subscriber topic connections between those Apps, each
  carrying ``frequency_hz``, ``latency_ms``, ``drop_rate_percent``, and a
  ``metrics_status``.
- **pipeline_status** - the Function's aggregate health, rolled up from every edge
  and from node reachability.

The metrics themselves are **not** measured by the plugin directly (it does not
subscribe to your data topics). They come entirely from a ``/diagnostics``
producer running elsewhere in your system. Getting that producer right is the
single most important prerequisite - read the next section before doing
anything else.

Prerequisites
-------------

The ``/diagnostics`` producer contract
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The plugin subscribes to ``/diagnostics`` (``diagnostic_msgs/msg/DiagnosticArray``)
and keys its per-topic metrics map on ``DiagnosticStatus.name``. **That name must be
the exact, fully-qualified ROS topic name** (leading ``/``, no aliasing, no
shortened form) - byte-for-byte the same string ``ros2 topic list`` would show for
the topic the graph provider should attach the sample to.

Recognized ``DiagnosticStatus.values`` keys:

.. list-table::
   :header-rows: 1
   :widths: 30 70

   * - Key
     - Mapped to
   * - ``frame_rate_msg``
     - ``frequency_hz`` (primary source; see the header-stamp note below)
   * - ``frame_rate_node``
     - ``frequency_hz`` fallback, used only when ``frame_rate_msg`` is present
       but exactly ``0``
   * - ``current_delay_from_realtime_ms``
     - ``latency_ms``
   * - ``expected_frequency``
     - The topic's expected frequency for this sample. When present, it wins
       over any graph-provider-side configuration (global default or
       per-function override) for that topic.
   * - ``drop_rate_percent`` / ``drop_rate``
     - ``drop_rate_percent``
   * - ``total_dropped_frames``
     - Recognized (keeps the sample from being treated as irrelevant) but not
       currently mapped to any output field.

A negative ``frame_rate_msg`` or ``frame_rate_node`` (a "not measured yet"
sentinel some producers emit) is rejected outright and never used.

.. note::

   **Drop-rate degradation is not automatic.** It only fires against a producer
   that actually emits ``drop_rate_percent`` or ``drop_rate``. The reference
   producer, ``greenwave_monitor`` (see below), does **not** emit either key -
   it only reports ``total_dropped_frames``, which is recognized (it keeps the
   sample from being skipped as irrelevant) but never converted to a rate. Run
   ``greenwave_monitor`` and the drop-rate threshold in
   :doc:`/config/graph-provider` will simply never trip, however unhealthy the
   real drop rate is - it becomes active only against a producer that emits one
   of the two mapped keys.

Producer vocabulary is not automatic
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The recognized keys and the ``DiagnosticStatus.name`` == topic-name rule above
are this plugin's entire contract - a producer must speak both, or it does not
work. In particular, **a stock ``diagnostic_updater``-based producer will not
work without adapting it.** ``diagnostic_updater`` emits its own key vocabulary
(e.g. ``"Actual frequency (Hz)"``, not ``frame_rate_msg``) and names
``DiagnosticStatus.name`` after the diagnostic task (e.g. ``"topic_monitor:
/powertrain/engine/temperature"`` or similar), not the bare, fully-qualified
topic name the graph provider matches against. Neither the keys nor the name
match what this plugin looks for, so every edge on that topic stays
``"pending"`` forever - silently, with no error anywhere. To use
``diagnostic_updater``, either write a custom ``DiagnosticTask`` that emits the
recognized key vocabulary above and sets ``.name`` to the plain topic name, or
run a separate producer that already speaks it (like ``greenwave_monitor``,
next).

The header-stamp requirement
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

``frame_rate_msg`` is usually computed from a message's header timestamp. Message
types with **no header field** cannot be rated that way, so a compliant producer
reports ``frame_rate_msg: 0`` for them and puts the real measured rate in
``frame_rate_node`` instead. The graph provider falls back to ``frame_rate_node``
only in that exact case (``frame_rate_msg`` present and exactly ``0``) - never for
a missing ``frame_rate_msg``, and never overriding a positive ``frame_rate_msg``.

The failure mode, stated plainly
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

**No ``/diagnostics`` producer publishing for a topic means every edge on that
topic reports ``metrics_status: "pending"`` forever.** It never transitions to
``"active"`` or ``"error"`` on its own - there is no timeout that turns "no data
was ever seen" into an error state, by design (an edge that was never observed is
not the same failure as one that stopped being observed). If you bring up the
graph provider and see every edge stuck on ``pending``, this is almost always the
cause - see :ref:`graph-provider-pending-troubleshooting` below.

Reference producer: ``greenwave_monitor``
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

``greenwave_monitor`` (package and executable both named ``greenwave_monitor``) is
the reference ``/diagnostics`` producer used by this project's own integration
tests. It resolves its monitored-topic list **once, at startup**, and never
re-resolves it - a topic with no live publisher yet when it starts is silently
skipped for the rest of its lifetime. Always start it strictly after the
publishers of every topic it should monitor are already advertised in the ROS
graph.

.. code-block:: bash

   ros2 run greenwave_monitor greenwave_monitor --ros-args \
     -p gw_monitored_topics:="['/powertrain/engine/temperature']" \
     -p gw_time_check_preset:="header_with_nodetime_fallback"

Parameter shape:

- ``gw_monitored_topics`` (list of strings) - fully-qualified topic names to
  monitor with no per-topic expected-frequency tracking.
- ``gw_frequency_monitored_topics`` (dict) - ``topic -> {expected_frequency,
  tolerance}``. When set for a topic, the ``expected_frequency`` stamped into
  ``/diagnostics`` wins over any graph-provider-side configuration for that
  topic (see :doc:`/config/graph-provider`).
- ``gw_time_check_preset`` (string) - e.g. ``"header_with_nodetime_fallback"``,
  matching greenwave's own example configuration.

``greenwave_monitor`` also publishes ``/diagnostics`` at only ~1 Hz, with the
first useful (non-transient) reading typically arriving a few seconds after
startup - budget your polling accordingly.

Turning the plugin on
----------------------

The graph provider is a standalone ``GatewayPlugin`` - see :doc:`plugin-system`
for the general plugin loading mechanism. ``gateway.launch.py`` auto-detects
``ros2_medkit_graph_provider`` at launch time: if the package is installed
alongside the gateway, the plugin is loaded automatically with no configuration
required. To load it manually (a custom launch setup, or a non-default install
layout):

.. code-block:: yaml

   ros2_medkit_gateway:
     ros__parameters:
       plugins: ["graph_provider"]
       plugins.graph_provider.path: "/path/to/lib/ros2_medkit_graph_provider/libros2_medkit_graph_provider_plugin.so"

See :doc:`/config/graph-provider` for the full set of tunable thresholds.

Functions are populated by manifest **and** by runtime discovery - namespaces
turn into Functions automatically (``create_functions_from_namespaces``,
enabled by default). The ``x-medkit-graph`` endpoint is not manifest-only: any
Function, however it was discovered, gets the capability.

Aligning the expected frequency for this walkthrough
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The worked example below publishes at 2 Hz, but the global
``expected_frequency_hz_default`` is 30 Hz (see :doc:`/config/graph-provider`).
Without an override, ``2 / 30 = 0.067`` falls below the default
``degraded_frequency_ratio`` of ``0.5``, and the graph would read
``"degraded"``, not ``"healthy"``. To get the ``"healthy"`` result shown below,
add a per-function override for the ``engine-monitoring`` Function to a params
YAML file:

.. code-block:: yaml

   ros2_medkit_gateway:
     ros__parameters:
       plugins.graph_provider.function_overrides.engine-monitoring.expected_frequency_hz: 2.0

and load it via ``config_file`` - **not** a bare ``ros2 launch ... key:=value``
(silently dropped) or a CLI ``-p`` flag (fails to parse: ``engine-monitoring``
has a hyphen, which the per-function override mechanism does not accept from
the ROS 2 CLI - see :doc:`/config/graph-provider`'s note on override delivery):

.. code-block:: bash

   ros2 launch ros2_medkit_gateway gateway.launch.py \
     config_file:=/path/to/engine-monitoring-overrides.yaml

The Discovery Path
-------------------

1. **List functions** to find the one you care about:

   .. code-block:: bash

      curl -H "Authorization: Bearer $TOKEN" http://localhost:8080/api/v1/functions | jq

2. **Read the Function's detail** and follow its capability href. A Function
   detail response carries an ``"x-medkit-graph"`` link exactly while this
   plugin is loaded - the key is absent on a gateway running without it,
   because nothing would answer at that URI:

   .. code-block:: bash

      curl -H "Authorization: Bearer $TOKEN" http://localhost:8080/api/v1/functions/engine-monitoring | jq '."x-medkit-graph"'
      # "/api/v1/functions/engine-monitoring/x-medkit-graph"

3. **GET the graph** at that href.

A Worked Example
-----------------

The example below matches this project's own end-to-end test topology: a
Function ``engine-monitoring`` hosting an ``engine-temp-sensor`` App (publishes
``/powertrain/engine/temperature`` at 2 Hz) and an ``engine-temp-monitor`` App
(subscribes to it), with a real ``greenwave_monitor`` watching that topic.

.. code-block:: bash

   curl -H "Authorization: Bearer $TOKEN" http://localhost:8080/api/v1/functions/engine-monitoring/x-medkit-graph | jq

.. code-block:: json

   {
     "x-medkit-graph": {
       "schema_version": "2.0.0",
       "graph_id": "engine-monitoring-graph",
       "timestamp": "2026-07-20T14:03:11.204Z",
       "scope": {
         "type": "function",
         "entity_id": "engine-monitoring"
       },
       "pipeline_status": "healthy",
       "bottleneck_edge": null,
       "topics": [
         {
           "topic_id": "topic-1",
           "name": "/powertrain/engine/temperature"
         }
       ],
       "nodes": [
         {
           "entity_id": "engine-temp-sensor",
           "node_status": "reachable"
         },
         {
           "entity_id": "engine-temp-monitor",
           "node_status": "reachable"
         }
       ],
       "edges": [
         {
           "edge_id": "edge-1",
           "source": "engine-temp-sensor",
           "target": "engine-temp-monitor",
           "topic_id": "topic-1",
           "transport_type": "unknown",
           "metrics": {
             "source": "/greenwave_monitor",
             "publisher_count": 1,
             "frequency_hz": 2.02,
             "latency_ms": 3.8,
             "drop_rate_percent": 0.0,
             "metrics_status": "active"
           }
         }
       ]
     }
   }

``publisher_count`` reflects a live query of the ROS graph for this edge's
DATA topic (``/powertrain/engine/temperature``), independent of
``/diagnostics``. Here it is ``1`` - a single publisher, so the measured
``frequency_hz`` is trustworthy. If a second publisher ever appeared on that
topic (a duplicate node, a leftover process), this edge would instead carry
``"publisher_count": 2`` and ``"rate_ambiguous": true``, meaning
``frequency_hz`` is now a rate summed across both publishers and could be
inflated. See :doc:`/config/graph-provider`'s ``multi_publisher_rate`` setting
for how to control the response to that ambiguity.

``metrics.source`` is the node that published the ``/diagnostics`` sample this
edge was last updated from, resolved from the live ROS graph. With a single
``/diagnostics`` publisher (the case here) it resolves on every RMW. Telling
several simultaneous ``/diagnostics`` publishers apart per sample needs an RMW
whose message publisher GID matches the graph endpoint GID (``rmw_fastrtps_cpp``);
on an RMW without that (for example ``rmw_cyclonedds_cpp``) a sample from one of
several publishers is left unattributed and ``metrics.source`` is omitted rather
than guessed.

If a second App pair existed in the same Function without a matching
``greenwave_monitor`` entry in ``gw_monitored_topics``, its edge would appear
in the same document with ``"metrics_status": "pending"`` and no
``metrics.source`` key at all - both apps stay ``reachable``, the edge is just
silent. See :ref:`graph-provider-pending-troubleshooting`.

Cyclic Subscription (Live Updates)
-----------------------------------

The plugin registers a cyclic-subscription sampler for the ``x-medkit-graph``
resource, so a client can receive periodic graph snapshots over Server-Sent
Events instead of polling. See :doc:`/api/rest` for the general
cyclic-subscription API; the graph-specific parts are below.

Each tick carries the whole graph document for the function. The resource URI
must therefore end at ``x-medkit-graph`` - appending a path below it is refused
with 400.

Create the subscription:

.. code-block:: bash

   curl -X POST http://localhost:8080/api/v1/functions/engine-monitoring/cyclic-subscriptions \
     -H "Content-Type: application/json" \
     -d '{
       "resource": "/api/v1/functions/engine-monitoring/x-medkit-graph",
       "interval": "normal",
       "duration": 300
     }'

.. code-block:: json

   {
     "id": "sub_001",
     "observed_resource": "/api/v1/functions/engine-monitoring/x-medkit-graph",
     "event_source": "/api/v1/functions/engine-monitoring/cyclic-subscriptions/sub_001/events",
     "protocol": "sse",
     "interval": "normal"
   }

Then stream events from ``event_source``. Each frame's payload is the same
``{"x-medkit-graph": {...}}`` wrapper the plain ``GET`` returns:

.. code-block:: bash

   curl -N http://localhost:8080/api/v1/functions/engine-monitoring/cyclic-subscriptions/sub_001/events

.. code-block:: text

   data: {"timestamp":"2026-07-20T14:03:12.204Z","payload":{"x-medkit-graph":{"schema_version":"2.0.0","graph_id":"engine-monitoring-graph","timestamp":"2026-07-20T14:03:12.204Z","scope":{"type":"function","entity_id":"engine-monitoring"},"pipeline_status":"healthy","bottleneck_edge":null,"topics":[{"topic_id":"topic-1","name":"/powertrain/engine/temperature"}],"nodes":[{"entity_id":"engine-temp-sensor","node_status":"reachable"},{"entity_id":"engine-temp-monitor","node_status":"reachable"}],"edges":[{"edge_id":"edge-1","source":"engine-temp-sensor","target":"engine-temp-monitor","topic_id":"topic-1","transport_type":"unknown","metrics":{"source":"/greenwave_monitor","publisher_count":1,"frequency_hz":2.03,"latency_ms":3.7,"drop_rate_percent":0.0,"metrics_status":"active"}}]}}}

Before the producer has warmed up, the same stream carries the edge as
``"metrics_status": "pending"`` with no ``metrics.source`` key - a client
watching the stream will see that transition to ``"active"`` live, without
reconnecting.

Reading the Output
-------------------

``node_status`` / ``last_seen`` (per node)
   - ``reachable`` - the app is currently present in the ROS graph.
   - ``unreachable`` - the app is not currently present. The node also carries
     ``last_seen``: an ISO 8601 timestamp of the last introspection cycle that
     saw it online, when known (omitted for an app that has never been seen
     online).

``metrics_status`` (per edge)
   - ``pending`` - no ``/diagnostics`` sample has ever been merged for this
     topic. Permanent until real data arrives; see the prerequisites above.
   - ``active`` - a fresh sample was merged within the freshness window
     (see :doc:`/config/graph-provider`). Tracks freshness, not field
     completeness: an edge with a zero-rate warmup sample from the producer
     is still ``active``.
   - ``error`` - a sample was merged at some point, but the newest one has been
     outside the freshness window for longer than ``stale_grace_sec`` (default
     ``2.0`` s - a single late sample does not flip this immediately; see
     :doc:`/config/graph-provider`). Carries ``"error_reason": "metrics_stale"``
     - the only reachable value of that field. This is a real, currently-broken
     data flow, distinct from ``pending`` (never observed).

``metrics.publisher_count`` / ``metrics.rate_ambiguous`` (per edge)
   ``publisher_count`` is the live publisher count on this edge's DATA topic,
   queried directly from the ROS graph - independent of ``/diagnostics``, and
   present even while ``metrics_status`` is still ``pending``. When it is
   greater than ``1``, the edge also carries ``rate_ambiguous: true``:
   ``frequency_hz`` is a topic-level arrival rate summed across every
   publisher, so a duplicate or leftover publisher can inflate it and mask a
   genuinely slow pipeline as healthy. Treat ``frequency_hz`` on such an edge
   as suspect, not authoritative. See :doc:`/config/graph-provider`'s
   ``multi_publisher_rate`` setting for the policy that controls whether
   ``frequency_hz`` is still shown once this ambiguity exists.

``pipeline_status`` (per graph)
   - ``broken`` - at least one edge is ``error``/``metrics_stale``.
   - ``degraded`` - no edge is in error, but at least one measured edge's
     frequency ratio (measured / expected) is below
     ``degraded_frequency_ratio``, or its drop rate exceeds
     ``drop_rate_percent_threshold``, or at least one scoped node is
     ``unreachable``. A dead node carries no topics, so it contributes no
     edge, but the Function is not healthy while one of its nodes is down -
     ``node_status`` names which one.
   - ``healthy`` - everything else, including a graph where every edge is
     still ``pending``. A Function with no ``/diagnostics`` coverage at all
     reads as ``healthy``, not broken - a pipeline that was never observed is
     not evidence of a broken one.

``bottleneck_edge``
   Present only when ``pipeline_status`` is ``degraded``; the ``edge_id`` of
   the edge with the lowest frequency ratio among all edges that have one.

``topic_id`` / ``edge_id``
   **Positional, not stable.** They are assigned by enumeration order
   (``topic-1``, ``topic-2``, ... and ``edge-1``, ``edge-2``, ...) each time
   the document is built, and will renumber whenever the set of topics or
   edges in the Function changes (a node joins or leaves, a topic starts or
   stops being published). Do not persist them or use them as a stable
   reference across requests - identify an edge by its ``(source, target,
   topic_id)`` within a single response instead.

See Also
--------

- :doc:`/config/graph-provider` - Configuration keys, defaults, and per-function overrides
- :doc:`/api/rest` - Full ``x-medkit-graph`` schema reference
- :doc:`plugin-system` - General plugin loading mechanism
- :doc:`/troubleshooting` - :ref:`graph-provider-pending-troubleshooting`
