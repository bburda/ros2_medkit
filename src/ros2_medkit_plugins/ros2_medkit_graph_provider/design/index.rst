ros2_medkit_graph_provider
===========================

This section contains design documentation for the ros2_medkit_graph_provider plugin.

Overview
--------

The ``ros2_medkit_graph_provider`` package implements a gateway plugin that generates
live dataflow graph documents for SOVD Function entities. It models the ROS 2 topic
graph as a directed graph of nodes (apps) and edges (topic connections), enriched
with frequency, latency, and drop rate metrics from ``/diagnostics``. The graph
document powers the ``x-medkit-graph`` vendor extension endpoint and cyclic
subscription sampler.

Architecture
------------

The following diagram shows the plugin's main components and data flow.

.. plantuml::
   :caption: Graph Provider Plugin Architecture

   @startuml ros2_medkit_graph_provider_architecture

   skinparam linetype ortho
   skinparam classAttributeIconSize 0

   title Graph Provider - Plugin Architecture

   package "ros2_medkit_gateway" {
       interface GatewayPlugin
       interface IntrospectionProvider
       class PluginContext {
           +register_capability()
           +register_sampler()
           +get_entity_snapshot()
       }
   }

   package "ROS 2" {
       class "/diagnostics" as diag_topic <<topic>>
   }

   package "ros2_medkit_graph_provider" {

       class GraphProviderPlugin {
           - topic_metrics_: map<string, TopicMetrics>
           - topic_metrics_order_: deque<string>
           - last_seen_by_app_: map<string, string>
           - config_: ConfigOverrides
           --
           + introspect(input): IntrospectionResult
           + {static} build_graph_document(): json
           --
           - subscribe_to_diagnostics()
           - diagnostics_callback(msg, msg_info)
           - resolve_publisher_source(msg_info): optional<string>
           - build_current_graph(func_id)
           - build_graph_from_entity_cache(func_id)
           - build_state_snapshot(input, timestamp)
           - resolve_config(func_id)
           - load_parameters()
       }

       class TopicMetrics {
           + frequency_hz: optional<double>
           + latency_ms: optional<double>
           + drop_rate_percent: optional<double>
           + expected_frequency_hz: optional<double>
           + source: optional<string>
           + last_update_ns: int64
       }

       class GraphBuildState {
           + topic_metrics: map
           + last_seen_by_app: map
           + now_ns: int64
       }

       class GraphBuildConfig {
           + expected_frequency_hz_default: 30.0
           + degraded_frequency_ratio: 0.5
           + drop_rate_percent_threshold: 5.0
           + freshness_headroom_factor: 3.0
           + freshness_floor_sec: 5.0
           + stale_grace_sec: 2.0
           + multi_publisher_rate: annotate
       }
   }

   GraphProviderPlugin .up.|> GatewayPlugin
   GraphProviderPlugin .up.|> IntrospectionProvider
   GraphProviderPlugin --> PluginContext
   GraphProviderPlugin --> diag_topic : subscribes
   GraphProviderPlugin *--> TopicMetrics : caches
   GraphProviderPlugin ..> GraphBuildState : builds
   GraphProviderPlugin ..> GraphBuildConfig : configured by

   @enduml

Graph Document Schema
---------------------

The plugin generates a JSON document per Function entity with the following structure:

- **schema_version**: ``"2.0.0"`` - a real, semver contract on the document's shape and
  field semantics. A minor bump is additive/backward-compatible (new optional field, new
  enum value a tolerant client can ignore); a major bump means the shape or the meaning of
  an existing field changed and old parsing logic may break.
- **graph_id**: ``"{function_id}-graph"``
- **timestamp**: ISO 8601 millisecond-precision timestamp
- **scope**: Function entity that owns the graph
- **pipeline_status**: ``"healthy"``, ``"degraded"``, or ``"broken"``
- **bottleneck_edge**: Edge ID with the lowest frequency ratio (only when degraded)
- **topics**: List of topics with **positional** IDs (``topic-1``, ``topic-2``, ...,
  assigned by enumeration order on every build). They renumber whenever the topic set
  changes and must never be persisted or treated as stable across requests.
- **nodes**: List of app entities with reachability status. A node whose
  ``node_status`` is ``"unreachable"`` also carries ``last_seen``: an ISO 8601
  timestamp of the last introspection cycle that saw the app online, when known
  (omitted for an app that has never been seen online).
- **edges**: Publisher-subscriber connections with per-edge metrics, keyed by
  equally positional ``edge-1``, ``edge-2``, ... IDs

Edge metrics include frequency, latency, drop rate, and a ``metrics_status`` field:

- ``"active"`` - A ``/diagnostics`` sample was merged within the freshness window
- ``"pending"`` - No ``/diagnostics`` sample has ever been merged for this topic
- ``"error"`` - A sample was merged in the past, but the newest one has been older than
  the freshness window continuously for longer than ``stale_grace_sec`` (see
  `Stale-Grace Debounce`_ below) (``error_reason: "metrics_stale"`` - the only
  reachable value)

Pipeline Status Logic
---------------------

The overall pipeline status is determined by aggregating edge states:

1. **broken** - At least one edge has ``metrics_status: "error"`` (``metrics_stale`` -
   a real, currently-broken data flow)
2. **degraded** - At least one edge has frequency below the degraded ratio threshold or
   drop rate above it, or at least one scoped node is ``unreachable``. A dead node
   carries no topics and so contributes no edge, but the Function is not healthy while
   one of its nodes is down.
3. **healthy** - Everything else, including a graph where every edge is still ``pending``
   (a pipeline that was never observed is not evidence of a broken one)

When the status is ``"degraded"``, the ``bottleneck_edge`` field identifies the edge
with the lowest frequency-to-expected ratio, helping operators pinpoint the
constraint in the dataflow pipeline.

Data Sources
------------

Diagnostics Subscription
~~~~~~~~~~~~~~~~~~~~~~~~

The plugin subscribes to ``/diagnostics`` and parses ``DiagnosticStatus`` messages
for topic-level metrics, keyed on ``DiagnosticStatus.name`` (the fully-qualified
topic name). Recognized keys:

- ``frame_rate_msg`` - Mapped to ``frequency_hz``; falls back to ``frame_rate_node``
  when ``frame_rate_msg`` is present but exactly ``0`` (header-less message types
  cannot be rated from a header timestamp)
- ``current_delay_from_realtime_ms`` - Mapped to ``latency_ms``
- ``drop_rate_percent`` / ``drop_rate`` - Mapped to ``drop_rate_percent``
- ``expected_frequency`` - Mapped to ``expected_frequency_hz``; wins over any
  graph-provider-side configuration for that topic when present
- ``total_dropped_frames`` - Recognized (keeps the sample from being skipped as
  irrelevant) but not currently mapped to an output field

Drop-rate degradation only fires against a producer that actually emits
``drop_rate_percent`` or ``drop_rate``. The reference producer,
``greenwave_monitor``, does **not** emit either key - it only reports
``total_dropped_frames`` (recognized, but never converted to a rate) - so the
drop-rate threshold is inert against ``greenwave_monitor`` out of the box; it
only activates against a producer that emits one of the two mapped keys.

Each ``DiagnosticArray`` message resolves to exactly one publisher (via publisher
GID matching against ``get_publishers_info_by_topic("/diagnostics")``), stamped as
``metrics.source`` on every topic that message updates, latest-wins: a later
message whose publisher cannot be resolved clears ``source`` again rather than
retaining the last successfully-attributed name, so ``source`` and the sample's
freshness stamp always describe the same, most recent message. No GID match
leaves ``source`` unresolved for that update - never a fabricated name or a
vendor literal; a pending or unresolved edge simply omits the ``source`` key.

Separately, every edge also carries ``publisher_count`` - the live publisher
count on its DATA topic, queried directly from the ROS graph (independent of
``/diagnostics``, and emitted even while the edge is still ``pending``).
``frequency_hz`` is a topic-level arrival rate summed across every publisher on
the topic name, so when ``publisher_count`` is greater than 1 the edge also
carries ``rate_ambiguous: true`` - a duplicate or leftover publisher can inflate
the summed rate and mask a genuinely slow pipeline as healthy. See
:doc:`/config/graph-provider`'s ``multi_publisher_rate`` setting for the policy
that controls whether ``frequency_hz`` is still shown (and allowed to drive the
degraded verdict) once this ambiguity exists.

A bounded cache (max 512 topics) with FIFO eviction (oldest-inserted topic evicted
first, tracked via an insertion-ordered deque) prevents unbounded memory growth.

Entity Cache
~~~~~~~~~~~~

On every request (HTTP route and cyclic-subscription sampler alike), the plugin
rebuilds the graph fresh from the current entity cache
(``PluginContext::get_entity_snapshot()``) - there is no cross-request graph cache
to go stale. This ensures the response always reflects the latest node and topic
state.

Function Scoping
~~~~~~~~~~~~~~~~

Graph documents are scoped to individual Function entities. The plugin resolves
which apps belong to a function by checking the function's ``hosts`` list against
app IDs and component IDs. Only topics that connect scoped apps appear as edges.
System topics (``/parameter_events``, ``/rosout``, ``/diagnostics``, NITROS topics)
are filtered out.

Design Decisions
----------------

Extracted Plugin
~~~~~~~~~~~~~~~~

The graph provider was extracted from the gateway core into a standalone plugin
package. This follows the same pattern as the linux introspection plugins: the
gateway loads graph_provider as a ``.so`` at runtime, keeping the core gateway
free of diagnostics-specific logic. The plugin can be omitted from deployments
that do not need dataflow visualization.

Static build_graph_document
~~~~~~~~~~~~~~~~~~~~~~~~~~~

The ``build_graph_document()`` method is static and takes all inputs explicitly
(function_id, IntrospectionInput, GraphBuildState, GraphBuildConfig, timestamp).
This makes the graph generation logic fully testable without instantiating the
plugin or its ROS 2 dependencies.

Per-Function Config Overrides
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The plugin supports per-function configuration overrides for thresholds
(expected frequency, degraded ratio, drop rate, staleness grace, multi-publisher
rate policy). This allows operators to set different health baselines for
different subsystems - for example, a camera pipeline at 30 Hz vs. a LiDAR
pipeline at 10 Hz. Overrides can only be delivered via a params YAML loaded
with ``config_file`` - see :doc:`/config/graph-provider` for why the ROS 2 CLI
``-p`` path does not work for a hyphenated function id, and why
``gateway.launch.py`` does not forward arbitrary plugin parameters.

Monotonic Freshness Clock
~~~~~~~~~~~~~~~~~~~~~~~~~

``TopicMetrics::last_update_ns`` and ``GraphBuildState::now_ns`` are both stamped
from ``std::chrono::steady_clock``, never ``system_clock``. Freshness is computed
as a subtraction of the two (``now_ns - last_update_ns``); a backward wall-clock
step (an NTP correction, common on a device booting before time sync) would make
that subtraction go negative against a non-monotonic clock and could read a
genuinely dead topic as fresh forever. The document's human-readable ``timestamp``
field is the one place that still uses ``system_clock`` - it is never diffed
against anything.

Stale-Grace Debounce
~~~~~~~~~~~~~~~~~~~~~

An edge does not flip to ``error``/``metrics_stale`` the instant its age crosses
the freshness window; it must stay outside that window continuously for more
than ``stale_grace_sec`` (default ``2.0``, overridable per Function). This is a
stateless function of ``(now_ns, last_update_ns, window_sec, grace_sec)`` with no
separate onset/tick state to maintain, so it absorbs a single late
DDS/executor-jitter ``/diagnostics`` sample without ``pipeline_status`` flapping
to ``"broken"`` and back on the very next on-time sample. ``stale_grace_sec: 0.0``
reproduces the original, un-debounced point-in-time behavior exactly. See
:doc:`/config/graph-provider` for the full formula.

Multi-Publisher Rate Defense
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

``frequency_hz`` is derived from ``frame_rate_msg``/``frame_rate_node``, both a
subscriber-side arrival rate summed across every publisher on the topic name -
the plugin cannot attribute an incoming ``/diagnostics`` sample to a single
publisher. A duplicate or leftover publisher on the same topic can therefore
inflate the summed rate enough to mask a stalled primary publisher as healthy.
The plugin queries the live ROS graph for each DATA topic's publisher count
(independent of ``/diagnostics``) and emits it as ``publisher_count`` on every
edge on that topic, flagging ``rate_ambiguous: true`` whenever it is greater than
one. The ``multi_publisher_rate`` setting (default ``"annotate"``) controls whether
``frequency_hz`` keeps driving the degraded verdict once that ambiguity exists,
or is suppressed for safety-critical deployments that would rather show no rate
than an untrustworthy one. Suppression removes the misleading number, not the
health verdict - ``rate_ambiguous`` already flags the ambiguity. See
:doc:`/config/graph-provider` for the full policy reference.

Cyclic Subscription Sampler
~~~~~~~~~~~~~~~~~~~~~~~~~~~

The plugin registers a sampler via ``PluginContext::register_sampler()`` for the
``x-medkit-graph`` resource. This allows clients to create cyclic subscriptions
that receive periodic graph snapshots over SSE, enabling live dashboard updates
without polling the HTTP endpoint.
