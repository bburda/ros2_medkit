REST API Reference
==================

The ros2_medkit gateway exposes a RESTful API for interacting with ROS 2 systems.
All endpoints are prefixed with ``/api/v1``.

.. note::

   Entity endpoints (``/components``, ``/apps``) share the same handler implementations.
   The examples use ``/components`` but the same patterns apply to ``/apps``.

.. contents:: Table of Contents
   :local:
   :depth: 2

Client Request Headers
----------------------

Two headers a client may send are read across many endpoints rather than
belonging to one. Both are optional, and both are declared per-operation in the
generated OpenAPI document, so a generated client sees them on exactly the
operations that read them.

``X-Client-Id``
   Identifies the calling client for :doc:`resource locking <locking>`. Read by
   every lock-participating write - those operations also carry
   ``x-medkit-lock-guarded: true`` and declare a ``409``, and
   :ref:`locking-blocked-operations` lists them. While a lock protects an
   entity's collection, only the client holding it may write; every other
   caller, including one that sends no ``X-Client-Id``, is answered ``409``.
   The lock endpoints themselves also read it: ``POST``/``PUT``/``DELETE``
   ``/locks`` require it, and the two ``GET`` routes use it only to fill in the
   ``owned`` field.

   ``DELETE /api/v1/faults`` is the one route that reads it without ever
   answering ``409``; it silently skips faults on entities locked by another
   client and still answers ``204``. Nothing on the response reports the skip -
   ``X-Medkit-Local-Only`` is about aggregated peers, not locks - so re-read the
   entity's faults to see what survived.

   Everything in this entry describes a gateway with ``locking.enabled`` on.
   With it off the header is declared on the ``/locks`` endpoints only, since
   those are the only routes that still read it; see :doc:`locking`.

``X-Medkit-No-Fan-Out``
   Answer from this gateway alone: do not query aggregated peers and do not
   merge their items. Read by the **per-entity** resource-collection list
   endpoints (data, operations, configurations, faults, logs) and by ``GET
   /api/v1/version-info`` - exactly the operations that declare it in the
   OpenAPI document.

   The gateway sets it on its own outbound peer requests, which stops
   bidirectional aggregation from recursing **on the routes that check it**.
   The global ``GET /api/v1/faults`` does not check it, so it neither declares
   the header nor honours it (see :ref:`the fan-out design note
   <aggregation-fan-out>`).

   **Presence-only.** The value is never read, so ``X-Medkit-No-Fan-Out:
   false`` suppresses fan-out exactly like any other value. The OpenAPI schema
   is a string rather than a boolean for that reason. Omit the header to get
   the aggregated answer.

Server Capabilities
-------------------

``GET /api/v1/``
   Get server capabilities and entry points.

   **Example Response:**

   .. code-block:: json

      {
        "name": "ROS 2 Medkit Gateway",
        "version": "0.7.0",
        "api_base": "/api/v1",
        "endpoints": [
          "GET /api/v1/health",
          "GET /api/v1/areas",
          "GET /api/v1/components",
          "GET /api/v1/apps",
          "GET /api/v1/functions",
          "GET /api/v1/faults",
          "..."
        ],
        "capabilities": {
          "discovery": true,
          "data_access": true,
          "operations": true,
          "async_actions": true,
          "configurations": true,
          "faults": true,
          "logs": true,
          "bulk_data": true,
          "cyclic_subscriptions": true,
          "triggers": true,
          "updates": false,
          "authentication": false,
          "tls": false,
          "aggregation": false
        }
      }

   The ``capabilities.aggregation`` flag is ``true`` when the aggregation
   subsystem is enabled on this gateway (i.e. ``aggregation.enabled=true``
   in config, which wires up an ``AggregationManager``). It does NOT
   require peers to be present - a gateway with aggregation enabled but
   zero peers still reports ``true`` and still emits the
   aggregation-only response fields (``peers`` on ``/health``, which may
   be an empty array; ``x-medkit.contributors`` on entities, which will
   contain only ``"local"`` until a peer contributes). Clients can
   feature-detect those fields using this flag instead of probing for
   field presence. ``warnings`` is **not** one of them: it is served on
   every ``/health`` response regardless of this flag.

``GET /api/v1/version-info``
   Get gateway version and status information.

   **Example Response:**

   .. code-block:: json

      {
        "items": [
          {
            "version": "1.0.0",
            "base_uri": "/api/v1",
            "vendor_info": {
              "version": "0.7.0",
              "name": "ros2_medkit"
            }
          }
        ]
      }

``GET /api/v1/health``
   Health check endpoint. Returns HTTP 200 if gateway is operational.

   The body always includes these x-medkit extension fields:

   - ``warnings`` - array of structured operator-actionable warnings,
     empty when there are no active anomalies. Each warning carries
     ``code``, ``message``, ``entity_ids``, ``ros_node_fqns`` and
     ``peer_names``. The three identifier arrays are always present, and a
     code with nothing to say in one of them sends it empty.
     ``entity_ids`` holds addressable SOVD entity ids only - never ROS node
     names, which go in ``ros_node_fqns``. See :doc:`warning_codes` for the
     stable list of codes and what each one fills in.
   - ``warning_schema_version`` - integer contract version for the
     ``warnings`` array. Clients key on this instead of string-matching
     codes. See :doc:`warning_codes` ``Schema versioning``.

   .. note::

      The OpenAPI component for a warning object is named ``HealthWarning``.
      It was ``HealthAggregationWarning`` in earlier releases, renamed when
      warnings stopped being aggregation-specific. The wire shape of the
      object did not change, so ``warning_schema_version`` does not move for
      the rename - but a client regenerated from the OpenAPI document will
      see the generated type change name.

   When aggregation is enabled (``capabilities.aggregation == true`` in
   the root response), the body additionally includes:

   - ``peers`` - array of peer status objects for every configured or
     discovered peer. Each carries ``name``, ``url`` and ``status``
     (``"online"`` or ``"offline"``).

   The ``discovery`` object carries a ``linking`` sub-object describing
   how manifest apps were bound to runtime ROS nodes: ``linked_count``,
   ``orphan_count``, ``binding_conflicts``, ``unmanifested_policy`` (the
   configured ``config.unmanifested_nodes`` value) and, when the linker
   produced any, ``warnings`` (an array of strings). It appears only when
   a linker ran, which means hybrid mode **with the runtime layer
   enabled**: ``runtime_only`` and ``manifest_only`` run no linker, and
   neither does a hybrid gateway configured with
   ``discovery.runtime.enabled: false``.

   The body also always includes two subscription-pool vendor-extension
   sections, populated from atomic reads so ``/health`` never blocks even
   when the sampling pool is under load:

   - ``x-medkit-subscription-executor`` - state of the single-writer
     worker that owns the pool's subscription node. Fields:
     ``worker_alive``, ``degraded``, ``queue_depth``,
     ``queue_max_depth_observed``, ``queue_dropped``, ``tasks_completed``,
     ``tasks_failed``, ``last_task_latency_us``, ``max_task_latency_us``,
     ``current_task_age_ms``, ``watchdog_trips``, ``graph_events_received``.
     External monitors (k8s liveness, Docker HEALTHCHECK, systemd watchdog)
     should page on ``degraded == true``.
   - ``x-medkit-data-provider`` - pool-level counters: ``pool_size``,
     ``pool_cap``, ``pool_hits``, ``pool_misses``, ``evictions_total``,
     ``type_change_events``, ``graph_events_received``,
     ``concurrent_cold_waits``.

   See :doc:`/design/ros2_medkit_gateway/ros2_subscription_architecture`
   for the underlying pool design that produces these counters.

   .. note::

      Security: ``/health`` is currently reachable without
      authentication by default (``auth.enabled`` defaults to
      ``false``), and even with auth enabled the endpoint is readable
      by the ``viewer``, ``operator``, and ``configurator`` roles. The
      ``peers`` array enumerates every configured peer's name and URL,
      which reveals deployment topology. This is by design for
      operator observability in trusted LANs, but on shared-infra or
      multi-tenant installs you should front the endpoint with an
      authenticating reverse proxy or restrict the peer-name field to
      admin-gated callers at the ingress.

   **Example Response.** A hybrid gateway with one declared app, six
   undeclared ROS nodes and ``unmanifested_nodes: error``, captured from a
   running gateway. The three ``x-medkit-*`` statistics objects are omitted
   here for length; ``peers`` appears only when aggregation is enabled.

   .. code-block:: json

      {
        "status": "healthy",
        "timestamp": 1786538411000000000,
        "warning_schema_version": 2,
        "warnings": [
          {
            "code": "unmanifested_nodes",
            "message": "6 running ROS node(s) are not declared in the manifest while unmanifested_nodes is set to 'error' (listed in ros_node_fqns). The gateway keeps serving. Declare them in the manifest, or relax the policy to 'warn' or 'ignore'.",
            "entity_ids": [],
            "ros_node_fqns": [
              "/powertrain/engine/rpm_sensor",
              "/ros2_medkit_gateway",
              "/_param_client_node",
              "/ros2_medkit_gateway_fault_clients",
              "/ros2_medkit_gateway_lifecycle_state_reader",
              "/ros2_medkit_gateway_sub"
            ],
            "peer_names": []
          }
        ],
        "discovery": {
          "mode": "hybrid",
          "strategy": "hybrid",
          "pipeline": {
            "layers": ["manifest", "runtime"],
            "total_entities": 9,
            "enriched_count": 0,
            "conflict_count": 0,
            "conflicts": [],
            "id_collisions": 0,
            "filtered_by_gap_fill": 0
          },
          "linking": {
            "linked_count": 1,
            "orphan_count": 6,
            "binding_conflicts": 0,
            "unmanifested_policy": "error"
          }
        }
      }

   With aggregation enabled the body additionally carries ``peers``, and a
   Component announced by more than one peer adds a second warning:

   .. code-block:: json

      {
        "peers": [
          {"name": "peer_b", "url": "http://peer-b:8080", "status": "online"},
          {"name": "peer_c", "url": "http://peer-c:8080", "status": "online"}
        ],
        "warnings": [
          {
            "code": "leaf_id_collision",
            "message": "Component 'ecu-x' is announced by multiple peers (peer_b, peer_c); routing falls back to last-writer-wins which is non-deterministic. Resolve by renaming the Component on one side or by modelling it as a hierarchical parent (declare a child Component with parentComponentId='ecu-x' on the owning peer).",
            "entity_ids": ["ecu-x"],
            "ros_node_fqns": [],
            "peer_names": ["peer_b", "peer_c"]
          }
        ]
      }

Discovery Endpoints
-------------------

Areas
~~~~~

``GET /api/v1/areas``
   List all areas (logical/physical groupings).

   **Example Response:**

   .. code-block:: json

      {
        "items": [
          {
            "id": "powertrain",
            "name": "Powertrain",
            "href": "/api/v1/areas/powertrain"
          }
        ]
      }

``GET /api/v1/areas/{area_id}``
   Get area capabilities and metadata.

``GET /api/v1/areas/{area_id}/contains``
   List components contained in this area.

``GET /api/v1/areas/{area_id}/components``
   List components in a specific area.

   .. note::

      **ros2_medkit extension:** Areas support resource collections beyond the SOVD spec,
      which only defines them for apps and components. Areas provide ``/data``,
      ``/data-categories``, ``/data-groups``, ``/operations``, ``/configurations``,
      ``/faults``, ``/logs`` (namespace prefix aggregation), read-only ``/bulk-data``,
      and ``/triggers``. See :ref:`sovd-compliance` for details.

Components
~~~~~~~~~~

``GET /api/v1/components``
   List all components with their operations and capabilities.

   **Example Response:**

   .. code-block:: json

      {
        "items": [
          {
            "id": "temp_sensor",
            "name": "temp_sensor",
            "href": "/api/v1/components/temp_sensor"
          }
        ]
      }

``GET /api/v1/components/{component_id}``
   Get component capabilities including available resource collections.

   .. note::

      **ros2_medkit extension:** When a component carries an asset-identity
      nameplate - from the manifest ``identity:`` block (see
      :doc:`/config/manifest-schema`) or a protocol device-info read (e.g. the
      OPC UA BuildInfo/DI nameplate) - both the list and detail responses
      include it under ``x-medkit.identity``. Only populated fields are
      emitted (camelCase), ``extra`` holds vendor-specific keys, and
      ``_provenance`` records which source set each field (keys use the
      snake_case field names; ``extra`` entries are prefixed with
      ``extra.``).

   .. code-block:: json

      {
        "id": "plc_runtime",
        "name": "PLC Runtime",
        "x-medkit": {
          "source": "opcua",
          "identity": {
            "manufacturer": "SelfPatch Devices",
            "model": "SPX-1000",
            "serialNumber": "SN-0001-TEST",
            "hardwareRevision": "HW-A2",
            "softwareVersion": "SW-3.4.5",
            "networkEndpoint": "opc.tcp://plc.local:4840",
            "extra": {
              "buildNumber": "build-4567"
            },
            "_provenance": {
              "manufacturer": "opcua",
              "serial_number": "opcua",
              "extra.buildNumber": "opcua"
            }
          }
        }
      }

``GET /api/v1/components/{component_id}/hosts``
   List apps hosted on this component (SOVD 7.6.2.4).

``GET /api/v1/components/{component_id}/depends-on``
   List component dependencies.

Apps
~~~~

``GET /api/v1/apps``
   List all apps discovered by the gateway.

   The set of apps is populated either from the static manifest (manifest or hybrid mode)
   or via heuristic runtime discovery of ROS 2 nodes (see :doc:`/tutorials/heuristic-apps`).
   This endpoint may return an empty list if no apps are discovered or if app discovery is
   disabled in the gateway configuration.

``GET /api/v1/apps/{app_id}``
   Get capabilities for a single discovered app.

``GET /api/v1/apps/{app_id}/is-located-on``
   Return the parent component that hosts this app.

   The response follows the standard ``items`` wrapper and returns:

   - ``0`` items when the app has no associated host component
   - ``1`` item when the host component is resolved
   - ``1`` item with ``x-medkit.missing=true`` when the app references a host
     component that cannot currently be resolved

   **Example Response:**

   .. code-block:: json

      {
        "items": [
          {
            "id": "temp-sensor-hw",
            "name": "Temperature Sensor",
            "href": "/api/v1/components/temp-sensor-hw"
          }
        ],
        "x-medkit": {
          "total_count": 1
        },
        "_links": {
          "self": "/api/v1/apps/engine-temp-sensor/is-located-on",
          "app": "/api/v1/apps/engine-temp-sensor"
        }
      }

   Unknown apps return ``404 App not found`` with ``parameters.app_id``.

``GET /api/v1/apps/{app_id}/belongs-to``
   Return the area that contains this app via its parent component.

   Per SOVD (ISO 17978-3 §7.6), the corresponding
   ``belongs-to`` URI reference in ``GET /apps/{app_id}`` is only emitted when
   the app has a parent component (i.e. is not standalone). Standalone apps do
   not expose this subresource in HATEOAS and the endpoint will return an empty
   ``items`` collection if called directly.

   The response follows the standard ``items`` wrapper and returns:

   - ``0`` items when the app has no associated host component (standalone app)
   - ``0`` items when the parent component has no assigned area
   - ``1`` item when the area is resolved
   - ``1`` item with ``x-medkit.missing=true`` when the parent component references
     an area that cannot currently be resolved
   - ``1`` item with ``x-medkit.missing=true`` and ``x-medkit.unresolved_component``
     set to the dangling component id when the app references a parent component
     that cannot currently be resolved (manifest broken / component removed)

   **Example Response:**

   .. code-block:: json

      {
        "items": [
          {
            "id": "engine",
            "name": "Engine",
            "href": "/api/v1/areas/engine"
          }
        ],
        "x-medkit": {
          "total_count": 1
        },
        "_links": {
          "self": "/api/v1/apps/engine-temp-sensor/belongs-to",
          "app": "/api/v1/apps/engine-temp-sensor"
        }
      }

   Unknown apps return ``404 App not found`` with ``parameters.app_id``.

Functions
~~~~~~~~~

``GET /api/v1/functions``
   List all functions (requires manifest mode or hybrid mode).

``GET /api/v1/functions/{function_id}``
   Get function capabilities.

``GET /api/v1/functions/{function_id}/hosts``
   List apps that host this function.

``GET /api/v1/functions/{function_id}/x-medkit-graph``
   Get a function-scoped topology snapshot with per-topic metrics and pipeline status.
   Served by the ``ros2_medkit_graph_provider`` plugin. **Requires a real
   ``/diagnostics`` producer** publishing ``DiagnosticStatus`` messages keyed on the
   exact fully-qualified topic name - without one, every edge stays ``"pending"``
   forever. See :doc:`/tutorials/graph-provider` for the full prerequisites and a
   worked walkthrough, and :doc:`/config/graph-provider` for the threshold reference.

``GET /api/v1/x-medkit-watchdog``
   Get the graph watchdog's reliability status: which entities it has observed, which are
   armed (past the bringup-quiesce warmup and therefore eligible to raise), and the cached
   lifecycle state of any managed lifecycle nodes. Served by the
   ``ros2_medkit_graph_watchdog`` plugin. Read-only; it raises nothing and changes nothing.

   The payload carries ``schema_version``, the configured ``warmup_cycles``, a global state,
   and a per-entity map. An entity that is present but not yet armed is normal during
   bringup: the watchdog holds every raise until an entity has been continuously present for
   ``warmup_cycles`` ticks, so a joining node does not read as a failure.

   Use it to answer "why has the watchdog not reported anything yet" without reading logs.

   **Example Response:**

   .. code-block:: json

      {
        "x-medkit-graph": {
          "schema_version": "2.0.0",
          "graph_id": "perception_graph-graph",
          "timestamp": "2026-03-08T12:00:00.000Z",
          "scope": {
            "type": "function",
            "entity_id": "perception_graph"
          },
          "pipeline_status": "degraded",
          "bottleneck_edge": "edge-1",
          "topics": [
            {
              "topic_id": "topic-1",
              "name": "/camera/front/image_raw"
            },
            {
              "topic_id": "topic-2",
              "name": "/camera/front/camera_info"
            }
          ],
          "nodes": [
            {
              "entity_id": "camera_front",
              "node_status": "reachable"
            },
            {
              "entity_id": "detector",
              "node_status": "reachable"
            }
          ],
          "edges": [
            {
              "edge_id": "edge-1",
              "source": "camera_front",
              "target": "detector",
              "topic_id": "topic-1",
              "transport_type": "unknown",
              "metrics": {
                "source": "/greenwave_monitor",
                "publisher_count": 2,
                "rate_ambiguous": true,
                "frequency_hz": 12.5,
                "latency_ms": 4.2,
                "drop_rate_percent": 0.0,
                "metrics_status": "active"
              }
            },
            {
              "edge_id": "edge-2",
              "source": "camera_front",
              "target": "detector",
              "topic_id": "topic-2",
              "transport_type": "unknown",
              "metrics": {
                "publisher_count": 1,
                "frequency_hz": null,
                "latency_ms": null,
                "drop_rate_percent": 0.0,
                "metrics_status": "pending"
              }
            }
          ]
        }
      }

   **Field Notes:**

   - ``schema_version``: semver contract on the document's shape and field semantics.
     A minor bump is additive/backward-compatible (new optional field, new enum value a
     tolerant client can ignore); a major bump means an existing field's shape or
     meaning changed and old parsing logic may break.
   - ``pipeline_status``: overall graph state, one of ``healthy``, ``degraded``, ``broken``.
     A graph where every edge is still ``pending`` reads as ``healthy`` - a pipeline
     never observed is not evidence of a broken one. A scoped node that is
     ``unreachable`` makes the status at least ``degraded``: a dead node carries no
     topics and so contributes no edge, but the Function is not healthy while it is down.
   - ``node_status``: per-node reachability, one of ``reachable``, ``unreachable``
   - ``last_seen``: present only on a node whose ``node_status`` is ``unreachable``;
     an ISO 8601 millisecond-precision timestamp of the last introspection cycle
     that saw the app online, when known (omitted for an app that has never been
     seen online). A node whose ``node_status`` is ``unreachable`` always has an
     empty ``topics.publishes``/``topics.subscribes`` list, so it never appears as
     ``source`` or ``target`` of any edge in the same document:

     .. code-block:: json

        {
          "entity_id": "old_lidar_node",
          "node_status": "unreachable",
          "last_seen": "2026-03-08T11:59:42.017Z"
        }

   - ``topic_id`` / ``edge_id``: **positional, not stable.** Assigned by enumeration
     order on every build (``topic-1``, ``topic-2``, ... / ``edge-1``, ``edge-2``, ...)
     and renumbered whenever the topic/edge set changes. Do not persist them or use
     them as a cross-request reference.
   - ``metrics_status``: per-edge telemetry state, one of:

     - ``pending`` - no ``/diagnostics`` sample has ever been merged for this topic
       (permanent until real data arrives)
     - ``active`` - a sample was merged within the freshness window (tracks freshness,
       not field completeness)
     - ``error`` - a sample was merged in the past, but the newest one has been older
       than the freshness window continuously for longer than ``stale_grace_sec``
       (default ``2.0`` s - a single late sample does not flip this immediately; see
       :doc:`/config/graph-provider`)
   - ``error_reason``: present only when ``metrics_status`` is ``error``; the only
     reachable value is ``metrics_stale``
   - ``metrics.source``: the resolved fully-qualified node name that published the
     ``/diagnostics`` message this edge's metrics were last updated from. **Omitted**
     on ``pending`` edges and on any edge whose most recent sample could not be
     attributed to a specific publisher - never a fabricated name or a hardcoded
     vendor literal. With a single ``/diagnostics`` publisher the sample resolves
     on every RMW. Telling several simultaneous ``/diagnostics`` publishers apart
     per sample needs an RMW whose message publisher GID matches the graph endpoint
     GID (``rmw_fastrtps_cpp``); on an RMW without that (e.g. ``rmw_cyclonedds_cpp``)
     a sample from one of several publishers is left unattributed (omitted) rather
     than guessed.
   - ``metrics.publisher_count``: live publisher count on this edge's DATA topic,
     from the ROS graph (independent of ``/diagnostics``). Emitted whenever that
     graph query resolved - even at ``1``, and even while ``metrics_status`` is
     still ``pending`` - and omitted only when the query never ran or came back
     empty (never a fabricated ``0``).
   - ``metrics.rate_ambiguous``: present (``true``) only when ``publisher_count``
     is greater than ``1``. ``frequency_hz`` is a topic-level arrival rate summed
     across every publisher on the topic, so a duplicate or leftover publisher can
     inflate it and mask a genuinely slow pipeline as healthy - this is the
     operator-facing signal that the rate number is suspect. See
     :doc:`/config/graph-provider`'s ``multi_publisher_rate`` setting for the
     policy controlling whether ``frequency_hz`` is still shown (and allowed to
     drive the degraded verdict) once this is true.
   - ``transport_type``: reserved and currently unpopulated. Always the literal
     ``"unknown"``.

   .. note::

      **ros2_medkit extension:** Functions support resource collections beyond the SOVD spec.
      ``/data`` and ``/operations`` aggregate from hosted apps (per SOVD). Additionally,
      ``/configurations``, ``/faults``, ``/logs`` aggregate from hosts, read-only
      ``/bulk-data`` is available, ``/cyclic-subscriptions`` and ``/triggers`` are
      supported, and the vendor resource ``/x-medkit-graph`` exposes a function-scoped
      graph snapshot. See :ref:`sovd-compliance` for details.

.. _member-qualified-ids:

Item Ids and Their Providers
----------------------------

An entity that draws its items from members - an Area, a Function, or a
Component with hosted apps - lists what its members provide. Each listed item
carries the members that contribute it in ``x-medkit.member_ids``, so a caller
can always see where an item came from.

**Ids stay bare.** An item id is the ROS name the member itself uses: the topic
path for ``/data``, the service or action short name for ``/operations``. This
is the ordinary case and it does not change with aggregation - in runtime
discovery every App hangs off the single host Component, so almost every entity
draws from members.

**Except when the id is ambiguous.** When more than one item in the merged
collection carries the same id, each of those copies is addressed with the
member that owns it::

   <member_id>:<item_id>

The split is on the first colon: an entity id is restricted to alphanumerics,
underscore and hyphen and so never contains one, while an item name can.

Ambiguity is decided on the merged collection, after peer fan-out, because that
is where it becomes visible - two gateways each holding one ``calibrate`` both
consider it unique. In practice:

- ``/operations`` - two members exposing one short name at different ROS paths
  are two items with one id, so both are qualified
  (``primary_calibration:calibrate``, ``peer_calibration:calibrate``).
- ``/data`` - a topic path names one topic however many members publish and
  subscribe to it; those merge into a single item, so the bare path is kept and
  every contributor is named in ``member_ids``. A path is qualified only when
  two gateways each contribute an item under it.

**When one member carries the short name twice.** An operation's wire id is the
last segment of its ROS path, so ``left/calibrate`` and ``right/calibrate`` on
one node are two operations called ``calibrate``. The member half names that
same member for both copies and separates nothing, so those items take the ROS
path, leading slash stripped, as their item half::

   robot/left/calibrate                       # on the App itself
   primary_calibration:robot/left/calibrate   # on an entity that aggregates it

The form is decided per provider: a short name its own provider carries once
keeps that short name, whatever another provider does with the same name. The
split at the first colon is unchanged, because a ROS path carries no colon, and
the path form is the one ``/data`` already uses for a topic.

**A member half from a peer is read through the collision rename.** A peer names
its own leaves as its own tree names them, and an App whose id collided with a
local one is merged here under ``<peer>__<id>`` - so the name the peer sends
names the LOCAL leaf. Items arriving through the peer fan-out are re-attributed
to the id the merge gave their owner, both in ``x-medkit.member_ids`` and in the
member half of the item id, so ``secondary_gateway__shared_sensor:calibrate``
addresses the peer's copy and ``shared_sensor:calibrate`` the local one.

**Availability on a listed item describes its member.** ``x-medkit.available``
is emitted only as ``false``, and only when the gateway that owns the item is
not answering; absence means the item can be served. An entity declared on this
gateway alone can host a member another gateway runs - no peer contributes the
entity, so its collection fan-out never runs - and that says nothing about the
member. The item is listed as usual, and the request for it is dispatched to the
member's own route.

What this means for a request:

- A bare id that names one item works, on every route. Every client that sends
  the ROS short name keeps working, and it is what the generated OpenAPI
  document describes.
- ``POST /{entity}/operations/{id}/executions`` with a bare id that more than
  one member provides is refused with ``400 invalid-request``, naming the
  qualified form and listing the members in ``parameters.member_ids``. Running
  whichever member was walked first without saying which one ran is the defect
  this removes. A short name that ONE member carries at two ROS paths is
  refused the same way, listing those paths in ``parameters.ros2_paths``.
  Either refusal carries ``parameters.operation_ids``: the ids that do address
  what collided, as the collection lists them, so the client sends one back
  rather than deriving it.
- A qualified id is accepted on the single-item routes. A member half that
  names no member of the entity is ``404``, and so is an item half that member
  does not provide - which is what tells an absent item apart from an item that
  exists and currently carries no data. A member half followed by nothing names
  no item and is ``404`` as well.
- ``GET /{entity}/operations/{id}`` and
  ``GET /{entity}/operations/{id}/executions`` refuse exactly what the execution
  refuses, with the same body. Reading an operation under an id that names
  several of them would describe one without saying which, and the same id is a
  ``400`` the moment the caller runs it. The collection never offers such an id,
  so only a stale one arrives, and it leaves with ``parameters.operation_ids``.
  An unambiguous bare id reads and lists exactly as before.

Listing the Executions of an Operation
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

``GET /{entity}/operations/{id}/executions`` resolves ``{id}`` by the rule
above, so a member-qualified id and a ROS-path id both work, and the answer has
three distinct forms:

- an **action** returns the goals it holds, newest first;
- a **service** returns ``200`` with an empty ``items`` array. A service call
  completes inside its own ``POST`` and leaves no execution resource, so its
  collection exists and is permanently empty. Whether an operation can ever have
  executions is read from ``asynchronous_execution`` on the operation itself;
- an id that names **no operation** is ``404 operation-not-found``, and an
  unknown member half is ``404 resource-not-found`` naming that half - so a typo
  is never answered as an operation that simply has not been run.

Goals live on the gateway that sent them, so an id naming a peer-owned member is
dispatched to that member's own route exactly as the ``POST`` was. A goal
started through an aggregating entity is therefore listed through it too.

Where a Member-Qualified Request is Served
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

An aggregating entity holds no resources of its own, and its members can belong
to different gateways. A request naming one member is therefore served by the
gateway that owns **that member**, on the member's own entity route:

.. code-block:: text

   POST /api/v1/functions/vehicle_health/operations/peer_calibration:calibrate/executions

is answered, when ``peer_calibration`` belongs to a peer, by

.. code-block:: text

   POST /api/v1/apps/peer_calibration/operations/calibrate/executions

on that peer, and the peer's response is what the client receives. The ROS
service or topic behind the id only exists on the owner's graph, so no other
gateway can answer. A member this gateway owns is served locally exactly as
before. This applies to ``GET`` and ``PUT`` of a single ``/data`` item, to
``POST`` of an ``/operations`` execution, and to ``GET``, ``PUT`` and
``DELETE`` of a single ``/configurations`` item.

**An id needs no member half to reach its owner.** A ``/data`` item one member
provides keeps its bare id - qualification follows ambiguity, not aggregation -
so the bare topic path is the id the collection hands back, and it is dispatched
by the member the tree records as providing that topic:

.. code-block:: text

   GET /api/v1/functions/vehicle_health/data/chassis%2Fbrakes%2Fpressure

is answered, when every member providing ``/chassis/brakes/pressure`` belongs to
one peer, by

.. code-block:: text

   GET /api/v1/apps/pressure_sensor/data/chassis/brakes/pressure

on that peer. A topic a member this gateway runs provides is sampled here,
unchanged; a topic whose providers are spread across gateways names no single
place, and the local graph answers it as before. ``/operations`` resolves a bare
id the same way - the operation is resolved first, and the member owning its ROS
path is where the execution is sent.

``/configurations`` keeps its own id scheme, ``<app_id>:<param_name>``, and the
member half is the app id. Because nothing on the owning gateway is aggregating,
the parameter is addressed there by its bare name:

.. code-block:: text

   PUT /api/v1/functions/vehicle_health/configurations/peer_calibration:calibration_offset

is answered, when ``peer_calibration`` belongs to a peer, by

.. code-block:: text

   PUT /api/v1/apps/peer_calibration/configurations/calibration_offset

so the value comes from - and the write lands on - the ROS node that actually
declares the parameter. ``GET /{entity}/configurations`` is unaffected: peer
parameters reach that listing through the collection fan-out, and the ids it
offers are the ids the single-item routes accept.

A member half is recognised when the text before the first colon names a member
of the addressed entity. How many ROS nodes this gateway resolves for that
entity does not enter into it, because a member another gateway runs reports no
ROS binding here and so resolves none: an aggregating entity whose members are
all peer-owned resolves nothing at all, and one that resolves a single local
node can still have peer-owned members beside it. Both take
``<app_id>:<param_name>`` exactly as an entity with several local nodes does.

A prefix naming no member is part of the parameter name, which is what keeps a
parameter whose own name contains a colon addressable, and an entity's own id is
never read as a member half of itself.

Reachability is decided before anything is forwarded. A member retained while
its gateway is silent answers ``504 not-responding`` naming the member (see
:ref:`retained-entities`) rather than a ``502`` from a connection that could not
be made.

``X-Medkit-No-Fan-Out`` does not change this. The header bounds the collection
fan-out that merges peer items into a listing; a request naming one member
already names its owner, so it is one hop and is answered by that owner whether
or not the header is present.

Ambiguity is a property of the declared tree, not of who is reachable right
now. A peer's declared operations are held locally, so the same request gets
the same answer whether or not that peer is currently answering, and deciding
it costs no network call. It cannot be changed by anything a client sends.

.. _retained-entities:

Entities of a Silent Peer
~~~~~~~~~~~~~~~~~~~~~~~~~

When a peer stops answering, the entities it **declared in its manifest** are
retained and marked unavailable; the ones it merely discovered from its live
ROS graph disappear, because nothing can observe that graph any more. A
retained entity:

- stays listed and stays addressable, so the tree does not change shape when a
  link drops;
- reports ``x-medkit.available: false`` and ``x-medkit.is_online: false``;
- keeps the operations it last reported. They stay listed on the aggregating
  entity, each marked ``x-medkit.available: false``, and they still count
  towards ambiguity - so an id that two members provide stays qualified and its
  bare form stays refused whether or not either member is answering. A response
  that suppressed fan-out (``X-Medkit-No-Fan-Out``) omits peer-owned items,
  because the peers were never asked, but still qualifies what it does list;
- answers any request addressed to it with ``504`` and the SOVD standard code
  ``not-responding``, naming the member - rather than being forwarded to the
  silent peer and surfacing as a ``502``, or falling through to a local read
  that returns ``200`` with an empty body.

``/health`` is unchanged: the peer itself is listed there with
``status: "offline"``. Availability of an entity and health of a peer are
separate questions and are reported separately.

``x-medkit.available`` is emitted **only when false**, so an absent field means
the entity is reachable. An aggregating gateway reads the field back off its
peers with that same default, which is what carries the fact past one hop: in a
chain ``A <- B <- C``, ``B`` marks ``C``'s declared entities unavailable when
``C`` goes quiet, and ``A`` reports them the same way. An App also carries
``x-medkit.is_online``; a Component has no second signal, so for a Component
this field is the only one.

.. note::

   ``/configurations`` predates this rule and keeps its own: on an entity whose
   parameters come from more than one node, **every** parameter id is
   ``<app_id>:<param_name>``, and a bare id is refused on write. Its items carry
   ``x-medkit.source`` (a single app id), not ``member_ids``. The node count
   decides which ids the listing OFFERS; which ids it ACCEPTS is decided by the
   member set, so the qualified form works on an entity whose members are all
   peer-owned too. See :ref:`configuration-endpoints`.

Data Endpoints
--------------

Read and publish data from ROS 2 topics. Item ids follow
:ref:`member-qualified-ids`.

``GET /api/v1/components/{id}/data``
   Read all topic data from an entity.

   **Example Response:**

   .. code-block:: json

      {
        "items": [
          {
            "name": "temperature",
            "data_id": "powertrain%2Fengine%2Ftemperature",
            "type": "std_msgs/msg/Float64",
            "value": {"data": 85.5},
            "timestamp": "2025-01-15T10:30:00Z"
          }
        ],
        "x-medkit": {
          "entity_id": "temp_sensor",
          "total_count": 1
        }
      }

``GET /api/v1/components/{id}/data/{topic_path}``
   Read specific topic data. Topic path is URL-encoded (``/`` → ``%2F``).

   **Example:**

   .. code-block:: bash

      curl http://localhost:8080/api/v1/components/temp_sensor/data/powertrain%2Fengine%2Ftemperature

``PUT /api/v1/components/{id}/data/{topic_path}``
   Publish to a topic.

   - **Content-Type:** application/json
   - **200:** Message published successfully
   - **400:** Invalid message format
   - **401:** Unauthorized (when auth enabled)

   **Example:**

   .. code-block:: bash

      curl -X PUT http://localhost:8080/api/v1/components/brake_actuator/data/chassis%2Fbrakes%2Fcommand \
        -H "Content-Type: application/json" \
        -d '{"data": 50.0}'

Operations Endpoints
--------------------

Execute ROS 2 services and actions. Operation ids follow
:ref:`member-qualified-ids`: a short name that only one member exposes is used
bare, one that several expose is addressed ``<member_id>:<operation>``, and one
that a single member exposes at two ROS paths is addressed by the path itself,
without its leading slash.

List Operations
~~~~~~~~~~~~~~~

``GET /api/v1/components/{id}/operations``
   List all operations (services and actions) for an entity.

   **Example Response:**

   .. code-block:: json

      {
        "items": [
          {
            "id": "calibrate",
            "name": "calibrate",
            "type": "service",
            "service_type": "std_srvs/srv/Trigger",
            "schema": {
              "request": {},
              "response": {"success": "bool", "message": "string"}
            }
          },
          {
            "id": "long_calibration",
            "name": "long_calibration",
            "type": "action",
            "action_type": "example_interfaces/action/Fibonacci",
            "schema": {
              "goal": {"order": "int32"},
              "result": {"sequence": "int32[]"},
              "feedback": {"partial_sequence": "int32[]"}
            }
          }
        ],
        "x-medkit": {
          "entity_id": "calibration",
          "total_count": 2
        }
      }

``GET /api/v1/components/{id}/operations/{operation_id}``
   Get operation details and schema.

Execute Operations
~~~~~~~~~~~~~~~~~~

``POST /api/v1/components/{id}/operations/{operation_id}/executions``
   Execute an operation (service call or action goal).

   - **Content-Type:** application/json
   - **200:** Service call completed (sync)
   - **202:** Action goal accepted (async)
   - **400:** Invalid input
   - **404:** Operation not found

   **Service Example (synchronous):**

   .. code-block:: bash

      curl -X POST http://localhost:8080/api/v1/components/calibration/operations/calibrate/executions \
        -H "Content-Type: application/json" \
        -d '{}'

   **Action Example (asynchronous):**

   .. code-block:: bash

      curl -X POST http://localhost:8080/api/v1/components/calibration/operations/long_calibration/executions \
        -H "Content-Type: application/json" \
        -d '{"order": 10}'

   **Action Response (202 Accepted):**

   .. code-block:: json

      {
        "id": "abc123-def456",
        "status": "running"
      }

   The ``202`` carries a ``Location`` header naming the new execution. It is
   the request path plus the execution id, so it stays inside the collection
   the caller addressed: a POST to ``/api/v1/functions/powertrain/...`` is
   answered with a ``/api/v1/functions/powertrain/...`` execution URI, never a
   ``/components/`` one.

``GET /api/v1/components/{id}/operations/{operation_id}/executions``
   List all executions for an operation. Available on every entity type that
   lists the operation - areas, components, apps and functions. Actions return
   the goals started through this entity; a service returns an empty ``items``
   array, because a service call leaves no execution resource behind. An id
   naming no operation is ``404``.

``GET /api/v1/components/{id}/operations/{operation_id}/executions/{execution_id}``
   Get execution status and result.

   Executions belong to the entity they were started on. Reading, updating or
   cancelling one through a different entity's URI answers ``404`` even when
   the execution id exists, and the listing above shows an entity only the
   executions started through it. The same action reached through two entities
   (an app and the function that aggregates it) therefore keeps two separate
   execution collections.

   **Example Response (completed action):**

   .. code-block:: json

      {
        "status": "completed",
        "capability": "execute",
        "parameters": {"sequence": [0, 1, 1, 2, 3, 5, 8, 13, 21, 34]},
        "x-medkit": {
          "goal_id": "abc123def456789a0b1c2d3e4f506172",
          "ros2_status": "succeeded",
          "ros2": {
            "action": "/powertrain/engine/long_calibration",
            "type": "example_interfaces/action/Fibonacci"
          }
        }
      }

   ``status`` carries the SOVD execution status and is one of ``pending``,
   ``running``, ``completed``, ``failed``. ``parameters`` carries the action's
   most recent feedback.

   .. note::

      **Reading the outcome of a cancel.** ``status`` cannot express it on its
      own: a cancelled goal and a goal that failed by itself both render as
      ``failed``, and a goal that is still cancelling renders as ``running``.
      ``x-medkit.ros2_status`` carries the underlying ROS 2 goal state
      verbatim - ``accepted``, ``executing``, ``canceling``, ``succeeded``,
      ``canceled``, ``aborted`` - and is the field to read when a
      ``DELETE``/``PUT``-stop answered ``504`` and the outcome has to be
      established by polling.

``PUT /api/v1/components/{id}/operations/{operation_id}/executions/{execution_id}``
   Send a control command to a running execution. ROS 2 actions implement the
   SOVD ``stop`` capability (mapped to action cancel):

   .. code-block:: json

      {"capability": "stop"}

   - **202:** Stop accepted - the goal is cancelling; ``Location`` points at
     the execution status resource. Also returned when the cancel response
     was lost but the action's status stream already shows the goal
     cancelling.
   - **400:** The action server rejected the stop
     (``x-medkit-ros2-action-rejected``, ``return_code`` 1-3), or the
     capability is unsupported (``freeze`` / ``reset`` / unknown -
     ``invalid-parameter``)
   - **404:** Execution not found
   - **409:** ``execute`` on an already-running execution
     (``precondition-not-fulfilled``)
   - **500:** Transport failure while sending the cancel
     (``x-medkit-ros2-action-unavailable``)
   - **503:** Cancel service not available - the action server is gone
     (``x-medkit-ros2-action-unavailable``)
   - **504:** No response from the action server within the cancel budget and
     the status stream does not show the goal cancelling: the outcome is
     unknown - poll the execution status resource (``not-responding``)

``DELETE /api/v1/components/{id}/operations/{operation_id}/executions/{execution_id}``
   Cancel a running execution.

   - **204:** Execution cancelled. Also returned when the cancel response was
     lost but the action's status stream already shows the goal cancelling.
   - **400:** The action server answered and rejected the cancel
     (``x-medkit-ros2-action-rejected``, ``return_code`` 1-3). Note
     ``return_code`` 2 means the *action server* no longer knows the goal
     while the gateway still tracks it - the request will not start
     succeeding on retry.
   - **404:** Execution not found - the *gateway* no longer tracks it
     (``resource-not-found``)
   - **500:** Transport failure while sending the cancel
     (``x-medkit-ros2-action-unavailable``)
   - **503:** Cancel service not available - the action server is gone
     (``x-medkit-ros2-action-unavailable``)
   - **504:** No response from the action server within the cancel budget and
     the status stream does not show the goal cancelling: the outcome is
     unknown - poll the execution status resource (``not-responding``)

.. note::

   **Executions on an aggregate.** ``GET``, ``PUT`` and ``DELETE`` on a single
   execution resolve the operation id in the route to the member that owns it
   and are dispatched to that member's gateway, exactly as ``POST`` and the
   executions listing are - a goal lives on the gateway that sent it. So every
   id the listing hands out is addressable through the same path it was listed
   under, and the ``Location`` a dispatched ``PUT`` or ``POST`` returns names
   the member's own route, which this gateway resolves to the same member. An
   operation id that does not resolve to exactly one owned operation is answered
   locally, keyed on the execution id alone, so a locally-owned execution is
   unaffected and an id naming no goal still gets ``404``. A member whose
   gateway is silent answers ``504 not-responding`` before anything is
   forwarded.

.. note::

   **Cancel budget.** Both routes above are bounded by
   ``service_call_timeout_sec`` (default 10 s, clamped to 1-3600; see
   :doc:`../config/server`) plus up to 2 s spent discovering the action's
   cancel service, so the worst case a client should allow is
   ``service_call_timeout_sec + 2 s``. Configuring a budget shorter than that
   discovery wait does not shorten the discovery wait - a cancel issued before
   the cancel service has been discovered still spends up to 2 s there before
   the response wait starts.

Lifecycle Endpoints
-------------------

Read the lifecycle status of an entity and request lifecycle transitions.
Available for apps and components only.

Lifecycle control is delegated to a ``LifecycleProvider`` registered by a
substrate-owning plugin (ROS 2 lifecycle nodes, process/container/systemd, host
reboot). When no provider is registered for the entity, the status read returns
a value derived from runtime state and the transitions return ``501``.

Read Status
~~~~~~~~~~~

``GET /api/v1/apps/{app_id}/status``

``GET /api/v1/components/{component_id}/status``
   Return the current lifecycle status of the entity.

   The response ``status`` is ``ready`` or ``notReady``. Each supported
   transition is advertised as a URI field (``start``, ``restart``,
   ``force-restart``, ``shutdown``, ``force-shutdown``), present only when the
   entity's provider supports it. Without a provider, status is derived from
   runtime state (an App is ``ready`` while its node is online; the host
   Component is ``ready`` while reachable, any other Component is ``ready`` while
   at least one hosted App is online) and no transition URIs are returned.

   - **200:** Lifecycle status
   - **404:** Entity not found

   .. code-block:: json

      {
        "status": "ready",
        "restart": "/api/v1/apps/temp_sensor/status/restart"
      }

Request Transition
~~~~~~~~~~~~~~~~~~~

``PUT /api/v1/apps/{app_id}/status/{transition}``

``PUT /api/v1/components/{component_id}/status/{transition}``
   Request a lifecycle transition. ``{transition}`` is one of ``start``,
   ``restart``, ``force-restart``, ``shutdown``, ``force-shutdown``. The call is
   asynchronous: it returns on acceptance and the client polls the status read.

   RBAC: ``start`` / ``restart`` / ``force-restart`` require the ``operator``
   role; the destructive ``shutdown`` / ``force-shutdown`` require
   ``configurator``.

   - **202:** Transition accepted (the ``Location`` header points to the status URI)
   - **403:** Two different refusals share this status. The auth middleware
     rejects a caller without the role above, ahead of the handler, in the
     RFC 6749 shape (``{"error": "insufficient_scope", ...}``). The lifecycle
     provider rejects the transition itself in the SOVD shape
     (``{"error_code": "insufficient-access-rights", ...}``). Read
     ``error`` vs ``error_code`` to tell them apart.
   - **404:** Entity not found
   - **409:** A precondition was not fulfilled (``precondition-not-fulfilled``)
   - **501:** No lifecycle provider is registered for the entity (``not-implemented``)

   .. code-block:: bash

      curl -X PUT http://localhost:8080/api/v1/apps/temp_sensor/status/restart

.. _configuration-endpoints:

Configurations Endpoints
------------------------

Manage ROS 2 node parameters.

.. note::

   Parameter ids do not follow :ref:`member-qualified-ids`. On an entity backed
   by more than one node every parameter id is ``<app_id>:<param_name>``,
   whether or not that name is ambiguous, and a bare id is refused on ``PUT``
   and ``DELETE`` with ``400 invalid-request``. ``GET`` accepts the bare form
   and returns the first node that answers. Items carry the owning app in
   ``x-medkit.source``.

   The ``<app_id>`` half is a member id, so ``GET``, ``PUT`` and ``DELETE`` of a
   qualified id are served by the gateway that owns that app, on its own
   ``/apps/{app_id}/configurations/{param_name}`` route. See
   :ref:`member-qualified-ids` for the dispatch and its ``504`` case. The
   qualified form is accepted on any entity that has the named member, including
   one whose members are all peer-owned and one that runs a single node of its
   own - the ids the listing offers are unchanged in either case.

``GET /api/v1/components/{id}/configurations``
   List all parameters for an entity.

   **Example Response:**

   .. code-block:: json

      {
        "items": [
          {
            "id": "publish_rate",
            "name": "publish_rate",
            "type": "double"
          },
          {
            "id": "sensor_id",
            "name": "sensor_id",
            "type": "string"
          }
        ],
        "x-medkit": {
          "entity_id": "temp_sensor",
          "total_count": 2
        }
      }

``GET /api/v1/components/{id}/configurations/{param_name}``
   Get a specific parameter value.

``PUT /api/v1/components/{id}/configurations/{param_name}``
   Set a parameter value.

   - **Content-Type:** application/json
   - **200:** Parameter updated
   - **400:** Invalid value - the node rejected it, or it cannot be converted to
     the parameter's type at all (e.g. a mixed-type array). Both are the
     caller's body, so both are ``400``, never ``500``.
   - **404:** Parameter not found

   **Example:**

   .. code-block:: bash

      curl -X PUT http://localhost:8080/api/v1/components/temp_sensor/configurations/publish_rate \
        -H "Content-Type: application/json" \
        -d '{"data": 20.0}'

``DELETE /api/v1/components/{id}/configurations/{param_name}``
   Reset parameter to default value.

``DELETE /api/v1/components/{id}/configurations``
   Reset all parameters to default values.

   - **204:** every member of the entity was reset
   - **207:** some were not, and the body names each one

   This gateway resets a parameter by calling the parameter service on its own
   ROS graph, so a member another gateway runs is not reset by this request. Such
   a member is listed in the ``207`` body with ``success: false`` and an error
   naming the gateway that owns it, so a caller is never told a reset covered
   parameters it did not reach. Reset it on that gateway, through the member's
   own ``/apps/{app_id}/configurations`` route.

   .. code-block:: json

      {
        "entity_id": "vehicle_health",
        "results": [
          {
            "node": "/powertrain/engine/calibration",
            "app_id": "primary_calibration",
            "success": true,
            "details": {"reset_count": 2, "failed_count": 0}
          },
          {
            "app_id": "peer_calibration",
            "success": false,
            "error": "Not reset here: 'peer_calibration' is owned by gateway 'secondary_gateway'. Reset it on that gateway, through its own /apps/peer_calibration/configurations route."
          }
        ]
      }

   ``details`` carries the per-parameter outcome of the nodes this gateway did
   reset, including on an entry that failed - a partial reset names the
   parameters it could not restore.

Resource Locking
----------------

SOVD resource locking for preventing concurrent modification of entity state.
See :doc:`locking` for the full API reference.

Faults Endpoints
----------------

Query and manage faults.

.. note::

   Faults are reported by ROS 2 nodes via the FaultReporter library, not via REST API.
   The gateway queries faults from the ros2_medkit_fault_manager node.

.. note::

   **Per-entity fault scope (``/{entity-path}/faults`` routes).** The gateway keys
   faults by ``fault_code`` only, and a fault's ``reporting_sources`` set is the
   union of every app that has reported that code. Per-entity routes apply a
   strict all-sources scope check: a fault is in scope for an entity iff **every**
   entry in ``reporting_sources`` is an app owned by that entity (exact FQN
   match, or strict path-child).

   This means a ``fault_code`` reported by apps in two different entities
   (for example ``SENSOR_TIMEOUT`` reported by both the lidar and the
   temperature sensor app) is **not** visible or clearable through either
   entity's per-entity routes - per-fault routes return ``404``, collection
   responses omit it, and per-entity ``DELETE`` skips it. To see, list, or
   clear such shared faults use the global ``GET /api/v1/faults`` /
   ``DELETE /api/v1/faults`` routes.

``GET /api/v1/faults``
   List all faults across the system.

``GET /api/v1/components/{id}/faults``
   List all faults for an entity.

   Both endpoints accept an optional ``?status=`` query parameter:

   +-----------------+--------------------------------------------------+
   | Value           | Returns                                          |
   +=================+==================================================+
   | *(default)*     | ``PREFAILED`` + ``CONFIRMED`` (active faults)    |
   +-----------------+--------------------------------------------------+
   | ``pending``     | ``PREFAILED`` only                               |
   +-----------------+--------------------------------------------------+
   | ``confirmed``   | ``CONFIRMED`` only                               |
   +-----------------+--------------------------------------------------+
   | ``cleared``     | ``CLEARED`` + ``HEALED`` + ``PREPASSED``         |
   |                 | (SOVD "cleared" semantics)                       |
   +-----------------+--------------------------------------------------+
   | ``healed``      | ``HEALED`` + ``PREPASSED`` only                  |
   +-----------------+--------------------------------------------------+
   | ``all``         | All statuses                                     |
   +-----------------+--------------------------------------------------+

   **Example Response:**

   .. code-block:: json

      {
        "items": [
          {
            "fault_code": "LIDAR_RANGE_INVALID",
            "severity": "ERROR",
            "message": "Invalid range configuration: min_range > max_range",
            "timestamp": "2025-01-15T10:30:00Z",
            "source": "lidar_driver"
          }
        ],
        "x-medkit": {
          "entity_id": "lidar_sensor",
          "total_count": 1
        }
      }

``GET /api/v1/components/{id}/faults/{fault_code}``
   Get details of a specific fault including environment data.

   **Example Response (200 OK):**

   .. code-block:: json

      {
        "item": {
          "code": "MOTOR_OVERHEAT",
          "fault_name": "Motor temperature exceeded threshold",
          "severity": 2,
          "status": {
            "aggregatedStatus": "active",
            "testFailed": "1",
            "confirmedDTC": "1",
            "pendingDTC": "0"
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
              "data": 105.5,
              "x-medkit": {
                "topic": "/motor/temperature",
                "message_type": "sensor_msgs/msg/Temperature",
                "full_data": {"temperature": 105.5, "variance": 0.1},
                "captured_at": "2026-02-04T10:30:00.123Z"
              }
            },
            {
              "type": "rosbag",
              "name": "fault_recording",
              "bulk_data_uri": "/apps/motor_controller/bulk-data/rosbags/fault_MOTOR_OVERHEAT_1738664999000",
              "size_bytes": 1234567,
              "duration_sec": 6.0,
              "format": "mcap"
            }
          ]
        },
        "x-medkit": {
          "occurrence_count": 3,
          "reporting_sources": ["/powertrain/motor_controller"],
          "severity_label": "ERROR"
        }
      }

   **Status Object:**

   The ``status`` object follows SOVD fault status specification:

   - ``aggregatedStatus``: Overall status (``active``, ``passive``, ``cleared``)
   - ``testFailed``: Test failed indicator (``0`` or ``1``)
   - ``confirmedDTC``: Confirmed DTC indicator (``0`` or ``1``)
   - ``pendingDTC``: Pending DTC indicator (``0`` or ``1``)

   **Snapshot Types:**

   - ``freeze_frame``: Data captured at fault confirmation. Entity frames for
     faults that were already confirmed when the gateway started are captured
     at gateway start instead and carry ``"capture_origin": "startup"`` in
     their ``x-medkit`` block. For a plugin-backed entity that reports its
     link down, the values are the plugin's last known ones and may predate
     the confirmation by the length of the outage; such entries carry
     ``connected`` (the payload's link flag, ``false`` for the loss-of-comms
     case) and ``source_timestamp`` (the payload's own timestamp, verbatim)
     in ``x-medkit``, both only when the plugin's payload reports them.
     ``captured_at`` always dates the capture, not the values.
   - ``rosbag``: Recording file available via bulk-data endpoint

   **Response codes:**

   - **200:** Fault details
   - **400:** ``fault_code`` empty or longer than 256 characters
   - **404:** Fault not found, reported by an app outside this entity's scope,
     or declined by the fault manager
   - **503:** Fault manager unavailable

``DELETE /api/v1/components/{id}/faults/{fault_code}``
   Clear a fault.

   - **204:** Fault cleared
   - **400:** ``fault_code`` empty or longer than 256 characters
   - **404:** Fault not found, reported by an app outside this entity's scope,
     or declined by the fault manager
   - **503:** Fault manager unavailable

.. note::

   ``503`` on these two routes means the fault manager did not answer - it is
   absent, still starting, or timed out. A fault manager that answers and
   declines the request is reported as ``404``, not ``503``: it is reachable
   and healthy, and the request is what it would not serve. That covers a
   ``fault_code`` it does not hold and one it will not accept - it restricts
   codes to alphanumerics, underscore, hyphen and dot, a narrower set than the
   ``maxLength`` the OpenAPI document publishes, so a short code containing
   anything else is admitted by the gateway and answered ``404``.

   Both nodes bound ``fault_code`` at the published 256 characters, so every
   length the document admits reaches the fault manager.

``DELETE /api/v1/components/{id}/faults``
   Clear all faults for an entity.

   Accepts the optional ``?status=`` query parameter (same values as ``GET /faults``).
   Without it, clears pending and confirmed faults.

   - **204:** Faults cleared (or none to clear)
   - **400:** Invalid status parameter
   - **503:** Fault manager unavailable

``DELETE /api/v1/faults``
   Clear all faults across the system *(ros2_medkit extension, not SOVD)*.

   Accepts the optional ``?status=`` query parameter (same values as ``GET /faults``).
   Without it, clears pending and confirmed faults.

   - **204:** Faults cleared (or none to clear)
   - **400:** Invalid status parameter
   - **503:** Fault manager unavailable

Logs Endpoints
--------------

Query and configure the /rosout ring buffer for an entity. Supported entity types:
**areas** (aggregated from hosted apps, merged with a namespace prefix query), **components**
(aggregated from hosted apps, merged with a namespace prefix query), **apps** (exact FQN match;
external apps by bare entity id), and **functions** (aggregated from hosted apps).

.. note::

   By default, log entries are sourced from the ``/rosout`` ROS 2 topic. ros2_medkit retains
   the 200 most recent entries per node in an in-memory ring buffer (configurable via
   ``logs.buffer_size`` in ``gateway_params.yaml``). A ``LogProvider`` plugin can replace the
   storage backend or take full ownership of the log pipeline (see plugin development docs).

``GET /api/v1/components/{id}/logs``
   Query log entries aggregated from the component's hosted apps. Child apps resolve via
   the entity cache using the same rule as fault scoping: an external app (plugin-provided,
   no ROS binding) is queried by its bare entity id, every other app by its exact FQN, and
   an external component also returns entries stored under its own id. When the component
   declares a non-empty namespace, a namespace prefix query runs in addition and the results
   are merged and deduplicated, so ROS nodes under the namespace keep contributing even when
   the component also hosts external apps. The response always carries
   ``x-medkit.aggregation_level=component`` and ``aggregated=true``; the ``app_count`` and
   ``aggregation_sources`` fields are populated only when at least one hosted source
   resolves.

``GET /api/v1/apps/{id}/logs``
   Query log entries for the specific app. A ROS-bound app is queried by its exact FQN; an
   external app by its bare entity id (the id ``add_log_entry`` and fault reporting use).

**Query parameters:**

.. list-table::
   :header-rows: 1
   :widths: 20 80

   * - Parameter
     - Description
   * - ``severity``
     - Minimum severity filter (``debug`` | ``info`` | ``warning`` | ``error`` | ``fatal``).
       The stricter of this parameter and the entity's configured ``severity_filter`` is applied.
       Without this parameter, the entity's configured ``severity_filter`` (default: ``debug``)
       determines the minimum level. Empty or absent = use entity config only.
   * - ``context``
     - Substring filter applied to the log entry's logger name (``context.node`` in the response).
       Maximum length: 256 characters. Empty or absent = no filter.

**Response 200:**

.. code-block:: json

   {
     "items": [
       {
         "id": "log_42",
         "timestamp": "2026-01-15T10:30:00.123456789Z",
         "severity": "warning",
         "message": "Calibration drift detected",
         "context": {
           "node": "powertrain/engine/temp_sensor",
           "function": "read_sensor",
           "file": "temp_sensor.cpp",
           "line": 99
         }
       }
     ]
   }

The ``context.function``, ``context.file``, and ``context.line`` fields are omitted when empty/zero.

**Severity values** map directly to the ROS 2 log levels:

.. list-table::
   :header-rows: 1
   :widths: 15 15 70

   * - Value
     - ROS 2 level
     - Meaning
   * - ``debug``
     - DEBUG (10)
     - Fine-grained diagnostic information
   * - ``info``
     - INFO (20)
     - Normal operational messages
   * - ``warning``
     - WARN (30)
     - Non-fatal anomalies
   * - ``error``
     - ERROR (40)
     - Errors that may require attention
   * - ``fatal``
     - FATAL (50)
     - Critical failures

``GET /api/v1/components/{id}/logs/configuration`` / ``GET /api/v1/apps/{id}/logs/configuration``
   Return the current log configuration for the entity.

   **Response 200:**

   .. code-block:: json

      {
        "severity_filter": "debug",
        "max_entries": 100
      }

``PUT /api/v1/components/{id}/logs/configuration`` / ``PUT /api/v1/apps/{id}/logs/configuration``
   Update the log configuration for the entity. All body fields are optional.

   **Request body:**

   .. code-block:: json

      {
        "severity_filter": "warning",
        "max_entries": 500
      }

   ``severity_filter`` - minimum severity to return in query results (``debug`` | ``info`` | ``warning`` |
   ``error`` | ``fatal``). Entries below this level are excluded from queries. Default: ``debug``.

   ``max_entries`` - maximum number of entries returned per query. Must be between 1 and 10,000
   (inclusive). Default: ``100``.

   **Response 204:** No content.

   - **400:** Invalid ``severity_filter`` or ``max_entries`` value

Bulk Data Endpoints
-------------------

Access, upload, and delete large binary data (rosbags, calibration files, firmware, etc.)
associated with entities. Read endpoints (GET) support all entity types. Write endpoints
(POST, DELETE) are supported for components and apps only.

List Categories
~~~~~~~~~~~~~~~

``GET /api/v1/{entity-path}/bulk-data``

List available bulk-data categories for an entity. Returns the union of rosbag categories
(from the fault manager) and configured categories (from ``bulk_data.categories``).

**Supported entity paths:**

- ``/apps/{app-id}``
- ``/components/{component-id}``
- ``/areas/{area-id}``
- ``/functions/{function-id}``
- ``/areas/{area-id}/subareas/{subarea-id}``
- ``/components/{component-id}/subcomponents/{subcomponent-id}``

**Example:**

.. code-block:: bash

   curl http://localhost:8080/api/v1/apps/motor_controller/bulk-data

**Response (200 OK):**

.. code-block:: json

   {
     "items": ["rosbags", "calibration", "firmware"]
   }

List Bulk Data Items
~~~~~~~~~~~~~~~~~~~~

``GET /api/v1/{entity-path}/bulk-data/{category}``

List all bulk-data items in a category for the entity.

**Example:**

.. code-block:: bash

   curl http://localhost:8080/api/v1/apps/motor_controller/bulk-data/rosbags

**Response (200 OK):**

.. code-block:: json

   {
     "items": [
       {
         "id": "fault_MOTOR_OVERHEAT_1738664999000",
         "name": "fault_MOTOR_OVERHEAT_1738664999000 recording 2026-02-04T10:30:00.000Z",
         "mimetype": "application/x-mcap",
         "size": 1234567,
         "creation_date": "2026-02-04T10:30:00.000Z",
         "x-medkit": {
           "fault_codes": ["MOTOR_OVERHEAT", "MOTOR_STALL"],
           "duration_sec": 6.0,
           "format": "mcap",
           "recording_id": "fault_MOTOR_OVERHEAT_1738664999000"
         }
       }
     ]
   }

For ``rosbags``, the descriptor ``id`` is the recording id - the bag directory
name - and it is what the download URL takes. There is **one descriptor per
recording**, not one per fault: faults confirmed in one burst share a single
recording, and ``x-medkit.fault_codes`` lists every fault attached to it. A
recording therefore reports its size once. One fault code can appear on several
descriptors, one per occurrence it kept, told apart by ``creation_date``, which
is the time that recording was made.

Download Bulk Data
~~~~~~~~~~~~~~~~~~

``GET /api/v1/{entity-path}/bulk-data/{category}/{id}``

Download a specific bulk-data file.

**Response Headers:**

- ``Content-Type``: the media type of the stored artifact - see below
- ``Content-Disposition``: ``attachment; filename="<name>"``. For a rosbag the
  name is ``<recording_id>.<format>``: the recording actually served, which for
  a pre-#620 fault-code URL is not the segment the client sent, and the format
  is the one persisted at capture time (``mcap`` or ``sqlite3``). For every
  other category it is the stored item's own name, e.g. ``report.zip``.
- ``Accept-Ranges``: ``bytes`` - the download is served by a range-aware
  provider, so a client may fetch part of the file
- ``Access-Control-Expose-Headers``: ``Content-Disposition``

**Media types.** The OpenAPI document declares
``application/x-mcap``, ``application/x-sqlite3`` and
``application/octet-stream`` for this response, followed by ``*/*``. That is
not hedging: for the ``rosbags`` category the type is derived from the
recorded storage format and is one of the three named types, but every other
category serves back the media type recorded when the file was uploaded, which
is chosen by the uploading client. Uploading a ``text/csv`` makes the download
serve ``text/csv``. The named types are declared because they *are*
derivable; ``*/*`` is declared because the rest genuinely is not.

There is no response schema, for either status. The body is raw file content,
and OpenAPI 3.1 has no way to say "bytes" - ``format: binary`` was an OpenAPI
3.0 idiom that 3.1 dropped when it aligned with JSON Schema 2020-12. A
schema-free media type entry is the accurate description.

**Range requests.** A request carrying a ``Range`` header is answered with
**206 Partial Content** and a ``Content-Range: bytes <start>-<end>/<total>``
header instead of ``200``; the body is the requested slice. Several ranges in
one request are answered as a single ``multipart/byteranges`` body, which is
declared on the 206 only - the 200 can never carry it.

**Example:**

.. code-block:: bash

   curl -O -J http://localhost:8080/api/v1/apps/motor_controller/bulk-data/rosbags/fault_MOTOR_OVERHEAT_1738664999000

   # One byte range
   curl -H 'Range: bytes=0-1023' \
     http://localhost:8080/api/v1/apps/motor_controller/bulk-data/rosbags/550e8400-e29b-41d4-a716-446655440000

**Response Codes:**

- **200 OK**: File content
- **206 Partial Content**: The byte range requested via ``Range``, with ``Content-Range``
- **404 Not Found**: Entity, category, or bulk-data ID not found
- **416 Range Not Satisfiable**: The ``Range`` header could not be parsed. Not
  specific to this endpoint - see :ref:`rest-range-rejection`.

Upload Bulk Data
~~~~~~~~~~~~~~~~

``POST /api/v1/{entity-path}/bulk-data/{category}``

Upload a new bulk-data file to the specified category. Files are sent as
``multipart/form-data``. The ``rosbags`` category is read-only and cannot be
used for uploads.

**Supported entity types:** components, apps only. Areas and functions return 405.

**Form Fields:**

.. list-table::
   :header-rows: 1
   :widths: 20 15 65

   * - Field
     - Required
     - Description
   * - ``file``
     - Yes
     - The file to upload (binary data with filename and content type).
   * - ``description``
     - No
     - Human-readable description of the file.
   * - ``metadata``
     - No
     - JSON string with arbitrary key-value metadata.

**Example:**

.. code-block:: bash

   curl -X POST http://localhost:8080/api/v1/components/motor_controller/bulk-data/calibration \
     -F "file=@calibration_data.bin;type=application/octet-stream" \
     -F "description=Motor calibration parameters v2.1" \
     -F 'metadata={"version": "2.1", "author": "engineer_01"}'

**Response (201 Created):**

.. code-block:: json

   {
     "id": "calibration_1739612345000000000_ab12cd34",
     "name": "calibration_data.bin",
     "mimetype": "application/octet-stream",
     "size": 4096,
     "creation_date": "2026-03-15T14:30:00.000Z",
     "description": "Motor calibration parameters v2.1",
     "x-medkit": {
       "version": "2.1",
       "author": "engineer_01"
     }
   }

**Response Headers:**

- ``Location``: ``/api/v1/components/motor_controller/bulk-data/calibration/calibration_1739612345000000000_ab12cd34``

**Error Responses:**

- **400 Bad Request**: Missing ``file`` field, unknown category, or ``rosbags`` category
- **405 Method Not Allowed**: Upload attempted on areas or functions
- **413 Payload Too Large**: File exceeds ``bulk_data.max_upload_size``

Delete Bulk Data
~~~~~~~~~~~~~~~~

``DELETE /api/v1/{entity-path}/bulk-data/{category}/{id}``

Delete a specific bulk-data item. The ``rosbags`` category is managed by the
fault manager and cannot be deleted via this endpoint.

**Supported entity types:** components, apps only. Areas and functions return 405.

**Example:**

.. code-block:: bash

   curl -X DELETE http://localhost:8080/api/v1/components/motor_controller/bulk-data/calibration/calibration_1739612345000000000_ab12cd34

**Response Codes:**

- **204 No Content**: Item deleted successfully
- **400 Bad Request**: ``rosbags`` category (managed by fault manager)
- **404 Not Found**: Entity, category, or bulk-data ID not found
- **405 Method Not Allowed**: Delete attempted on areas or functions

Software Updates
----------------

Manage software update packages with an async prepare/execute lifecycle.
The updates feature requires a plugin implementing ``UpdateProvider`` to be loaded
via the plugin framework (see :doc:`/config/server`).
Without such a plugin, all endpoints return ``501 Not Implemented``.

``GET /api/v1/updates``
   List all registered update packages.

   **Query Parameters:**

   - ``origin`` (optional): Filter by origin (``remote`` or ``proximity``)
   - ``target-version`` (optional): Filter by target version

   **Example Response (200 OK):**

   .. code-block:: json

      {
        "items": ["firmware-v2.1", "calibration-update-3"]
      }

``POST /api/v1/updates``
   Register a new update package.

   **Request Body:**

   .. code-block:: json

      {
        "id": "firmware-v2.1",
        "update_name": "Firmware Update v2.1",
        "automated": true,
        "origins": ["remote"],
        "duration": 600,
        "size": 52428800,
        "updated_components": ["ecu_main"],
        "affected_components": ["ecu_main", "ecu_secondary"]
      }

   **Response (201 Created):**

   .. code-block:: json

      {
        "id": "firmware-v2.1"
      }

   **Response Headers:**

   - ``Location``: ``/api/v1/updates/firmware-v2.1``

``GET /api/v1/updates/{id}``
   Get full metadata for a specific update package.

   **Response (200 OK):**

   Returns the JSON metadata as registered.

   - **404 Not Found:** Package does not exist

``DELETE /api/v1/updates/{id}``
   Delete an update package.

   - **204 No Content:** Package deleted
   - **404 Not Found:** Package does not exist
   - **409 Conflict:** Operation in progress for this package

``PUT /api/v1/updates/{id}/prepare``
   Trigger preparation of an update (download, verify, check dependencies).
   Runs asynchronously - poll the status endpoint for progress.

   - **202 Accepted:** Preparation started
   - **404 Not Found:** Package does not exist
   - **409 Conflict:** Operation already in progress

   **Response Headers:**

   - ``Location``: ``/api/v1/updates/{id}/status``

``PUT /api/v1/updates/{id}/execute``
   Trigger execution of a prepared update (install). Only succeeds after
   prepare has completed.

   - **202 Accepted:** Execution started
   - **400 Bad Request:** Package not prepared
   - **404 Not Found:** Package does not exist
   - **409 Conflict:** Operation already in progress

   **Response Headers:**

   - ``Location``: ``/api/v1/updates/{id}/status``

``PUT /api/v1/updates/{id}/automated``
   Trigger automated update (prepare + execute in one step). Only works
   for packages that support automated mode.

   - **202 Accepted:** Automated update started
   - **400 Bad Request:** Package does not support automated mode
   - **404 Not Found:** Package does not exist
   - **409 Conflict:** Operation already in progress

   **Response Headers:**

   - ``Location``: ``/api/v1/updates/{id}/status``

``GET /api/v1/updates/{id}/status``
   Get the current status and progress of an update operation.

   **Example Response (200 OK):**

   .. code-block:: json

      {
        "status": "inProgress",
        "progress": 65,
        "sub_progress": [
          {"name": "download", "progress": 100},
          {"name": "verify", "progress": 30}
        ],
        "x-medkit": {
          "phase": "preparing"
        }
      }

   **Status values:** ``pending``, ``inProgress``, ``completed``, ``failed``

   A successful ``POST /api/v1/updates`` seeds a ``pending`` status for the package,
   so this endpoint returns ``200`` with ``{"status": "pending", "x-medkit": {"phase": "none"}}``
   immediately after registration, before any ``prepare`` or ``execute`` call.

   **Vendor extension ``x-medkit.phase``** (non-standard, SOVD-compatible):
   ``none``, ``preparing``, ``prepared``, ``executing``, ``executed``,
   ``failed``, ``deleting``. Differentiates "prepare completed" (``status``
   ``completed`` + ``x-medkit.phase`` ``prepared``) from "execute completed"
   (``status`` ``completed`` + ``x-medkit.phase`` ``executed``). Clients that
   only consume the standard ``status`` field continue to work unchanged.

   When ``status`` is ``failed``, an ``error`` object is included:

   .. code-block:: json

      {
        "status": "failed",
        "error": {
          "error_code": "internal-error",
          "message": "Download failed: connection timeout"
        }
      }

   - **404 Not Found:** Package is not registered

Cyclic Subscriptions
--------------------

Cyclic subscriptions provide periodic push-based delivery of any SOVD resource collection
via Server-Sent Events (SSE). A client creates a subscription specifying the resource URI
(data, faults, configurations, logs, or ``x-`` vendor extensions) and a delivery interval.
The server then pushes the latest value at the requested frequency.

Subscriptions are temporary - they do not survive server restart.

**Supported collections:**

- ``data`` - Topic data. Requires a resource path naming the topic, e.g. ``/data/temperature``
- ``faults`` - Fault list. Streamed as a whole; no resource path
- ``configurations`` - Parameter values. Streamed as a whole; no resource path
- ``logs`` - Application log entries from ``/rosout``. Streamed as a whole; no resource path
- ``x-*`` - Vendor extensions (e.g. ``x-medkit-graph``). Streamed as a whole; no resource path

A collection that is streamed as a whole delivers every item of that collection on
every tick. A resource URI naming a single item of such a collection is refused with
400 ``x-medkit-invalid-resource-uri`` rather than accepted and answered with the whole
collection.

**Interval values:**

- ``fast`` - 50ms sampling period
- ``normal`` - 200ms sampling period (default)
- ``slow`` - 500ms sampling period

``POST /api/v1/{entity_type}/{entity_id}/cyclic-subscriptions``
   Create a new cyclic subscription.

   Response: **201 Created** with a ``Location`` header pointing to the new
   subscription.

   **Applies to:** ``/apps``, ``/components``, ``/functions``

   **Request Body:**

   .. code-block:: json

      {
        "resource": "/api/v1/apps/temp_sensor/data/temperature",
        "protocol": "sse",
        "interval": "normal",
        "duration": 300
      }

   **Fields:**

   - ``resource`` (string, required): Full SOVD resource URI to observe
     (e.g. ``/api/v1/apps/{id}/data/{topic}``, ``/api/v1/apps/{id}/faults``,
     ``/api/v1/functions/{id}/x-medkit-graph``). The URI ends at the collection
     unless that collection's sampler narrows its payload to a named resource
   - ``protocol`` (string, optional): Transport protocol. Only ``"sse"`` supported. Default: ``"sse"``
   - ``interval`` (string, required): One of ``fast``, ``normal``, ``slow``
   - ``duration`` (integer, required): Subscription lifetime in seconds.
     Must be > 0 and <= ``sse.max_duration_sec`` (default: 3600)

   **Error responses:**

   - **400** ``invalid-parameter`` - Invalid interval, duration <= 0, or duration exceeds max
   - **400** ``x-medkit-invalid-resource-uri`` - Malformed resource URI, path traversal,
     ``data`` without a topic path, or a resource path on a collection that is streamed
     as a whole
   - **400** ``x-medkit-entity-mismatch`` - Resource URI references different entity than route
   - **400** ``x-medkit-collection-not-supported`` - Entity doesn't support the collection
   - **400** ``x-medkit-collection-not-available`` - No data provider registered for collection
   - **400** ``x-medkit-unsupported-protocol`` - Requested protocol not available
   - **503** ``service-unavailable`` - Max subscription capacity reached

   **Response 201 Created:**

   .. code-block:: json

      {
        "id": "sub_001",
        "observed_resource": "/api/v1/apps/temp_sensor/data/temperature",
        "event_source": "/api/v1/apps/temp_sensor/cyclic-subscriptions/sub_001/events",
        "protocol": "sse",
        "interval": "normal"
      }

``GET /api/v1/{entity_type}/{entity_id}/cyclic-subscriptions``
   List all active cyclic subscriptions for an entity. Returns ``{"items": [...]}``.

``GET /api/v1/{entity_type}/{entity_id}/cyclic-subscriptions/{id}``
   Get details of a single subscription.

``PUT /api/v1/{entity_type}/{entity_id}/cyclic-subscriptions/{id}``
   Update ``interval`` and/or ``duration`` of an existing subscription.
   Only provided fields are updated. Updating ``duration`` resets the
   expiry timer from the current time (not from the original creation time).

   **Request Body:**

   .. code-block:: json

      {
        "interval": "fast",
        "duration": 600
      }

``DELETE /api/v1/{entity_type}/{entity_id}/cyclic-subscriptions/{id}``
   Cancel and remove a subscription. Returns 204 No Content.

``GET /api/v1/{entity_type}/{entity_id}/cyclic-subscriptions/{id}/events``
   SSE event stream. Connect to receive periodic data updates.

   **Response Headers:**

   .. code-block:: text

      Content-Type: text/event-stream
      Cache-Control: no-cache
      Connection: keep-alive

   **Event Format (EventEnvelope):**

   .. code-block:: text

      data: {"timestamp":"2026-02-14T10:30:00.250Z","payload":{"id":"/temperature","data":{"data":23.5}}}

   The stream auto-closes when the duration expires, the client disconnects,
   or the subscription is deleted.

**Multi-collection examples:**

Subscribe to faults on a component:

.. code-block:: json

   {
     "resource": "/api/v1/components/ecu1/faults",
     "interval": "slow",
     "duration": 600
   }

Subscribe to a specific configuration parameter:

.. code-block:: json

   {
     "resource": "/api/v1/apps/temp_sensor/configurations/calibration_offset",
     "interval": "normal",
     "duration": 120
   }

Scripts
-------

Upload, manage, and execute diagnostic scripts on entities.
*(ISO 17978-3, 7.15)*

Scripts are available on **Components** and **Apps** entity types.
The feature must be enabled by setting ``scripts.scripts_dir`` in the gateway configuration.

Script Error Statuses
~~~~~~~~~~~~~~~~~~~~~

Beyond the usual 400 / 404 / 500, and 501 on every script endpoint when no
scripts backend is configured, each endpoint answers only what its own backend
call can produce. With the built-in backend:

.. list-table::
   :header-rows: 1
   :widths: 45 55

   * - Endpoint
     - Extra statuses
   * - ``POST .../scripts`` (upload)
     - **413** ``script-file-too-large`` - file over the configured size limit
   * - ``DELETE .../scripts/{script_id}``
     - **409** ``script-managed`` (manifest-owned, not editable) or
       ``script-running``
   * - ``POST .../scripts/{script_id}/executions``
     - **429** ``script-concurrency-limit``
   * - ``PUT .../executions/{execution_id}``
     - **409** ``script-not-running``
   * - ``DELETE .../executions/{execution_id}``
     - **409** ``script-running``

The listing and read endpoints (``GET .../scripts``,
``GET .../scripts/{script_id}``, ``GET .../executions/{execution_id}``) add
nothing to the blanket set.

The 429 is the **script manager's** concurrency limit, not the HTTP rate
limiter's: it is answered whether or not ``rate_limiting.enabled`` is set, and
carries no ``Retry-After`` or ``X-RateLimit-*`` headers. See
:ref:`rate-limiting` for the other 429. When the limiter is on, both can answer
429 on the execution-start route and the document can only describe one: the
route's own declaration wins, so that operation's 429 is documented as the
script manager's, without the limiter's headers. The body shape is the same
either way.

``script-already-exists`` is defined for backends that maintain their own
registry (a plugin with a SQLite store, say); the built-in backend generates
ids and never returns it.

Upload Script
~~~~~~~~~~~~~

``POST /api/v1/{entity_type}/{entity_id}/scripts``
   Upload a diagnostic script via ``multipart/form-data``.

   - **file** (required): The script file (Python, bash, or sh)
   - **metadata** (optional): JSON with name, description, parameters_schema

   Response: **201 Created** with ``Location`` header pointing to the new script.

   .. note::

      Uploads can be disabled by setting ``scripts.allow_uploads: false`` in the
      gateway configuration. When disabled, POST returns 400. Pre-deployed
      manifest scripts remain available for execution.

List Scripts
~~~~~~~~~~~~

``GET /api/v1/{entity_type}/{entity_id}/scripts``
   List all scripts for an entity. Returns ``{"items": [...]}``.

Get Script
~~~~~~~~~~

``GET /api/v1/{entity_type}/{entity_id}/scripts/{script_id}``
   Get metadata for a specific script.

Delete Script
~~~~~~~~~~~~~

``DELETE /api/v1/{entity_type}/{entity_id}/scripts/{script_id}``
   Delete an uploaded script. Returns **204 No Content**.
   Returns **409** if the script is manifest-managed or currently executing.

Start Execution
~~~~~~~~~~~~~~~

``POST /api/v1/{entity_type}/{entity_id}/scripts/{script_id}/executions``
   Start a new execution of a script.

   **Request Body:**

   .. code-block:: json

      {
        "execution_type": "now",
        "parameters": {"threshold": 0.1}
      }

   .. list-table::
      :header-rows: 1
      :widths: 25 15 10 50

      * - Attribute
        - Type
        - Conv
        - Description
      * - ``execution_type``
        - string
        - M
        - When to run. The shipped backend accepts only ``now``; see the note below
      * - ``parameters``
        - object
        - O
        - Input parameters for the script
      * - ``proximity_response``
        - string
        - O
        - Co-location proof token

   .. note::

      The built-in script backend supports only ``now``. Other execution types
      (``on_restart``, ``now_and_on_restart``, ``once_on_restart``) require a
      plugin-provided ScriptProvider and will return 400 ``invalid-parameter``
      if not supported.

   Response: **202 Accepted** with ``Location`` header pointing to the execution status.

Get Execution Status
~~~~~~~~~~~~~~~~~~~~

``GET /api/v1/{entity_type}/{entity_id}/scripts/{script_id}/executions/{execution_id}``
   Poll the status of a script execution.

   Status values: ``prepared``, ``running``, ``completed``, ``failed``, ``terminated``

Terminate Execution
~~~~~~~~~~~~~~~~~~~

``PUT /api/v1/{entity_type}/{entity_id}/scripts/{script_id}/executions/{execution_id}``
   Send a termination action to a running execution.

   **Request Body:**

   .. code-block:: json

      {"action": "stop"}

   Action values: ``stop`` (SIGTERM), ``forced_termination`` (SIGKILL).

Delete Execution
~~~~~~~~~~~~~~~~

``DELETE /api/v1/{entity_type}/{entity_id}/scripts/{script_id}/executions/{execution_id}``
   Remove a completed/terminated execution resource. Returns **204 No Content**.
   Returns **409** if the execution is still running.

Triggers
--------

Triggers provide condition-based push notifications for resource changes via
Server-Sent Events (SSE). Unlike cyclic subscriptions - which poll a resource
at a fixed interval and push every sample - triggers evaluate a condition
against each change and only fire when the condition is met.

**Key differences from Cyclic Subscriptions:**

- Cyclic subscriptions push data at a fixed interval (``fast``/``normal``/``slow``)
  regardless of whether the value changed
- Triggers are event-driven: they only fire when a specific condition is satisfied
  (e.g., value changed, entered a range, reached a threshold)
- Triggers support persistence across gateway restarts (``persistent: true``)
- Triggers can be one-shot (fire once, then auto-terminate) or multishot (continuous)

**Supported entity types:** ``/areas``, ``/components``, ``/apps``, ``/functions``

.. note::

   **ros2_medkit extension:** SOVD defines triggers for apps and components only.
   ros2_medkit extends trigger support to areas and functions, allowing
   hierarchy-scoped monitoring. Area-level triggers catch changes from all
   descendant entities within the area.

**Observable resource collections:**

- ``data`` - Topic data changes (driven by ``TriggerTopicSubscriber``)
- ``faults`` - Fault state transitions (created, updated, cleared)
- ``operations`` - Operation execution completions
- ``updates`` - Software update status changes
- ``logs`` - Log entries matching configured severity (x-medkit extension)

Create Trigger
~~~~~~~~~~~~~~

``POST /api/v1/{entity_type}/{entity_id}/triggers``
   Create a new condition-based trigger.

   Response: **201 Created** with a ``Location`` header pointing to the new
   trigger.

   **Request Body:**

   .. code-block:: json

      {
        "resource": "/api/v1/apps/temp_sensor/data/powertrain%2Fengine%2Ftemperature",
        "trigger_condition": {
          "condition_type": "LeaveRange",
          "lower_bound": 20.0,
          "upper_bound": 80.0
        },
        "path": "/data",
        "protocol": "sse",
        "multishot": true,
        "persistent": false,
        "lifetime": 300,
        "log_settings": {
          "severity": "warning",
          "marker": "Temperature threshold exceeded"
        }
      }

   **Fields:**

   .. list-table::
      :header-rows: 1
      :widths: 20 10 70

      * - Field
        - Required
        - Description
      * - ``resource``
        - Yes
        - Full SOVD resource URI to observe (e.g. ``/api/v1/apps/{id}/data/{topic}``,
          ``/api/v1/apps/{id}/faults``, ``/api/v1/areas/{id}/faults``).
          Must reference the same entity as the route.
      * - ``trigger_condition``
        - Yes
        - Object with ``condition_type`` and condition-specific parameters.
          See `Trigger Conditions`_ below.
      * - ``path``
        - No
        - JSON Pointer within the resource payload to evaluate. When set, the
          condition is evaluated against the value at this path instead of the
          full payload.
      * - ``protocol``
        - No
        - Transport protocol. Only ``"sse"`` is supported. Default: ``"sse"``.
      * - ``multishot``
        - No
        - If ``true``, the trigger fires repeatedly. If ``false``, the trigger
          auto-terminates after the first event. Default: ``false``.
      * - ``persistent``
        - No
        - If ``true``, the trigger survives gateway restarts (when
          ``on_restart_behavior`` is ``"restore"``). Default: ``false``.
      * - ``lifetime``
        - No
        - Time-to-live in seconds. The trigger auto-terminates after this
          duration. Must be a positive integer. Omit for no expiry.
      * - ``log_settings``
        - No
        - Temporary log entry injected when the trigger fires.
          Accepts ``severity`` (log level: ``debug``, ``info``, ``warning``,
          ``error``, ``fatal``; default: ``info``) and ``marker`` (descriptive
          message text; default: ``"Trigger fired"``). The log entry includes
          trigger metadata (trigger ID, condition type, resource URI).

   **Response 201 Created:**

   .. code-block:: json

      {
        "id": "trig_001",
        "status": "active",
        "observed_resource": "/api/v1/apps/temp_sensor/data/powertrain%2Fengine%2Ftemperature",
        "event_source": "/api/v1/apps/temp_sensor/triggers/trig_001/events",
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

   **Error Responses:**

   - **400** ``invalid-parameter`` - Missing or invalid ``resource``, ``trigger_condition``,
     ``condition_type``, ``lifetime``, or condition-specific parameters
   - **400** ``x-medkit-invalid-resource-uri`` - Malformed resource URI or path traversal
   - **400** ``x-medkit-entity-mismatch`` - Resource URI references a different entity than
     the route
   - **503** ``service-unavailable`` - Maximum trigger capacity reached
     (configurable via ``triggers.max_triggers``)

List Triggers
~~~~~~~~~~~~~

``GET /api/v1/{entity_type}/{entity_id}/triggers``
   List all triggers for an entity.

   **Response 200:**

   .. code-block:: json

      {
        "items": [
          {
            "id": "trig_001",
            "status": "active",
            "observed_resource": "/api/v1/apps/temp_sensor/faults",
            "event_source": "/api/v1/apps/temp_sensor/triggers/trig_001/events",
            "protocol": "sse",
            "trigger_condition": {"condition_type": "OnChange"},
            "multishot": true,
            "persistent": false,
            "lifetime": 300
          }
        ]
      }

Get Trigger
~~~~~~~~~~~

``GET /api/v1/{entity_type}/{entity_id}/triggers/{trigger_id}``
   Get details of a single trigger.

   **Response 200:** Same schema as creation response.

   - **404** ``resource-not-found`` - Trigger not found or belongs to a different entity

Update Trigger
~~~~~~~~~~~~~~

``PUT /api/v1/{entity_type}/{entity_id}/triggers/{trigger_id}``
   Update the lifetime of an existing trigger. Updating ``lifetime`` resets the
   expiry timer from the current time.

   **Request Body:**

   .. code-block:: json

      {
        "lifetime": 600
      }

   **Response 200:** Updated trigger object (same schema as creation response).

   - **400** ``invalid-parameter`` - Missing or invalid ``lifetime``
   - **404** ``resource-not-found`` - Trigger not found

Delete Trigger
~~~~~~~~~~~~~~

``DELETE /api/v1/{entity_type}/{entity_id}/triggers/{trigger_id}``
   Remove a trigger. Any active SSE connection for this trigger is closed.

   - **204** No Content - Trigger deleted
   - **404** ``resource-not-found`` - Trigger not found

Trigger Events (SSE Stream)
~~~~~~~~~~~~~~~~~~~~~~~~~~~

``GET /api/v1/{entity_type}/{entity_id}/triggers/{trigger_id}/events``
   SSE event stream for a trigger. Connect to receive events when the trigger
   condition is met. The stream sends keepalive comments every 15 seconds.

   **Response Headers:**

   .. code-block:: text

      Content-Type: text/event-stream
      Cache-Control: no-cache

   **Frame format:**

   Each event is one SSE frame carrying an ``id:`` field and a ``data:`` field
   holding the JSON ``TriggerEventFrame``:

   .. code-block:: text

      id: 1
      data: {"timestamp":"2026-03-19T10:30:00.250Z","payload":{"data":{"data":85.5}}}

   The id counts events on this connection from 1, and unlike the fault stream
   this route does not read ``Last-Event-ID`` - so the id is a position within
   one connection, not a replay cursor, and reconnecting restarts it at 1.

   A brief disconnect does not by itself lose events. Each trigger holds a
   queue of up to 100 pending events, filled as conditions fire whether or not
   a client is attached, and drained on the next connection - so a multishot
   trigger that is reconnected to promptly delivers what it buffered.

   The queue is in memory and belongs to the trigger's lifetime, not to the
   connection, so anything that ends or resets the trigger takes the queue with
   it. Known cases: overflow past 100 discards the oldest; a single-shot
   trigger terminates on firing, after which its stream answers ``404``; the
   ``lifetime`` expiring discards the trigger's whole state; deleting the
   trigger, or restarting the gateway, does the same. Restart loses the queue
   even for a ``persistent`` trigger - persistence stores the trigger and its
   last observed value, never its pending events. Treat the buffer as a
   convenience across a reconnect, not as a delivery guarantee; if you need
   one, poll the underlying resource rather than relying on the stream.

   While no event is pending, the stream sends a comment line rather than a
   frame, every 15 seconds:

   .. code-block:: text

      :keepalive

   **TriggerEventFrame fields:**

   - ``timestamp`` (string) - ISO 8601 timestamp of when the event was generated
   - ``payload`` (object) - The observed resource's value at the moment the
     condition fired - the whole value, not the ``path`` sub-document the
     condition was evaluated against

   There is no ``error`` member. A trigger frame exists only because a
   condition fired, so there is no failed-evaluation case to report; a
   resource that cannot be read simply produces no frame. The *cyclic
   subscription* stream is the one that reports a failed sample inline - see
   ``SubscriptionEventFrame`` - and the two are easy to confuse because they
   are otherwise the same shape.

   The stream closes when:

   - The trigger's ``lifetime`` expires
   - The trigger is deleted
   - A one-shot trigger fires (``multishot: false``)
   - The client disconnects
   - The gateway shuts down
   - Maximum SSE client limit is reached (503 on connect)

   **Example:**

   .. code-block:: bash

      curl -N http://localhost:8080/api/v1/apps/temp_sensor/triggers/trig_001/events

   - **404** ``resource-not-found`` - Trigger not found or expired
   - **503** ``service-unavailable`` - Maximum SSE client limit reached

Trigger Conditions
~~~~~~~~~~~~~~~~~~

The ``trigger_condition`` object in the creation request specifies when the
trigger fires. Four standard condition types are supported:

.. list-table::
   :header-rows: 1
   :widths: 18 32 50

   * - Condition Type
     - Parameters
     - Behavior
   * - ``OnChange``
     - (none)
     - Fires whenever the current value differs from the previous value.
       First evaluation always fires.
   * - ``OnChangeTo``
     - ``target_value`` (any JSON value, required)
     - Fires when the current value equals the target AND differs from the
       previous value. First evaluation checks target only.
   * - ``EnterRange``
     - ``lower_bound`` (number, required), ``upper_bound`` (number, required)
     - Fires when a numeric value transitions from outside the inclusive range
       [lower_bound, upper_bound] to inside it. Requires a previous value
       (first evaluation does not fire).
   * - ``LeaveRange``
     - ``lower_bound`` (number, required), ``upper_bound`` (number, required)
     - Fires when a numeric value transitions from inside the inclusive range
       [lower_bound, upper_bound] to outside it. Requires a previous value
       (first evaluation does not fire).

Plugins can register custom condition evaluators with ``x-`` prefixed names
(e.g., ``x-threshold-count``) via the ``ConditionRegistry``.

Configuration
~~~~~~~~~~~~~

Configure triggers in ``gateway_params.yaml``:

.. code-block:: yaml

   ros2_medkit_gateway:
     ros__parameters:
       triggers:
         # Enable/disable the trigger subsystem (default: true)
         # When false, trigger endpoints return 501
         enabled: true

         # Maximum concurrent triggers across all entities (default: 1000)
         # Returns HTTP 503 when this limit is reached
         max_triggers: 1000

         # Behavior on gateway restart for persistent triggers
         # "reset": Clear all triggers on restart (default)
         # "restore": Reload persistent triggers from storage
         on_restart_behavior: "reset"

         # Trigger persistence storage
         storage:
           # Path to SQLite database for persistent triggers
           # Empty string = in-memory only (default)
           # Example: "/var/lib/ros2_medkit/triggers.db"
           path: ""

Persistence
~~~~~~~~~~~

Triggers created with ``"persistent": true`` are stored in a SQLite database.
On gateway restart, their behavior depends on the ``on_restart_behavior``
configuration:

- **reset** (default): All triggers are cleared on restart, regardless of
  the ``persistent`` flag. This is the safest option for development.
- **restore**: Persistent triggers are reloaded from the database. Their
  ``previous_value`` state is preserved, allowing range-based conditions
  (EnterRange, LeaveRange) to evaluate correctly without losing context.

Non-persistent triggers are always cleared on restart.

Restore happens once, while the gateway starts. The number of triggers it put
back is logged, so a restart that restored fewer than expected is visible in
the gateway log rather than only in a later 404.

The gateway also removes triggers whose entity has left discovery. A restored
trigger is exempt from that until its entity has been discovered at least once:
immediately after a restart nothing has been discovered yet, and an entity that
has merely not been reported yet has not disappeared. A restored trigger whose
entity never appears stays listed and can be deleted through the API.

A restored ``data`` trigger re-resolves its topic from the entity cache rather
than from the topic name it was stored with, and that attempt is governed by
the same rule as the record: while the entity has never been discovered the
attempt keeps running and never gives up, so a trigger whose entity takes
minutes to appear still subscribes and still fires. The gateway logs a warning
naming the trigger once the entity has been missing for longer than the
resolution budget, so an entity that never appears is visible rather than
silent. Once the entity has been discovered, the budget applies as usual and a
resource path that still cannot be resolved to a topic is given up on with a
warning naming the trigger.

Planned Stops
-------------

A **planned stop** is a window of wall-clock time during which faults are expected: a
changeover, a maintenance weekend, a line move. An operator declares one at runtime and the
gateway then reports every fault whose current cycle started inside it with
``x-medkit.expected`` - on ``GET /faults``, on the per-entity fault lists, on a fault's detail
document and on the ``/faults/stream`` frames.

Declaring a window changes **nothing** about how faults are handled. An expected fault is
confirmed, healed, cleared, captured and audited exactly as any other; no code path in the
fault manager consults a window. The flag is there so a reader can tell an expected fault from
a surprise, and the evidence chain still describes the plant that existed.

The windows live in the fault manager, next to the faults they describe, and survive a
restart. Retention is bounded by count (``planned_stop.max_windows``), never by age, and never
drops a window that is still running.

What makes a fault expected
~~~~~~~~~~~~~~~~~~~~~~~~~~~

A fault is expected when its ``first_occurred`` lies in ``[from, to]``, inclusive at both ends.
``first_occurred`` is the start of the fault's **current cycle**, not its first report ever: it
is reset when a FAILED event raises a CLEARED fault again. A code that failed during a
changeover and failed again after it was acknowledged is therefore expected for the first cycle
and a surprise for the second - which is what an operator asking "did anything break outside
the stop" means.

When several windows cover a fault, ``planned_stop_id`` names the **earliest-starting** one, so
the fault list and the event stream never disagree about which window applies.

In an aggregated deployment the windows are per fault manager. Each peer's gateway derives the
flag for its own faults and the aggregator relays what the peer said; there is no cross-peer
window logic.

.. list-table::
   :header-rows: 1
   :widths: 34 12 54

   * - Endpoint
     - Role
     - Description
   * - ``POST /api/v1/x-medkit-planned-stops``
     - operator
     - Declare a window. ``201`` with the stored window and a ``Location`` header
   * - ``GET /api/v1/x-medkit-planned-stops``
     - viewer
     - List windows, newest declaration first. ``?active=true`` keeps only the ones containing
       now
   * - ``GET /api/v1/x-medkit-planned-stops/{planned_stop_id}``
     - viewer
     - One window
   * - ``DELETE /api/v1/x-medkit-planned-stops/{planned_stop_id}``
     - operator
     - End a window early. ``200`` with the ended window, not ``204``: the caller has to be told
       where ``to`` landed

Declaring a window
~~~~~~~~~~~~~~~~~~

.. code-block:: bash

   curl -X POST http://localhost:8080/api/v1/x-medkit-planned-stops \
     -H 'Content-Type: application/json' \
     -d '{"from": "2026-09-06T18:00:00Z", "to": "2026-09-06T22:00:00Z", "reason": "line changeover"}'

.. code-block:: json

   {
     "id": "1788716982473405798-1",
     "from": "2026-09-06T18:00:00.000Z",
     "to": "2026-09-06T22:00:00.000Z",
     "reason": "line changeover",
     "declared_by": "shift_lead",
     "declared_at": "2026-09-06T17:58:12.004Z",
     "ended_early": false
   }

Request fields:

- ``from`` (optional) - ISO 8601 in UTC (``Z`` or ``+00:00``). Omit it for a stop that starts
  now. A window wholly in the past is accepted and marks the faults it covers: a stop is a fact
  about the plant, not about when someone typed it in.
- ``to`` (required) - ISO 8601 in UTC, strictly after ``from``. There is no maximum duration.
- ``reason`` (required) - carried verbatim on every fault the window marks.
- ``declared_by`` (optional) - defaults to the authenticated client id, or ``anonymous`` when
  authentication is off.

Windows may overlap. Refusals are all ``400``: ``to`` at or before ``from``, a time that is not
ISO 8601 in UTC (a real offset such as ``+02:00`` is refused rather than converted), and an
instant outside ``1970-01-01T00:00:00Z`` to ``2038-01-19T03:14:07Z``, which is the range a
``builtin_interfaces/Time`` can carry.

Ending a window early
~~~~~~~~~~~~~~~~~~~~~

.. code-block:: bash

   curl -X DELETE http://localhost:8080/api/v1/x-medkit-planned-stops/1788716982473405798-1

``to`` moves to the moment of the request and ``ended_early`` becomes true. Faults whose cycle
started before that instant stay expected; faults raised after it do not. A window whose end
has already passed answers ``400`` with vendor code ``x-medkit-planned-stop-ended`` - when a
stop actually finished is not something a later request gets to rewrite. An unknown id answers
``404``.

Reading the flag
~~~~~~~~~~~~~~~~

``GET /api/v1/faults`` (and every per-entity fault list) carries the flag on each item and the
tally beside them:

.. code-block:: json

   {
     "items": [
       {
         "fault_code": "MOTOR_STALL",
         "status": "CONFIRMED",
         "first_occurred": 1788716990.12,
         "x-medkit": {"expected": true, "planned_stop_id": "1788716982473405798-1"}
       }
     ],
     "x-medkit": {"count": 1, "expected_count": 1}
   }

``planned_stop_id`` is present only when ``expected`` is true. ``expected_count`` counts the
items this gateway derived a flag for; items merged from an aggregated peer carry the peer's
own flag and are not recounted.

Both fault lists take ``?expected=true|false|all``. Omitting it means ``all``: the flag is extra
information about a fault, never a reason to hide it by default.

The ``/faults/stream`` frames carry the same pair inside their ``x-medkit`` object, alongside
the ``entity_type`` / ``entity_id`` hint. ``expected`` is sent as an explicit ``false`` rather
than omitted, so a consumer can tell "not expected" from "this gateway could not say".

Fault Triggers (threshold rules)
--------------------------------

Fault triggers are runtime threshold rules on an app's discovered data points:
the engine polls each rule's value and reports a fault while the value stays
crossed (level-triggered), then auto-clears it when the value recovers. They
are a sibling of the SOVD notification ``/triggers`` collection above and never
overload it; rules persist to a JSON store and survive a restart.

The routes are part of the generated OpenAPI spec (``/api/v1/docs``, tag
``FaultTriggers``), so generated clients and Swagger UI discover them the same
way as every other endpoint.

The engine runs only when ``fault_triggers.enabled`` is true *and* at least one
plugin is loaded. Without it the routes stay mounted and answer ``501``
(``not-implemented``) - the same shape the ``/updates`` and ``/triggers`` gates
use, so a client can tell "this build has no threshold engine" apart from "no
such app or rule".

``GET /api/v1/apps/{app_id}/fault-triggers``
   List the app's rules. The owning app is the one in the path; it is not
   repeated in the item, and neither is the engine's internal cross latch.

   .. code-block:: json

      {
        "items": [
          {
            "id": "ftr_1",
            "data_name": "level",
            "operator": ">=",
            "threshold": 80.0,
            "fault_code": "TANK_OVERFILL",
            "severity": "CRITICAL",
            "active": true
          }
        ]
      }

``POST /api/v1/apps/{app_id}/fault-triggers``
   Create a rule. Required: ``data_name``, ``operator`` (``>``, ``<``, ``>=``,
   ``<=``, ``==``), ``threshold`` (number), ``fault_code``, ``severity``
   (``INFO``/``WARNING``/``ERROR``/``CRITICAL``). Optional: ``active``
   (default ``true``). Returns ``201`` with the created rule and a ``Location``
   header pointing to it.

   Validation: ``400`` (``invalid-parameter``) for missing/invalid fields or a
   ``data_name`` the app does not expose (when enumerable); ``404``
   (``entity-not-found``) when the app itself was never discovered; ``409``
   (``precondition-not-fulfilled``) when the ``fault_code`` is already used by
   another rule - fault codes are global to the fault store, so two rules
   sharing one would fight over the same fault.

``DELETE /api/v1/apps/{app_id}/fault-triggers/{trigger_id}``
   Remove a rule (``204``). A fault currently asserted by the rule is cleared;
   the correlation cascade is skipped so the clear stays scoped to the rule's
   own fault.

.. _rate-limiting:

Rate Limiting
-------------

The gateway supports token-bucket-based rate limiting to protect endpoints from abuse. Rate limiting is disabled by default and can be enabled via configuration parameters.

Configuration
~~~~~~~~~~~~~

You can configure global and per-client RPM (requests per minute) limits:

- ``rate_limiting.enabled``: ``true`` to enable.
- ``rate_limiting.global_requests_per_minute``: Overarching limit across all clients.
- ``rate_limiting.client_requests_per_minute``: Limit per individual client IP.

Endpoint limits can also be overridden with patterns:

- ``rate_limiting.endpoint_limits``: List of ``"pattern:rpm"`` strings. For example, ``["/api/v1/*/operations/*:10"]`` limits execution calls without affecting other data endpoints.

Response Headers
~~~~~~~~~~~~~~~~

When rate limiting is enabled, the gateway includes the following HTTP response headers on every check:

- ``X-RateLimit-Limit``: The effective RPM limit applied.
- ``X-RateLimit-Remaining``: Number of requests remaining in the current minute window.
- ``X-RateLimit-Reset``: Unix epoch time (in seconds) when the limit bucket resets.

Rejection (429 Too Many Requests)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

If a request exceeds the available tokens, it is rejected with an HTTP 429 status code and a ``Retry-After`` header indicating the number of seconds to wait before retrying.

**Example Response:**

.. code-block:: json

   {
     "error_code": "rate-limit-exceeded",
     "message": "Too many requests. Please retry after 10 seconds.",
     "parameters": {
       "retry_after": 10,
       "limit": 60,
       "reset": 1739612355
     }
   }

.. _rest-authentication:

Authentication Endpoints
------------------------

JWT-based authentication with Role-Based Access Control (RBAC).

The ``/auth/*`` endpoints, and the authentication middleware guarding every
other route, answer errors in the RFC 6749 section 5.2 shape rather than the
SOVD ``GenericError`` used everywhere else:

.. code-block:: json

   {
     "error": "invalid_grant",
     "error_description": "Refresh token is expired or unknown"
   }

``/auth/authorize`` and ``/auth/token`` accept the request body as either
``application/json`` or ``application/x-www-form-urlencoded``, the encoding
RFC 6749 clients default to. ``/auth/revoke`` accepts JSON only, and per
RFC 7009 section 2.2 answers ``200`` whether or not the submitted token was
valid - so it never returns ``401``.

These three endpoints are the only ones the middleware lets through
unauthenticated whatever ``require_auth_for`` says - a caller has to be able to
obtain a token before it has one. They are also the only operations the served
OpenAPI document publishes with an empty ``security: []`` requirement; every
other operation names the role the gateway's permission table grants for its
path, so ``GET /api/v1/docs`` is where a client reads which role an endpoint
needs. See :ref:`rest-role-required` below.

.. seealso::

   :doc:`/tutorials/authentication` for configuration details.

.. _rest-role-required:

Which role an endpoint needs
----------------------------

Every route declares its weakest permitted caller where it is registered, and
that one declaration produces both the entries the middleware matches against
and the ``security`` requirement published for the operation. The served
document is therefore the reference: read
``paths.<path>.<method>.security[0].bearerAuth[0]`` from
``GET /api/v1/docs``.

The shape of the assignment:

* ``viewer`` - every ``GET`` the gateway itself serves, including the SSE
  streams, the bulk-data downloads and the capability descriptions. Routes a
  plugin mounts are the exception and are ``admin`` whatever their method (see
  below).
* ``operator`` - runtime writes: operation executions, clearing faults,
  publishing data, locks, cyclic subscriptions, triggers, fault-trigger rules,
  bulk-data upload and delete, starting and controlling script executions, and
  the ``start`` / ``restart`` / ``force-restart`` lifecycle transitions.
* ``configurator`` - changes to how the system is configured: configuration
  writes and resets, log configuration, script upload and delete, the
  ``/updates`` write verbs (register, prepare, execute, automated, delete -
  reading an update or its status is ``viewer``), and the ``shutdown`` /
  ``force-shutdown`` transitions.
* ``admin`` - everything above, plus every route mounted outside the route
  registry. That is what covers plugin-served routes, which no per-route
  declaration describes; they publish ``admin`` and nothing weaker reaches
  them.

Enforcement fails closed - a path no entry matches is refused - so the
published role is what the gateway demands rather than a separate claim about
it. What is enforced at all is a deployment setting: with ``auth.enabled``
false no role is published or checked, and with ``require_auth_for: write`` a
``GET`` is served without a token even though its operation names a role.

``POST /api/v1/auth/authorize``
   Authenticate with client credentials.

   **Request:**

   .. code-block:: json

      {
        "grant_type": "client_credentials",
        "client_id": "admin",
        "client_secret": "admin_secret_key"
      }

   **Response:**

   .. code-block:: json

      {
        "access_token": "eyJhbGciOiJIUzI1NiIs...",
        "token_type": "Bearer",
        "expires_in": 3600,
        "refresh_token": "dGhpcyBpcyBhIHJlZnJlc2g...",
        "scope": "admin"
      }

``POST /api/v1/auth/token``
   Refresh access token.

   **Request:**

   .. code-block:: json

      {
        "grant_type": "refresh_token",
        "refresh_token": "dGhpcyBpcyBhIHJlZnJlc2g..."
      }

``POST /api/v1/auth/revoke``
   Revoke a token.

   **Request:**

   .. code-block:: json

      {"token": "dGhpcyBpcyBhIHJlZnJlc2g..."}

Vendor Extension Endpoints (Plugins)
-------------------------------------

Plugin-registered endpoints use the ``x-medkit-`` prefix following the SOVD vendor extension
mechanism. These endpoints are only available when the corresponding plugin is loaded
(see :doc:`/tutorials/linux-introspection`).

.. warning::

   The procfs plugin exposes process command lines (``/proc/{pid}/cmdline``) via HTTP.
   Command lines may contain sensitive data (API keys, passwords passed as arguments).
   Enable authentication when using the procfs plugin in production environments.

.. note::

   Vendor extension endpoints are registered dynamically by plugins. They do not appear in
   the ``GET /`` root endpoint list. Use entity capability responses (``GET /apps/{id}``,
   ``GET /components/{id}``) to discover available extensions via the ``capabilities`` field.

Linux Process Introspection (x-medkit-procfs)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Requires: ``procfs_introspection`` plugin.

``GET /api/v1/apps/{id}/x-medkit-procfs``
   Get process-level information for a single app.

   **Response 200:**

   .. code-block:: json

      {
        "pid": 1234,
        "ppid": 1,
        "state": "S",
        "exe": "/usr/bin/talker",
        "cmdline": "/usr/bin/talker --ros-args __node:=talker __ns:=/demo",
        "rss_bytes": 524288,
        "vm_size_bytes": 2097152,
        "threads": 4,
        "cpu_user_ticks": 1520,
        "cpu_system_ticks": 340,
        "cpu_user_seconds": 15.2,
        "cpu_system_seconds": 3.4,
        "uptime_seconds": 123.45
      }

   - **404:** Process not found (node not running or PID cache miss)
   - **503:** Failed to read process information

``GET /api/v1/components/{id}/x-medkit-procfs``
   Aggregate process info for all apps in the component. Processes are
   deduplicated by PID (multiple nodes in the same process appear once).

   **Response 200:**

   .. code-block:: json

      {
        "processes": [
          {
            "pid": 1234,
            "node_ids": ["talker", "listener"],
            "...": "same fields as app endpoint"
          }
        ]
      }

Systemd Unit Introspection (x-medkit-systemd)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Requires: ``systemd_introspection`` plugin and ``libsystemd``.

``GET /api/v1/apps/{id}/x-medkit-systemd``
   Get systemd unit information for the app's process.

   **Response 200:**

   .. code-block:: json

      {
        "unit": "ros2-talker.service",
        "unit_type": "service",
        "active_state": "active",
        "sub_state": "running",
        "restart_count": 2,
        "watchdog_usec": 5000000
      }

   ``restart_count`` and ``watchdog_usec`` are only meaningful for service units.
   For other unit types (timer, mount, etc.) they are always 0.

   - **404:** Process not found or not managed by a systemd unit
   - **503:** Failed to query systemd properties

``GET /api/v1/components/{id}/x-medkit-systemd``
   Aggregate systemd unit info for all apps in the component. Units are
   deduplicated by unit name.

   **Response 200:**

   .. code-block:: json

      {
        "units": [
          {
            "unit": "ros2-talker.service",
            "node_ids": ["talker", "listener"],
            "...": "same fields as app endpoint"
          }
        ]
      }

Container Introspection (x-medkit-container)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Requires: ``container_introspection`` plugin. Supports the unified cgroup
hierarchy (v2), the legacy one (v1), and hybrid hosts, under both the
``host`` and ``private`` cgroup namespace modes.

``GET /api/v1/apps/{id}/x-medkit-container``
   Get container information for the app's process.

   **Response 200:**

   .. code-block:: json

      {
        "container_id": "a1b2c3d4e5f6...",
        "runtime": "docker",
        "memory_limit_bytes": 1073741824,
        "memory_limit_state": "limited",
        "cpu_quota_us": 100000,
        "cpu_period_us": 100000,
        "cpu_quota_state": "limited"
      }

   Fields ``memory_limit_bytes`` and ``cpu_quota_us`` are present only when a limit is in
   force. ``cpu_period_us`` is present whenever the CPU limit was read at all, including
   when the quota is unlimited. ``memory_limit_state`` and ``cpu_quota_state`` are
   always present and carry one of ``limited``, ``unlimited``, ``unreadable`` or
   ``unavailable``, so a client can tell an unconstrained container from one whose limit
   files could not be read. ``cpu_quota_state`` covers the quota and its period together.

   ``cpu_quota_us`` is the CFS bandwidth limit. It does not reflect the set of CPUs the
   container is pinned to (``--cpuset-cpus``), which is visible only through
   ``sched_getaffinity()``; the effective CPU budget needs both.

   ``container_id`` is empty when the cgroup namespace hides it (``--cgroupns=private``
   reports the namespace root as the path). The container is still recognised from the
   markers its runtime leaves behind, and the limits are still reported.

   - **404:** Process not found or not running in a container
   - **503:** The cgroup of the process could not be determined at all

``GET /api/v1/components/{id}/x-medkit-container``
   Aggregate container info for all apps in the component. Containers are
   deduplicated by container ID.

   **Response 200:**

   .. code-block:: json

      {
        "containers": [
          {
            "container_id": "a1b2c3d4e5f6...",
            "node_ids": ["talker", "listener"],
            "...": "same fields as app endpoint"
          }
        ]
      }

x-medkit-topic-beacon
~~~~~~~~~~~~~~~~~~~~~

Provided by the ``ros2_medkit_topic_beacon`` plugin (not available when the
plugin is not loaded). Returns beacon metadata for an entity populated from
``MedkitDiscoveryHint`` messages published by the entity's node via a ROS 2
topic (push-based).

``GET /api/v1/apps/{id}/x-medkit-topic-beacon``

``GET /api/v1/components/{id}/x-medkit-topic-beacon``

**Example:**

.. code-block:: bash

   curl http://localhost:8080/api/v1/apps/engine_temp_sensor/x-medkit-topic-beacon

**Response (200 OK):**

.. code-block:: json

   {
     "entity_id": "engine_temp_sensor",
     "status": "active",
     "age_sec": 1.234,
     "stable_id": "",
     "display_name": "Engine Temperature Sensor",
     "transport_type": "shared_memory",
     "negotiated_format": "",
     "process_id": 12345,
     "process_name": "sensor_node",
     "hostname": "robot-1",
     "component_id": "powertrain",
     "function_ids": ["monitoring"],
     "depends_on": [],
     "metadata": {"custom_key": "custom_value"}
   }

**Status values:**

- ``active`` - Hint is within the configured ``beacon_ttl_sec``
- ``stale`` - Hint is past TTL but within ``beacon_expiry_sec``

**Notes:**

- ``age_sec`` is the elapsed time in seconds since the last hint was received.
- When no beacon data exists for an entity, the endpoint returns 404 with
  error code ``x-medkit-beacon-not-found`` (not an ``"unknown"`` status).

**Response Codes:**

- **200 OK** - Beacon data found and returned
- **404 Not Found** (code: ``ERR_ENTITY_NOT_FOUND``) - Entity does not exist
- **404 Not Found** (code: ``x-medkit-beacon-not-found``) - Entity exists but no beacon data received

x-medkit-param-beacon
~~~~~~~~~~~~~~~~~~~~~

Provided by the ``ros2_medkit_param_beacon`` plugin (not available when the
plugin is not loaded). Returns beacon metadata for an entity populated by
polling ROS 2 node parameters matching a configured prefix (pull-based).

``GET /api/v1/apps/{id}/x-medkit-param-beacon``

``GET /api/v1/components/{id}/x-medkit-param-beacon``

**Example:**

.. code-block:: bash

   curl http://localhost:8080/api/v1/apps/engine_temp_sensor/x-medkit-param-beacon

**Response (200 OK):**

The response schema is identical to ``x-medkit-topic-beacon``. See above for
the full field listing.

**Response Codes:**

- **200 OK** - Beacon data found and returned
- **404 Not Found** (code: ``ERR_ENTITY_NOT_FOUND``) - Entity does not exist
- **404 Not Found** (code: ``x-medkit-beacon-not-found``) - Entity exists but no beacon data received

Error Responses
---------------

Every error carries the SOVD ``GenericError`` body - a flat object, not a
nested ``error`` envelope:

.. code-block:: json

   {
     "error_code": "entity-not-found",
     "message": "Entity not found",
     "parameters": {
       "entity_id": "unknown_component"
     }
   }

``error_code`` and ``message`` are always present. ``parameters`` is
cause-specific and omitted when there is nothing to add.

A vendor-specific failure carries a **fourth** key. The gateway rewrites
``error_code`` to the sentinel ``vendor-error`` and moves the real
``x-medkit-*`` code into ``vendor_code``, so a generic SOVD client sees a code
it knows while the precise one stays available:

.. code-block:: json

   {
     "error_code": "vendor-error",
     "vendor_code": "x-medkit-gateway-shutdown",
     "message": "Gateway is shutting down"
   }

Match on ``error_code``, and on ``vendor_code`` when ``error_code`` is
``vendor-error``. Do not match on ``message`` - it is prose and changes.

The ``/auth/*`` endpoints are the one exception: they answer RFC 6749
section 5.2 ``{"error": "...", "error_description": "..."}`` instead, as does
the authentication middleware on the 401 and 403 it returns ahead of any
route. See :ref:`rest-authentication`.

Common Error Codes
~~~~~~~~~~~~~~~~~~

Standard SOVD codes appear in the response's ``error_code`` field.
Vendor-specific ``x-medkit-*`` codes are enveloped: the response carries
``error_code: "vendor-error"`` with the precise code in ``vendor_code``.

.. list-table::
   :header-rows: 1
   :widths: 30 15 55

   * - Error Code
     - HTTP Status
     - Description
   * - ``entity-not-found``
     - 404
     - The requested entity does not exist
   * - ``resource-not-found``
     - 404
     - The requested resource (topic, service, parameter) does not exist
   * - ``operation-not-found``
     - 404
     - The named operation does not exist on this entity
   * - ``invalid-request``
     - 400
     - Malformed request body (not valid JSON, or not an object). The same code
       also appears below with a 409, on a lock acquire collision - read the
       status, not the code alone, to tell the two apart.
   * - ``invalid-parameter``
     - 400
     - A field or query parameter failed validation. ``parameters.parameter``
       names the offending one.
   * - ``collection-not-supported``
     - 400
     - This entity type does not serve the requested resource collection
   * - ``precondition-not-fulfilled``
     - 409
     - The request conflicts with current state (e.g. a duplicate
       ``fault_code`` on a fault-trigger rule)
   * - ``lock-broken``
     - 409
     - A guarded write was refused because another client holds a lock on the
       entity. ``parameters.lock_id`` names it.
   * - ``invalid-request``
     - 409
     - The request conflicts with the current state of the resource. This is
       **not** lock-specific - operations use it to refuse re-executing a
       running operation, for instance - so do not assume lock semantics or a
       lock-shaped ``parameters``. ``message`` identifies the conflict and
       ``parameters`` varies with it. For the lock cases and what each one
       carries, see :ref:`the lock refusal table <rest-lock-refusals>`. The
       same code also appears with 400 for a malformed body - see the row
       above.
   * - ``insufficient-access-rights``
     - 403
     - A lifecycle provider refused the transition (``AccessDenied``). Not a
       lock error - the lock routes never emit this code.
   * - ``forbidden``
     - 403
     - The caller is not the owner of the lock it tried to release or extend
       (``LockManager``'s ``lock-not-owner``).
   * - ``payload-too-large``
     - 413
     - Upload exceeds the configured size limit
   * - ``not-implemented``
     - 501
     - The feature is not enabled, or no backend is configured for it
   * - ``service-unavailable``
     - 503
     - A backing service (fault store, subscription manager) refused
   * - ``not-responding``
     - 504
     - The underlying ROS 2 entity did not respond in time; the outcome of
       the request is unknown
   * - ``rate-limit-exceeded``
     - 429
     - Client exceeded its request quota
   * - ``internal-error``
     - 500
     - Unhandled failure. ``parameters.details`` carries the cause.
   * - ``vendor-error``
     - varies
     - A vendor-specific failure; read ``vendor_code`` for the real code
   * - ``x-medkit-plugin-error``
     - 400-599
     - Plugin provider returned an error. Status varies by plugin. Message truncated to 512 chars.
   * - ``x-medkit-gateway-shutdown``
     - 503
     - Gateway is in the process of shutting down. **Do not retry against the same
       gateway instance** - the process is going away. Clients should fail over to
       another gateway or surface the outage to the operator.
   * - ``x-medkit-subscribe-failed``
     - 500
     - Could not create the underlying ROS 2 subscription (rcl error during slot
       creation). Transient: retry once after a short backoff. Persistent failure
       usually indicates a publisher type mismatch or a missing IDL package.
   * - ``x-medkit-resource-sample-failed``
     - n/a
     - A cyclic subscription's sampler could not read the resource on this
       tick. Delivered inside the SSE frame's ``error`` object, never as an
       HTTP status; the stream stays open and the next tick is retried.
   * - ``x-medkit-cold-wait-cap-exceeded``
     - 503
     - Too many concurrent /data callers are waiting on cold (publisher-but-no-data)
       topics. Retry with exponential backoff. ``params.cold_wait_cap`` carries the
       configured cap. Tune via ``data_provider.cold_wait_cap`` and
       ``data_provider.max_parallel_samples`` if this fires under normal load.
   * - ``x-medkit-ros2-topic-unavailable``
     - 404
     - The data resource names a topic the ROS 2 graph does not currently have
   * - ``x-medkit-ros2-service-unavailable``
     - 500
     - A ROS 2 service call backing an operation failed
   * - ``x-medkit-ros2-action-rejected``
     - 400
     - A ROS 2 action server rejected the goal
   * - ``x-medkit-ros2-action-unavailable``
     - 500
     - A ROS 2 action execution failed
   * - ``x-medkit-ros2-parameter-read-only``
     - 403
     - The configuration parameter is declared read-only on the node
   * - ``x-medkit-update-not-found``
     - 404
     - No update package with that id
   * - ``x-medkit-update-already-exists``
     - 400
     - An update package with that id is already registered
   * - ``x-medkit-update-in-progress``
     - 409
     - Another update is executing, or this one is being deleted
   * - ``x-medkit-update-not-prepared``
     - 400
     - ``execute`` was called before ``prepare`` completed
   * - ``x-medkit-update-not-automated``
     - 400
     - ``automated`` was requested on a package whose ``automated`` is false
   * - ``x-medkit-script-already-exists``
     - 409
     - A script with that id already exists
   * - ``x-medkit-managed-script``
     - 409
     - The script is manifest-managed and cannot be modified over REST
   * - ``x-medkit-script-running``
     - 409
     - The script has a running execution and cannot be deleted
   * - ``x-medkit-script-not-running``
     - 409
     - A control action was sent to an execution that is not running
   * - ``x-medkit-concurrency-limit``
     - 429
     - The script backend's concurrent-execution limit was reached
   * - ``x-medkit-script-too-large``
     - 413
     - The uploaded script exceeds the configured size limit
   * - ``x-medkit-ros2-node-unavailable``
     - 503
     - The node backing a configuration read or write did not answer in time
   * - ``x-medkit-invalid-resource-uri``
     - 400
     - A trigger or subscription ``resource`` URI does not parse
   * - ``x-medkit-entity-mismatch``
     - 400
     - A trigger or subscription ``resource`` URI names a different entity than
       the route it was posted to
   * - ``x-medkit-collection-not-supported``
     - 400
     - The ``resource`` URI names a collection triggers and subscriptions
       cannot observe
   * - ``x-medkit-collection-not-available``
     - 400
     - The ``resource`` URI names a collection this entity does not serve
   * - ``x-medkit-unsupported-protocol``
     - 400
     - The requested subscription ``protocol`` has no registered transport
   * - ``x-medkit-planned-stop-ended``
     - 400
     - The planned-stop window has already ended and cannot be ended again. Distinct from a 404 on an
       unknown id: the window is there, and when the stop actually finished is not something a later
       request gets to rewrite

The table is kept complete by a check rather than by review:
``scripts/check_error_codes_documented.py`` (ctest
``gateway_error_codes_documented``) fails if any ``ERR_*`` declared in
``error_codes.hpp`` and named anywhere in the gateway's sources, its headers,
or an in-tree plugin is missing from **this table** - a mention elsewhere in
this guide does not count. So the claim it backs is exactly: every error code
this repository can put on the wire appears above.

What that check does not reach, and this sentence therefore does not claim: a
third-party plugin may raise codes declared nowhere in this repository, and the
statuses and descriptions in the third column are read from the emitters by
hand. One code is excluded by name in the script, with its reason -
``x-medkit-internal-forwarded``, a framework sentinel the error writer returns
on before rendering anything.

Several codes are reached by more than one internal cause. Locking is where
that matters most, because its outcomes are spread across five rows above;
this is every refusal the lock routes can produce:

.. _rest-lock-refusals:

.. list-table:: Lock refusals, by internal cause
   :header-rows: 1
   :widths: 26 12 24 38

   * - Internal cause
     - Status
     - ``error_code``
     - Notes
   * - ``lock-conflict``
     - 409
     - ``invalid-request``
     - Entity already locked. Carries ``existing_lock_id``; retryable with
       ``break_lock``
   * - ``lock-not-breakable``
     - 409
     - ``invalid-request``
     - What a ``break_lock`` retry returns when the held lock forbids it. Also
       carries ``existing_lock_id``
   * - ``lock-not-owner``
     - 403
     - ``forbidden``
     - Releasing or extending a lock held by another client
   * - ``lock-not-found``
     - 404
     - ``resource-not-found``
     - No lock on the entity, or it has expired
   * - ``invalid-expiration``
     - 400
     - ``invalid-parameter``
     - Expiration or extension exceeding the configured maximum. The
       non-positive branch never gets this far - the handler rejects it first
       with its own ``invalid-parameter``
   * - ``lock-required``
     - 409
     - ``invalid-request``
     - The entity's manifest requires a lock for this collection and the caller
       holds none. Carries ``details``, ``entity_id`` and ``collection``, and -
       unlike the two rows above - **no** ``existing_lock_id``, because no lock
       exists to name
   * - (guarded write)
     - 409
     - ``lock-broken``
     - A write refused because another client's lock covers the collection.
       Always carries ``entity_id`` and ``collection``; carries ``lock_id`` only
       when the blocking lock could be identified. Comes from the request
       handler rather than from ``LockManager``

Three refusals the manager can construct are shadowed by an earlier handler
check and so do not reach a client in that form: ``lock-disabled`` (the lock
routes answer ``501`` ``not-implemented`` before ``LockManager`` is consulted),
unknown scope, and non-positive expiration (``LockHandlers`` validates both
against the same vocabulary first and emits its own ``400``
``invalid-parameter``, with ``parameters.invalid_scope`` naming the offending
scope).

On ``PUT`` / ``DELETE .../locks/{lock_id}`` the ``404`` a client normally meets
is the handler's - it checks the lock exists and belongs to the entity before
delegating, and its body carries ``lock_id`` and ``entity_id``. The manager's
own parameter-free ``404`` is still reachable in the narrow window where the
lock expires between those two lookups.

.. _rest-range-rejection:

Range Rejection (416)
~~~~~~~~~~~~~~~~~~~~~

Every operation in the OpenAPI document declares **416 Range Not Satisfiable**,
including operations that have nothing to do with file downloads. This is not
over-declaration. The HTTP layer parses the ``Range`` header before routing the
request, so a syntactically invalid ``Range`` is rejected before any handler
runs - on any path, including paths that do not exist:

.. code-block:: bash

   $ curl -i -H 'Range: furlongs=1-2' http://localhost:8080/api/v1/health
   HTTP/1.1 416 Range Not Satisfiable

The body is the usual ``GenericError`` shape, which is why the document
declares it as such rather than as a body-less response: the HTTP layer itself
writes 416 with an empty body, and the gateway's global error handler then
fills any body-less error response with a ``GenericError``.

Only the six bulk-data download routes declare a ``Range`` *request* parameter,
because they are the only routes where sending one is useful. 416 is
nevertheless reachable everywhere.

416 is not the only status answered this way, and the ``error_code`` in the
body it produces is a placeholder - see the next section.

.. _rest-framework-error-bodies:

Framework-Produced Error Bodies
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Some errors are answered by the HTTP layer itself, before any gateway handler
runs and sometimes before routing. cpp-httplib produces these with an **empty
body**, and the gateway's global error handler then fills any body-less error
response with a ``GenericError`` so that clients always receive the same
envelope. Statuses reaching a client this way include:

- **400** - malformed request line or headers, or an unparseable
  ``multipart/form-data`` boundary
- **413** - a form-urlencoded payload over the built-in length cap
- **414** - request URI too long
- **416** - unparseable ``Range`` header (see :ref:`rest-range-rejection`)
- **500** - an exception escaping a handler

**The ``error_code`` on these bodies is a placeholder.** The global handler
writes ``resource-not-found`` regardless of the actual status, because it runs
after the fact and has no way to know why the HTTP layer rejected the request.
So a 413 and a 416 both arrive carrying ``"error_code":
"resource-not-found"``. (The 404 an unrouted request produces goes through the
same path, where that code happens to be right - which is why the mismatch is
easy to miss on the statuses above.)

**Read the HTTP status, not the ``error_code``, whenever the status was not
produced by a handler.** The codes listed under `Common Error Codes`_ are
accurate only for errors the gateway itself raises. The ``parameters.status``
field on these bodies repeats the real status, which is the reliable field.

Plugin Entity Delegation
~~~~~~~~~~~~~~~~~~~~~~~~

Entities created by gateway plugins (via ``IntrospectionProvider``) have their
data, operations, and faults requests transparently routed to the owning plugin's
``DataProvider``, ``OperationProvider``, or ``FaultProvider``. The response format
is determined by the plugin. If the plugin returns an error, the response uses the
``x-medkit-plugin-error`` vendor code with an ``entity_id`` parameter identifying
the affected entity. See :doc:`/tutorials/plugin-system` for details on per-entity
provider routing.

URL Encoding
------------

Topic and parameter paths containing ``/`` must be URL-encoded:

.. list-table::
   :header-rows: 1
   :widths: 50 50

   * - Original Path
     - URL Encoded
   * - ``/powertrain/engine/temperature``
     - ``powertrain%2Fengine%2Ftemperature``
   * - ``/chassis/brakes/command``
     - ``chassis%2Fbrakes%2Fcommand``

.. _sovd-compliance:

SOVD Compliance
---------------

The gateway implements a **pragmatic subset** of the SOVD (Service-Oriented Vehicle
Diagnostics) standard. We follow SOVD where it matters for interoperability -
endpoint contracts, data model, entity hierarchy - but extend it where ROS 2
use cases benefit.

**SOVD-Aligned Capabilities:**

- Discovery (``/areas``, ``/components``, ``/apps``, ``/functions``)
- Data access (``/data``) with topic sampling and JSON serialization
- Operations (``/operations``, ``/executions``) with async action support
- Configurations (``/configurations``)
- Faults (``/faults``) with ``environment_data`` and SOVD status object
- Logs (``/logs``) with severity filtering and per-entity configuration
- Bulk Data (``/bulk-data``) with custom categories and rosbag downloads
- Software Updates (``/updates``) with async prepare/execute lifecycle
- Cyclic Subscriptions (``/cyclic-subscriptions``) with SSE-based delivery
- Scripts (``/scripts``) with upload, execution, and lifecycle management
- Triggers (``/triggers``) with condition-based push notifications

**Pragmatic Extensions:**

The SOVD spec defines resource collections only for apps and components. ros2_medkit
extends this to areas and functions where aggregation makes practical sense.

The matrix below transcribes ``EntityCapabilities::for_type``, which drives the
collection check in ``validate_collection_access_typed``. It is **not** where
the ``capabilities`` array of ``GET /{entity-type}/{id}`` comes from: that array
is built from a second, independent list, the ``CapabilityBuilder::Capability``
vector each handler in ``discovery_handlers.cpp`` assembles. The two surfaces
overlap but are not the same set - the component array also carries ``status``,
``subcomponents``, ``hosts`` and ``depends-on``, and the area array
``subareas``, ``contains`` and ``components``, none of which are resource
collections and so none of which appear in the table.

Nor is it what an entity's ``/docs`` sub-document lists. That document is a
projection of the routes the gateway registers (see `Capability Description
(OpenAPI Docs)`_ below), so a collection appears there when a route answers it
and not otherwise - the table cannot make it appear or disappear.

The transcription is by hand. What is checked mechanically is the property the
table exists to describe:
``test_openapi_contract::test_every_advertised_collection_is_served`` takes the
first discovered entity of each type, follows every non-templated ``href`` in
its ``capabilities`` array **and** every path in its ``/docs`` sub-document that
declares a ``GET`` against a live gateway, and fails on a 404 - so it covers
both surfaces, including where they disagree. Paths with no ``GET`` are skipped
because a 404 there says nothing: ``PUT /{type}/{id}/status/restart`` has no GET
to answer. Its fixture discovers no areas, so the Areas column below is covered
by the ``EntityCapabilities`` unit tests instead, which assert the per-type
lists directly. ``501`` is a served answer, not a missing one: see
``data-categories`` and ``data-groups`` below.

Collections named by the SOVD standard that the gateway does **not** serve
per entity - ``data-lists``, ``modes`` and ``communication-logs`` - are absent
from the table and from every capability list. ``updates`` is server-scoped
only (``/api/v1/updates``), never mounted under an entity.

.. list-table:: Resource Collection Support Matrix
   :header-rows: 1
   :widths: 20 16 16 16 16 16

   * - Resource
     - Areas
     - Components
     - Apps
     - Functions
     - SOVD Spec
   * - data
     - aggregated
     - yes
     - yes
     - aggregated
     - apps, components
   * - data-categories
     - 501
     - 501
     - 501
     - 501
     - apps, components
   * - data-groups
     - 501
     - 501
     - 501
     - 501
     - apps, components
   * - operations
     - aggregated
     - yes
     - yes
     - aggregated
     - apps, components
   * - configurations
     - aggregated
     - yes
     - yes
     - aggregated
     - apps, components
   * - faults
     - aggregated
     - yes
     - yes
     - aggregated
     - apps, components
   * - logs
     - prefix match
     - prefix match
     - exact match
     - from hosts
     - apps, components
   * - bulk-data
     - read-only
     - full CRUD
     - full CRUD
     - read-only
     - apps, components
   * - cyclic-subscriptions
     - \-
     - yes
     - yes
     - yes
     - apps, components
   * - scripts
     - \-
     - yes
     - yes
     - \-
     - apps, components
   * - locks
     - \-
     - yes
     - yes
     - \-
     - apps, components
   * - triggers
     - yes (x-medkit)
     - yes
     - yes
     - yes (x-medkit)
     - apps, components
   * - fault-triggers
     - \-
     - \-
     - yes (x-medkit)
     - \-
     - not in SOVD

Three rows depend on configuration, and the two advertising surfaces answer
differently, which is worth stating rather than leaving to be discovered:

- ``locks``: the routes are always registered for components and apps and answer
  ``501`` when there is no lock manager (``locking.enabled`` off). The
  ``capabilities`` entry and the ``locks`` URI field follow the lock manager; the
  ``/docs`` sub-document lists ``/locks`` unconditionally, because registration
  is unconditional and the sub-document reports registrations.
- ``scripts``: the same shape. ``ScriptManager`` is constructed unconditionally,
  so all eight script routes are always registered for components and apps, and
  they answer ``501`` until a backend exists - either a plugin
  ``ScriptProvider`` or a non-empty ``scripts.scripts_dir``. The
  ``capabilities`` entry follows the backend; the sub-document lists
  ``/scripts`` unconditionally.
- ``fault-triggers``: always registered and always advertised, for apps only;
  with no fault-trigger engine running the routes answer ``501``.

Other extensions beyond SOVD:

- Vendor extension fields using ``x-medkit`` prefix (per SOVD extension mechanism)
- ``DELETE /faults`` - Clear all faults globally
- ``GET /faults/stream`` - SSE real-time fault notifications. Each event payload carries an
  optional ``x-medkit`` SOVD payload-extension object with ``entity_type`` and ``entity_id``
  fields when the gateway can resolve the fault's first reporting source back to an entity,
  so consumers can hit ``/{entity_type}/{entity_id}/bulk-data/rosbags/{fault_code}`` directly
  without enumerating entities - that address serves the fault's newest recording. To reach an
  older one, list ``/bulk-data/rosbags`` and use the descriptor ``id``. Resolution is snapshotted at event arrival; the entire
  ``x-medkit`` object is omitted when no entity can be resolved.
- ``/health`` - Health check with discovery pipeline diagnostics
- ``/version-info`` - Gateway version information
- ``/docs`` - OpenAPI capability description
- SSE fault streaming - Real-time fault notifications
- ``x-medkit`` extension fields in responses

**Cross-Gateway Resource Aggregation:**

When aggregation is enabled, per-entity resource collection endpoints perform
real-time fan-out to peer gateways. The affected endpoints are: data,
operations, faults, configurations, logs, and the global ``GET /api/v1/faults``
endpoint. ``GET /api/v1/faults/stream`` also reaches every healthy peer, but it
is not an ``items`` merge: it holds one connection open per peer for as long as
a client is attached. See the fault-streaming section below. The gateway sends the same request to all healthy peers, merges their
``items`` arrays into the local response, and returns the combined result.

If some peer requests fail during fan-out (peer unreachable or non-2xx
response), the response includes vendor metadata indicating partial results:

.. code-block:: json

   {
     "items": [],
     "x-medkit": {
       "partial": true,
       "failed_peers": ["secondary_gateway"],
       "peer_failures": [{"peer": "secondary_gateway", "reason": "timeout"}]
     }
   }

``failed_peers`` names which peers contributed nothing. ``peer_failures`` says
why, one entry per name, with ``reason`` one of:

.. list-table::
   :header-rows: 1
   :widths: 20 80

   * - ``reason``
     - Meaning
   * - ``timeout``
     - A budget ran out. Which one is named in the message: the connect budget
       means the peer never accepted the connection, a read or write budget
       means it did and the gateway stopped waiting.
   * - ``unreachable``
     - No usable answer arrived: connection refused, no route, TLS or redirect
       failure, or the peer died while the answer was arriving.
   * - ``canceled``
     - This gateway stopped the call, which happens during its own shutdown.
   * - ``error-status``
     - The peer answered with a status outside 2xx.
   * - ``too-large``
     - The peer's body passed the peer-response size limit.
   * - ``invalid-response``
     - The peer answered with something that is not the JSON the route
       promises.

Without ``peer_failures`` a client cannot tell a busy subsystem from a dead
one, which is the whole of what a partial answer is asked.

When all peers respond successfully, these fields are omitted. See the
:doc:`aggregation configuration guide </config/aggregation>` for setup details.

**A forwarded request that runs out of time.** ``GET``, ``POST``, ``PUT``,
``DELETE`` and ``PATCH`` on a resource a peer owns are proxied to that peer.
Two outcomes that used to be one:

.. list-table::
   :header-rows: 1
   :widths: 12 30 58

   * - Status
     - ``error_code``
     - When
   * - ``504``
     - ``not-responding``
     - A budget ran out before the peer answered. The message names which one
       and its value, because the connect, read and write budgets are three
       different keys and only one of them was the problem. The request may
       still be running on the peer.
   * - ``502``
     - ``vendor-error`` / ``x-medkit-peer-unavailable``
     - Nothing answered - refused, no route, or the peer died mid-answer.

**A local operation that runs out of time.** ``POST
/{entity}/operations/{op}/executions`` answers ``504`` ``not-responding`` when
the backing ROS 2 service or action did not answer inside
``service_call_timeout_sec``, with a message naming the endpoint and
the budget; ``503`` when the endpoint is not on the graph at all; and ``500``
for any other transport failure. Previously all three answered ``500`` with a
fixed ``"Service call failed"`` / ``"Action execution failed"``, and a client
had to match on the text in ``parameters.details`` to learn it was a timeout.

**Fault streaming on an aggregating gateway.** ``GET /faults/stream`` on a
gateway with peers relays their streams into its own. Each relayed event
carries ``x-medkit.peer`` naming the gateway that raised the fault. The relay
is open only while a client is attached, because it holds one SSE client slot
on each peer and ``sse.max_clients`` defaults to 2. Replay via
``Last-Event-ID`` covers this gateway's own ids: two peers number their events
independently, so a reconnecting client resumes from what the aggregator has
buffered rather than from each peer's own history. A request carrying
``X-Medkit-No-Fan-Out`` is served from the local graph only, which is what
stops a chain of aggregating gateways relaying one event round the loop.

Because those relayed connections count against the peer's own
``sse.max_clients``, ``GET /health`` now reports how much of that cap is in
use:

.. code-block:: json

   {
     "status": "healthy",
     "x-medkit-sse": {"connected_clients": 1, "max_clients": 2}
   }

Without it, an operator refused a stream with ``503`` on a gateway nobody
appears to be watching has nothing to look at. The object is omitted when the
gateway has no SSE client tracker.

``GET /{entity}/configurations`` applies the same honesty to its *local*
backing nodes. This only arises for entities backed by more than one ROS 2
node; a single-node entity has no partial state, since its one node either
answers or the whole request fails. When several backing nodes contribute and
some answer while others do not, the response stays ``200`` but flags itself
``partial`` and names the failed nodes: nodes that were down or unresponsive
(``503``-class) in ``unavailable_nodes``, any other per-node failure in
``failed_nodes``. The parameters of the nodes that *did* answer are still
returned - a partial list never discards them:

.. code-block:: json

   {
     "items": [
       {"id": "use_sim_time", "name": "use_sim_time", "type": "parameter"}
     ],
     "x-medkit": {
       "partial": true,
       "unavailable_nodes": ["/down_node"]
     }
   }

This matches ``GET /{entity}/configurations/{param}``, which returns ``503``
``x-medkit-ros2-node-unavailable`` for the same backing-node outage.

When every backing node responds, ``unavailable_nodes`` and ``failed_nodes`` are
omitted. Note that ``partial`` may still appear on its own from the peer
fan-out described above, without any local unavailable node.

When *no* backing node answers, the route fails rather than returning an empty
partial list: it surfaces the highest-severity per-node failure, so a
node-unavailability outage returns ``503`` while a purely internal failure -
for example every backing node reporting shutdown - returns ``500``. This route
could not return ``500`` before; clients that previously saw ``503`` for a
manager shutdown now see ``500``.

Capability Description (OpenAPI Docs)
--------------------------------------

The gateway provides self-describing OpenAPI 3.1.0 capability descriptions at any level
of the API hierarchy. Append ``/docs`` to any valid path to receive a context-scoped
OpenAPI spec describing the available operations at that level.

How much of that description is derived from the handlers rather than asserted
beside them - and, for each mechanism, what keeps the two from drifting apart -
is set out in :doc:`/design/ros2_medkit_gateway/openapi_derivation`.

Every scoped spec is a **projection of the root document**: the paths at or
below the requested path, with the ids the request named substituted into the
templates and the ``in: path`` parameters those substitutions answered removed.
For a projected path, what a scoped spec says about an operation is what the
root spec says about it - status codes, schemas, roles and all.

Which prefixes resolve at all is narrower than which paths the gateway serves.
``PathResolver`` recognises a fixed set of resource-collection keywords -
``data``, ``data-categories``, ``data-groups``, ``operations``, ``faults``,
``configurations``, ``logs``, ``bulk-data``, ``cyclic-subscriptions``,
``triggers``, ``updates``, ``hosts`` - and nothing else. ``locks``, ``status``,
``scripts`` and ``fault-triggers`` answer ``200`` on the collection and are
advertised as URI fields on the entity detail response, but
``<entity-path>/docs`` answers ``404`` for them. Read the root document for
those.

The concrete data and operation item paths described below are the one thing in
a scoped spec that is *built* from the entity cache rather than projected. What
they add is the path itself - one key per discovered topic, service and action,
which SOVD asks for and one ``/data/{data_id}`` registration cannot give - and
the ``x-sovd-*`` extensions that go with it.

.. warning::

   **They add no payload schema today, on any gateway.** All four sites that
   build a ``TopicData`` push an empty type
   (``thread_safe_entity_cache.cpp``), so a topic's ROS 2 type never reaches
   this builder in any discovery mode. That is structural, not a property of
   one fixture. The ``/data`` listing does resolve the type - it is under
   ``x-medkit.ros2.type`` and ``x-medkit.type_info`` on each item - by a path
   this projection does not use. Until that type is wired through, read the
   listing for a topic's shape and treat these paths as addresses rather than
   schemas.

Everything else a built item says is copied from the projected route it sits
beside, because the two are the *same route*: ``/apps/x/data/temperature`` is
served by the handler registered at ``/apps/{app_id}/data/{data_id}``. Four
things are copied - the declared ``security``, the responses, the
``x-medkit-lock-guarded`` marker, and the non-path parameters (in practice
``X-Client-Id``; the fan-out header is declared on collection *listing* routes,
which are never an item's sibling). Path parameters are not, because a concrete
path has no placeholder for them, and ``operationId`` is not, because it must
stay unique across the document.

So a built item declares the same ``401``/``403`` components the middleware
actually answers with, the same ``416``, and the whole lock contract together -
the ``409``, the marker and ``X-Client-Id``, which
:ref:`locking <locking-blocked-operations>` treats as one declaration. It
follows ``auth.require_auth_for`` and ``locking.enabled`` for the same reason:
the projection does, and this is a copy of it.

**Responses are copied too, including the 2xx.** The gateway envelopes every
read - ``GET .../data/{data_id}`` answers ``DataValue``, ``GET
.../operations/{operation_id}`` answers ``OperationDetail`` - so a body built
from the ROS 2 message or service-response type would be a second,
contradictory answer for one route rather than a more specific one. The request
body is copied on the same terms: a built ``PUT`` publishes its own envelope
only where the topic's type is known, and inherits ``$ref: DataWriteRequest``
otherwise, which today is always.

A built operation item carries a ``GET`` only. The gateway registers no
``POST`` at ``/{entity}/operations/{operation_id}`` - execution is
``POST /{entity}/operations/{operation_id}/executions``, which the projection
publishes beside it - so a ``POST`` on the concrete path answers ``404``. The
ROS service-response schema belongs to that execution result and is not
published anywhere today.

Both scopes work this way, and the generator has to know which it is in: a
scoped spec substitutes the ids it was given into the path **keys**, so at
``<entity>/data/docs`` the sibling is still ``/data/{data_id}`` while at
``<entity>/data/<topic>/docs`` it has already become ``/data/<topic>``. A built
item whose sibling is not found is discarded rather than published
un-inherited. Where a projection sits at that key - specific-resource scope -
the projection survives; where none does, the item is simply absent. That
second case is reachable: a nested path such as
``/areas/robot/components/robot-controller/data/temperature/docs`` projects
nothing, and answers with empty ``paths``.

``GET /api/v1/docs``
   Returns the full OpenAPI spec for the gateway root, including all server-level
   endpoints, entity collections, and global resources.

``GET /api/v1/{entity-collection}/docs``
   The subtree under the collection (e.g. ``/apps/docs``, ``/components/docs``):
   the listing, the entity detail template, and everything below it.

``GET /api/v1/{entity-type}/{entity-id}/docs``
   The subtree under one entity, with its id substituted - the detail endpoint
   and every resource route registered for that entity type. Templates deeper
   than the entity (``{data_id}``, ``{fault_code}``) stay templated and keep
   their parameters.

``GET /api/v1/{entity-type}/{entity-id}/{resource}/docs``
   The subtree under one resource collection. For ``data`` and ``operations``
   this also carries one concrete path per discovered topic / service / action,
   whose payload schema is generated from the ROS 2 type - the one thing a route
   registration cannot know, and the only part of any scoped spec that is not a
   projection.

Each scoped spec carries the ``components/schemas`` entries its own ``$ref``
chains reach, not the full DTO set the root spec ships.

**Features:**

- Specs include SOVD extensions: ``x-sovd-version`` on every spec, and
  ``x-sovd-data-category`` / ``x-sovd-name`` /
  ``x-sovd-cyclic-subscription-supported`` on the concrete data and operation
  item paths described above
- Each operation declares exactly one success status, derived from the handler's
  C++ return type. The few operations whose handler can genuinely answer with one
  of several success shapes (``POST .../operations/{operation_id}/executions``,
  ``DELETE .../faults/{fault_code}``,
  ``DELETE .../configurations``) carry ``x-medkit-alternates: true`` and list every
  alternative under its own status code. A generated client can therefore branch on
  status only where that marker is present.
- The concrete data and operation item paths in an entity-level or
  resource-level spec come from the runtime entity cache, so they change as the
  ROS 2 graph does
- Specs are cached per entity cache generation for performance. The cache holds
  each document serialized, not parsed, so what it costs in memory is close to
  what the document costs on the wire; it is bounded both by entry count and by
  total bytes, and is emptied whenever either bound is reached or the entity
  cache generation changes. A document larger than the whole byte budget is
  served but not cached. None of this is observable from a response: the two
  ``/docs`` routes answer ``application/json`` with the same 2-space-indented
  body whether it came from the cache or was just generated
- The two ``/docs`` routes are themselves in the root spec, as
  ``getCapabilityDescription`` (``/docs``) and ``getScopedCapabilityDescription``
  (``/{entity_path}/docs``). The second one's ``entity_path`` parameter spans
  several path segments - it is the whole prefix, e.g. ``apps/temp_sensor/data`` -
  so a generated client must send its slashes unescaped.
- Routes mounted by a loaded plugin are in the root spec too, provided the plugin
  exports ``describe_plugin_routes`` (see :doc:`/tutorials/plugin-system`). They
  carry ``x-medkit-plugin-served: true``, and the tag each one declares is added
  to the document's global tag list. A plugin route whose path lies under a
  scoped path appears in that scoped spec as well, for the same reason a
  registry route does - both are projected from the same merged set. Where a
  plugin describes a path the registry already holds, the registry's description
  is the one published. A plugin that does not export the symbol serves routes
  that appear nowhere in any spec.

**Configuration:**

- ``docs.enabled`` (bool, default: ``true``) - Set to ``false`` to disable
  the ``/docs`` endpoints. Returns 501 when disabled.

**Swagger UI (optional):**

When built with ``-DENABLE_SWAGGER_UI=ON``, the gateway serves an interactive
Swagger UI at ``/api/v1/swagger-ui`` with embedded assets (no CDN dependency).

**Error Responses:**

- **404:** No capability description available for the requested path
- **501:** Capability description is disabled (``docs.enabled=false``)

See Also
~~~~~~~~

- :doc:`/config/discovery-options` for merge pipeline configuration
- :doc:`/tutorials/authentication` - Configure authentication
- :doc:`/config/server` - Server configuration options
