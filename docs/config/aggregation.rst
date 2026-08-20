Aggregation Configuration
=========================

This reference describes all aggregation-related configuration options for
multi-instance peer aggregation in ros2_medkit_gateway.

.. contents:: Table of Contents
   :local:
   :depth: 2

Overview
--------

Aggregation allows multiple gateway instances to federate their entity trees
into a single unified API. A primary gateway merges entities from peer gateways
and transparently forwards requests for remote entities.

All aggregation parameters are under the ``aggregation`` key in
``gateway_params.yaml`` or can be set via ROS 2 parameters.

Quick Start
-----------

.. code-block:: bash

   # Enable aggregation with a static peer
   ros2 run ros2_medkit_gateway gateway_node --ros-args \
       -p aggregation.enabled:=true \
       -p aggregation.peer_urls:="['http://192.168.1.10:8080']" \
       -p aggregation.peer_names:="['peer_b']"

Or in ``gateway_params.yaml``:

.. code-block:: yaml

   ros2_medkit_gateway:
     ros__parameters:
       aggregation:
         enabled: true
         peer_urls: ["http://192.168.1.10:8080"]
         peer_names: ["peer_b"]

Core Parameters
---------------

.. list-table::
   :header-rows: 1
   :widths: 30 10 10 50

   * - Parameter
     - Type
     - Default
     - Description
   * - ``aggregation.enabled``
     - bool
     - ``false``
     - Master switch for peer aggregation. When disabled, the gateway operates
       in standalone mode with no peer communication.
   * - ``aggregation.timeout_ms``
     - int
     - ``2000``
     - HTTP timeout in milliseconds for all peer communication: health checks,
       entity fetching, and request forwarding. Increase for high-latency
       networks.

mDNS Discovery Parameters
--------------------------

.. list-table::
   :header-rows: 1
   :widths: 30 10 10 50

   * - Parameter
     - Type
     - Default
     - Description
   * - ``aggregation.announce``
     - bool
     - ``false``
     - Broadcast this gateway's presence via mDNS. Other gateways on the local
       network can discover this instance automatically. Opt-in to avoid
       surprising network behavior.
   * - ``aggregation.discover``
     - bool
     - ``false``
     - Browse for peer gateways via mDNS. When a new peer is found, it is
       automatically added to the peer list. Opt-in to avoid surprising
       network behavior.
   * - ``aggregation.mdns_service``
     - string
     - ``"_medkit._tcp.local"``
     - mDNS service type used for announcement and browsing. All gateways in
       the same aggregation cluster must use the same service type.
   * - ``aggregation.mdns_name``
     - string
     - ``""``
     - mDNS instance name for announcement and self-discovery filtering.
       Defaults to the system hostname (via ``gethostname()``). Must be unique
       per gateway instance. Set explicitly when running multiple gateways on
       the same host - otherwise they share the same hostname and filter each
       other out as "self".

.. _aggregation-security:

Security Parameters
--------------------

These parameters control authentication forwarding and transport security
for peer communication.

.. list-table::
   :header-rows: 1
   :widths: 30 10 10 50

   * - Parameter
     - Type
     - Default
     - Description
   * - ``aggregation.forward_auth``
     - bool
     - ``false``
     - Forward the client's ``Authorization`` header to peer gateways. When
       ``false`` (default), auth tokens are **never** sent to peers - this
       prevents token leakage to untrusted or mDNS-discovered peers. Only
       enable when all peers are trusted and share the same JWT configuration.
   * - ``aggregation.require_tls``
     - bool
     - ``false``
     - Require HTTPS for all peer URLs. When ``true``, any peer URL using
       ``http://`` is rejected at startup (static peers) or on discovery
       (mDNS peers) with an ERROR log. When ``false``, ``http://`` peers
       produce a WARN log about cleartext communication.
   * - ``aggregation.peer_scheme``
     - string
     - ``"http"``
     - URL scheme used when constructing URLs for mDNS-discovered peers.
       mDNS SRV records provide hostname and port but not the URL scheme.
       Set to ``"https"`` when all peers use TLS. This does not affect
       static peer URLs (which include the scheme explicitly).
   * - ``aggregation.max_discovered_peers``
     - int
     - ``50``
     - Maximum number of peers that can be added via mDNS discovery. Range:
       1-1000, clamped with a warning. Prevents
       unbounded growth of the peer list from rogue mDNS announcements on the
       local network. Static peers (configured via ``peer_urls``/``peer_names``)
       do not count against this limit. When the limit is reached, additional
       discovered peers are rejected with a WARN log.

.. warning::

   When ``forward_auth`` is enabled but ``require_tls`` is disabled, the
   gateway logs a warning at startup. Authorization tokens may be sent to
   peers over cleartext HTTP, exposing them to network sniffers.

.. warning::

   When ``forward_auth`` is enabled, **all peers** (including mDNS-discovered
   ones) receive the client's auth token. A malicious peer on the local
   network could harvest these tokens. For production deployments:

   1. Set ``forward_auth: true`` only if all peers are trusted.
   2. Set ``require_tls: true`` to prevent tokens from flowing in cleartext.
   3. Set ``peer_scheme: "https"`` so mDNS-discovered peers also use TLS.
   4. Consider disabling mDNS discovery (``discover: false``) and using
      only static peers with known ``https://`` URLs.

Static Peers
------------

Static peers are configured as parallel arrays: ``peer_urls[i]`` pairs with
``peer_names[i]``. Both arrays must have the same length. Empty-string entries
are ignored.

.. list-table::
   :header-rows: 1
   :widths: 30 10 10 50

   * - Parameter
     - Type
     - Default
     - Description
   * - ``aggregation.peer_urls``
     - string[]
     - ``[""]``
     - List of peer gateway base URLs (e.g.,
       ``["http://192.168.1.10:8080", "http://192.168.1.11:8080"]``).
       Each URL must include the scheme and port.
   * - ``aggregation.peer_names``
     - string[]
     - ``[""]``
     - List of human-readable names for peers (e.g.,
       ``["arm_controller", "base_platform"]``).
       Used as prefix for collision resolution (e.g., ``peername__entity_id``)
       and in the routing table. Must be unique across all peers.

Scenario Examples
-----------------

Star Topology
~~~~~~~~~~~~~

One primary gateway aggregates from three subsystem gateways:

.. code-block:: yaml

   # Primary gateway (host-A, port 8080)
   ros2_medkit_gateway:
     ros__parameters:
       aggregation:
         enabled: true
         timeout_ms: 3000
         announce: false
         discover: false  # Use static peers only
         peer_urls: ["http://192.168.1.10:8080", "http://192.168.1.11:8080", "http://192.168.1.12:8080"]
         peer_names: ["arm_controller", "base_platform", "sensor_array"]

The leaf gateways do not need aggregation enabled - they serve their own
entities independently. Only the primary gateway needs aggregation.

Chain Topology
~~~~~~~~~~~~~~

Gateway A aggregates from B, which aggregates from C:

.. code-block:: yaml

   # Gateway A (top-level aggregator)
   ros2_medkit_gateway:
     ros__parameters:
       server:
         port: 8080
       aggregation:
         enabled: true
         peer_urls: ["http://gateway-b:8080"]
         peer_names: ["subsystem_b"]

.. code-block:: yaml

   # Gateway B (mid-level aggregator)
   ros2_medkit_gateway:
     ros__parameters:
       server:
         port: 8080
       aggregation:
         enabled: true
         peer_urls: ["http://gateway-c:8080"]
         peer_names: ["subsystem_c"]

.. code-block:: yaml

   # Gateway C (leaf - no aggregation needed)
   ros2_medkit_gateway:
     ros__parameters:
       server:
         port: 8080
       aggregation:
         enabled: false

Gateway A sees entities from A + B + C. Gateway B sees entities from B + C.

mDNS-Only Discovery
~~~~~~~~~~~~~~~~~~~~

Fully automatic peer discovery with no static configuration:

.. code-block:: yaml

   # All gateways use the same config
   ros2_medkit_gateway:
     ros__parameters:
       aggregation:
         enabled: true
         announce: true
         discover: true
         mdns_service: "_medkit._tcp.local"
         # No static peers - all discovery via mDNS

All gateways on the same network segment automatically find each other. When a
gateway starts, it announces itself and discovers existing peers. When a gateway
stops, it sends an mDNS goodbye and peers remove it automatically.

.. note::

   mDNS requires multicast network support. Docker containers using bridge
   networking may not support mDNS - use static peers or host networking
   instead.

Mixed Static + mDNS
~~~~~~~~~~~~~~~~~~~~

Combine static peers for known infrastructure with mDNS for dynamic discovery:

.. code-block:: yaml

   ros2_medkit_gateway:
     ros__parameters:
       aggregation:
         enabled: true
         announce: true
         discover: true
         # Always connect to the base platform
         peer_urls: ["http://base-platform:8080"]
         peer_names: ["base_platform"]
         # Additional peers discovered via mDNS at runtime

.. note::

   By default, ``Authorization`` headers are **not** forwarded to peers
   (``forward_auth: false``). If your peers require authentication, set
   ``forward_auth: true`` and ensure all peers use the same JWT
   configuration. See :ref:`Security Parameters <aggregation-security>`
   for details on securing peer communication.

.. note::

   ``X-Client-Id`` is always forwarded, and is not governed by
   ``forward_auth``. It names the caller rather than granting it anything:
   a lock on a peer-owned entity is held on the peer and judged there, so a
   forwarded request that arrived without the name would be a different
   caller than the one holding the lock. Authority still travels only in
   ``Authorization``, which is forwarded when the deployment says the peer
   is trusted with it.

Secure Aggregation (TLS + Auth)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Production deployment with TLS enforcement and auth forwarding:

.. code-block:: yaml

   ros2_medkit_gateway:
     ros__parameters:
       server:
         tls:
           enabled: true
           cert_file: "/etc/ros2_medkit/certs/cert.pem"
           key_file: "/etc/ros2_medkit/certs/key.pem"
       aggregation:
         enabled: true
         forward_auth: true      # All peers share JWT config
         require_tls: true       # Reject any http:// peers
         peer_scheme: "https"    # mDNS-discovered peers use HTTPS
         announce: true
         discover: true
         peer_urls: ["https://gateway-b:8080"]
         peer_names: ["subsystem_b"]

Entity Merge Behavior
---------------------

When aggregation is enabled, entities from peers are merged with local entities:

- **Areas, Functions, and Components**: Merged by ID. If both local and remote
  have the same ID, they become one entity. Areas and Functions represent logical
  groupings that span hosts. Components represent physical hosts or ECUs defined
  in manifests - the same Component across peers is the same physical entity.

- **Apps**: Prefixed on collision. If a remote App has the same ID as a local
  one, the remote App's ID is prefixed with ``peername__`` (double underscore).
  For example, App ``camera_driver`` from peer ``arm`` becomes
  ``arm__camera_driver``. Apps represent individual ROS 2 nodes with unique
  behavior.

Requests for remote entities are transparently forwarded to the owning peer.
The routing table maps entity IDs to peer names.

An entity that draws its resources from members - an Area, a merged Function, a
hierarchical parent Component - is deliberately absent from that table, because
its members can sit on different gateways and routing it whole would discard
every member the other contributors hold. A request that names one member is
routed instead: it is re-addressed to that member's own entity route on the
gateway that owns it, so

.. code-block:: text

   POST /api/v1/functions/vehicle_health/operations/peer_calibration:calibrate/executions

becomes, on the peer that runs ``peer_calibration``,

.. code-block:: text

   POST /api/v1/apps/peer_calibration/operations/calibrate/executions

The same routing applies to a single ``/data`` item and to a single
``/configurations`` parameter. A configuration id is ``<app_id>:<param_name>``,
and on the owning gateway the parameter is addressed by its bare name, so

.. code-block:: text

   PUT /api/v1/functions/vehicle_health/configurations/peer_calibration:calibration_offset

becomes

.. code-block:: text

   PUT /api/v1/apps/peer_calibration/configurations/calibration_offset

A member half is recognised when the text before the first colon names a member
of the addressed entity, so the qualified form works on an aggregating entity
whose members are all owned by peers - a parent gateway that runs no ROS node of
its own - and on one that runs a single node beside peer-owned members. Neither
shape changes the ids the entity's listing offers.

``DELETE /api/v1/{entity_type}/{id}/configurations`` resets the nodes this
gateway runs. A member another gateway runs is not reset by it, and the response
says so: ``207`` instead of ``204``, with that member named and the gateway that
owns it named with it.

Reachability is answered before anything is forwarded, so a member whose gateway
is silent gets ``504 not-responding`` rather than a ``502`` from a failed
connection. A member this gateway owns is served here, unchanged.

See :doc:`../design/ros2_medkit_gateway/aggregation` for detailed merge logic
and architecture diagrams.

Health and Partial Results
--------------------------

Entity collection endpoints (``GET /api/v1/areas``, ``/components``, ``/apps``,
``/functions``) serve data from the local entity cache, which is populated
during periodic cache refresh cycles. These endpoints do not perform real-time
fan-out to peers, so they always return successfully with whatever entities
were last cached.

Per-entity resource collection endpoints perform real-time fan-out via
``fan_out_get()`` to collect items from all healthy peers and merge them
into the local response. This applies to:

- ``GET /api/v1/{entity_type}/{id}/data``
- ``GET /api/v1/{entity_type}/{id}/operations``
- ``GET /api/v1/{entity_type}/{id}/faults``
- ``GET /api/v1/{entity_type}/{id}/configurations``
- ``GET /api/v1/{entity_type}/{id}/logs``
- ``GET /api/v1/faults`` (global fault list)

If a peer request fails during fan-out (peer unreachable or non-2xx response),
the response body includes:

- ``x-medkit.partial: true`` in the JSON response body
- ``x-medkit.failed_peers`` listing which peers failed

This allows clients to detect degraded responses and take appropriate
action. Individual entity requests for remote entities (e.g.,
``GET /api/v1/apps/{id}``) return ``502 Bad Gateway`` if the owning peer
is unreachable.

Peer Refresh Completeness
~~~~~~~~~~~~~~~~~~~~~~~~~

A cache refresh reads a peer over several requests: the four entity lists, the
nested ``subareas`` and ``subcomponents`` collections, the per-entity detail
that carries a Component's relationships and a Function's hosts, and each app's
``operations``. If any of them cannot be read - connection failure, a status
the route has no other meaning for, an oversized body, unparsable JSON - the
refresh for that peer is discarded whole. A partial picture is never published
as a complete one, and the peer's last complete declaration is left in place.

What clients see then depends on the peer's health check:

- Health check fails: the retained declaration is served with
  ``x-medkit.available: false`` (and ``x-medkit.is_online: false`` for Apps),
  because a request addressed there cannot arrive.
- Health check passes: the retained declaration is served unchanged and the
  incomplete refresh is logged at ``WARN``. Availability is untouched - the
  peer can still be reached; this gateway merely failed to read all of it.

Two statuses are read rather than treated as failures:

- ``404`` on a nested collection route means the peer runs a gateway version
  that does not expose the route. Those members are omitted, the rest of the
  peer merges normally, and the absent routes are logged once per refresh at
  ``WARN``.
- ``504`` with error code ``not-responding`` on any route hanging off an entity
  - its detail, or one of its nested collections - means the peer holds that id
  and the gateway contributing it has gone quiet, which is the answer an
  aggregating peer gives for a declaration it is retaining. In a chain topology
  this is how the far end reports a dead leaf, so the entity is kept as the
  peer's list named it and marked ``x-medkit.available: false``. A nested
  collection answering that way costs only the members that route carries;
  treated as a failure it would discard the whole peer on every refresh, so one
  unreachable member would freeze this gateway's view of everything that peer
  holds. A ``504`` without ``not-responding`` is not a statement about an entity
  and still discards the refresh.

Availability is also read back off the wire. ``x-medkit.available`` is emitted
only when false, so an absent field means the entity is reachable, and that is
the default this gateway parses it with. It matters most beyond one hop: an App
also carries ``x-medkit.is_online``, but a Component has no second signal, so
without the read-back the head of a three-gateway chain reports a leaf behind a
dead gateway as reachable. Retention never contradicts what a peer said - it
only ever sets ``available`` to false, and it does so when the peer itself
stopped answering, which already covers everything behind it.

.. _aggregation-breaking-changes:

Breaking Changes (Entity Model Simplification)
-----------------------------------------------

.. warning::

   These changes affect **all discovery modes**, not just aggregation.
   Users running in ``runtime_only`` mode without aggregation enabled will
   see different API responses after upgrading.

The entity model has been aligned with the SOVD specification (ISO 17978).
Synthetic/heuristic Area and Component creation from ROS 2 namespaces has
been removed. The following behavioral changes apply:

**API response changes:**

- ``GET /api/v1/components`` now returns a **single host-level Component**
  (from ``HostInfoProvider``) instead of one synthetic Component per ROS 2
  namespace.
- ``GET /api/v1/areas`` now returns an **empty list** in runtime-only mode
  (was namespace-based). Areas come from manifest only.
- ``GET /api/v1/functions`` is now **populated** in runtime-only mode (was
  always empty). Each ROS 2 namespace becomes a Function entity, controlled
  by the ``create_functions_from_namespaces`` parameter (default: ``true``).
- **Apps** are still created from ROS 2 nodes with ``source: "heuristic"``.

**Removed configuration parameters (8 total):**

The following parameters no longer exist. The gateway will log a warning
and ignore them if present in config.

From ``discovery.runtime``:

- ``create_synthetic_areas``
- ``create_synthetic_components``
- ``grouping_strategy``
- ``synthetic_component_name_pattern``
- ``topic_only_policy``
- ``min_topics_for_component``

From ``discovery.merge_pipeline.gap_fill``:

- ``allow_heuristic_areas``
- ``allow_heuristic_components``

**Cross-layer impact:**

The entity model change affects consumers beyond the gateway REST API:

- **Web UI** (``ros2_medkit_web_ui``): The ``ComponentWithOperations`` type
  has stale ``area``, ``namespace``, and ``fqn`` fields that assumed
  per-namespace Components. These fields are empty or absent for the single
  host-level Component.
- **Foxglove extension** (``ros2_medkit_foxglove_extension``): The
  ``EntityBrowserPanel`` loads Areas first to build the entity tree. In
  runtime-only mode, the Areas list is now empty, so the panel shows no
  top-level grouping until a manifest is provided.
- **MCP server** (``ros2_medkit_mcp``): Clients that reference synthetic
  Component IDs (e.g., ``powertrain_engine_component``) will receive 404
  errors. Update tool calls to use the single host-level Component ID or
  switch to Apps/Functions.

**Migration path:**

- If you relied on per-namespace Components, switch to ``hybrid`` or
  ``manifest_only`` mode and declare Components explicitly in a manifest.
- If you relied on namespace-based Areas, declare them in a manifest.
- Namespace grouping is now handled by Function entities instead of Areas
  and Components.
