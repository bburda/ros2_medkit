Multi-Instance Aggregation
==========================

This document describes the design of multi-instance peer aggregation in
ros2_medkit_gateway. It covers the entity model changes (SOVD alignment),
entity merge logic, request routing, and mDNS-based auto-discovery.

.. contents:: Table of Contents
   :local:
   :depth: 3

Overview
--------

A single ros2_medkit_gateway instance discovers and serves the ROS 2 entities on
its local machine. In production systems, robots often run multiple processes
across several hosts, containers, or network segments. Multi-instance aggregation
allows a **primary gateway** to transparently merge entities from one or more
**peer gateways** into a single unified API. Clients see one entity tree and do
not need to know which gateway owns which entity.

.. plantuml::
   :caption: Multi-Instance Aggregation - High-Level Architecture

   @startuml aggregation_overview

   skinparam linetype ortho
   skinparam classAttributeIconSize 0

   class Client << external >>

   package "Primary Gateway" {
       class AggregationManager {
           + fetch_all_peer_entities()
           + fan_out_get()
           + forward_request()
           + check_all_health()
       }
       class EntityMerger {
           + merge_areas()
           + merge_functions()
           + merge_components()
           + merge_apps()
           + get_routing_table()
       }
       class MdnsDiscovery {
           + start()
           + stop()
       }
       class EntityCache
   }

   package "Peer Gateway B" {
       class "REST_API_B" as restB
   }

   package "Peer Gateway C" {
       class "REST_API_C" as restC
   }

   Client --> AggregationManager : HTTP request
   AggregationManager --> EntityMerger : merge remote entities
   AggregationManager --> restB : PeerClient
   AggregationManager --> restC : PeerClient
   AggregationManager --> EntityCache : update with merged entities
   MdnsDiscovery ..> AggregationManager : add_discovered_peer()
   EntityMerger ..> EntityCache : merged entity set

   @enduml

Entity Model (SOVD Alignment)
-----------------------------

Prior to this feature, the gateway used only **Areas**, **Components**, and
**Apps** as entity types. The SOVD spec (ISO 17978) defines a richer hierarchy
where **Components** represent physical hardware (ECUs, hosts) and **Functions**
represent logical capabilities. This feature aligns the entity model:

- **Area** - Physical or logical domain. Manifest-defined only (never
  auto-generated from namespaces).
- **Component** - Physical host or ECU. In runtime-only mode, the gateway
  creates a single Component from the local hostname using ``HostInfoProvider``.
  All Apps discovered on that host are children of this Component.
- **App** - Individual ROS 2 node (unchanged).
- **Function** - Logical capability grouping. In runtime-only mode, each
  ROS 2 namespace becomes a Function entity, grouping the Apps that share
  that namespace. In manifest mode, Functions are explicitly declared.

This mapping aligns with the SOVD view where a Component is "what hosts the
software" and a Function is "what the software does".

.. plantuml::
   :caption: SOVD Entity Model Alignment

   @startuml entity_model

   skinparam linetype ortho
   skinparam classAttributeIconSize 0

   title SOVD Entity Hierarchy

   class Area {
       id: string
       namespace_path: string
       description: string
   }

   class Component {
       id: string
       name: string
       description: string
       area: string
       source: string
   }

   class App {
       id: string
       name: string
       namespace_path: string
       component: string
       source: string
   }

   class Function {
       id: string
       name: string
       hosts: list
       source: string
   }

   Area "1" *--> "*" Component : contains
   Component "1" *--> "*" App : hosts
   Function "1" o--> "*" App : groups

   note right of Component
       In runtime mode: single host
       Component from HostInfoProvider
       with hostname, OS, arch.
   end note

   note right of Function
       In runtime mode: each namespace
       becomes a Function entity.
       In manifest mode: explicitly declared.
   end note

   @enduml

HostInfoProvider
~~~~~~~~~~~~~~~~

``HostInfoProvider`` reads local system information (hostname, OS from
``/etc/os-release``, CPU architecture from ``uname``) and creates a single
``Component`` entity representing the physical host. The Component ID is the
sanitized hostname (lowercase, dots replaced with underscores, truncated to
256 characters).

This replaces the old "synthetic component per namespace" behavior in
runtime-only discovery mode. The single host Component gives the entity tree a
physically meaningful root, and namespace-based grouping is handled by Function
entities instead.

Resource Collections on Functions and Areas
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Functions and Areas support the same resource collections as Components and Apps:

- **data** - Aggregated topic data from all hosted entities
- **operations** - Aggregated services and actions from hosted entities
- **configurations** - Aggregated parameters from hosted entities
- **faults** - Aggregated faults from hosted entities
- **logs** - Aggregated log entries from hosted entities

Requests to ``/functions/{id}/data`` are fan-out queries that collect data from
all entities listed in the Function's ``hosts`` field. Similarly, Area resource
collection requests aggregate from all Components contained in that Area.

Entity Merge Logic
------------------

When entities arrive from a peer gateway, the ``EntityMerger`` applies
type-specific merge rules:

.. plantuml::
   :caption: Entity Merge Logic by Type

   @startuml entity_merge

   title Entity Merge Rules

   start
   :Receive remote entity from peer;
   if (Entity type?) then (Area or Function)
       if (Same ID exists locally?) then (yes)
           :Merge into existing entity\n(combine relations/tags);
       else (no)
           :Add as new entity;
           :Add routing entry;
       endif
   elseif (Component) then
       if (Same ID exists locally?) then (yes)
           :Merge metadata (tags, description);
           :Add provisional routing entry;
       else (no)
           :Add as new entity;
           :Add routing entry;
       endif
   else (App)
       if (Same ID exists locally?) then (yes)
           :Prefix remote ID with\npeername__ separator;
       else (no)
           :Keep original ID;
       endif
       :Add routing entry;
   endif
   :Tag with peer source metadata;
   :Append "peer:<name>" to contributors;
   stop

   @enduml

After every peer has been merged, ``AggregationManager`` runs a
classification pass over the full Component set:

.. plantuml::
   :caption: Component Classification (post-merge)

   @startuml component_classification

   title Component Classification (post-merge)

   start
   :Collect all merged Components
   and per-peer Component claims;
   repeat
     :Pick next Component;
     if (Referenced as parent_component_id
     by any other Component?) then (yes)
       :Classify as hierarchical parent;
       :Remove routing entry\n(serve locally like an Area);
     else (no)
       :Classify as leaf (ECU);
       :Keep routing entry\n(forward to owning peer);
       if (Claimed by >1 peer?) then (yes)
         :Emit LeafCollisionWarning\n(last-writer-wins for routing);
       endif
     endif
   repeat while (more Components?)
   :Attach warnings to /health.warnings;
   stop

   @enduml

**Merge rules summary:**

- **Areas**: Merge by ID. If both local and remote have the same Area ID (e.g.,
  ``root``), they are combined into one entity. Remote-only Areas are added with
  ``source: "peer:<name>"``. No routing table entry is created for merged Areas
  because the local gateway owns the merged entity.

- **Functions**: Merge by ID, combining the ``hosts`` lists from both sides.
  If both gateways expose a ``navigation`` Function, the merged entity lists
  hosts from both gateways. Same ownership semantics as Areas.

- **Components**: Merge by ID, combining tags and metadata. Components
  represent either a single physical ECU *or* a hierarchical parent that
  groups other Components across ECUs (``parent_component_id`` is used to
  model the hierarchy). The ownership rule is applied symmetrically to how
  Areas handle shared roots:

  * **Leaf Component** - no other Component in the merged set references it
    as ``parent_component_id``. Leaves are tied to exactly one ECU, so on
    collision the peer owns the runtime state (data, logs, hosts,
    operations, faults). Leaves get a routing table entry and every request
    - detail endpoint and all sub-resources - is forwarded to the peer.
  * **Hierarchical parent Component** - referenced as
    ``parent_component_id`` by at least one other Component in the merged
    set (local, remote, or transitively merged from any peer). The parent
    itself has no runtime state; it only groups its children. The parent is
    served locally with the merged view (tags/description/``contributors``
    combined) exactly like an Area. No routing table entry is created for
    a hierarchical parent, even when multiple peers announce it.

  Classification happens after all peers have been merged
  (``classify_component_routing`` in ``aggregation/classification.hpp``) so
  that sub-components arriving from different peers still unlock parent
  behaviour on the primary.

  **Multi-peer leaf collisions** (two or more peers announce the same leaf
  Component ID) are surfaced as structured ``/health.warnings`` entries and
  RCLCPP_WARN log lines. Routing falls back to last-writer-wins;
  rejection would not fix the deployment and would only take the gateway
  offline.

  **Cross-snapshot instability under peer churn.** The "last writer" in
  last-writer-wins is determined by the order of healthy peers in the
  merge snapshot (``aggregation_manager.cpp`` iterates ``peers_`` and
  filters via ``is_healthy()``). Two consequences operators should plan
  around:

  - If a colliding peer goes unhealthy between merges, it drops out of
    the snapshot. The remaining peer becomes the new last-writer and
    routing silently flips to it. The request itself cannot fail (the
    dead peer cannot serve it), so this flip is required behaviour -
    but the ``/health.warnings`` entry also disappears (only one
    claimant is left), which hides the routing change from a snapshot
    comparison. Alert on the transition from ``warnings`` non-empty to
    empty, not just on the presence of warnings.
  - Insertion order within ``peers_`` is stable across a merge but can
    vary across gateway restarts, so two fresh primaries with the same
    peer list may pick different last-writers. Sticky routing across
    snapshots is intentionally not implemented - it would mask a
    deployment anomaly rather than surface it. Resolve collisions at
    the manifest level instead.

  **Malformed parent_component_id.** ``classify_component_routing``
  validates every ``parent_component_id`` edge before running
  hierarchical-parent detection. Self-parent references, parent IDs not
  present in the merged Component set, and cycles
  (``A -> B -> ... -> A``) are dropped with a diagnostic on
  ``malformed_parent_warnings`` (logged by the aggregation manager via
  ``RCLCPP_WARN``). Affected Components fall back to leaf routing so a
  misconfigured peer cannot mask itself behind a phantom parent.

- **Apps**: Prefix on collision. If a remote App has the same ID as a local one,
  the remote entity's ID is prefixed with ``peername__`` (double underscore
  separator). Apps represent individual ROS 2 nodes with unique behavior - two
  Apps with the same ID from different peers are different entities.

The ``EntityMerger::SEPARATOR`` constant (``__``) is used as the prefix
separator for Apps. The routing table maps ``entity_id -> peer_name`` for
entities whose runtime state lives on the peer: remote-only Areas and
Functions, leaf Components (remote-only or collision-merged), and
remote-only or prefixed Apps. Hierarchical parent Components are not in the
routing table - they are served locally.

Provenance (``x-medkit.contributors``)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Every merged entity carries a ``contributors`` list in its ``x-medkit``
block that names each source which contributed to the merged view. The
list is populated during merge:

- Each local entity is seeded with ``"local"`` before the peer merge loop
  runs.
- ``EntityMerger`` appends ``"peer:<name>"`` on every Area / Component /
  Function collision and on every remote-only addition, without knowing
  yet whether a Component will later be classified as a hierarchical
  parent or a leaf. The classification pass runs afterwards and only
  rewrites the routing table; ``contributors`` reflects the merge
  inputs. Appends are deduplicated, so merging with the same peer twice
  never produces duplicate entries.
- Apps that collide receive only ``"peer:<name>"`` because the
  prefix strategy turns them into distinct entities.
- Outputs are sorted with ``"local"`` first (when present) and
  ``"peer:<name>"`` entries alphabetically, so clients and snapshot
  tests can rely on a stable order regardless of peer merge order.

Clients (web UI, MCP, Foxglove, VDA 5050 agent) can use ``contributors``
to distinguish a locally-owned entity from one that came in over
aggregation, and to display which peers participated in a hierarchical
parent's view. In daisy-chain topologies each hop surfaces only its
direct upstream; an operator looking at the top-level aggregator sees
``"peer:<direct_neighbour>"`` and must drill into the neighbour to see
its own contributor list.

Request Routing
---------------

When a request arrives for an entity, ``HandlerContext`` checks the routing
table. If the entity is local, processing continues normally. If the entity
maps to a peer, the request is forwarded transparently:

.. plantuml::
   :caption: Request Routing - Local vs Remote

   @startuml request_routing

   actor Client
   participant "Primary Gateway" as primary
   participant HandlerContext as ctx
   participant AggregationManager as agg
   participant "Peer Gateway" as peer

   Client -> primary : GET /api/v1/components/robot_arm__lidar/data
   primary -> ctx : validate_entity_for_route()
   ctx -> ctx : Look up "robot_arm__lidar" in routing table
   ctx --> primary : entity found, peer = "robot_arm"

   primary -> agg : forward_request("robot_arm", req, res)
   agg -> peer : GET /api/v1/components/lidar/data
   note right
       Path rewritten: strips peer prefix
       from entity ID before forwarding
   end note
   peer --> agg : 200 OK + JSON data
   agg --> primary : Copy response to client
   primary --> Client : 200 OK + JSON data

   == Local Entity ==

   Client -> primary : GET /api/v1/apps/my_node/data
   primary -> ctx : validate_entity_for_route()
   ctx -> ctx : "my_node" not in routing table
   ctx --> primary : entity found, local

   primary -> primary : Process locally via DataAccessManager
   primary --> Client : 200 OK + JSON data

   @enduml

Member Dispatch
---------------

The routing table answers "who owns this entity", and for an entity that draws
its resources from members that question has no single answer: an Area, a
merged Function and a hierarchical parent Component all have members on both
sides of the link, which is exactly why they are deliberately absent from the
table. Routing one of them whole would hand the request to one peer and discard
every member the other contributors hold.

A request naming one member is a different question, and it does have a single
answer. ``HandlerContext::dispatch_to_member`` re-addresses such a request to
the member's own entity route - ``/apps/{member}/...`` or
``/components/{member}/...`` - on the gateway the routing table names for that
member:

.. code-block:: text

   POST /api/v1/functions/vehicle_health/operations/peer_calibration:calibrate/executions
     -> POST /api/v1/apps/peer_calibration/operations/calibrate/executions   (on the peer)

   GET  /api/v1/functions/vehicle_health/data/pressure_sensor:chassis/brakes/pressure
     -> GET  /api/v1/apps/pressure_sensor/data/chassis/brakes/pressure       (on the peer)

   PUT  /api/v1/functions/vehicle_health/configurations/peer_calibration:calibration_offset
     -> PUT  /api/v1/apps/peer_calibration/configurations/calibration_offset (on the peer)

Each collection keeps its own id scheme and each hands the same two halves to
the dispatch. ``/data`` and ``/operations`` qualify only an ambiguous id and
carry ``x-medkit.member_ids``; ``/configurations`` qualifies every id on a
multi-node entity as ``<app_id>:<param_name>`` and carries
``x-medkit.source``. What the two schemes agree on is what the dispatch needs:
the member half is an entity id, and the item half is the id the member's own
route uses. Nothing on the owning gateway is aggregating, so the item half is
sent bare - a parameter as its plain name, a topic as its plain path.

Which half is which is decided from the entity's MEMBER SET, not from the number
of ROS nodes the local walk resolves for the entity. The two answer different
questions. A member another gateway runs announces no ROS binding, so it
contributes no node here: the node count measures how much of the entity is
local, and reading an id from it refuses the qualified form on exactly the two
deployments that need it most - an aggregator that runs no node of its own,
where the count is zero, and a gateway running one node beside peer-owned
members, where the count is one. The member set includes what the peers
contributed, so it says the same thing on every deployment.

A prefix that names no member is part of the parameter name. That is what makes
the rule self-protecting: a parameter whose own name contains a colon stays
addressable, and no id that resolves today moves, because a split only happens
where the prefix matches a real member. An entity's own id is not a member half
of itself - it separates nothing - and a member half with an empty item after it
is not one either, since one path segment shorter is the member's configurations
COLLECTION and answering there would hand a list to a caller that asked for one
value.

An operation's item half is its short name, except where the member carrying it
exposes that short name at more than one ROS path. There the member half names
one member for both copies and cannot separate them, so the item half is the
ROS path with its leading slash stripped - and it stays that on the member's own
route too, because the member has the same two operations under the same short
name:

.. code-block:: text

   POST /api/v1/components/vehicle-ecu/operations/dual_calibration:testrig/dual/left/calibrate/executions
     -> POST /api/v1/apps/dual_calibration/operations/testrig/dual/left/calibrate/executions

The member's own gateway is the only one that can answer: the ROS service, the
topic and the parameter behind the id exist on its graph and nowhere else. What
this gateway holds for a peer-owned member is a declaration, which is why the
local walk's record of "does this member provide this item" is consulted only
once the member is known to be served here - on a peer-owned member it holds no
topics at all, and no node FQN to ask for a parameter, so every one of them
would be a miss.

The order inside ``dispatch_to_member`` is load-bearing:

1. **Reachability, before anything is sent.** A member retained while its
   gateway is silent is answered from what it declared: ``504`` with the SOVD
   code ``not-responding``, naming the member. Forwarding to a dead peer instead
   produces a socket failure dressed as ``502``, which reports this gateway as
   broken rather than the link as down.
2. **Ownership.** No routing entry means this gateway owns the member and the
   handler carries on.
3. **Addressing.** The member is looked up in the cache to decide whether it is
   an App or a Component, because that decides which collection its route lives
   under.
4. **Forward.** ``AggregationManager::forward_request`` is called with the
   built path. The overload taking an explicit target applies the same
   ``/api/v1/`` SSRF guard and the same ``<peer>__`` prefix rewrite to that path
   as the two-argument form applies to the incoming one - the target is
   assembled from client-supplied ids and is exactly as untrusted.

Reset-all, ``DELETE /{entity}/configurations``, is not a member-qualified
request and does not go through this dispatch: it names no member, and its
members can sit on several gateways at once, so there is no single owner to hand
it to. This gateway resets what it runs, by calling the parameter services on
its own ROS graph. A member owned by a peer is therefore not reset, and the
response says so rather than implying otherwise - ``207`` with that member named
and its owning gateway named with it, instead of a ``204`` that would report a
reset of parameters the entity lists and this request never touched.

The wire is committed by the forward, so the handler returns
``HandlerContext::forwarded_sentinel_error()``: the typed router recognises the
``x-medkit-internal-forwarded`` code and renders nothing, the same channel the
remote-entity branch of ``validate_entity_for_route`` uses.

Termination does not depend on a hop count. The target is the member's own route
on the gateway that owns it, and there that entity is local, so the receiving
gateway serves it rather than forwarding again. A ``X-Medkit-No-Fan-Out`` header
on the incoming request is propagated but does not suppress the dispatch:
suppression bounds collection fan-out, while a member-qualified request names
its owner and is one hop by construction.

**Entity collection endpoints** (``GET /api/v1/areas``, ``/components``,
``/apps``, ``/functions``) serve from the local entity cache, which is
populated during periodic cache refresh cycles that fetch entities from all
healthy peers.

**Per-entity resource collections** (data, operations, faults, configurations,
logs) and the global ``GET /api/v1/faults`` endpoint use real-time **fan-out**
via ``fan_out_get()`` (handlers call the ``merge_peer_items()`` helper from
``fan_out_helpers.hpp``): the primary gateway sends the same request to all
healthy peers, collects the responses, and merges the ``items`` arrays. If some
peers fail, the response body includes ``x-medkit.partial: true`` and
``x-medkit.failed_peers``. Fan-out requests include an
``X-Medkit-No-Fan-Out`` header to prevent recursive loops when peers have
bidirectional aggregation.

**Target-filtered fan-out.** For per-entity paths, ``merge_peer_items()``
asks ``AggregationManager::get_peer_contributors(id)`` for the list of
peers that host or contribute to the entity, and passes it as a filter to
``fan_out_get()``. Requests reach only those peers; non-contributors are
never queried so they cannot appear in ``failed_peers``. The set unions:

- The routing table (remote leaves, collision-renamed peer-only entities).
- A ``peer_contributors_by_entity_`` map maintained alongside the routing
  table. ``gateway_node`` rebuilds both after every discovery cycle by
  walking ``contributors`` on the merged Areas/Components/Apps/Functions,
  stripping the ``"peer:"`` prefix and accumulating peer names per id.
  Merged Areas/Functions with ID collisions and hierarchical parent
  Components - both deliberately stripped from the routing table - still
  reach their peers through this map.

When the resolved list is empty (local-only entity), fan-out is skipped:
no peer hosts the entity, so hitting peers would only produce spurious
``partial: true`` / ``failed_peers``. Global endpoints (paths with no
entity id, e.g. ``GET /api/v1/faults``) pass a ``nullptr`` filter and keep
fan-out-to-all-healthy behavior.

Entities freshly announced on a peer but not yet reflected in the local
routing/contributor tables (a brief window between discovery cycles) are
treated as local-only: their per-entity fan-out is deferred until the
next cycle rebuilds the tables. This is a deliberate trade-off against
re-enabling the spurious ``partial: true`` path.

.. warning::

   Fan-out is synchronous on the httplib handler thread. Each request blocks
   for up to ``timeout_ms`` (default 2000ms) waiting for the slowest healthy
   peer (parallel via ``std::async``, so max-not-sum across peers).
   ``merge_peer_items()`` skips fan-out when ``healthy_peer_count() == 0`` to
   avoid blocking after a peer outage is detected by health checks, but during
   the window between a peer going down and the next health check cycle, handler
   threads can block. Under concurrent load, this could exhaust httplib's thread
   pool. Consider reducing ``aggregation.timeout_ms`` for deployments with many
   per-entity fan-out consumers.

Peer Discovery
--------------

Peers can be configured statically in the YAML config or discovered
automatically via mDNS.

Static Peers
~~~~~~~~~~~~

Configure peers directly in ``gateway_params.yaml`` using parallel arrays:

.. code-block:: yaml

   aggregation:
     enabled: true
     peer_urls: ["http://192.168.1.10:8080", "http://192.168.1.11:8080"]
     peer_names: ["arm_controller", "base_platform"]

Static peers are always present in the peer list regardless of mDNS settings.

mDNS Auto-Discovery
~~~~~~~~~~~~~~~~~~~~

``MdnsDiscovery`` uses multicast DNS (via the ``mjansson/mdns`` header-only C
library) to announce and discover gateway instances on the local network.

- **Announce**: A background thread responds to mDNS queries for the configured
  service type (default: ``_medkit._tcp.local``). Other gateways on the network
  discover this instance automatically.

- **Browse**: A background thread periodically sends mDNS queries and processes
  responses. When a new peer is found, ``AggregationManager::add_discovered_peer()``
  is called. When a peer sends a goodbye, ``remove_discovered_peer()`` is called.

mDNS discovery works alongside static peers. A gateway can have both static
and dynamically discovered peers.

Health Monitoring
~~~~~~~~~~~~~~~~~

``AggregationManager`` calls ``check_all_health()`` during each entity cache
refresh cycle. Refresh is primarily driven by rclcpp graph events (polled
every 100 ms), with ``refresh_interval_ms`` (default: 30000 ms) providing a
safety backstop for the case a graph event is missed. Each ``PeerClient``
GETs ``/api/v1/health`` on its peer. If the health check fails, the peer is
marked unhealthy and excluded from fan-out queries and entity fetching.

When a peer recovers (health check succeeds again), it is automatically
re-included.

``PeerClient::fetch_entities()`` reads a peer over several requests and either
describes it whole or reports failure: a dead connection, a status a route has
no other meaning for, an oversized body or unparsable JSON on any of them fails
the fetch, because a picture missing a branch is indistinguishable on the wire
from a peer that does not have that branch. Two statuses carry a meaning of
their own: a ``404`` on a nested collection route (``/subareas``,
``/subcomponents``, an app's ``/operations``) identifies a peer running a
gateway that predates the route and is reported in
``PeerEntities::absent_routes`` for the caller to log; a ``504`` with error code
``not-responding`` on a Component's detail is the peer reporting that the
gateway contributing that Component has gone quiet - what a middle gateway in a
chain answers for a declaration it is retaining - so the Component is kept as
the list named it and marked unavailable.
``AggregationManager`` never records a failed fetch as the peer's declaration,
so the last complete one survives; it re-checks that peer's health to decide
whether to replay it marked unavailable (health check failed) or exactly as it
was last read (health check still passes).

The aggregator also publishes its own ``/health`` response with two
additional fields when aggregation is enabled (x-medkit extensions on our
own endpoint, outside the SOVD core contract):

- ``peers`` - array of peer status objects describing each configured or
  discovered peer (URL, name, reachability, last-seen timestamp).
- ``warnings`` - array of operator-actionable aggregation warnings. The
  array is always present (possibly empty) when aggregation is active, so
  clients do not have to differentiate "no warnings" from "aggregation
  disabled" (use ``/.capabilities.aggregation`` in the root endpoint for
  that).

Warning objects carry ``code`` (stable machine-readable identifier,
documented in ``warning_codes.hpp``), ``message`` (human-readable text
including a remediation hint), ``entity_ids`` (SOVD IDs touched by the
anomaly), and ``peer_names`` (peers involved). The only code emitted
today is ``leaf_id_collision`` - see the classification section for the
detection algorithm and the fall-back routing behaviour.

Stream Proxy
~~~~~~~~~~~~

For streaming connections (e.g., SSE fault subscriptions), the ``StreamProxy``
interface provides transport-agnostic event proxying. The ``SSEStreamProxy``
implementation connects to a peer's SSE endpoint and relays events back to the
primary gateway's client. Each ``StreamEvent`` carries the ``peer_name`` so the
aggregator can attribute events to their source.

Deployment Topologies
---------------------

Star Topology
~~~~~~~~~~~~~

One primary gateway aggregates from multiple leaf gateways. Best for robots
with a central controller and peripheral subsystems:

.. code-block:: text

   Client
     |
   Primary (host-A)
    / | \
   B  C  D  (leaf gateways)

Each leaf gateway discovers its own ROS 2 subsystem. The primary merges all
entities and serves a unified view.

Chain Topology
~~~~~~~~~~~~~~

Gateways are chained: A aggregates from B, which aggregates from C. Each
level in the chain sees the merged view of everything downstream:

.. code-block:: text

   Client -> A -> B -> C

Gateway A sees entities from A + B + C. Gateway B sees entities from B + C.
Gateway C sees only its own entities.

This is useful for layered systems where subsystems have their own aggregation
level (e.g., a fleet gateway that aggregates per-robot gateways, which in turn
aggregate per-subsystem gateways).

Containers on Same Host
~~~~~~~~~~~~~~~~~~~~~~~

Multiple ROS 2 subsystems run in separate containers on the same host. Each
container runs its own gateway instance. A host-level gateway aggregates from
all containers via ``localhost`` or Docker network:

.. code-block:: text

   Client
     |
   Host Gateway (port 8080)
    /           \
   Container A   Container B
   (port 8081)   (port 8082)

mDNS discovery handles container-to-container communication automatically
when containers share a network. Static peers work for bridge-networked
containers where mDNS does not cross network boundaries.

Key Classes
-----------

``PeerClient``
    HTTP client for communicating with a single peer gateway. Supports health
    checking, entity fetching, transparent request forwarding (proxy), and
    JSON-parsed responses for fan-out merging. Thread-safe via atomic health
    flag and mutex-guarded lazy client creation.

``EntityMerger``
    Stateless merge engine that combines local and remote entity sets using
    type-specific rules (merge by ID for Area/Function/Component, prefix on
    collision for App). Produces a provisional routing table mapping remote
    entity IDs to peer names - the Component entries are later refined by
    ``classify_component_routing``.

``classify_component_routing`` (``aggregation/classification.hpp``)
    Pure free function that takes the fully merged Component set plus the
    per-peer ``PeerClaim`` list and returns a ``ClassifiedRouting`` with
    hierarchical-parent Components removed from the routing table (served
    locally with merged view) and leaves kept. Multi-peer leaf collisions
    surface as ``LeafCollisionWarning`` entries consumed by
    ``/health.warnings``.

``AggregationManager``
    Central coordinator that manages the set of ``PeerClient`` instances, runs
    health checks, maintains the routing table, and provides fan-out and
    forwarding APIs. Thread-safe via ``shared_mutex``.

``MdnsDiscovery``
    Background service for announcing and browsing mDNS services. Runs
    announce and browse threads. Invokes callbacks when peers are found or
    removed.

``StreamProxy`` / ``SSEStreamProxy``
    Transport-agnostic interface for proxying streaming connections to peers.
    ``SSEStreamProxy`` implements SSE-based event relaying with a background
    reader thread.

``HostInfoProvider``
    Reads local host system info (hostname, OS, architecture) and produces a
    single ``Component`` entity representing the physical host. Used in
    runtime-only discovery to replace synthetic per-namespace Components.

Security Considerations
-----------------------

Multi-instance aggregation introduces an attack surface where a malicious or
compromised peer can inject data into the primary gateway's entity tree. The
following defenses are in place:

Peer URL Validation (mDNS-discovered peers)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

``add_discovered_peer()`` rejects URLs that:

- Use a non-HTTP(S) scheme (e.g., ``ftp://``, ``file://``)
- Resolve to loopback, link-local, or unspecified addresses (prevents SSRF to
  localhost services or cloud metadata endpoints like ``169.254.169.254``)
- Point to well-known cloud metadata hostnames (``metadata.google``)

Static peers bypass address validation (loopback is valid for same-host
deployments) but still require HTTP(S) scheme.

TLS Enforcement
~~~~~~~~~~~~~~~

When ``require_tls: true``, peers with ``http://`` URLs are rejected (both
static and discovered). This prevents cleartext communication on untrusted
networks.

Privileged Port Rejection (mDNS)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The mDNS browse callback rejects SRV records with ports below 1024. Privileged
ports are system services (SSH, HTTP, DNS) that should never be SOVD peer
gateways. This prevents rogue mDNS announcements from redirecting requests to
system services.

Entity ID Validation
~~~~~~~~~~~~~~~~~~~~

Entity IDs received from peer JSON responses are validated before being used in
URL paths for detail fetches. IDs must match ``[a-zA-Z0-9_-]{1,256}``. This
prevents path traversal attacks where a malicious peer returns IDs like
``../etc/passwd`` that would be interpolated into HTTP request paths.

Per-Collection Limits
~~~~~~~~~~~~~~~~~~~~~

Each entity collection (areas, components, apps, functions) is limited to 1000
entities per peer response. If a peer returns more, the entire fetch for that
peer is rejected. This prevents a malicious peer from causing excessive HTTP
requests via N+1 detail fetches.

Response Size Limits
~~~~~~~~~~~~~~~~~~~~

All HTTP responses from peers are limited to 10 MB (``MAX_PEER_RESPONSE_SIZE``).
Responses exceeding this limit are rejected.

Per-Peer Entity Limits
~~~~~~~~~~~~~~~~~~~~~~

``fetch_and_merge_peer_entities()`` accepts a ``max_entities_per_peer`` parameter
(default: 10000) that limits the total number of entities from a single peer
across all collections.

Max Discovered Peers
~~~~~~~~~~~~~~~~~~~~

The ``max_discovered_peers`` config (default: 50) limits the number of peers
that can be added via mDNS discovery. Static peers do not count against this
limit.

Static Peer Protection
~~~~~~~~~~~~~~~~~~~~~~

``remove_discovered_peer()`` only iterates discovered peers, never static peers.
This prevents a rogue mDNS goodbye message from removing a statically configured
peer by matching its name.

Auth Header Forwarding
~~~~~~~~~~~~~~~~~~~~~~

By default, ``forward_auth`` is ``false`` - the primary gateway does NOT forward
client Authorization headers to peers. This prevents token leakage to untrusted
or mDNS-discovered peers. Enable only when all peers are trusted.

Function Host Remapping
~~~~~~~~~~~~~~~~~~~~~~~

When App ID collision causes prefixing (e.g., ``camera_driver`` becomes
``peer_b__camera_driver``), Function entities that reference the original App ID
in their ``hosts`` list are automatically remapped to use the prefixed ID. This
ensures Function host references remain valid after merge.
