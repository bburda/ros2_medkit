Warning Codes
=============

The ``/api/v1/health`` endpoint surfaces operator-actionable anomalies in the
top-level ``warnings`` array (x-medkit extension). These are conditions the
gateway flags without taking itself offline: ``status`` stays ``"healthy"``
and every endpoint keeps serving.
Each entry has the shape:

.. code-block:: json

   {
     "code": "leaf_id_collision",
     "message": "Component 'ecu-x' is announced by multiple peers (peer_b, peer_c); routing falls back to last-writer-wins which is non-deterministic. Resolve by renaming the Component on one side or by modelling it as a hierarchical parent (declare a child Component with parentComponentId='ecu-x' on the owning peer).",
     "entity_ids": ["ecu-x"],
     "ros_node_fqns": [],
     "peer_names": ["peer_b", "peer_c"]
   }

All five keys are always present. The three identifier arrays are empty
rather than omitted when a code has nothing to put in them, so a client can
read them without a presence check.

Codes are stable machine-readable identifiers: renaming a code is a
breaking change for downstream consumers that key on the string.

The canonical list of codes is maintained in
``src/ros2_medkit_gateway/include/ros2_medkit_gateway/core/http/warning_codes.hpp``;
this page mirrors it for API consumers.

.. list-table::
   :header-rows: 1
   :widths: 25 75

   * - Code
     - Meaning / Remediation
   * - ``leaf_id_collision``
     - More than one peer announces the same **leaf** (non-hierarchical)
       Component ID during aggregation merge. Routing falls back to
       last-writer-wins, so requests for the affected Component reach one
       peer non-deterministically. Resolve by renaming the Component on one
       side, or by modelling it as a hierarchical parent - declare a child
       Component with ``parentComponentId`` pointing at the colliding ID on
       the owning peer. The warning lists every claiming peer in
       ``peer_names``.
   * - ``unmanifested_nodes``
     - One or more running ROS nodes are not declared in the manifest while
       ``config.unmanifested_nodes`` is set to ``error``. This is a report,
       not an outage: the gateway keeps serving and ``status`` stays
       ``"healthy"``. Resolve by declaring the nodes in the manifest, or by
       relaxing the policy to ``warn`` or ``ignore``. The same document
       carries ``discovery.linking.unmanifested_policy`` and
       ``discovery.linking.orphan_count``, so a monitor can branch on
       ``orphan_count > 0 && unmanifested_policy == "error"`` without reading
       the message text.

``warnings`` is always an array on the ``/health`` response - empty when no
anomalies are active, non-empty otherwise - whether or not aggregation is
enabled. Aggregation-specific codes can only appear when aggregation is
configured (``GET /`` -> ``capabilities.aggregation`` is ``true``), but the
array itself is not conditional on it.

The identifier arrays
---------------------

A warning object carries three identifier arrays. All three are always
present; a code that has nothing to say in one of them sends an empty array
rather than omitting the field, so a client never has to distinguish "absent"
from "none".

.. list-table::
   :header-rows: 1
   :widths: 22 78

   * - Field
     - Contains
   * - ``entity_ids``
     - Addressable SOVD entity ids, and nothing else. **This holds for every
       code, present and future.** Every value satisfies the entity-id rules
       (alphanumerics, ``_`` and ``-``, at most 256 characters), so
       ``GET /{collection}/{id}`` can be built from it directly.
   * - ``ros_node_fqns``
     - ROS node fully-qualified names, e.g.
       ``/powertrain/engine/rpm_sensor``. These are *not* entity ids - a
       leading or embedded ``/`` is rejected in an entity id because it
       conflicts with URL routing - so they have their own field.
   * - ``peer_names``
     - The aggregation peers involved in the anomaly.

Per code:

.. list-table::
   :header-rows: 1
   :widths: 25 25 25 25

   * - Code
     - ``entity_ids``
     - ``ros_node_fqns``
     - ``peer_names``
   * - ``leaf_id_collision``
     - the colliding Component id
     - empty
     - the announcing peers
   * - ``unmanifested_nodes``
     - empty
     - every undeclared node
     - empty

``unmanifested_nodes`` reports nodes rather than entities on purpose, and not
because an undeclared node necessarily lacks an entity - most of them have
one. The reasons are that the reported set is deliberately wider than the
entity tree, and that the node name is the stable identifier of the two:

- The list is taken from the unfiltered runtime view, before gap-fill, before
  the namespace filters and before the unmanifested-node policy itself. Nodes
  that those filters remove are exactly the ones an operator most needs
  named, and they have no entity to point at.
- An App id is derived from the bare node name and only becomes
  namespace-qualified if another node on the graph happens to share that
  name. An unrelated node starting anywhere can therefore change an existing
  orphan's App id between two ``/health`` polls, while its FQN cannot change.

Schema versioning
-----------------

Alongside ``warnings`` the ``/health`` response exposes an integer
``warning_schema_version``, present on every response regardless of whether
any warnings are active. Typed clients key on this field to decide which
codes they can feature-detect without reverting to string-matching every time
a new anomaly class is added.

The contract is:

- Current version: ``2``. Version 2 added the ``unmanifested_nodes`` code and
  the ``ros_node_fqns`` field, and made ``warnings`` and
  ``warning_schema_version`` unconditional; version 1 emitted them only when
  aggregation was enabled and had no ``ros_node_fqns``.
- Bumped by one whenever a code is added, removed, or the shape of a
  warning object changes.
- Within a given version, every code listed on this page is guaranteed to
  appear verbatim; clients seeing an unknown code at a known version
  should log-and-ignore rather than fail.
- Across versions, clients are expected to treat unknown codes as
  future-compatible: log-and-ignore, do not crash.

Clients that need strong typing (MCP tools, Web UI badges, Foxglove
panels) should branch on ``warning_schema_version`` before mapping codes
onto internal enums.
