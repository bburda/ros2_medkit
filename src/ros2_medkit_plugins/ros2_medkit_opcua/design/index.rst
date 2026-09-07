==================
ros2_medkit_opcua
==================

Design notes for the OPC-UA gateway plugin.

Motivation
==========

Mixed ROS 2 + PLC diagnostics
-----------------------------

Industrial robot deployments are rarely pure ROS 2. A typical cell combines ROS 2 manipulators
with PLC-controlled fixtures, conveyors, safety relays, and process equipment. Operators want a
single diagnostic surface that covers both sides.

The OPC-UA plugin lets ``ros2_medkit_gateway`` expose OPC-UA PLC tags as first-class SOVD entities,
giving diagnostic tools and MCP agents one consistent API across the whole cell.

Architecture
============

The plugin is loaded by ``ros2_medkit_gateway`` at runtime via ``dlopen`` and implements two
interfaces:

- ``GatewayPlugin`` - lifecycle (``configure``, ``set_context``, ``get_routes``, ``shutdown``) and
  vendor REST endpoints.
- ``IntrospectionProvider`` - emits an ``IntrospectionResult`` describing the SOVD entities
  generated from the PLC node map, so the gateway's merge pipeline can place them in the entity tree.

::

    OPC-UA server (PLC, OpenPLC, Siemens S7, Beckhoff, ...)
             |  opc.tcp, port 4840
             v
    OpcuaClient (open62541pp wrapper)
             |
             v
    OpcuaPoller  --- threshold alarms ---> OpcuaPlugin::on_alarm_change
             |                                     |
             |  PollSnapshot                       | ros2_medkit_msgs/ReportFault
             v                                     v  (or ClearFault when alarm clears)
    OpcuaPlugin::publish_values          ros2_medkit_fault_manager
             |                                     |
             v                                     v
    std_msgs/Float32 topics            SOVD faults on PLC entities
    (one per numeric node)

Node map schema
===============

All mapping lives in YAML (``config/tank_demo_nodes.yaml`` is the reference example):

- ``area_id`` / ``component_id`` - SOVD tree placement for the PLC runtime
- ``nodes`` - list of entries, each with:

  - ``node_id`` - OPC-UA node identifier (e.g. ``"ns=2;i=1"``)
  - ``entity_id`` - SOVD app the value belongs to
  - ``data_name`` - short name used in REST URLs
  - ``display_name``, ``unit``, ``data_type``, ``writable`` - metadata.
    ``writable`` is ignored in a read-only build (see below)
  - ``min_value`` / ``max_value`` - optional write range check
  - ``alarm`` - optional fault definition (``fault_code``, ``severity``, ``threshold``,
    ``above_threshold`` direction)

The plugin binary is completely PLC-agnostic; pointing it at a different node map file
reuses the same build for a different PLC.

Poll vs subscription mode
=========================

The plugin supports two data paths, selected by the ``prefer_subscriptions`` config flag:

**Polling (default)** - a background thread reads all mapped nodes at a fixed interval
(``poll_interval_ms``, default 1000 ms). Simple, predictable, no event loop needed, sufficient
for diagnostic use cases where latency below one second does not add value.

**Subscription** - registers OPC-UA monitored items and receives change notifications at
``subscription_interval_ms``. Lower CPU cost on large node sets, but requires a running OPC-UA
client event loop. Kept as an opt-in path because many PLC servers have limits on the number
of active subscriptions.

Both modes share the same ``PollSnapshot`` output structure, so downstream consumers
(``OpcuaPlugin::publish_values``, REST handlers, fault mapping) are agnostic to the source.

Alarm -> fault mapping
======================

Each node in the map may declare an ``alarm`` block. The poller compares the current value
against the configured threshold on every poll; transitions across the threshold produce
edge-triggered callbacks:

- **Active -> fault reported** via ``/fault_manager/report_fault`` with the configured
  severity and message
- **Cleared -> fault cleared** via ``/fault_manager/clear_fault`` by fault code

The plugin keeps per-fault state only long enough to detect edges; the fault manager owns
persistence and fault lifecycle.

Read-only is a property of the build
====================================

Four of the five protocol plugins cannot write to their device at all. OPC UA can,
and config-less discovery would mark a point writable straight from the server's
``CurrentWrite`` bit. A plant asking "can this box change a controller" cannot be
answered by a configuration value, because a configuration value can be changed
without rebuilding or shipping anything. So the answer is carried by the
binary.

``MEDKIT_OPCUA_READ_ONLY`` is a CMake cache option, default ``ON``. With it on:

- ``OpcuaClient::write_value``, the ``open62541pp`` Write service functions and
  templates it reaches, the vendor route handler
  ``OpcuaPlugin::handle_plc_operations`` and the value-coercion helper are all
  outside ``#if`` and are never compiled.
- The link removes what the compiler alone cannot. ``-fvisibility=hidden`` covers
  the sources compiled into the module but not the static archives it links, so
  without further flags open62541's own ``UA_Client_write*`` primitives sit in the
  object *and* in its dynamic symbol table, where a caller holding the ``.so``
  can ``dlsym`` one and drive a controller with it even though no route reaches
  it. ``-Wl,--exclude-libs,ALL`` takes the archives out of the export table, and
  ``-ffunction-sections -fdata-sections`` plus ``-Wl,--gc-sections`` then let the
  linker drop them from the object entirely, because nothing references them once
  the C++ write path is gone. The read-only object exports the six plugin entry
  points and nothing from the OPC-UA stack.
- The section flags are set on ``open62541-object`` and ``open62541-plugins``,
  which is where upstream compiles its C sources - ``open62541`` itself is
  assembled from ``$<TARGET_OBJECTS:...>`` and compiles nothing. Upstream adds the
  same two flags, but only under ``Release`` and ``MinSizeRel``, so a default-type
  build previously kept twelve ``UA_*_write*`` symbols in the read-only object as
  local, unexported code. Setting them here makes the property independent of
  ``CMAKE_BUILD_TYPE``, and ``test_opcua_build_variant`` asserts the absence of
  that whole family so the difference cannot come back unnoticed.
- What is left of open62541 inside the object is one shared transport, not a
  write path. open62541 is a single static library, so removing the write code
  does not remove what it shared with the read code: the generic dispatcher
  ``__UA_Client_Service`` (used by read, browse and ConditionRefresh), the binary
  encoders (23 ``*_encodeBinary`` symbols), and the generated ``UA_TYPES``
  descriptors, which the table references as a whole so the linker cannot drop
  individual entries. Measured on the read-only object, those descriptors include
  ``WriteRequest``, ``WriteValue``, ``WriteResponse``, ``AddNodes``,
  ``DeleteNodes``, ``AddReferences``, ``SetMonitoringMode``,
  ``SetPublishingMode`` and ``TransferSubscriptions``; ``HistoryUpdate`` is
  absent.

  Descriptors are data. What makes them unreachable is that no function in the
  object composes any of those requests, none of these symbols is exported, and a
  read-only build registers three GET routes - no route, node-map key or config
  key supplies a NodeId, a method id or an attribute id. So the claim the package
  makes is the exact one: every C++ path that composes a Write or a condition
  method call is absent, and no OPC UA symbol is exported. Not "no OPC UA
  machinery at all", which would be false.

- Subscription and monitored-item creation stay in both variants. They change
  server-side session state, which is not controller data, and the read path
  cannot receive values or alarms without them.
- ``NodeMap::load`` forces ``writable`` to false and warns once when the file asked
  otherwise; ``AutoBrowser`` does not compile the ``infer_writable`` inference, so
  the server's ``CurrentWrite`` bit is never read.
- No entity registers the ``x-plc-operations`` capability, ``list_operations``
  emits no ``set_<name>`` entry, and ``get_routes`` does not register the write
  route.
- ``write_data`` and the value-write half of ``execute_operation`` return 501 as
  their first statement, before any node lookup, with a message naming the build
  property. SOVD spells "the entity does not support this" as 501 (fault
  deletion, data lists, subscriptions and triggers all use it); 403 is defined
  once, for a valid token with insufficient permissions, which is the one reading
  that is wrong here - no credential reaches a path that is not in the binary. They stay declared because the gateway reaches the plugin through the
  ``DataProvider`` / ``OperationProvider`` interfaces; the refusal reaches a client
  as SOVD vendor code ``x-medkit-plugin-error``, which is the code the gateway
  assigns to every plugin provider error.
- ``acknowledge_fault`` / ``confirm_fault`` go the same way. A method call that
  changes alarm state on the server is a write to the controller, and the rule is
  that a read-only build carries no write path at all, not merely no value writes.
  ``OpcuaClient::call_condition_method`` - the entry point that issues the Part 9
  Acknowledge (i=9111) and Confirm (i=9113) calls - is compiled only in a
  write-capable build, ``list_operations`` offers neither operation on an
  event-alarm entity, and ``execute_operation`` refuses both by name before any
  condition lookup.
- ``ConditionRefresh`` stays, and so does the generic ``OpcuaClient::call_method``
  it rides on. Part 9 5.5.7 makes it a request for the server to replay the
  conditions it already holds to the calling subscription; it changes nothing on
  the controller, and without it a restart loses the active fault set. That is why
  the guard sits on the condition-method entry point rather than on ``call_method``,
  and why neither ``call_method`` nor ``opcua::services::call`` is a write marker -
  measured, they are present in both variants.

The acceptance is an inspection of the built object, not a reading of the
configuration: ``test_opcua_build_variant`` runs ``nm`` over the plugin ``.so`` and
asserts the write symbols are absent, the read symbols present, the dynamic symbol
table free of OPC-UA machinery, and the six plugin entry points still exported. It
runs in both variants with opposite expectations, so a symbol list that stopped
matching anything fails in the write-capable build instead of passing everywhere.

Type-aware writes (write-capable build)
=======================================

``POST /apps/{id}/x-plc-operations/{op}`` accepts a JSON body ``{"value": ...}``. The handler:

1. Looks up the node by ``data_name`` (with optional ``set_`` prefix in the op name).
2. Rejects read-only nodes with a 400 error.
3. Coerces the JSON value to the node's declared ``data_type`` (``bool``, ``int``, ``string``,
   otherwise ``double``) and returns a typed 400 on mismatch.
4. Applies the range check (``min_value`` / ``max_value``) if configured.
5. Calls ``OpcuaClient::write_value`` which uses the node's OPC-UA data type to build the
   correct binary payload.

This avoids the common failure mode of writing a ``float64`` to an OPC-UA ``REAL`` (float32)
node and getting silent truncation or a server-side rejection.

ROS 2 topic bridge
==================

``set_context`` creates one ``std_msgs/Float32`` publisher per numeric node, named from the
``ros2_topic`` field of the node map. After every poll, ``publish_values`` pushes the current
values onto these topics. Non-numeric nodes (strings) are skipped with a one-time warning per
node.

This lets ROS 2 consumers (rqt plots, nav2 behavior trees, diagnostic aggregators) consume
PLC state without having to speak OPC-UA directly.

Security
========

The current implementation uses anonymous OPC-UA authentication with ``SecurityPolicy=None``.
This is explicitly **not suitable for untrusted networks** - the plugin logs a warning on
startup. Proper username/password and certificate-based authentication are planned.

Supported PLC servers
=====================

The plugin speaks plain OPC-UA and supports all four node identifier types
defined by OPC 10000-6 section 5.3.1.10 (``i=`` numeric, ``s=`` string,
``g=`` GUID, ``b=`` opaque ByteString). This means the same binary works
against any compliant OPC-UA server without a code change - only the
``node_map.yaml`` changes per vendor. ``config/tank_demo_nodes.yaml`` carries
ready-to-paste examples for OpenPLC (this demo), Siemens S7-1500 TIA Portal,
Beckhoff TwinCAT 3, Allen-Bradley via Kepware, and KUKA KR C5.

What the plugin can NOT do today (regardless of vendor):

- Complex OPC-UA types: only scalar Variables (``int``, ``float``, ``bool``,
  ``string``) are mapped. Structures, arrays, enums, unions and
  ExtensionObjects are ignored.
- Vendor information models: profiles like Euromap 77 (injection molding),
  Siemens DI (device integration), and PA-DIM (process automation) are not
  interpreted natively. Tags exposed through those models can still be
  mapped manually via their node IDs, but the plugin does not understand
  the model semantics.
- Native OPC-UA Alarms & Conditions: the plugin detects alarms by applying
  thresholds to polled values. Servers that publish native AlarmCondition
  events (e.g. Siemens ``AlarmConditionType``) are not subscribed to.

Future work
===========

Tracked issues in the `ros2_medkit issue tracker
<https://github.com/selfpatch/ros2_medkit/issues>`_:

- `#367 <https://github.com/selfpatch/ros2_medkit/issues/367>`_
  Certificate-based OPC-UA authentication (Basic256Sha256)
- `#368 <https://github.com/selfpatch/ros2_medkit/issues/368>`_
  Optional auto-browse of the OPC-UA address space to seed the node map
  (deferred pending validated user demand; UaExpert and ``python -m
  asyncua.tools.uals`` are recommended for now)
- `#366 <https://github.com/selfpatch/ros2_medkit/issues/366>`_
  Vendor ``open62541pp`` inline so the package can be added to the
  rosdistro release list

Untracked (open the issue if you hit the pain):

- Hot-reload of the node map without restarting the plugin
- Complex OPC-UA type support (structures, arrays, enums)
- Vendor information model bindings (Euromap 77, Siemens DI, PA-DIM)


Native ``AlarmConditionType`` event subscription (issue #386)
=============================================================

The plugin subscribes to native OPC-UA Part 9 ``AlarmConditionType`` events
emitted by vendor PLCs, in addition to the threshold-based polling path. Both
modes coexist in a single ``node_map.yaml`` (different YAML keys) and feed the
same ``fault_manager`` service.

Configuration
-------------

Two YAML forms describe alarms; an entry can use one but never both:

.. code-block:: yaml

   nodes:
     # Threshold-based (existing): polls the scalar value and raises a fault
     # when it crosses the configured threshold.
     - node_id: 'ns=2;i=2'
       entity_id: tank_process
       data_name: tank_temperature
       data_type: float
       alarm:
         fault_code: TANK_OVERHEAT
         severity: ERROR
         threshold: 80.0
         above_threshold: true

   event_alarms:
     # Native AlarmConditionType (new): subscribes to events emitted from
     # the source NodeId and bridges them through the state machine below.
     - alarm_source: 'ns=4;s=Alarms.Overpressure'
       entity_id: tank_process
       fault_code: PLC_OVERPRESSURE
       severity_override: ERROR        # optional - else derived from event Severity
       message: 'Tank overpressure'    # optional - else event Message field

State machine
-------------

Inputs from each event payload (positional ``EventFilter`` select clauses):

- ``ConditionId`` - NodeId; null means the notification is not an A&C Condition
  (a plain BaseEvent / system message), gated by rule 0 below before the rest
- ``EnabledState.Id`` - bool
- ``ShelvingState.CurrentState.Id`` - NodeId; non-Unshelved => suppressed
- ``ActiveState.Id`` - bool
- ``AckedState.Id`` - bool
- ``ConfirmedState.Id`` - bool
- ``BranchId`` - NodeId; non-null means historical branch (Part 9 §5.5.2.12)

Decision order, first match wins:

+-----+--------------------------------------+---------------------------------------+
| #   | Condition                            | Outcome                               |
+=====+======================================+=======================================+
| 0   | ``ConditionId == null``              | not a Condition: dropped for every    |
|     | (non-condition event, e.g. a         | alarm source - explicit and auto -    |
|     | system message)                      | before the state machine runs         |
+-----+--------------------------------------+---------------------------------------+
| 1   | ``BranchId != null``                 | history-only (no SOVD update)         |
+-----+--------------------------------------+---------------------------------------+
| 2   | ``EnabledState == false``            | clear if was active, else no-op       |
+-----+--------------------------------------+---------------------------------------+
| 3   | ``ShelvingState/CurrentState/Id``    | clear if was active, else no-op.      |
|     | in {``TimedShelved`` (i=2930),       | A null/unset/unknown ``Id`` is        |
|     | ``OneShotShelved`` (i=2932)}         | treated as ``Unshelved`` (some        |
|     |                                      | servers leave the optional field      |
|     |                                      | uninitialized).                       |
+-----+--------------------------------------+---------------------------------------+
| 4   | ``ActiveState == true``              | ``CONFIRMED`` (idempotent)            |
+-----+--------------------------------------+---------------------------------------+
| 5a  | ``ActiveState == false`` and         | internal ``HEALED`` state; ``/faults``|
|     | not (``Acked`` and ``Confirmed``)    | still shows ``CONFIRMED`` (see note   |
|     |                                      | below)                                |
+-----+--------------------------------------+---------------------------------------+
| 5b  | ``ActiveState == false``,            | ``CLEARED``                           |
|     | ``Acked == true`` and                |                                       |
|     | (``Confirmed == true`` or            |                                       |
|     | not ``require_confirm_for_clear``)   |                                       |
+-----+--------------------------------------+---------------------------------------+

Rule 0 is enforced in the poller's ``on_event`` (via ``is_condition_event``)
before any state-machine input is built, and applies to every alarm source -
explicit ``event_alarms`` (including a source-level ``fault_code`` catch-all on
``ns=0;i=2253``) and zero-config ``auto_alarms`` alike. A drop on an explicit
source is additionally logged once as an operator warning, since a real alarm
that genuinely lacks a ConditionId would otherwise vanish without a trace.

``require_confirm_for_clear`` (default ``true``) gates rule 5b on both
Acknowledge and Confirm. Some servers do not implement the optional ``Confirm``
transition - Siemens S7-1500 supports ``Acknowledge`` / ``ConditionRefresh`` /
``AddComment`` but not ``Confirm`` - so ``ConfirmedState`` never becomes true
and the alarm would latch in ``HEALED`` forever. Setting the flag ``false``
(YAML ``require_confirm_for_clear`` / env ``OPCUA_REQUIRE_CONFIRM_FOR_CLEAR``)
clears such an alarm on ``Acknowledge`` alone. The default is unchanged
(spec-strict, no regression) and the relaxed path still requires
acknowledgement; it needs real-S7-1500 validation (issue #478).

``Retain`` is NOT used by this live state-machine table (lifecycle is driven by
Active / Acked / Confirmed; per Part 9 §5.5.2.10 ``Retain`` controls visibility
during ``ConditionRefresh`` bursts). It IS used in the read-based reconnect
replay path (below) as the interesting-state filter, mirroring what
``ConditionRefresh`` would replay. The SOVD ``PREFAILED`` state has no native
equivalent and is reserved for the threshold-polling pre-trigger path.

.. note::

   **HEALED is internal-only.** Rule 5a transitions the bridge's internal
   ``SovdAlarmStatus`` to ``Healed`` and emits the ``ReportHealed`` action,
   but ``OpcuaPlugin::on_event_alarm`` deliberately treats that action as a
   no-op. The reason: ``ros2_medkit_msgs/srv/ReportFault`` has only
   ``EVENT_FAILED`` / ``EVENT_PASSED`` verbs, and routing ``EVENT_PASSED``
   on every latch would feed the fault_manager debounce engine - which has
   its own statistical ``HEALED`` semantics (``healing_threshold`` cycles
   of PASSED events) and may auto-clear the fault, defeating Part 9's
   mandatory ack/confirm contract. Until ``ros2_medkit_msgs/msg/Fault``
   gains a ``STATUS_LATCHED`` (or equivalent) value, ``/faults`` keeps
   ``status=CONFIRMED`` for a latched alarm; the next ``Cleared`` (rule 5b)
   removes the entry.

   This is a known UX gap: an operator looking at ``/faults`` cannot
   distinguish "alarm physically active" from "alarm physically cleared,
   awaiting confirm". The ack/confirm SOVD operations work end-to-end; the
   visibility limitation is tracked as a follow-up.

Severity mapping
----------------

OPC-UA severity is a 1-1000 scalar. The plugin maps it to selfpatch's SOVD
severity buckets:

- 1-200    -> ``INFO``
- 201-500  -> ``WARNING``
- 501-800  -> ``ERROR``
- 801-1000 -> ``CRITICAL``

This is the selfpatch convention, **not** IEC 62682 - that spec defines a
1-1000 priority scale but no normative band names. ``severity_override`` on
an ``event_alarms`` entry takes precedence when set.

ConditionRefresh
----------------

After creating event monitored items the plugin invokes ``ConditionRefresh``
so the server pushes any condition that fired before the subscription started.
The same call fires on every successful reconnect.

Per Part 9 §5.5.7 ``ConditionRefresh`` is a Method of the **ConditionType**, so
the Call's ObjectId is the well-known ConditionType node ``i=2782`` (method
``i=3875``), NOT the Server object ``i=2253``. Calling it on the Server object
was the root cause of ``BadNodeIdUnknown`` rejections on conformant servers,
including Siemens S7-1500, which documents ``ConditionRefresh`` as supported
(only the optional ``ConditionRefresh2`` is not). With the corrected target the
method is expected to succeed on Siemens, making the read fallback unnecessary
there (issue #478).

The bracketing ``RefreshStartEventType`` (i=2787) and ``RefreshEndEventType``
(i=2788) are recognized and used to set a diagnostic flag; live notifications
arriving during the burst are applied normally because the state machine is
driven by per-condition ``ConditionId`` and runs idempotently.

Read-based replay fallback
--------------------------

When ``ConditionRefresh`` is rejected (``condition_replay_strategy`` =
``read`` / ``auto``) the plugin browses each ``alarm_source`` for AlarmCondition
instances - hierarchical Object children plus targets of the ``HasCondition``
reference (``i=9006``), recursing one level - reads their current state, and
drives the same state machine. It mirrors ``ConditionRefresh`` semantics:

- only conditions with ``Retain == true`` form the active set;
- a ``Disabled`` condition (``EnabledState/Id = false``) is never active;
- a transient browse/read failure keeps the condition (it is not dropped and
  must not be reconciled away).

**Safety gate (issue #478).** The reconcile that clears faults for conditions
no longer present NEVER fires for a source that has not positively exposed at
least one Condition instance node. An EventNotifier-only server (S7-1500 models
no per-condition instance nodes) returns a successful but empty browse; rather
than wiping the tracked active alarms, the plugin preserves them, logs an
operator warning, and relies on ``ConditionRefresh`` / live transitions. This
prevents the net-negative outcome where a reconnect erased a still-active alarm.

Acknowledge / Confirm round-trip
--------------------------------

Two SOVD operations appear on every entity that has at least one event-mode
alarm declared, in a write-capable build. Both change condition state on the
controller, so the default read-only build neither lists nor performs them (see
"Read-only is a property of the build" above):

- ``POST /apps/{entity}/operations/acknowledge_fault/executions``
- ``POST /apps/{entity}/operations/confirm_fault/executions``

Body:

.. code-block:: json

   { "fault_code": "PLC_OVERPRESSURE", "comment": "operator on radio" }

The plugin resolves ``(entity_id, fault_code)`` to the live ``ConditionId``
maintained by the poller, then calls the inherited
``AcknowledgeableConditionType`` method (``Acknowledge`` ``i=9111`` or
``Confirm`` ``i=9113``) on that NodeId. The latest ``EventId`` ``ByteString``
captured from the most recent notification is passed as the first argument;
without it servers return ``BadEventIdUnknown`` (Part 9 §5.7.3).

Vendor matrix
-------------

+----------------------+---------------------+-------------------------------+
| Vendor / runtime     | AlarmConditionType  | Notes                         |
+======================+=====================+===============================+
| Siemens S7-1500      | yes (FW V2.9+)      | ProDiag, Program_Alarm,       |
|                      |                     | system diagnostics.           |
|                      |                     | ConditionRefresh +            |
|                      |                     | Acknowledge supported, NO     |
|                      |                     | Confirm (set                  |
|                      |                     | require_confirm_for_clear     |
|                      |                     | false). EventNotifier-only:   |
|                      |                     | no condition instance nodes   |
+----------------------+---------------------+-------------------------------+
| Beckhoff TwinCAT 3   | yes (TF6100)        | ``Confirm`` propagates to     |
|                      |                     | PLC code; ``Ack`` does not    |
+----------------------+---------------------+-------------------------------+
| Rockwell ControlLogix| yes (via FactoryTalk| Tag-based alarms bridged by   |
|                      | Linx FW 16.20+)     | the gateway                   |
+----------------------+---------------------+-------------------------------+
| CodeSys 3.5+         | yes (alarm manager  | Custom severity mapping       |
|                      | provider library)   |                               |
+----------------------+---------------------+-------------------------------+
| OpenPLC v3           | no                  | Scalar variables only;        |
|                      |                     | use threshold mode            |
+----------------------+---------------------+-------------------------------+

Out of scope
------------

- ``ShelvingState`` write operations (``TimedShelve`` / ``OneShotShelve`` /
  ``Unshelve``). The plugin reads the state to suppress active alarms but
  does not yet expose operator UI to set it.
- OPC-UA branch reasoning beyond ``BranchId``-based suppression.
  Re-fires are tracked via ``fault_manager`` ``occurrence_count`` plus the
  ``/faults/stream`` SSE history.
- Auto-discovery of alarm sources via ``Server.GeneratedEvents`` browse
  (tracked in #368 alongside scalar auto-discovery).
- ``Quality`` (StatusCode) propagation to a SOVD ``status_quality`` field.
  Requires an additive field on the ``ReportFault.srv`` schema; tracked
  separately.
