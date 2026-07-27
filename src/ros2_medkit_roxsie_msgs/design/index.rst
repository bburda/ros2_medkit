ros2_medkit_roxsie_msgs
=======================

This section contains design documentation for the ros2_medkit_roxsie_msgs package.

Overview
--------

Siemens SIMATIC ROS Connector (ROXSIE) generates, from a YAML configuration, both the
PLC-side blocks and a ROS 2 package that exchange declared data between a SIMATIC PLC
and ROS 2 over shared memory on the host. The generated node sits on the ROS 2 graph,
so anything already listening to that graph can consume the data.

Alarms are the gap. The configuration surface covers declared data topics, while the
alarm system is a separate mechanism inside the CPU: numbered alarms, coming and going
states, acknowledgement, and values latched at the moment the alarm fired. A machine
that combines a SIMATIC PLC with ROS 2 therefore still keeps two fault stories that
never meet, and joining them today means reading alarm bits over a protocol and
rebuilding their meaning from an engineering export, or copying alarms by hand into a
second list on the panel.

This package defines the two messages that close the gap. It has no runtime code - only
``.msg`` definitions compiled by ``rosidl`` into C++ and Python bindings.

Design
------

Two directions, deliberately asymmetric.

**Alarms flow out in full.** Every alarm event the CPU alarm system produces is published
as ``AlarmEvent``: which alarm, coming or going, acknowledged or not, the PLC's own
timestamp, and the values latched when it fired. The bridge turns each event into a fault
with a freeze-frame and an audit trail.

**Only a status word flows back in.** The diagnostics side does not push faults into the
PLC. It publishes ``DiagnosticsStatus``, four fixed values that the generated node writes
into a consumed data block: heartbeat, worst severity, active count, coarse class bits.

The asymmetry is the point:

* Fault text is dynamic and unbounded. String handling inside a PLC costs determinism,
  which is the one property a PLC exists to provide.
* A fault list inside the PLC would be a second copy of the fault list, and two copies
  drift apart. The value of one source of truth disappears the moment it is duplicated.
* What the machine needs from the diagnostics side is a condition it can act on: a lamp
  for the operator, a bit for an interlock. Everything richer is read over REST by
  whoever needs it, at the moment they need it.

Alarm texts never travel at runtime. The generator emits a static alarm number to text
table next to the generated code, and the bridge resolves the text on the ROS 2 side.

Acknowledgement is one shared state. Whoever acknowledges first - an operator on the
panel or a client on the ROS 2 side - every other consumer sees the same value rather
than keeping a private copy.

Architecture
------------

.. plantuml::
   :caption: ROXSIE alarm integration - data flow

   @startuml roxsie_alarm_integration_architecture

   skinparam linetype ortho

   package "SIMATIC PLC" {
       [CPU alarm system] as AS
       [ROXSIE generated blocks] as GEN
       [Consumed status block] as DB
       [Signal column\nMachine interlocks] as SIG
   }

   package "ROS 2" {
       [ROXSIE generated node] as NODE
       [ros2_medkit_roxsie_bridge] as BR
       [FaultManager] as FM
       [SOVD gateway] as GW
   }

   [Panel page, dashboards,\nmaintenance clients] as CONS

   AS --> GEN : alarm events
   GEN --> NODE : shared memory
   NODE --> BR : AlarmEvent
   BR --> FM : ReportFault
   FM --> BR : fault events
   BR --> NODE : DiagnosticsStatus
   NODE --> GEN : shared memory
   GEN --> DB
   DB --> SIG
   FM -- GW
   GW --> CONS : HTTPS

   @enduml

Deployment note: the generated node exchanges data with the PLC over shared memory on
the same host, but publishes on the ROS 2 graph over the host network. The diagnostics
side therefore does not have to run on the controller; it subscribes like any other ROS 2
participant.

What the Generator Has to Provide
---------------------------------

The messages describe the wire. The generated code is what makes the alarms reachable at
all, so its obligations are part of the contract:

* **Observe the alarm system, not a data block mirror.** Alarm events live in the CPU
  message system, not in ordinary data blocks. Reaching them needs either a native
  subscription or program code that reads pending alarms and forwards them.
* **Latch values at trigger time.** ``associated_values`` must be what the alarm system
  captured when the alarm fired. Re-reading them afterwards produces a plausible but
  wrong freeze-frame.
* **Timestamp at the source.** ``plc_timestamp`` comes from the PLC clock in UTC, not
  from the moment of publishing.
* **Re-publish pending alarms after a restart.** A node that starts fresh must send the
  alarms that are currently active, otherwise an active alarm silently disappears from
  the diagnostics side for as long as it stays active.
* **Emit the alarm number to text table.** Generated once from the TIA project alongside
  the code, consumed on the ROS 2 side. The table travels with the deployment, not with
  each event.
* **Write the consumed status block.** Four values into the data block, so that a
  watchdog in the PLC program can turn a stalled heartbeat into its own alarm.

Message Definitions
-------------------

.. list-table::
   :header-rows: 1
   :widths: 30 70

   * - Message
     - Purpose
   * - ``AlarmEvent.msg``
     - One alarm event from the CPU alarm system: identifier, source, class, coming or
       going, acknowledgement state, PLC timestamp, latched values
   * - ``DiagnosticsStatus.msg``
     - Aggregate diagnostics state towards the PLC: heartbeat, worst severity, active
       count, coarse class bits, scope

Lifecycle
---------

A PLC alarm and a medkit fault already model the same thing, so the mapping is direct
rather than invented.

.. list-table::
   :header-rows: 1
   :widths: 40 60

   * - Alarm event
     - Fault behaviour
   * - ``STATE_COMING``
     - Reported as a FAILED event; debounce confirms it and a freeze-frame is captured
       from the latched values
   * - ``STATE_GOING``
     - Reported as a PASSED event; the fault heals but is not deleted, so history stays
       for the audit trail
   * - ``ACK_ACKNOWLEDGED``
     - The fault is cleared; the acknowledgement is the same state on both sides

Failure Behaviour
-----------------

.. list-table::
   :header-rows: 1
   :widths: 40 60

   * - Failure
     - Behaviour
   * - Diagnostics layer or link down
     - The heartbeat stops and a watchdog in the PLC program raises its own alarm, so the
       plant sees that diagnostics are offline rather than a stale healthy state
   * - PLC or the generated node down
     - The bridge reports a communications fault on the ROS 2 side
   * - Bridge restarts
     - Currently pending alarms are re-published, so a restart does not silently lose
       active alarms

The first row is the load-bearing one: the failure mode of the diagnostics layer must
never be indistinguishable from a healthy machine.

Open Questions
--------------

#. Is ``alarm_id`` unique per CPU, or only per block? If only per block, the unique key
   has to be ``(source, alarm_id)`` and the bridge cannot key on the number alone.
#. Numeric or typed associated values? Numeric keeps the message fixed-size and cheap on
   the PLC side; typed is richer but reintroduces strings.
#. Can the generated code acknowledge an alarm in the CPU alarm system, or is
   acknowledgement panel-only? It decides whether an acknowledgement made on the ROS 2
   side can reach the native alarm list.
#. Who owns the class catalogue behind ``class_bitmask``, and is 16 bits enough?
#. Topic names and QoS for the two topics, and whether they are namespaced per cell.
