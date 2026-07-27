ros2_medkit_plc_msgs
====================

This section contains design documentation for the ros2_medkit_plc_msgs package.

Overview
--------

A machine that combines a PLC and ROS 2 has two fault stories that never meet. The
PLC has a mature alarm system: numbered alarms, coming and going states,
acknowledgement, and values latched at the moment the alarm fired. ROS 2 has faults,
freeze-frames and a diagnostics tree. Joining them today means reading alarm bits over
a protocol and rebuilding their meaning from an engineering spreadsheet, or copying
alarms by hand into a second list. Both lose semantics, and both end with two fault
lists that drift apart.

This package defines the interface that removes the guesswork. It has no runtime code -
only ``.msg`` definitions that are compiled by ``rosidl`` into C++ and Python bindings.

Design
------

Two directions, deliberately asymmetric.

**Alarms flow out in full.** Every alarm event the PLC alarm system produces is
published as ``PlcAlarm``: which alarm, coming or going, acknowledged or not, the PLC's
own timestamp, and the values latched when it fired. A bridge turns each event into a
fault with a freeze-frame and an audit trail.

**Only a status word flows back in.** The diagnostics side does not push faults into the
PLC. It publishes ``PlcDiagnosticsStatus``, four fixed values written into a small data
block: heartbeat, worst severity, active count, coarse class bits.

The asymmetry is the point:

* Fault text is dynamic and unbounded. String handling inside a PLC costs determinism,
  which is the one property a PLC exists to provide.
* A fault list inside the PLC would be a second copy of the fault list, and two copies
  drift apart. The value of one source of truth disappears the moment it is duplicated.
* What the machine needs from the diagnostics side is a condition it can act on: a lamp
  for the operator, a bit for an interlock. Everything richer is read over REST by
  whoever needs it, at the moment they need it.

Alarm texts never travel at runtime. The mapping from alarm number to text is static,
exported once from the engineering project, and resolved on the ROS 2 side.

Acknowledgement is one shared state. Whoever acknowledges first - an operator on the
panel or a client on the ROS 2 side - every other consumer sees the same value rather
than keeping a private copy.

Architecture
------------

.. plantuml::
   :caption: PLC alarm interface - data flow

   @startuml plc_alarm_interface_architecture

   skinparam linetype ortho

   package "PLC" {
       [Alarm system] as AS
       [PLC-side producer\n(e.g. ROXSIE generated node)] as GLUE
       [Status data block] as DB
       [Signal column\nMachine interlocks] as SIG
   }

   package "ROS 2" {
       [plc_alarm_bridge] as AB
       [plc_status_publisher] as SP
       [FaultManager] as FM
       [SOVD gateway] as GW
   }

   [Panel page, dashboards,\nmaintenance clients] as CONS

   AS --> GLUE : alarm events
   GLUE --> AB : PlcAlarm
   AB --> FM : ReportFault
   FM --> SP : fault events
   SP --> GLUE : PlcDiagnosticsStatus
   GLUE --> DB
   DB --> SIG
   FM -- GW
   GW --> CONS : HTTPS

   @enduml

Producer Requirements
---------------------

The interface only describes the wire. The publishing side runs on or next to the
PLC and is what makes the alarms reachable at all, so its obligations are part of
the contract:

* **Observe the alarm system, not a tag mirror.** Alarm events live in the CPU
  message system, not in ordinary data blocks. The producer needs a path to them,
  either a native subscription or program code that reads pending alarms and
  forwards them.
* **Latch values at trigger time.** ``associated_values`` must be the values the
  alarm system captured when the alarm fired, not values read later. Re-reading
  them afterwards produces a plausible but wrong freeze-frame.
* **Timestamp at the source.** ``plc_timestamp`` comes from the PLC clock in UTC.
  It is not the publish time.
* **Re-publish pending alarms after a restart.** A producer that starts fresh must
  send the alarms that are currently active, otherwise an active alarm silently
  disappears from the diagnostics side for as long as it stays active.
* **Ship the alarm number to text table.** Exported once from the engineering
  project, consumed on the ROS 2 side. The table travels with the deployment, not
  with each event.
* **Write the status block and let the PLC watch it.** The consumer side writes the
  four values into a data block; a watchdog in the PLC program turns a stalled
  heartbeat into its own alarm.

**First target: Siemens SIMATIC ROS Connector (ROXSIE).** It is a code generator
that, from a YAML configuration, produces both the PLC blocks and a ROS 2 package
for exchanging declared data between a SIMATIC PLC and ROS 2. The generated node
already sits on the ROS 2 graph on the host network, so it is the natural place to
publish alarm events and to consume the status message. Its configuration surface
today covers declared data topics; carrying alarm events is an addition on that
side, which is why this interface defines only the wire and leaves the production
of the events to the generator.

Nothing in the messages depends on that generator. Any producer that can meet the
obligations above interoperates, which is the point of keeping the definitions
vendor-neutral.

Message Definitions
-------------------

.. list-table::
   :header-rows: 1
   :widths: 30 70

   * - Message
     - Purpose
   * - ``PlcAlarm.msg``
     - One alarm event from a PLC alarm system: identifier, source, class, coming or
       going, acknowledgement state, PLC timestamp, latched values
   * - ``PlcDiagnosticsStatus.msg``
     - Aggregate diagnostics state towards a PLC: heartbeat, worst severity, active
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
     - The heartbeat stops and a watchdog in the PLC raises its own alarm, so the plant
       sees that diagnostics are offline rather than a stale healthy state
   * - PLC down or unreachable
     - The bridge reports a communications fault on the ROS 2 side
   * - Bridge restarts
     - Currently pending alarms are re-published, so a restart does not silently lose
       active alarms

The first row is the load-bearing one: the failure mode of the diagnostics layer must
never be indistinguishable from a healthy machine.

Open Questions
--------------

#. Is ``alarm_id`` unique per CPU, or only per block? If only per block, the unique key
   has to be ``(source, alarm_id)`` and consumers cannot key on the number alone.
#. Numeric or typed associated values? Numeric keeps the message fixed-size and cheap on
   the PLC side; typed is richer but reintroduces strings.
#. Can generated code acknowledge an alarm in the PLC alarm system, or is acknowledgement
   panel-only? It decides whether an acknowledgement made on the ROS 2 side can reach the
   native alarm list.
#. Who owns the class catalogue behind ``class_bitmask``, and is 16 bits enough?
#. A separate interface package, or these messages in the core message package? Separate
   lets a generator depend on the interface alone; core is one less package to release.
