ros2_medkit_roxsie_msgs
=======================

This section contains design documentation for the ros2_medkit_roxsie_msgs package.

Overview
--------

ROXSIE generates the PLC blocks and a ROS 2 package for the data you declare in the YAML
config, exchanged over shared memory on the host. The generated node sits on the ROS 2
graph, so anything already listening to that graph can read the data.

That covers process values. It does not cover alarms, because alarms live in the CPU
message system and not in a data block: numbered alarms, coming and going states,
acknowledgement, and the values the alarm system latched when the alarm fired.

So on a machine with a SIMATIC PLC and ROS 2 you still have two separate lists of what is
wrong. To join them today you either read alarm bits over a protocol and rebuild their
meaning from an engineering export, or someone copies the alarms by hand into a second
list on the panel. Both drift.

This package holds the two message definitions for that exchange. It has no runtime code,
only ``.msg`` definitions that ``rosidl`` compiles into C++ and Python bindings.

Design
------

The two directions are not the same size, on purpose. Every alarm goes out with its full
context as ``AlarmEvent``: which alarm, coming or going, acked or not, the PLC timestamp,
and the latched values. The bridge turns each event into a fault with a freeze-frame and
an audit trail. Back the other way we send four numbers as ``DiagnosticsStatus``, and the
generated node writes them into a consumed data block.

Alarm text has no fixed length and changes with the project. A PLC that handles strings
loses determinism, and determinism is why it is there. Writing the fault list into the PLC
would give us two lists, and two lists drift.

The PLC only needs something it can switch on: a lamp for the operator, a bit for an
interlock. Whoever wants the detail reads it over REST.

Alarm text does not travel at runtime. The generator writes an alarm number to text table
next to the code, and the bridge looks the text up on the ROS 2 side.

Acknowledgement is one shared value. The operator acks on the panel or a client acks over
REST, and both sides see the same thing.

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

The generated node talks to the PLC over shared memory on the same host, but publishes on
the ROS 2 graph over the network. So the diagnostics side does not have to run on the
controller. It subscribes like any other ROS 2 participant.

What the Generator Has to Provide
---------------------------------

The messages only describe the wire. The generated code is what gets the alarms out, so
these belong to the contract:

* **Read the CPU message system, not a data block mirror.** That is where alarm events
  are.
* **Put the latched values into the message.** If you read them again later, the
  freeze-frame looks right and is wrong.
* **Use the PLC clock for** ``plc_timestamp``, not the publish time.
* **After a restart, send the alarms that are still active.** Otherwise an active alarm
  disappears from our side until it goes away by itself.
* **Emit the alarm number to text table** when you generate the code.
* **Write the four values into the consumed data block**, so a watchdog in the program
  can raise its own alarm when the heartbeat stops.

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

A PLC alarm and a medkit fault model the same thing, so the mapping is direct.

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

The first row matters most. If our layer dies, the plant has to see it.

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
