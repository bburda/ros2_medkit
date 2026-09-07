ros2_medkit_roxsie_msgs
=======================

This section contains design documentation for the ros2_medkit_roxsie_msgs package.

Overview
--------

ROXSIE generates the PLC blocks and a ROS 2 package for the data you declare in the YAML
config, exchanged over shared memory on the host. The generated node sits on the ROS 2
graph, so anything already listening to that graph can read the data.

That covers process values. It does not cover alarms, because alarms live in the CPU
message system and not in a data block.

So on a machine with a SIMATIC PLC and ROS 2 you still have two separate lists of what is
wrong. To join them today you either read alarm bits over a protocol and rebuild their
meaning from an engineering export, or someone copies the alarms by hand into a second
list on the panel. Both drift.

This package holds the messages for that exchange. It has no runtime code, only ``.msg``
definitions that ``rosidl`` compiles into C++ and Python bindings.

A Picture, Not a Stream
-----------------------

``AlarmList`` carries every alarm that is active or waiting for acknowledgement, published
cyclically. Two reasons.

The transport is cyclic anyway. The PLC-side node reads a data block at a fixed rate, so a
picture is what it naturally has. An event stream would be built on top of that and would
need its own delivery guarantees.

A picture also survives a restart. Whoever restarts, the next cycle carries the full truth
again. An event stream loses whatever happened while it was away, and the consumer cannot
tell an empty stream from a healthy machine. The gap is not hypothetical: reading alarms
over OPC UA has the same hole, because no table of pending alarms is served on reconnect.

The bridge compares consecutive pictures. An entry that appears is reported as a fault, an
entry that disappears heals it. Debounce, freeze-frame capture and the audit trail are the
same ones every other fault source in medkit goes through.

Two Ways to Fill the List
-------------------------

**From bits.** The customer declares alarm bits in a data block and ROXSIE carries them
like any other declared data. Nothing new is needed on the generator side, so this path
works today. ``alarm_class``, ``priority``, ``producer``, ``plc_timestamp`` and
``associated_values`` stay empty, and the freeze-frame comes from the process topics the
same deployment already carries. Acknowledgement is a bit, so either side can set it.

**From the CPU alarm system.** Generated code reads the alarm system with ``Get_Alarm`` and
mirrors it into the data block. Still no OPC UA and no license. This fills everything the
message can hold:

* the event time from the CPU clock, rather than the moment ROS 2 noticed
* the alarm class, which says whether the alarm needs acknowledgement
* the priority the plant already engineers with, 0 to 16
* the values the alarm system latched when the alarm fired, capped by the CPU at 512 bytes
  across all of them

It also picks up alarms nobody declared. The CPU message system carries system
diagnostics, ProDiag, GRAPH, motion and security alarms, and ``producer`` says which one an
entry came from. A module fault or a drive alarm arrives without any extra engineering,
which is the strongest argument for this path.

The trade is acknowledgement. The CPU owns the acknowledged state and has no per-alarm
acknowledge instruction. ``Ack_Alarms`` takes ``EN``, ``MODE``, ``ERROR`` and ``STATUS``
and nothing else. ``MODE = 1`` confirms every pending alarm at once, in batches of 100,
with ``STATUS`` 7001 while more are pending (Siemens, 2026-09-03). So the bridge never
calls it. Acknowledging one alarm needs the panel or OPC UA, and acknowledgement from the
ROS 2 side stays a bit-path feature. On this path we mirror the state rather than write it.

One cost worth naming: ``Get_Alarm`` reads from a queue, one entry at a time, while the
data block has to hold a picture. The generated code has to keep that picture itself. That
is real work, not a flag.

Alarm Ids Move
--------------

``alarm_id`` is unique within a CPU, and that is all it promises. Copying a block and
recompiling can renumber alarms, and two CPUs on the same line can use the same number
(Siemens, 2026-09-03). So the bridge keys alarms on ``(PLC, alarm_id)`` and never trusts
the id alone across a project change.

Every ``AlarmList`` carries ``catalog_version``: a hash over the alarm catalog the
generator built the list from. The bridge keeps the same hash next to its mapping and
compares on every picture. If they differ, the project was rebuilt and the numbers may
have moved, so the bridge raises a fault on itself instead of resolving ids against the
wrong table. A hand-kept mapping fails the other way: it keeps pointing at the wrong alarm
and nobody notices.

Architecture
------------

.. plantuml::
   :caption: ROXSIE alarm integration - data flow

   @startuml roxsie_alarm_integration_architecture

   skinparam linetype ortho

   package "SIMATIC PLC" {
       [CPU alarm system] as AS
       [Alarm bits] as BITS
       [Alarm list block] as GEN
       [Consumed status block] as DB
       [Machine logic\nSignal column] as SIG
   }

   package "ROS 2" {
       [ROXSIE generated node] as NODE
       [ros2_medkit_roxsie_bridge] as BR
       [FaultManager] as FM
       [SOVD gateway] as GW
   }

   [Panel page, dashboards,\nmaintenance clients] as CONS

   AS --> GEN : Get_Alarm
   BITS --> GEN : bit path
   GEN --> NODE : shared memory
   NODE --> BR : AlarmList
   BR --> FM : ReportFault
   FM --> BR : fault events
   BR --> NODE : DiagnosticsStatus
   NODE --> DB : shared memory
   DB --> SIG
   FM -- GW
   GW --> CONS : HTTPS

   @enduml

The generated node talks to the PLC over shared memory on the same host, but publishes on
the ROS 2 graph over the network. So the diagnostics side does not have to run on the
controller. It subscribes like any other ROS 2 participant.

What Goes Back Into the PLC
---------------------------

One value and a heartbeat. ``DiagnosticsStatus`` says what the diagnostics side is asking
of the machine: nothing, something is abnormal, someone has to act, or stop.

It does not say what to light up. Machines already have a state machine, and where PackML
is used, a block that turns machine state into signal column bits according to
ISA TR88.00.02. Driving the lamp ourselves would put two writers on it. We hand over a
condition and let the machine's own logic decide what that means for the lamp, for an
interlock, or for a state transition.

The condition covers every active fault in scope, not only the ones that came from the PLC.
A navigation fault on the robot moves the same value, which is how a problem on the ROS 2
side reaches an operator who never looks at a screen.

``class_bitmask`` is there for machine logic that needs to interlock on a kind of fault
without reading a list. Which fault sets which bit is deployment configuration.

Message Definitions
-------------------

.. list-table::
   :header-rows: 1
   :widths: 30 70

   * - Message
     - Purpose
   * - ``Alarm.msg``
     - One alarm: identity, source, class, priority, producer, active and acknowledged
       flags, PLC timestamp, latched values
   * - ``AlarmList.msg``
     - The cyclic picture: every alarm active or not yet acknowledged, the catalog
       version it was built from, and whether the block was too small to hold it all
   * - ``DiagnosticsStatus.msg``
     - What the diagnostics side asks of the machine: heartbeat, condition, class bits,
       scope

Alarm text does not travel at runtime. The text belongs to the project, changes with the
project, and is resolved on the ROS 2 side from a table that ships with the deployment.
That table comes from a TIA Openness export of the project's PLC alarm text lists: alarm
instances with texts, info texts, classes and languages, repeatable when the project
changes. The ``.psc`` export does not carry it (Siemens, 2026-09-03). The bridge mapping is
generated from the same export, and ``catalog_version`` pins the two together.

Failure Behaviour
-----------------

.. list-table::
   :header-rows: 1
   :widths: 40 60

   * - Failure
     - Behaviour
   * - Diagnostics layer or link down
     - The heartbeat stops and a watchdog in the PLC program raises its own alarm, so the
       plant sees that diagnostics are down rather than a green light that means nothing
   * - PLC or the generated node down
     - The bridge reports a communications fault on the ROS 2 side
   * - Anything restarts
     - The next picture carries the full set of active alarms, so nothing is silently lost

The first row matters most. If our layer dies, the plant has to see it.

What Comes After
----------------

This package covers alarms from the PLC and one condition back. Two extensions follow, both
shaped by what a deployment needs rather than by what is interesting to build.

**OpenTelemetry as another fault source.** A cell is more than a PLC and a robot. The switch
that flaps, the controller that runs hot, the compute in another building that this machine
depends on - none of them speak the PLC alarm system, and most of them already speak
OpenTelemetry. Every one of those vendors ships its own API, and asking each of them for an
integration does not scale; asking all of them for OpenTelemetry does. An OTLP receiver next
to this bridge leaves three things to settle:

* **Which signal becomes a fault.** Log records at error level are the obvious answer.
  Metrics are the more interesting one, because ``ros2_medkit_fault_detection`` already
  evaluates thresholds and status words for PLC tags and can take a metric the same way.
* **Which entity a stream belongs to.** Resource attributes have to map onto the tree, or
  every host lands in one unattached pile.
* **What the operator sees.** Nothing new to wire: an IT fault in the same list already
  moves the condition that reaches the signal column.

**OpenTelemetry as an export.** The same faults served over REST can be emitted as
OpenTelemetry, and the per-scope condition as a metric. Two renderings of one state, not two
sources of truth. Teams that already run an observability pipeline add a source rather than
integrate an API.

Neither exists today. There is a hook where an exporter plugs in and nothing behind it.
Building it against a concrete consumer beats guessing at a generic one.

Open Questions
--------------

Answered by Siemens (Manuel Kreutz, 2026-09-03, after tryouts on hardware):

#. ``alarm_id`` is unique within a CPU, so inside one PLC the number is the key and
   ``(source, alarm_id)`` is not needed. Two CPUs can collide, so the bridge keys on
   ``(PLC, alarm_id)``. Copying a block and recompiling can change the id, which is why
   ``AlarmList.catalog_version`` exists.
#. The alarm number to text table lives in TIA under "PLC alarm text lists". The supported
   path is a TIA Openness export: alarm instances with texts, info texts, classes and
   languages, repeatable when the project changes. The ``.psc`` export does not carry it.
#. The generated data block holds 64 entries in the current generator version. The next
   version makes the length configurable, bounded only by CPU memory. The generator sizes
   the array from the export, and ``AlarmList.truncated`` covers the rest.
#. There is no per-alarm acknowledge in the CPU. ``Ack_Alarms`` takes ``EN``, ``MODE``,
   ``ERROR`` and ``STATUS``, and ``MODE = 1`` confirms every pending alarm, in batches of
   100. Acknowledgement from the ROS 2 side stays a bit-path feature. On the ``Get_Alarm``
   path the bridge mirrors the CPU state and never calls ``Ack_Alarms``.

Still open:

#. Numeric associated values, or typed? Numeric keeps the entry fixed size. Typed carries
   more and brings strings back.
#. Who emits the bridge mapping. The TIA Openness export carries what the mapping needs,
   so it can be generated from the export on the ROS 2 side. Whether ROXSIE emits it next
   to the generated code, so that one tool run produces both and stamps both with the same
   ``catalog_version``, is for the design phase.
