ros2_medkit_fault_manager
=========================

This section contains design documentation for the ros2_medkit_fault_manager project.

Architecture
------------

The following diagram shows the relationships between the main components of the fault manager.

.. plantuml::
   :caption: ROS 2 Medkit Fault Manager Class Architecture

   @startuml ros2_medkit_fault_manager_architecture

   skinparam linetype ortho
   skinparam classAttributeIconSize 0

   title ROS 2 Medkit Fault Manager - Class Architecture

   package "ROS 2 Framework" {
       class "rclcpp::Node" {
           +create_service()
           +get_logger()
           +now()
       }
   }

   package "ros2_medkit_msgs" {
       class "msg::Fault" {
           +fault_code: string
           +severity: uint8
           +description: string
           +first_occurred: Time
           +last_occurred: Time
           +occurrence_count: uint32
           +status: string
           +reporting_sources: string[]
       }

       class "srv::ReportFault" {
           +Request: fault_code, severity, description, source_id
           +Response: success, message
       }

       class "srv::ListFaults" {
           +Request: filter_by_severity, severity, statuses
           +Response: faults[]
       }

       class "srv::ClearFault" {
           +Request: fault_code
           +Response: success, message
       }
   }

   package "ros2_medkit_fault_manager" {

       class FaultManagerNode {
           + get_storage(): FaultStorage&
       }

       class CaptureThreadPool {
           +enqueue(fault_code) : EnqueueOutcome
           +shutdown()
           +dropped_captures() : uint64_t
           -workers_ : vector<thread>
           -queue_ : deque<string>
       }

       enum QueueFullPolicy {
           kRejectNewest
           kDropOldest
       }

       abstract class FaultStorage <<interface>> {
           + {abstract} report_fault(): bool
           + {abstract} list_faults(): vector<Fault>
           + {abstract} get_fault(): optional<Fault>
           + {abstract} clear_fault(): bool
           + {abstract} size(): size_t
           + {abstract} contains(): bool
       }

       class InMemoryFaultStorage {
           + report_fault(): bool
           + list_faults(): vector<Fault>
           + get_fault(): optional<Fault>
           + clear_fault(): bool
           + size(): size_t
           + contains(): bool
       }

       class FaultState <<struct>> {
           + to_msg(): Fault
       }
   }

   ' Relationships

   ' Inheritance
   FaultManagerNode -up-|> "rclcpp::Node" : extends
   InMemoryFaultStorage -up-|> FaultStorage : implements

   ' Composition
   FaultManagerNode *-down-> InMemoryFaultStorage : owns
   FaultManagerNode --> CaptureThreadPool : enqueues capture jobs

   ' InMemoryFaultStorage contains FaultStates
   InMemoryFaultStorage o-right-> FaultState : contains many

   ' FaultState converts to message
   FaultState ..> "msg::Fault" : converts to

   ' Node uses service types
   FaultManagerNode ..> "srv::ReportFault" : handles
   FaultManagerNode ..> "srv::ListFaults" : handles
   FaultManagerNode ..> "srv::ClearFault" : handles

   @enduml

Main Components
---------------

1. **FaultManagerNode** - The main ROS 2 node that provides fault management services
   - Extends ``rclcpp::Node``
   - Owns a ``FaultStorage`` implementation for fault state persistence
   - Provides three ROS 2 services for fault reporting, querying, and clearing
   - Validates input parameters (fault_code, severity, source_id)
   - Logs fault lifecycle events at appropriate severity levels

2. **FaultStorage** - Abstract interface for fault storage backends
   - Defines the contract for fault storage implementations
   - Enables pluggable storage backends (in-memory, persistent, distributed)
   - Future implementations can be added in Issue #8: Fault Persistence Options

3. **InMemoryFaultStorage** - Thread-safe in-memory implementation of FaultStorage
   - Uses ``std::map`` keyed by ``fault_code`` for O(log n) lookups
   - Protected by ``std::mutex`` for concurrent service request handling
   - Aggregates reports from multiple sources into single fault entries
   - Implements severity escalation (higher severity overwrites lower)
   - Tracks occurrence counts and all reporting sources

4. **FaultState** - Internal representation of a fault entry
   - Maps directly to ``ros2_medkit_msgs::msg::Fault`` via ``to_msg()``
   - Uses ``std::set`` for reporting_sources to ensure uniqueness
   - Tracks first and last occurrence timestamps
   - Manages fault status lifecycle with debounce (PREFAILED -> CONFIRMED -> CLEARED)

5. **CaptureThreadPool** - Bounded worker pool for snapshot capture
   Under a fault storm the node enqueues capture jobs into a bounded ``CaptureThreadPool``
   (``pool_size`` workers draining a ``queue_depth``-bounded queue) rather than spawning one
   thread per fault. The full-queue policy (``reject_newest``/``drop_oldest``) bounds memory,
   and the destructor joins the pool before rosbag teardown.

Services
--------

~/report_fault
~~~~~~~~~~~~~~

Reports a new fault or updates an existing one.

- **Input validation**: fault_code and source_id cannot be empty, event_type must be valid
- **Event types**: FAILED (fault detected) or PASSED (fault condition cleared)
- **Debounce**: FAILED events decrement counter, PASSED events increment counter
- **Aggregation**: Same fault_code from different sources creates a single fault entry
- **Severity escalation**: Fault severity is updated if a higher severity is reported
- **Returns**: ``accepted=true`` if event was processed

~/list_faults
~~~~~~~~~~~~~

Queries faults with optional filtering.

- **Status filter**: Filter by status (PREFAILED, PREPASSED, CONFIRMED, HEALED, CLEARED); defaults to CONFIRMED
- **Severity filter**: When ``filter_by_severity=true``, returns only faults of specified severity
- **Returns**: List of ``Fault`` messages matching the filter criteria

~/clear_fault
~~~~~~~~~~~~~

Clears (acknowledges) a fault by setting its status to CLEARED.

- **Input validation**: fault_code cannot be empty
- **Idempotent**: Clearing an already-cleared fault succeeds
- **Returns**: ``success=true`` if fault existed, ``success=false`` if not found

~/set_planned_stop and ~/get_planned_stop
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Declares, withdraws and reads the planned stop.

- **One declaration per manager**: the stop is a fact about the installation, not about a fault
- **Idempotent**: a request for the state the switch is already in succeeds, changes nothing and writes no audit record; ``was_active`` tells the caller which of the two happened
- **Store first**: the declaration is written before anything is muted, unmuted, announced or audited, and a store that refuses the write ends the request with ``success=false`` - a stop the manager could not record must not be one a restart comes back believing in
- **Audited**: when ``audit_log.enabled`` is set (off by default), each real transition appends ``planned_stop_started`` / ``planned_stop_ended`` under the ``__audit__`` fault code, with that transition's own reason and declarer
- **Persisted**: stored in the fault store, so the declaration outlives the process (SQLite backend), and ``~/get_planned_stop`` keeps serving it after the withdrawal with ``ended_at`` stamped
- **Returns**: ``success``, a message, and the state the switch was in before the call

Design Decisions
----------------

Thread Safety
~~~~~~~~~~~~~

All ``FaultStorage`` public methods acquire a mutex lock to ensure thread safety
when handling concurrent service requests. This is essential since ROS 2 service
callbacks may execute on different threads.

Fault Aggregation
~~~~~~~~~~~~~~~~~

Multiple reports of the same ``fault_code`` (from same or different sources) are
aggregated into a single fault entry. This provides:

- **Deduplication**: Prevents fault flooding from repeated reports
- **Source tracking**: Identifies all sources reporting the same fault
- **Occurrence counting**: Tracks how many times a fault was reported

Severity Escalation
~~~~~~~~~~~~~~~~~~~

When a fault is re-reported with a higher severity, the stored severity is updated.
This ensures the fault reflects the worst-case condition. Severity levels are ordered:
``INFO(0) < WARN(1) < ERROR(2) < CRITICAL(3)``.

Status Lifecycle (Debounce Model)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Faults follow an AUTOSAR DEM-style debounce lifecycle:

- **PREFAILED**: Debounce counter < 0 but above confirmation threshold (fault trending towards confirmation)
- **PREPASSED**: Debounce counter > 0 but below healing threshold (fault trending towards healing)
- **CONFIRMED**: Debounce counter <= confirmation threshold (e.g., -3). Fault is active and verified.
- **HEALED**: Debounce counter >= healing threshold (if healing enabled). Fault resolved by PASSED events.
- **CLEARED**: Fault manually acknowledged via ClearFault service

FAILED events decrement the debounce counter (towards confirmation).
PASSED events increment the debounce counter (towards healing).
CRITICAL severity bypasses debounce and confirms immediately.

The counter is always clamped to ``[confirmation_threshold, healing_threshold]`` so a long run of
one-sided events cannot push it out to the integer limits and delay the opposite transition.

CONFIRMED and HEALED are *latched* (hysteresis): once reached, the status holds until the counter
walks all the way to the opposite threshold. So PREFAILED/PREPASSED depend on the counter sign, but
a CONFIRMED or HEALED fault keeps its status while the counter is between the thresholds - a single
opposite-direction event cannot flip it. One consequence is a re-confirmation delay: a fault that
becomes active again is not back in the default (CONFIRMED-only) list until the counter has fallen by
up to ``healing_threshold - confirmation_threshold`` events. During that window ``last_occurred``
still reflects the activity; ``occurrence_count`` does not, because it counts the edge that started
the occurrence, not every report within it.

Thresholds must satisfy ``confirmation_threshold < 0 <= healing_threshold`` (``healing_threshold == 0``
means heal on a single PASSED event); the node validates the
merged per-entity config at startup, logs a warning, and falls back to safe defaults if not. When
healing is disabled, any HEALED row left by a previous (healing-enabled) run is reclassified to
CLEARED once at startup so it does not behave inconsistently under the latch.

Planned Stop as a Second Mute Source
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Maintenance produces faults that are real, correctly detected, and not news. The
planned stop marks them instead of dropping them: while it stands, a fault whose
cycle *starts* is registered in the correlation engine as muted with
``rule_id = "planned_stop"`` and the pseudo root cause ``PLANNED_STOP``, and
everything else about it - debounce, confirmation, snapshot and rosbag capture,
the audit records - happens exactly as it would otherwise. Dropping the report
instead was rejected because a fault that outlived the stop would then never be
raised again: reporters are level-triggered, so nothing re-announces a condition
whose report was thrown away.

**Only a cycle that starts inside the stop.** That same level-triggered shape is
why the mark is gated on the cycle boundary rather than on the report.
``ros2_medkit_diagnostic_bridge`` sends one FAILED per incoming
``DiagnosticArray`` - typically 1 Hz for as long as the condition holds - and
``FaultReporter::report()`` is documented to be called the same way. Marking on any
report would take a fault that was confirmed and announced *before* the stop off
the fault list within a second of the declaration, and announce it a second time at
the switch-off. The boundary is ``report_fault_event``'s own ``is_new`` - a new fault, or one
raised again after being cleared - plus a re-fail out of HEALED, because the manager
publishes ``EVENT_CLEARED`` at the heal and the confirmation that follows is
therefore fresh news. ``occurrence_count`` keeps its own, narrower definition.

**Ownership is the fact; the mute is derived from it.** The stop owns a fault
CYCLE, and that ownership is a flag on the fault row in the store - not a set in the
process, and not a timestamp comparison. Everything else follows: an owned fault is
muted unless a rule's mute overlays it, and the moment the overlay ends the engine
re-asserts the stop's mute (``reassert_planned_stop_mutes``, called from
``process_clear`` and from ``cleanup_expired``). A rule therefore borrows a fault
rather than taking it, which is what makes the two features compose in both
directions: the withdrawal leaves a rule-held fault muted, and a rule that lets go
mid-stop hands the fault back instead of dropping it out of the stop for good.

Deriving the mute also fixes what ``should_mute`` means. A repeat report of a fault
whose rule has stopped matching produces no correlation result at all, so the report
path used to treat it as unmuted and announce an update for a fault the list was
hiding. ``process_fault`` now answers with the state of the mute map, not with what
this particular report matched, and the PASSED path asks the same question rather
than defaulting to "not muted".

Ownership ends in exactly three places: the fault is acknowledged (both
acknowledgement paths - the correlation clear and the scoped per-entity clear that
skips it - and the store clears the flag with the CLEARED status), the fault is
auto-cleared with its root cause, or the stop is withdrawn.

**The mark survives a restart** because the flags do. Startup reads them back and
re-registers the mute. Without that, a reboot inside a weekend stop would make the
survivors silently visible and the switch-off would announce none of them: their
confirmations, suppressed before the reboot, would be lost for good. A correlation
rule's mute is not persisted anywhere, so a rule-muted fault comes back unmuted
after a restart unless the stop owns its cycle - a pre-existing property of
correlation rather than something the switch introduces.

**A withdrawal is ordered so a crash cannot swallow an announcement.** The
declaration is written first (so a store that refuses the write changes nothing, and
a restart never re-arms a stop the operator withdrew), the confirmations are
announced next, and the ownership flags are cleared last. A process that dies in
between leaves faults owned by a declaration that is already over, which is a state
the next startup recognises: it announces them and clears the flags of exactly
those faults. Clearing first would have turned a crash into permanent silence.

The recovery runs synchronously in the constructor, before the node has a service
or a timer. Deferring it - waiting for a subscriber to match, say - opens a window
in which an operator declares a NEW stop over the same services and new cycles
become owned by it; the deferred work then announces a fault the new stop is
holding and, if it clears ownership wholesale rather than the set it captured,
drops that fault's flag too. Running early costs an announcement that may reach no
subscriber, because nothing has matched a publisher that is milliseconds old and
the events topic is volatile. That is the same accepted loss as every other
publication made at startup, and the released faults are in the default fault list
regardless.

Withdrawing releases the rest and publishes ``EVENT_CONFIRMED`` once for each
released fault that is CONFIRMED. That republication is the point of marking
rather than dropping: the confirmation happened behind the mute, no consumer of
the event stream ever heard it, and the condition still stands on the machine.
What is withheld is exactly what a rule-muted symptom withholds - ``EVENT_CONFIRMED``
and ``EVENT_UPDATED`` - while ``EVENT_CLEARED`` is published as usual, so a fault
that heals or is acknowledged inside the stop still reports its end.

The switch is a service rather than a parameter because it carries a reason and a
declarer and produces an audit record, none of which a parameter can do; and it is
not a new HTTP route because the gateway already exposes a node's services as SOVD
operations on its App entity.

One cost falls on every deployment, correlation or not: the engine is now always
constructed, so the ``correlation.cleanup_interval_sec`` timer (5 s by default)
runs on every fault manager. With no rules loaded a tick takes the engine's mutex,
walks two empty containers - ``pending_root_causes_`` and ``pending_clusters_`` -
and re-asserts an empty ownership set. Measured at ~190 ns per call (1e6 calls on a
rules-free engine, three runs: 191.2, 194.7, 190.3 ns), which at the default
interval is ~2.3 us of CPU per minute. Building the engine lazily when a stop is
first declared would put a second construction path under the service handler to
save that.

Rosbag Black-Box Recording
~~~~~~~~~~~~~~~~~~~~~~~~~~

``RosbagCapture`` is a single-writer black box: one ring buffer, one open bag writer and
one post-fault window per fault manager. Everything below follows from that.

Subscriptions feed a ``std::deque`` of serialised messages, pruned to ``duration_sec`` of
history and to ``max_buffer_mb`` of RAM. Pruning is driven by message arrival, not by a
timer, and each arrival prunes the whole deque by age - so a single topic going quiet is
pruned like anything else, while a system where *everything* goes quiet keeps its last
window buffered rather than silently losing the final messages before it died.

On confirmation the whole buffer is moved out in one step and written to a new bag. If
``duration_after_sec > 0`` the writer stays open and the state machine enters the
post-fault window: ``recording_post_fault_`` is set, a one-shot timer is armed, and from
that point incoming messages bypass the buffer and are written straight to the open bag.
When the timer fires (or ``stop()`` runs first), the recording is finalised: the writer
closes and one metadata row is stored per fault the recording covers.

.. plantuml::

   @startuml
   skinparam backgroundColor transparent
   state "Buffering" as BUF
   state "Post-fault window" as POST
   state "Finalising" as FIN

   [*] --> BUF : start()
   BUF --> POST : confirm, buffer non-empty\n(flush + keep writer open)
   BUF --> POST : confirm, buffer EMPTY\n(post-fault-only bag)
   BUF --> BUF : confirm, duration_after_sec == 0\n(flush, close, store row)
   POST --> POST : confirm inside the window\n(attach, share the bag)
   POST --> FIN : timer expires / stop()
   FIN --> BUF : rows stored, writer closed
   @enduml

The window boundary
"""""""""""""""""""

The interesting transition is the second one. The branch is on the buffer, not on window
history: a confirmation that finds nothing buffered opens a post-fault-only recording,
whatever emptied the buffer.

Right after a window finalises the buffer is empty as far as the capture's own message
flow goes: the flush that opened the recording drained it, and every message published
during the window went into that bag instead of back into the buffer. A fault confirming
in that gap - typically the next fault of the same burst - therefore has no pre-fault
history to write.

Two things qualify "empty". A message arriving between the drain and the guard being
published is still buffered, and more importantly the broad topic modes (``all``,
``auto``, ``entity``) subscribe to ``/fault_manager/events``: reporting a fault publishes
an event there, so on a real fault manager the act of reporting refills the buffer, and
the boundary case belongs mainly to a narrowed capture - ``explicit``, a topic list,
``config``, or a broad mode excluding that topic. ``test_rosbag_boundary_scope.test.py``
excludes it for exactly this reason, and says so.

It is not the only one. A fault confirming before anything has been published on a
captured topic - at startup, or with ``lazy_start`` - finds the same empty buffer and gets
the same post-fault-only bag, with no window having closed beforehand.

Before, ``flush_to_bag()`` returned an empty path and the confirmation was abandoned with
a warning: no bag, no metadata row, no retry, and every later retrieval for that fault
failed permanently. Now the empty buffer is distinguished from a write failure, and it
opens a recording anyway - a post-fault-only bag. Because it is the ordinary post-roll
state machine, attachment, entity scoping, auto-cleanup and quota accounting all behave
as for any other recording.

Two states stay deliberately empty-handed:

- ``duration_after_sec == 0``: there is no window to record into and no history to write,
  so the fault gets no bag.
- an I/O failure (unwritable ``storage_path``, unusable storage backend): no post-roll is
  opened on top of it, so no row is ever stored for a bag that does not exist.

``flush_to_bag()`` reports ``kOk`` / ``kEmptyBuffer`` / ``kIoError`` rather than one
overloaded empty-string return, and the writer-open block lives in ``open_bag_writer()``
so it can run without any buffered messages.

Handing a recording over
""""""""""""""""""""""""

Confirmations run on the capture pool and the post-fault timer runs on the executor. The
node-level rosbag mutex serialises confirmations against each other but not against the
timer, so ``post_fault_timer_mutex_`` is the only thing ordering the two - and clearing
the recording guard is precisely the signal that lets the next confirmation open a bag of
its own. Everything that belongs to the recording therefore changes hands inside one
critical section: the guard, the start time and the writer. Releasing the guard first and
reaching for ``writer_mutex_`` afterwards leaves a gap in which the incoming confirmation
installs its writer and the outgoing finalise then destroys it, after which the new
recording writes through a null pointer - every message dropped - and still stores a row
for the empty bag it produced. The writer is *closed* only after all three of
``post_fault_timer_mutex_``, ``capture_topics_mutex_`` and ``writer_mutex_`` are released,
once it is exclusively the finalise's own, because flushing a bag is real I/O. It is not
closed unsynchronised, though. It is closed under a different lock, and that lock exists
for a reason that has nothing to do with this class's state.

Closing a bag, and the lock that is not the data lock
"""""""""""""""""""""""""""""""""""""""""""""""""""""

``~Writer`` can unload the storage plugin's shared library and ``open()`` loads it, and the
state both touch is **process-global**: ``class_loader`` keeps one registry of loaded
libraries keyed by library path, so every writer of a given format shares one
``rcutils_shared_library_t`` no matter which thread, which ``RosbagCapture`` or which
``FaultManagerNode`` created it. A close racing an open puts two threads into ``dlclose``
on the same handle. The winner closes it and zeroes the struct; the loser's ``dlclose``
reports "shared object not open" and then calls the now-null ``allocator.deallocate``,
and the process dies in ``rcutils_unload_shared_library`` with the instruction pointer at
zero. Reproduced outside this package with threads doing nothing but open/close loops on
one format: four threads x 200 iterations failed in 20 of 20 runs on ``mcap`` and in 20 of
20 on ``sqlite3``, so it belongs to neither backend and to no single instance.

An instance member cannot serialise process-global state, so the lock is a file-scope
``plugin_mutex()`` in ``rosbag_capture.cpp``, held across a writer's construction, its
``open()`` and its destruction, on every path that has one: both finalise paths,
``discard_active_writer()``, the destructor, and the storage-backend probe that every
capture runs while it is being constructed. ``open()`` is inside the lock deliberately -
the constructor reaches no loader, the open is what loads the plugin, and a lock around
construction and destruction alone still let failures through. The mutex is leaked on
purpose (a ``new std::mutex`` that is never deleted): a static mutex destroyed during
static destruction, while another thread is closing a bag, reopens the very window it
exists to close. With it, the same loops ran 0 failures in 112000 operations on each
backend.

It is deliberately **not** ``writer_mutex_``. That lock is taken by ``message_callback()``
for every message of a post-roll and by the flush loop for every buffered message, so
charging a close to it would stall the capture's own write path for the length of a flush
plus a ``metadata.yaml`` write. Keeping the close off the data lock was the original
design's call, and the cost it avoided is real; what it left unpaid was safety. Measured on
one workstation, single-threaded: a close costs about 0.37 ms for a bag with no messages
(200 samples) and about 1.1 ms for one holding 256 MB, the ring buffer's default RAM cap,
split at the default 50 MB per file (12 samples). A recording opening at that moment waits
behind that. The figure is small because closing flushes to the page cache and writes
``metadata.yaml``; it does not ``fsync``, so slow or synchronous storage will cost more.

The resulting order is ``node rosbag mutex -> post_fault_timer_mutex_ ->
{capture_topics_mutex_, writer_mutex_}`` plus ``plugin_mutex() -> writer_mutex_``, with
``buffer_mutex_`` never held across another lock. ``plugin_mutex()`` is taken in three
shapes: alone, to destroy a writer already moved out of ``active_writer_``; alone, across a
probe writer's construction, ``open()`` and destruction in ``default_storage_probe()``,
which never touches ``active_writer_`` at all; or as the outer of the pair in
``open_bag_writer()``. No path takes it while holding a lock of this class. That is what
fixes the shape every destruction site shares - move the writer out of ``active_writer_``
under ``writer_mutex_``, release ``writer_mutex_``, then destroy under ``plugin_mutex()``.
Resetting in place under ``writer_mutex_`` would add the reverse edge and deadlock against
a concurrent ``open_bag_writer()``. That cycle is reachable precisely because the close sits
outside ``post_fault_timer_mutex_``: a confirmation running at the same time is not held at
the attach check, so it can be inside ``open_bag_writer()`` holding the plugin lock while
the finalise holds ``writer_mutex_`` and asks for it. Moving the close back inside
``post_fault_timer_mutex_`` would remove the cycle and reintroduce the stall the design
declines to pay for. Paths that take the capture-topics or writer locks on their own release
each before taking the next, so no other reverse edge exists and the order is acyclic.

Honest durations
""""""""""""""""

``RosbagFileInfo::duration_sec`` is the span the recording was open, tracked from the
timestamp of its oldest written message (or from the moment the writer opened, for a
post-fault-only bag). Both storage paths used to hardcode the configured windows, which
made a post-fault-only bag and a bag flushed from a half-filled buffer both claim a full
pre-fault window.

The span is measured on the **monotonic** clock, not the wall clock that timestamps
messages: a wall clock that steps backwards mid-window turns an elapsed time negative,
and the duration is then reported as zero. Message ages necessarily start out as wall-clock
differences, so a flush converts the history it found into a monotonic origin once and
measures forward from there. That conversion is capped at how long the capture has been
running - no bag can hold more history than that - because a clock step between buffering a
message and flushing it would otherwise land in the figure in full.

Measurement stops when the writer leaves ``message_callback``'s reach, inside the handover
critical section. Closing the bag flushes it and writes ``metadata.yaml``, and the size
walk recurses the directory; both scale with the bag and the storage, and neither adds a
message. Charging them to the recording would inflate a stored ``duration_sec`` without
bound on slow storage.

It is a recording span, not a content span. A post-fault-only window during which nothing
was published still reports its window: that statement is more useful to an operator than
a zero indistinguishable from a broken artifact, and it is the only definition every path
can produce without timestamping each message inside the post-roll write path, which runs
under ``writer_mutex_`` in the hot path. Because the buffer is pruned only on arrival, the
span can also legitimately exceed ``duration_sec + duration_after_sec``.

If the metadata row cannot be stored, the bag is discarded rather than kept: retrieval is
keyed by fault code and quota accounting enumerates rows, so a bag with no row would
occupy disk that nothing can find and nothing can evict.

Known interval: between the empty-buffer flush and the moment the recording guard is
published, the writer is being created - directory creation plus a storage-backend open.
Messages arriving in that interval take the buffering path rather than the new bag, so a
post-fault-only recording can miss its first few milliseconds. They are not lost; they
stay in the ring buffer and serve the next fault as pre-history.
