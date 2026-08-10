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
up to ``healing_threshold - confirmation_threshold`` events. During that window ``occurrence_count``
and ``last_occurred`` still reflect the activity.

Thresholds must satisfy ``confirmation_threshold < 0 <= healing_threshold`` (``healing_threshold == 0``
means heal on a single PASSED event); the node validates the
merged per-entity config at startup, logs a warning, and falls back to safe defaults if not. When
healing is disabled, any HEALED row left by a previous (healing-enabled) run is reclassified to
CLEARED once at startup so it does not behave inconsistently under the latch.

Rosbag Black-Box Recording
~~~~~~~~~~~~~~~~~~~~~~~~~~

``RosbagCapture`` is a single-writer black box: one ring buffer, one open bag writer and
one post-fault window per fault manager. Everything below follows from that.

Subscriptions feed a ``std::deque`` of serialised messages, pruned to ``duration_sec`` of
history and to ``max_buffer_mb`` of RAM. Pruning is driven by message arrival, not by a
timer, so a topic that stops publishing keeps its last window buffered rather than
silently losing the final messages before it died.

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

The interesting transition is the second one. Right after a window finalises the buffer
is empty *by construction*: the flush that opened the recording drained it, and every
message published during the window went into that bag instead of back into the buffer.
A fault confirming in that gap - typically the next fault of the same burst - therefore
has no pre-fault history to write.

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
for the empty bag it produced. The writer is only *closed* outside the locks, once it is
exclusively the finalise's own, because flushing a bag is real I/O.

The resulting order is ``node rosbag mutex -> post_fault_timer_mutex_ ->
{capture_topics_mutex_, writer_mutex_}``, with ``buffer_mutex_`` never held across another
lock. Paths that take the capture-topics or writer locks on their own release each before
taking the next, so no reverse edge exists and the order is acyclic.

Honest durations
""""""""""""""""

``RosbagFileInfo::duration_sec`` is the wall-clock span the recording was open, tracked
from the timestamp of its oldest written message (or from the moment the writer opened,
for a post-fault-only bag). Both storage paths used to hardcode the configured windows,
which made a post-fault-only bag and a bag flushed from a half-filled buffer both claim a
full pre-fault window.

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
