^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ros2_medkit_fault_manager
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Optional append-only, hash-chained audit log of fault state transitions: each transition appends one immutable row (``record_hash = sha256(prev_hash + canonical(event))`` via OpenSSL EVP SHA-256) with a persisted chain head, a ``verify`` routine, a read API, and retention that seals a segment anchor before pruning. Time-based (PREFAILED->CONFIRMED) auto-confirmations are also audited. ``verify`` reads the chain head directly from the database, so deleting the newest row together with the head row is reported as tampering instead of silently recovering. ``BEFORE UPDATE`` / ``BEFORE DELETE`` triggers reject out-of-band edits as defense-in-depth. The chain is unkeyed and stored in a single writable file, so ``verify`` detects edits/deletions that did not recompute the chain (casual or accidental tampering); it is not a defence against an attacker who can rewrite the whole file. Off by default (`#483 <https://github.com/selfpatch/ros2_medkit/issues/483>`_)
* ``report_fault_event`` honors the new ``ReportFault.supersedes_source_id`` request field: when set, the named source is dropped from the fault's ``reporting_sources`` before the current source is added (both the in-memory and SQLite stores). This lets a reporter correct an attribution it made under a provisional source without leaving the stale source stuck in the append-only set, where the strict-AND per-entity ``/faults`` scope filter would keep the fault hidden. Empty (default) preserves the prior add-only behavior; a self-supersede (``supersedes_source_id == source_id``) is a no-op (`#467 <https://github.com/selfpatch/ros2_medkit/issues/467>`_)

0.6.0 (2026-06-22)
------------------
* Bounded concurrent snapshot capture under fault storms with a ``CaptureThreadPool`` and configurable capture pool / queue / overflow-policy parameters. The rosbag leg is serialized and the cooldown map is bounded, so a burst of simultaneous faults can no longer exhaust capture threads or grow memory without limit (`#456 <https://github.com/selfpatch/ros2_medkit/pull/456>`_)
* Entity-scoped rosbag capture by default (`#431 <https://github.com/selfpatch/ros2_medkit/pull/431>`_)
* Made rosbag capture enablement crash-safe (`#430 <https://github.com/selfpatch/ros2_medkit/pull/430>`_)
* Contributors: @bburda, @mfaferek93

0.5.0 (2026-06-08)
------------------
* ``ClearFault`` honors the new ``skip_correlation_auto_clear`` request flag so per-entity fault clears can opt out of cascade-clearing correlated symptom fault codes (`#395 <https://github.com/selfpatch/ros2_medkit/issues/395>`_)
* Three-layer protection against unbounded snapshot growth (bounded buffers plus pruning)
* Concurrency and lifetime hardening: serialize concurrent subscription creation in ``SnapshotCapture``, join capture threads in the ``FaultManagerNode`` destructor, and defense-in-depth shutdown guards to prevent teardown crashes across distros
* Aggregation security hardening and improved test coverage
* Build: adopt the centralized ``ROS2MedkitWarnings`` and ``ROS2MedkitSanitizers`` cmake modules and ``bugprone`` / ``special-member-functions`` clang-tidy checks
* Contributors: @bburda

0.4.0 (2026-03-20)
------------------
* Per-entity confirmation and healing thresholds via manifest configuration (`#269 <https://github.com/selfpatch/ros2_medkit/pull/269>`_)
* Default rosbag storage format changed from ``sqlite3`` to ``mcap``
* Support for namespaced fault manager nodes - gateway resolves service/topic names when the fault manager runs in a custom namespace
* Build: use shared cmake modules from ``ros2_medkit_cmake`` package
* Build: centralized clang-tidy configuration
* Contributors: @bburda

0.3.0 (2026-02-27)
------------------
* Accurate HIGHEST_SEVERITY reassignment and stale ``fault_to_cluster_`` cleanup (`#221 <https://github.com/selfpatch/ros2_medkit/pull/221>`_)
* Clean up ``pending_clusters_`` when fault cleared before ``min_count`` (`#211 <https://github.com/selfpatch/ros2_medkit/pull/211>`_)
* Multi-distro CI support for ROS 2 Humble, Jazzy, and Rolling (`#219 <https://github.com/selfpatch/ros2_medkit/pull/219>`_, `#242 <https://github.com/selfpatch/ros2_medkit/pull/242>`_)
* Contributors: @bburda, @eclipse0922

0.2.0 (2026-02-07)
------------------
* Initial rosdistro release
* Central fault management node with ROS 2 services:

  * ReportFault - report FAILED/PASSED events with debounce filtering
  * GetFaults - query faults with filtering by severity, status, correlation
  * ClearFault - clear/acknowledge faults

* Debounce filtering with configurable thresholds:

  * FAILED events decrement counter, PASSED events increment
  * Configurable confirmation_threshold (default: -1, immediate)
  * Optional healing support (healing_enabled, healing_threshold)
  * Time-based auto-confirmation (auto_confirm_after_sec)
  * CRITICAL severity bypasses debounce

* Dual storage backends:

  * SQLite persistent storage with WAL mode (default)
  * In-memory storage for testing/lightweight deployments

* Snapshot capture on fault confirmation:

  * Topic data captured as JSON with configurable topic resolution
  * Priority: fault_specific > patterns > default_topics
  * Stored in SQLite with indexed fault_code lookup
  * Auto-cleanup on fault clear

* Rosbag capture with ring buffer:

  * Configurable duration, post-fault recording, topic selection
  * Lazy start mode (start on PREFAILED) or immediate
  * Auto-cleanup of bag files, storage limits (max_bag_size_mb)
  * GetRosbag service for bag file metadata

* Fault correlation engine:

  * Hierarchical mode: root cause to symptom relationships
  * Auto-cluster mode: group similar faults within time window
  * YAML-based configuration with pattern wildcards
  * Muted faults tracking, auto-clear on root cause resolution

* FaultEvent publishing on ~/events topic for SSE streaming
* Wall clock timestamps (compatible with use_sim_time)
* Contributors: Bartosz Burda, Michal Faferek
