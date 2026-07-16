^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ros2_medkit_msgs
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* ``ReportFault.srv`` request gains an optional ``string supersedes_source_id`` field: a reporter can drop a previously-reported source from the fault's ``reporting_sources`` when re-reporting under a corrected source, so a fault first attributed to a provisional source (an action bridge before DDS discovery resolves the server node FQN) resolves to the right entity under the strict-AND per-entity ``/faults`` scope filter. Empty (default) preserves the prior add-only behavior; out-of-tree callers must rebuild against the new message (`#467 <https://github.com/selfpatch/ros2_medkit/issues/467>`_)
* Contributors: @bburda

0.6.0 (2026-06-22)
------------------
* No functional changes; version bump for the coordinated 0.6.0 release.
* Contributors: @bburda

0.5.0 (2026-06-08)
------------------
* New service definitions ``ListEntities.srv``, ``GetEntityData.srv``, ``GetCapabilities.srv`` and the ``EntityInfo.msg`` type for exposing the entity tree over ROS 2 services (`#330 <https://github.com/selfpatch/ros2_medkit/issues/330>`_)
* ``ClearFault.srv`` request gains a ``bool skip_correlation_auto_clear`` field so callers can opt out of cascade-clearing correlated symptom fault codes; out-of-tree callers must rebuild against the new message (`#395 <https://github.com/selfpatch/ros2_medkit/issues/395>`_)
* Contributors: @bburda, @mfaferek93

0.4.0 (2026-03-20)
------------------
* ``MedkitDiscoveryHint`` message type for beacon discovery publishers
* Contributors: @bburda

0.3.0 (2026-02-27)
------------------
* Multi-distro CI support for ROS 2 Humble, Jazzy, and Rolling (`#219 <https://github.com/selfpatch/ros2_medkit/pull/219>`_, `#242 <https://github.com/selfpatch/ros2_medkit/pull/242>`_)
* Contributors: @bburda

0.2.0 (2026-02-07)
------------------
* Initial rosdistro release
* Fault management messages:

  * Fault.msg - Core fault data model with severity levels (INFO/WARN/ERROR/CRITICAL)
    and debounce-based status lifecycle (PREFAILED/PREPASSED/CONFIRMED/HEALED/CLEARED)
  * FaultEvent.msg - Real-time event notifications (EVENT_CONFIRMED/EVENT_CLEARED/EVENT_UPDATED)
    with auto-cleared correlation codes
  * MutedFaultInfo.msg - Fault correlation muting metadata
  * ClusterInfo.msg - Fault clustering information

* Fault management services:

  * ReportFault.srv - Report fault events with FAILED/PASSED event model
  * GetFaults.srv - Query faults with filtering by severity, status, and correlation data
  * ClearFault.srv - Clear/acknowledge faults by fault_code
  * GetSnapshots.srv - Retrieve topic snapshots captured at fault time
  * GetRosbag.srv - Retrieve rosbag recordings associated with faults

* Contributors: Bartosz Burda, Michal Faferek
