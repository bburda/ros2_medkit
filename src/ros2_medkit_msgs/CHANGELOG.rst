^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ros2_medkit_msgs
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.7.0 (2026-08-27)
------------------
* ``GetRosbag.srv`` gains a ``recording_id`` request field (tried before ``fault_code`` and falling back to it when it names no recording, so a caller holding one identifier that may be either can set both; ``fault_code`` keeps its meaning of "the newest recording of this fault") and ``recording_id`` / ``fault_codes[]`` response fields. ``ListRosbags.srv`` gains a parallel ``recording_ids[]``. ``Snapshot.msg``'s ``bulk_data_id`` now carries a recording id for rosbag snapshots rather than the fault code; the field type is unchanged. Additive, but the service type hashes change, so the gateway and the fault manager must be deployed together (`#620 <https://github.com/selfpatch/ros2_medkit/issues/620>`_)
* ``last_passed`` is exposed on the fault wire, so a consumer can tell when the monitored condition was last observed healthy without inferring it from the status (`#573 <https://github.com/selfpatch/ros2_medkit/pull/573>`_)
* Rosbag recordings are addressed by recording id rather than by fault code, so a fault holding several recordings can expose each one (`#623 <https://github.com/selfpatch/ros2_medkit/pull/623>`_)
* The fault contract documentation now matches the implementation on three points that had drifted: ``occurrence_count`` is an edge counter that advances on a new fault and on every re-raise after CLEARED, ``first_occurred`` marks the start of the current occurrence rather than the first one ever, and healed clears and SSE loss accounting are described as the code implements them (`#618 <https://github.com/selfpatch/ros2_medkit/pull/618>`_)
* Contributors: @bburda, @mfaferek93

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
