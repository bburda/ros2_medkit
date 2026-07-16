^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ros2_medkit_action_status_bridge
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* A fault first raised before DDS discovery resolves the action server node is now corrected once the FQN becomes known, instead of staying stuck under the provisional action-interface source. The bridge records the provisional source, and on the next rescan re-reports the already-delivered fault under the resolved server FQN with ``ReportFault.supersedes_source_id`` set, so the FaultManager drops the provisional source and the fault resolves to the server node's entity under the strict-AND per-entity ``/faults`` scope filter. This refines the #466 "source fixed at first report" behavior: the source is still fixed for a fault whose server was already known, but a provisional attribution made before discovery is corrected rather than left permanently mis-scoped. A healed or not-yet-delivered fault is left untouched (`#467 <https://github.com/selfpatch/ros2_medkit/issues/467>`_)
* Contributors: @bburda

0.6.0 (2026-06-22)
------------------
* Initial release: generic action-status bridge. Watches every
  ``/<action>/_action/status`` topic and turns ABORTED goals into FaultManager
  faults (``<PREFIX>_<ACTION>_ABORTED``), heals on a non-failing terminal state.
  Captures the terminal action verdict that the diagnostic and log bridges
  cannot see (`#423 <https://github.com/selfpatch/ros2_medkit/pull/423>`_).
* Fault state is per-ACTION, derived from the whole ``GoalStatusArray`` on each
  message: a fault is raised only on the ``healthy -> failed`` transition and
  healed only on ``failed -> healthy``, so it is order-independent and resilient
  to dropped terminal messages. Per-goal dedup now only suppresses duplicate log
  lines.
* ``canceled_is_fault`` emits a status-aware ``<PREFIX>_<ACTION>_CANCELED`` code;
  such a fault heals on the next non-failing terminal or when the canceled goal
  ages out of the retained status array.
* Vanished actions (lifecycle deactivate, one-shot nodes) are pruned on rescan.
* Parameters are range-checked/normalized at load; an incompatible action status
  QoS is now warned about instead of silently dropping faults.
* Ships a default configuration so the bridge starts out of the box (`#459 <https://github.com/selfpatch/ros2_medkit/pull/459>`_)
* Fault source is fixed at first report instead of being re-attributed later: faults raised before discovery settles register the server FQN, faults are never attributed to the discovery placeholder node name, and the subscriptions and timer are reset in the destructor (`#466 <https://github.com/selfpatch/ros2_medkit/pull/466>`_)
* Contributors: @bburda, @mfaferek93
