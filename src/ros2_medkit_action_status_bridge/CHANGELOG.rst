^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ros2_medkit_action_status_bridge
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.7.0 (2026-08-27)
------------------
* A deferred fault report is retried instead of dropped, and delivered promptly through a fast retry timer rather than waiting for the next ordinary cycle (`#472 <https://github.com/selfpatch/ros2_medkit/pull/472>`_)
* Silent parameter coercion is closed off, including a NaN that passed a ``FloatingPointRange`` descriptor and two integer narrowing paths where an out-of-range value wrapped back into the legal band before its own check ran. A value the node refuses or corrects is now reported rather than applied quietly (`#607 <https://github.com/selfpatch/ros2_medkit/pull/607>`_)
* Build and test only: the package is instrumented for coverage (`#582 <https://github.com/selfpatch/ros2_medkit/pull/582>`_), every integration launch test runs on its own DDS domain taken at run time (`#551 <https://github.com/selfpatch/ros2_medkit/pull/551>`_, `#597 <https://github.com/selfpatch/ros2_medkit/pull/597>`_), and the suite synchronises on bridge discovery before raising a fault instead of assuming the graph has settled (`#504 <https://github.com/selfpatch/ros2_medkit/pull/504>`_)
* Contributors: @bburda, @mfaferek93

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
