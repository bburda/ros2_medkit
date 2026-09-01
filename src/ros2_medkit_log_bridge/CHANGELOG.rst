^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ros2_medkit_log_bridge
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.7.0 (2026-08-27)
------------------
* Two parameters were being swallowed. ``max_tracked_nodes`` was clamped into range with nothing written to the log, so a configuration file and the running process could disagree in silence. ``report_cooldown_sec`` was worse: the range check was written so that NaN passed it, then passed the "non-positive disables the cooldown" test, and reached ``Duration::from_seconds``, where the conversion is undefined - after which the cooldown window suppressed either every ERROR report or none, with no way to tell which from outside. Both are now validated with a positive, finite range test, and both configuration rows describe what the code does (`#607 <https://github.com/selfpatch/ros2_medkit/pull/607>`_)
* Integer parameters are read as the int64 a ROS parameter actually holds and clamped in that domain before narrowing, so an out-of-range ``severity_floor`` or ``max_tracked_nodes`` can no longer wrap back into the legal band and pass its own range check (`#607 <https://github.com/selfpatch/ros2_medkit/pull/607>`_)
* Build and test only: the package is instrumented for coverage (`#582 <https://github.com/selfpatch/ros2_medkit/pull/582>`_), every integration launch test runs on its own DDS domain taken at run time (`#551 <https://github.com/selfpatch/ros2_medkit/pull/551>`_, `#597 <https://github.com/selfpatch/ros2_medkit/pull/597>`_), and the log-bridge integration test waits for ``/rosout`` discovery before emitting its first line instead of racing it
* Contributors: @bburda

0.6.0 (2026-06-22)
------------------
* Initial release: promote ``/rosout`` log entries (WARN/ERROR/FATAL) to
  FaultManager faults, attributed to the originating node via a per-source
  FaultReporter, with auto-generated stable fault codes (`#422 <https://github.com/selfpatch/ros2_medkit/pull/422>`_)
* Ships a default configuration so the bridge starts out of the box (`#449 <https://github.com/selfpatch/ros2_medkit/pull/449>`_)
* Skips the medkit stack's own nodes by default, matching on the raw logger name (`#460 <https://github.com/selfpatch/ros2_medkit/pull/460>`_)
* Contributors: @mfaferek93
