^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ros2_medkit_fault_reporter
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* ``FaultReporter`` can now be constructed from an ``rclcpp_lifecycle::LifecycleNode`` (and from a plain ``rclcpp::Node`` reference or explicit node interfaces), enabling use inside lifecycle nodes (`#555 <https://github.com/selfpatch/ros2_medkit/issues/555>`_)
* Contributors: @zeerekahmad

0.6.0 (2026-06-22)
------------------
* No functional changes; version bump for the coordinated 0.6.0 release.
* Contributors: @bburda

0.5.0 (2026-06-08)
------------------
* ``LocalFilter`` now debounces ``PASSED`` events with the same threshold / window filtering as ``FAILED``, preventing rapid CONFIRMED/CLEARED status cycling from triggering unbounded snapshot recapture (`#308 <https://github.com/selfpatch/ros2_medkit/issues/308>`_)
* Build: adopt the centralized ``ROS2MedkitWarnings`` and ``ROS2MedkitSanitizers`` cmake modules
* Contributors: @bburda, @mfaferek93

0.4.0 (2026-03-20)
------------------
* Build: use shared cmake modules from ``ros2_medkit_cmake`` package
* Build: auto-detect ccache, centralized clang-tidy configuration
* Contributors: @bburda

0.3.0 (2026-02-27)
------------------
* Multi-distro CI support for ROS 2 Humble, Jazzy, and Rolling (`#219 <https://github.com/selfpatch/ros2_medkit/pull/219>`_, `#242 <https://github.com/selfpatch/ros2_medkit/pull/242>`_)
* Contributors: @bburda

0.2.0 (2026-02-07)
------------------
* Initial rosdistro release
* FaultReporter client library with simple API:

  * report(fault_code, severity, description) - report FAILED events
  * report_passed(fault_code) - report fault condition cleared
  * High-severity faults (ERROR, CRITICAL) bypass local filtering

* LocalFilter for per-fault-code threshold/window filtering:

  * Configurable threshold (default: 3 reports) and time window (default: 10s)
  * Prevents flooding FaultManager with duplicate reports
  * PASSED events always forwarded (bypass filtering)

* Configuration via ROS parameters (filter_threshold, filter_window_sec)
* Thread-safe implementation with mutex-protected config access
* Contributors: Bartosz Burda, Michal Faferek
