^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ros2_medkit_param_beacon
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.7.0 (2026-08-27)
------------------
* Build and test only: every integration launch test runs on its own DDS domain, taken when the test starts and released when it ends, so a crash frees the domain the same way an ordinary exit does (`#551 <https://github.com/selfpatch/ros2_medkit/pull/551>`_, `#597 <https://github.com/selfpatch/ros2_medkit/pull/597>`_). The usable band excludes the domains whose RTPS port slice falls inside the kernel ephemeral port range, where any process on the machine can steal a port and kill a node with a bind failure
* Build: the package is instrumented for coverage (`#582 <https://github.com/selfpatch/ros2_medkit/pull/582>`_)
* Contributors: @bburda

0.6.0 (2026-06-22)
------------------
* No functional changes; version bump for the coordinated 0.6.0 release.
* Contributors: @bburda

0.5.0 (2026-06-08)
------------------
* Migrated ``ParameterBeaconPlugin`` to the ``get_routes()`` plugin API
* Added shutdown guards and ``noexcept`` destructors that reset rclcpp resources before member destruction, preventing teardown SIGSEGV; the graph poll now swallows ``rcl`` "context invalid" during shutdown
* Added post-shutdown guard unit tests
* Build: adopt the centralized ``ROS2MedkitWarnings`` cmake module
* Contributors: @bburda

0.4.0 (2026-03-20)
------------------
* Initial release - parameter-based beacon discovery plugin
* ``ParameterBeaconPlugin`` with pull-based parameter reading for entity enrichment
* ``x-medkit-param-beacon`` vendor extension REST endpoint
* Poll target discovery from ROS graph in non-hybrid mode
* ``ParameterClientInterface`` for testable parameter access
* Contributors: @bburda
