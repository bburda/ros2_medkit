^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ros2_medkit_diagnostic_bridge
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.7.0 (2026-08-27)
------------------
* The ``hardware_id`` of a diagnostic message is passed through as the fault's source id, so a fault raised from ``/diagnostics`` is attributed to the device the publisher named rather than to the bridge (`#622 <https://github.com/selfpatch/ros2_medkit/pull/622>`_)
* A fault code can be extracted from a diagnostic message's key/value attributes, so a publisher that already carries its own code no longer has to encode it in the message name (`#527 <https://github.com/selfpatch/ros2_medkit/pull/527>`_)
* The Humble fallback warning builds again (`#622 <https://github.com/selfpatch/ros2_medkit/pull/622>`_)
* Build and test only: the package is instrumented for coverage (`#582 <https://github.com/selfpatch/ros2_medkit/pull/582>`_), every integration launch test runs on its own DDS domain taken at run time (`#551 <https://github.com/selfpatch/ros2_medkit/pull/551>`_, `#597 <https://github.com/selfpatch/ros2_medkit/pull/597>`_), and the suite re-emits and polls until the fault surfaces instead of asserting once against a graph that may not have settled (`#504 <https://github.com/selfpatch/ros2_medkit/pull/504>`_)
* Contributors: @bburda, @mfaferek93, @nnarain

0.6.0 (2026-06-22)
------------------
* Tests: label ``test_integration`` as an integration test so it runs in the integration suite instead of the unit set (`#443 <https://github.com/selfpatch/ros2_medkit/pull/443>`_)
* Contributors: @bburda

0.5.0 (2026-06-08)
------------------
* Build: adopt the centralized ``ROS2MedkitWarnings`` and ``ROS2MedkitSanitizers`` cmake modules
* Tests: use centralized ``ROS_DOMAIN_ID`` allocation for DDS isolation
* Contributors: @bburda

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
* Bridge node converting standard ROS 2 /diagnostics to FaultManager fault reports
* Severity mapping:

  * OK -> PASSED event (fault condition cleared)
  * WARN -> WARN severity FAILED event
  * ERROR -> ERROR severity FAILED event
  * STALE -> CRITICAL severity FAILED event

* Auto-generated fault codes from diagnostic names (UPPER_SNAKE_CASE)
* Custom name_to_code mappings via ROS parameters
* Stateless design: always sends PASSED for OK status (handles restarts)
* Contributors: Michal Faferek
