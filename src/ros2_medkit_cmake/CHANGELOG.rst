^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ros2_medkit_cmake
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.7.0 (2026-08-27)
------------------
* ``ROS_DOMAIN_ID`` isolation is allocated at run time instead of from a hand-maintained table. A test registered through ``medkit_add_gtest``, ``medkit_add_gmock``, ``medkit_add_launch_test``, ``medkit_add_pytest_test`` or ``medkit_add_wrapped_test`` takes a domain when it starts and holds it through an open socket for exactly as long as it runs, so a crash or a SIGKILL releases it the same way an ordinary exit does. The isolation reaches across packages, which a CTest ``RESOURCE_LOCK`` cannot, because colcon runs one ctest per package in parallel. ``medkit_test_needs_no_domain()`` opts out a test that creates no ROS node. The usable band excludes the domains whose RTPS port slice falls inside the kernel ephemeral port range (`#551 <https://github.com/selfpatch/ros2_medkit/pull/551>`_, `#597 <https://github.com/selfpatch/ros2_medkit/pull/597>`_)
* Launch tests are no longer registered in a system package build, where they failed as a group and were reported as passing anyway because bloom runs the test step permissively; the macro now owns their properties instead of each package setting them (`#608 <https://github.com/selfpatch/ros2_medkit/pull/608>`_, `#602 <https://github.com/selfpatch/ros2_medkit/issues/602>`_)
* clang-tidy analyses a package's translation units in parallel, with the job count capped for an 8 GB machine and packages serialised so the cap holds, cutting PR feedback time from about 56 minutes to about 35 (`#588 <https://github.com/selfpatch/ros2_medkit/pull/588>`_, `#590 <https://github.com/selfpatch/ros2_medkit/pull/590>`_)
* Every C++ package is instrumented for coverage, and every package that lints exports a compile database (`#582 <https://github.com/selfpatch/ros2_medkit/pull/582>`_)
* Sanitizer builds keep their asserts, and the build reports what ccache did (`#590 <https://github.com/selfpatch/ros2_medkit/pull/590>`_)
* Contributors: @bburda

0.6.0 (2026-06-22)
------------------
* ``ROS2MedkitTestDomain``: carve dedicated ``ROS_DOMAIN_ID`` ranges for the new log bridge (210-214) and action-status bridge (215-219) test suites out of the integration-tests range (`#422 <https://github.com/selfpatch/ros2_medkit/pull/422>`_)
* Contributors: @mfaferek93

0.5.0 (2026-06-08)
------------------
* ``ROS2MedkitWarnings.cmake`` module centralizes compiler warning flags across all packages, with selective ``-Werror`` (namespaced ``MEDKIT_ENABLE_WERROR``, defaults OFF) applied only to flags safe against external headers
* ``ROS2MedkitSanitizers.cmake`` module adds ASan/UBSan and TSan support for sanitizer CI jobs
* ``ROS2MedkitTestDomain.cmake`` centralizes ``ROS_DOMAIN_ID`` allocation for per-test DDS isolation
* Vendored cpp-httplib 0.14.3 as a build-farm fallback (``VENDORED_DIR`` parameter), marked as a SYSTEM include to suppress third-party warnings
* ``medkit_find_cpp_httplib`` caps cpp-httplib at ``< 0.20`` across both the pkg-config and ``find_package(httplib)`` tiers, so distros shipping 0.20+ (Ubuntu 26.04 ships 0.26, which dropped the multipart ``Request::has_file`` API the gateway uses) fall through to the vendored 0.14.3 header instead of failing the build; ``ROS2MedkitCompat.cmake`` extended to cover ROS 2 Lyrical / Ubuntu 26.04 Resolute (rclcpp 32, ``yaml_cpp_vendor`` target export, ``ament_target_dependencies`` removal in ament_cmake 2.8.5+), which replaces Rolling in CI (`#405 <https://github.com/selfpatch/ros2_medkit/pull/405>`_)
* Contributors: @bburda, @mfaferek93

0.4.0 (2026-03-20)
------------------
* Initial release - shared cmake modules extracted from gateway package (`#294 <https://github.com/selfpatch/ros2_medkit/pull/294>`_)
* ``ROS2MedkitCcache.cmake`` - auto-detect ccache for faster incremental rebuilds
* ``ROS2MedkitLinting.cmake`` - centralized clang-tidy configuration (opt-in locally, mandatory in CI)
* ``ROS2MedkitCompat.cmake`` - multi-distro compatibility shims for ROS 2 Humble/Jazzy/Rolling
* Contributors: @bburda
