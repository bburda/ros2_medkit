^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ros2_medkit_integration_tests
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.7.0 (2026-08-27)
------------------
* Every integration launch test runs on its own DDS domain, taken when the test starts and held through an open socket for exactly as long as it runs, so a crash or a SIGKILL releases it the same way an ordinary exit does. This replaces the hand-maintained per-package domain pools, which had to stay pairwise disjoint because colcon runs one ctest per package in parallel and a CTest ``RESOURCE_LOCK`` binds only inside a single ctest run (`#551 <https://github.com/selfpatch/ros2_medkit/pull/551>`_, `#597 <https://github.com/selfpatch/ros2_medkit/pull/597>`_)
* Feature tests are registered by glob: a file dropped into ``test/features/*.test.py`` is registered with its port and domain assigned automatically, so no explicit registration is needed and none can bypass ``GATEWAY_TEST_PORT``
* Inline gateways and fault managers go through the shared launch helpers rather than each test hand-building its own, which also fixed the SIGKILL these tests hit under coverage instrumentation (`#554 <https://github.com/selfpatch/ros2_medkit/pull/554>`_)
* The Python in this package is linted (`#598 <https://github.com/selfpatch/ros2_medkit/pull/598>`_), and the package is instrumented for coverage (`#582 <https://github.com/selfpatch/ros2_medkit/pull/582>`_)
* Flaky suites are fixed at their cause rather than retried: the OPC UA Alarms and Conditions test, the action-status and diagnostic-bridge fault paths, the operation-handlers fixture, ``type_info`` caching and the documentation linkcheck (`#504 <https://github.com/selfpatch/ros2_medkit/pull/504>`_, `#636 <https://github.com/selfpatch/ros2_medkit/pull/636>`_)
* Contributors: @bburda, @mfaferek93, @YueBit

0.6.0 (2026-06-22)
------------------
* New suites covering the SOVD entity status endpoints (REQ_INTEROP_076), lifecycle-aware app and component status (`#455 <https://github.com/selfpatch/ros2_medkit/pull/455>`_), and fault-storm capture liveness under bounded concurrency (`#456 <https://github.com/selfpatch/ros2_medkit/pull/456>`_)
* Contributors: @bburda

0.5.0 (2026-06-08)
------------------
* New peer-aggregation suites: peer aggregation, cross-ECU fan-out across all resource types, daisy-chain hierarchical aggregation, leaf-collision aggregation, and cross-ECU log aggregation
* New SOVD-aligned entity-model suites: runtime entity model, flat entity tree without areas, hybrid suppression, and per-entity fault scope isolation
* New OpenAPI conformance suites: ``test_openapi_callability`` and ``test_openapi_response_drift`` validate the published spec against live responses
* New graph-event discovery suite covering rclcpp graph-event driven discovery refresh
* Coverage for the ``GET /apps/{id}/belongs-to`` discovery endpoint, nested ``x-medkit`` vendor payloads (`#385 <https://github.com/selfpatch/ros2_medkit/issues/385>`_), version-info aggregation, and pending-after-register update status (`#378 <https://github.com/selfpatch/ros2_medkit/issues/378>`_)
* Tests: centralized ``ROS_DOMAIN_ID`` allocation and widened shutdown timeouts for sanitizer overhead
* Contributors: @bburda, @eclipse0922

0.4.0 (2026-03-20)
------------------
* Integration tests for SOVD resource locking (acquire, release, extend, fault clear with locks, expiration, parent propagation)
* Integration tests for SOVD script execution endpoints (all formats, params, output, failure, lifecycle)
* Integration tests for graph provider plugin (external plugin loading, entity introspection)
* Integration tests for beacon discovery plugins (topic beacon, parameter beacon)
* Integration tests for OpenAPI/docs endpoint
* Integration tests for logging endpoints (``/logs``, ``/logs/configuration``)
* Integration tests for linux introspection plugins (launch_testing and Docker-based)
* Port isolation per integration test via CMake-assigned unique ports
* ``ROS_DOMAIN_ID`` isolation for integration tests
* Build: use shared cmake modules from ``ros2_medkit_cmake`` package
* Contributors: @bburda

0.3.0 (2026-02-27)
------------------
* Refactored integration test suite into dedicated ``ros2_medkit_integration_tests`` package (`#227 <https://github.com/selfpatch/ros2_medkit/pull/227>`_)
* Multi-distro CI support for ROS 2 Humble, Jazzy, and Rolling (`#219 <https://github.com/selfpatch/ros2_medkit/pull/219>`_, `#242 <https://github.com/selfpatch/ros2_medkit/pull/242>`_)
* Contributors: @bburda
