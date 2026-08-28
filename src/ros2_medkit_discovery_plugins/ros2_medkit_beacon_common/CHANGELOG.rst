^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ros2_medkit_beacon_common
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.7.0 (2026-08-27)
------------------
* Build and test only: the package is instrumented for coverage (`#582 <https://github.com/selfpatch/ros2_medkit/pull/582>`_), and its tests take a DDS domain at run time from the shared allocator instead of a hand-maintained per-package pool (`#597 <https://github.com/selfpatch/ros2_medkit/pull/597>`_)
* Contributors: @bburda

0.6.0 (2026-06-22)
------------------
* No functional changes; version bump for the coordinated 0.6.0 release.
* Contributors: @bburda

0.5.0 (2026-06-08)
------------------
* Updated include paths for the gateway core / ROS 2 ``PluginContext`` layer split and removal of backwards-compat shim headers
* Build: adopt the centralized ``ROS2MedkitWarnings`` cmake module
* Contributors: @bburda

0.4.0 (2026-03-20)
------------------
* Initial release - shared utilities for beacon discovery plugins
* ``BeaconHintStore`` with TTL-based hint transitions and thread safety
* ``BeaconValidator`` input validation gate
* ``BeaconEntityMapper`` to convert discovery hints into ``IntrospectionResult``
* ``build_beacon_response()`` shared response builder
* ``BeaconPlugin`` base class for topic and parameter beacon plugins
* ``TokenBucket`` rate limiter with thread-safe access
* Contributors: @bburda
