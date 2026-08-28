^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ros2_medkit_serialization
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.7.0 (2026-08-27)
------------------
* Build and test only: the package is instrumented for coverage (`#582 <https://github.com/selfpatch/ros2_medkit/pull/582>`_), its tests take a DDS domain at run time from the shared allocator (`#597 <https://github.com/selfpatch/ros2_medkit/pull/597>`_), and clang-tidy no longer reports findings in vendored headers that the header filter matched
* Contributors: @bburda, @mfaferek93

0.6.0 (2026-06-22)
------------------
* ``TypeIntrospection`` resolves service and action ``type_info`` schemas lazily and caches them as shared, immutable per-type objects, so discovery no longer rebuilds and deep-copies operation schemas on every refresh (`#462 <https://github.com/selfpatch/ros2_medkit/pull/462>`_)
* Contributors: @bburda

0.5.0 (2026-06-08)
------------------
* ``TypeIntrospection`` relocated into this package as part of the gateway core / ROS 2 layer split
* Build: adopt the centralized ``ROS2MedkitWarnings`` and ``ROS2MedkitSanitizers`` cmake modules
* Contributors: @bburda

0.4.0 (2026-03-20)
------------------
* Enable ``POSITION_INDEPENDENT_CODE`` for MODULE target compatibility
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
* Runtime JSON to ROS 2 message serialization using vendored dynmsg C++ API
* TypeCache - thread-safe caching of ROS type introspection data with shared_mutex
  for read concurrency
* JsonSerializer - bidirectional JSON <-> ROS message conversion via dynmsg YAML bridge,
  including CDR serialization/deserialization for GenericClient/GenericSubscription
* ServiceActionTypes - helper utilities for resolving service and action internal types
  (request/response, goal/result/feedback)
* SerializationError exception hierarchy for structured error handling
* Contributors: Bartosz Burda
