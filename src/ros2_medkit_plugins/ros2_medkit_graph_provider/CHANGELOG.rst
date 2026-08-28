^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ros2_medkit_graph_provider
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.7.0 (2026-08-27)
------------------
* **Breaking:** the ``x-medkit-graph`` health model is reworked and ``schema_version`` is now ``"2.0.0"``. ``pipeline_status`` is derived from data freshness and node reachability instead of the previous status model, and no longer reports ``"broken"`` for an edge that simply has no ``/diagnostics`` coverage. The ``error_reason`` values ``node_offline``, ``topic_stale`` and ``no_data_source`` are gone; ``metrics_stale`` is the only reachable value. A consumer that matched on the old values needs a change (`#546 <https://github.com/selfpatch/ros2_medkit/pull/546>`_, `#545 <https://github.com/selfpatch/ros2_medkit/issues/545>`_)
* ``metrics.source`` reports the resolved publisher rather than a placeholder, falling back to the single publisher of a topic where the RMW does not expose the identity directly, and the negotiation topic filter is anchored to path segments so an unrelated topic whose name merely contains the segment is no longer matched (`#546 <https://github.com/selfpatch/ros2_medkit/pull/546>`_)
* Per-function configuration overrides take effect, an invalid override warns instead of being applied silently, and the freshness clock is monotonic with a stateless stale grace, so a wall-clock step no longer moves an edge in or out of staleness (`#546 <https://github.com/selfpatch/ros2_medkit/pull/546>`_)
* Rate measurement is defended against multi-publisher inflation, state is bounded, and shutdown is guarded against callbacks firing on a partially destroyed provider (`#546 <https://github.com/selfpatch/ros2_medkit/pull/546>`_)
* The plugin and its robustness fields are documented, and the never-executed stale-topic path is removed rather than left as dead code (`#546 <https://github.com/selfpatch/ros2_medkit/pull/546>`_)
* Contributors: @bburda

0.6.0 (2026-06-22)
------------------
* No functional changes; version bump for the coordinated 0.6.0 release.
* Contributors: @bburda

0.5.0 (2026-06-08)
------------------
* Migrated ``GraphProviderPlugin`` to the ``get_routes()`` plugin API and fixed a route-separator bug
* Added shutdown guards and ``noexcept`` destructors that reset rclcpp resources before member destruction, preventing teardown SIGSEGV
* Added post-shutdown guard unit tests; dropped the cpp-httplib source install from the Dockerfile (now vendored via ``ros2_medkit_cmake``)
* Build: adopt the centralized ``ROS2MedkitWarnings`` cmake module
* Contributors: @bburda

0.4.0 (2026-03-20)
------------------
* Initial release - extracted from ``ros2_medkit_gateway`` package
* ``GraphProviderPlugin`` for ROS 2 graph-based entity introspection
* Standalone external plugin package with independent build and test
* Locking support via ``PluginContext`` API
* Contributors: @bburda
