^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ros2_medkit_sovd_service_interface
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.7.0 (2026-08-27)
------------------
* A cyclic-subscription request naming a resource path the collection cannot stream is refused instead of being accepted and never delivering
* The sampler declaration stays inside the gateway, so the service interface no longer carries a second copy of it
* Build and test only: the package is instrumented for coverage (`#582 <https://github.com/selfpatch/ros2_medkit/pull/582>`_), and its tests take a DDS domain at run time from the shared allocator (`#597 <https://github.com/selfpatch/ros2_medkit/pull/597>`_)
* Contributors: @bburda

0.6.0 (2026-06-22)
------------------
* ``list_entity_faults`` handles a fault ``status`` returned as an object (not just a string), supporting peer-fault aggregation across daisy-chained gateways (`#419 <https://github.com/selfpatch/ros2_medkit/pull/419>`_)
* Contributors: @bburda

0.5.0 (2026-06-08)
------------------
* Initial release - gateway plugin that exposes the medkit entity tree and fault data over standard ROS 2 services, so other ROS 2 nodes can query entities and faults without going through the HTTP API (`#330 <https://github.com/selfpatch/ros2_medkit/issues/330>`_)
* Creates ROS 2 services under a configurable prefix (default ``/medkit``): ``list_entities``, ``list_entity_faults``, and ``get_capabilities``; a ``get_entity_data`` service is also registered but currently returns "not implemented" (use the HTTP REST API for topic data) pending (`#351 <https://github.com/selfpatch/ros2_medkit/issues/351>`_)
* Runs as a gateway MODULE plugin with read-only ``PluginContext`` access to the entity cache and fault manager; implements no provider interfaces
* Shutdown guards and ``noexcept`` destructors reset ROS resources before member destruction to prevent teardown crashes
* Contributors: @bburda, @mfaferek93
