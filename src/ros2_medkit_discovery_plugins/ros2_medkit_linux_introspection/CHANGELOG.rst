^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ros2_medkit_linux_introspection
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.7.0 (2026-08-27)
------------------
* Container CPU and memory limits are reported on every common cgroup layout, not just one. The reader parses the cgroup v1 line format alongside v2, and resolves the limit files under both ``cgroupns=host`` and ``cgroupns=private`` - the latter being the Docker default, where the container sees its own cgroup mounted directly at ``/sys/fs/cgroup`` and the previously built path did not exist. A limit that could not be read is now distinguished from a container that genuinely has no limit, instead of both being reported as unlimited (`#637 <https://github.com/selfpatch/ros2_medkit/pull/637>`_, `#604 <https://github.com/selfpatch/ros2_medkit/issues/604>`_)
* Limits are read from the cgroup that owns them rather than from the process's own leaf, a legacy controller's limit is no longer outranked by a hierarchy that does not set one, and a limit file is read to its end instead of to the first short read (`#637 <https://github.com/selfpatch/ros2_medkit/pull/637>`_)
* Containers are detected per process rather than once for the whole node, and the ctest suite exercises every supported cgroup layout against synthetic hierarchies (`#637 <https://github.com/selfpatch/ros2_medkit/pull/637>`_)
* Build and test only: the package is instrumented for coverage (`#582 <https://github.com/selfpatch/ros2_medkit/pull/582>`_), and its tests take a DDS domain at run time from the shared allocator (`#597 <https://github.com/selfpatch/ros2_medkit/pull/597>`_)
* Contributors: @bburda

0.6.0 (2026-06-22)
------------------
* No functional changes; version bump for the coordinated 0.6.0 release.
* Contributors: @bburda

0.5.0 (2026-06-08)
------------------
* Migrated the introspection plugins to the ``get_routes()`` plugin API
* Declared ``pkg-config`` as a ``buildtool_depend`` and fixed a route-separator bug
* Build: adopt the centralized ``ROS2MedkitWarnings`` and ``ROS2MedkitSanitizers`` cmake modules
* Contributors: @bburda

0.4.0 (2026-03-20)
------------------
* Initial release - Linux process introspection plugins for ros2_medkit gateway
* ``procfs_plugin`` - process-level diagnostics via ``/proc`` filesystem (CPU, memory, threads, file descriptors)
* ``systemd_plugin`` - systemd unit status and resource usage via D-Bus
* ``container_plugin`` - container runtime detection and cgroup resource limits
* ``PidCache`` with TTL-based refresh for efficient PID-to-node mapping
* ``proc_reader`` and ``cgroup_reader`` utilities with configurable proc root
* Cross-distro support for ROS 2 Humble, Jazzy, and Rolling
* Contributors: @bburda
