ros2_medkit_cmake
==================

This section contains design documentation for the ros2_medkit_cmake package.

Overview
--------

The ``ros2_medkit_cmake`` package is a build utility package that provides shared CMake
modules for all other ros2_medkit packages. It contains no runtime code - only CMake
macros and functions that are sourced via ``find_package(ros2_medkit_cmake REQUIRED)``
and ``include()``.

Modules
-------

The package provides these CMake modules, installed to the ament index:

1. **ros2_medkit_cmake-extras.cmake** - Ament extras hook

   - Automatically sourced after ``find_package(ros2_medkit_cmake)``
   - Appends the installed module directory to ``CMAKE_MODULE_PATH``
   - Enables transparent ``include(ROS2MedkitCcache)`` etc. in downstream packages

2. **ROS2MedkitCcache.cmake** - Compiler cache integration

   - Auto-detects ``ccache`` on the system
   - Sets ``CMAKE_C_COMPILER_LAUNCHER`` and ``CMAKE_CXX_COMPILER_LAUNCHER``
   - Respects existing launcher overrides (does not clobber explicit settings)
   - Must be included early in CMakeLists.txt, before ``add_library``/``add_executable``

3. **ROS2MedkitLinting.cmake** - Centralized clang-tidy configuration

   - Provides ``ENABLE_CLANG_TIDY`` option (default OFF; a local gate only, CI
     configures it OFF and runs ``run-clang-tidy`` over the compilation database)
   - Provides ``ros2_medkit_clang_tidy()`` function with optional ``HEADER_FILTER``,
     ``TIMEOUT`` and ``JOBS`` arguments
   - Provides ``ROS2_MEDKIT_CLANG_TIDY_JOBS`` (default ``min(host cores, 2)``),
     capped so one package fits an 8 GB machine; memory scales linearly at
     roughly 1.2 GiB per job. Switch it with ``./scripts/test.sh tidy --jobs <n>``
   - References the shared ``.clang-tidy`` config file from the installed module directory

4. **ROS2MedkitCompat.cmake** - Multi-distro compatibility layer

   - ``medkit_find_yaml_cpp()`` - Resolves yaml-cpp across Humble (no cmake target) and Jazzy (namespaced target)
   - ``medkit_find_cpp_httplib()`` - Finds cpp-httplib >= 0.14 via pkg-config, cmake config, or vendored fallback (``VENDORED_DIR`` param)
   - ``medkit_detect_compat_defs()`` - Detects rclcpp and rosbag2 versions, sets ``MEDKIT_RCLCPP_VERSION_MAJOR`` and ``MEDKIT_ROSBAG2_OLD_TIMESTAMP``
   - ``medkit_apply_compat_defs(target)`` - Applies compile definitions based on detected versions
   - ``medkit_target_dependencies(target ...)`` - Drop-in replacement for ``ament_target_dependencies`` that also works on Lyrical (where ``ament_target_dependencies`` was removed)

5. **ROS2MedkitTestDomain.cmake** - ``ROS_DOMAIN_ID`` allocation for tests

   - ``MEDKIT_DOMAIN_TABLE`` holds the per-package pools; a package names itself with
     ``medkit_init_test_domains(PACKAGE <name>)`` rather than repeating a range
   - ``medkit_set_test_domain(<test>)`` assigns the next domain from the pool and its
     ``RESOURCE_LOCK``
   - ``medkit_add_launch_test(<name> <file>)`` registers a launch test and assigns its domain
     in one call, so a launch test cannot be added without isolation
   - ``medkit_add_domain_allocation_test()`` registers the runtime guard described below

Design Decisions
----------------

Test Domains Avoid the Ephemeral Port Range
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

RTPS gives a DDS domain the UDP slice ``[7400 + 250 * d, 7400 + 250 * d + 249]``, and both
CycloneDDS and Fast-DDS bind inside it without ``SO_REUSEPORT``. The Linux kernel hands out
ephemeral ports from ``net.ipv4.ip_local_port_range``, 32768-60999 by default, which maps
back to domains 101-214. A domain in that band works until an unrelated process on the
machine is given one of its ports first, and then every node on it fails to start with
``failed to bind to ANY:<port>: address in use``.

Test domains are therefore drawn only from 1-100 and 215-231. Domain 0 is left to the
developer shell, and 232 is dropped because its slice runs past 65535.

Pools Are Reused, Under a Lock
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The safe band holds fewer domains than the workspace has tests, so a package's pool wraps
around and two of its tests can share a domain. Sharing is made harmless rather than
unlikely: the same call that assigns a domain puts a ``medkit_dds_domain_<id>``
``RESOURCE_LOCK`` on the test, and CTest will not schedule two tests holding one lock
concurrently. That is load-bearing because ``scripts/test.sh`` runs ``ctest -j $(nproc)``.

A lock only binds inside one CTest run, and colcon runs a separate CTest per package in
parallel, so isolation *between* packages comes from the pools being disjoint instead. The
module checks that rather than trusting it.

The Constraint Is Checked, Not Documented
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Earlier rounds of domain work fixed collisions between our own tests and left the ephemeral
range unexamined, because nothing checked it. Two checks now do:

- at configure time the module validates the entire table, not just the pool being asked
  for, so a package that is not currently being built still cannot hold an unsafe or
  overlapping range. The ephemeral range is read from
  ``/proc/sys/net/ipv4/ip_local_port_range`` and widened to at least the Linux default, so a
  host configured with a narrow range cannot accept an allocation that breaks elsewhere;
- ``test_dds_domain_allocation`` runs on the machine that executes the tests. It reads the
  generated CTest properties back and fails on a domain outside the pool, a domain inside
  that machine's live ephemeral range, or a missing domain lock.

Separate Package
~~~~~~~~~~~~~~~~

Shared CMake modules live in their own ament package rather than being inlined
into each consuming package. This avoids duplication and ensures all packages
use the same compatibility logic. Downstream packages declare
``<buildtool_depend>ros2_medkit_cmake</buildtool_depend>`` in their
``package.xml``.

Multi-Distro Strategy
~~~~~~~~~~~~~~~~~~~~~

Rather than maintaining separate branches per ROS 2 distribution, the compat
module detects version numbers at configure time and adapts. This keeps a single
source tree building on Humble, Jazzy, and Lyrical without ``#ifdef`` proliferation
in application code.
