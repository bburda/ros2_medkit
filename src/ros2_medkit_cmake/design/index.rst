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

The package provides four CMake modules installed to the ament index:

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

Design Decisions
----------------

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
