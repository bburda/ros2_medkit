# Copyright 2026 bburda
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 3.8)
project(ros2_medkit_cmake NONE)

find_package(ament_cmake REQUIRED)

# Install cmake modules to the ament share directory.
# The *-extras.cmake hook adds this path to CMAKE_MODULE_PATH automatically.
install(
  FILES
    cmake/ROS2MedkitCompat.cmake
    cmake/ROS2MedkitCcache.cmake
    cmake/ROS2MedkitCoverage.cmake
    cmake/ROS2MedkitLinting.cmake
    cmake/ROS2MedkitSanitizers.cmake
    cmake/ROS2MedkitTestDomain.cmake
    cmake/ROS2MedkitWarnings.cmake
  DESTINATION share/${PROJECT_NAME}/cmake
)

# Installed next to the modules, not in a scripts/ directory of its own, so
# ROS2MedkitTestDomain.cmake can find them from CMAKE_CURRENT_LIST_DIR under both
# a plain install and a --symlink-install.
install(
  PROGRAMS
    scripts/check_test_domains.py
    scripts/domain_crosstalk_peer.py
    scripts/medkit_domain.py
    scripts/medkit_domain_runner.py
    scripts/medkit_run_with_domain.py
  DESTINATION share/${PROJECT_NAME}/cmake
)

# The shared lint configs are files of this package and are installed next to
# the cmake modules, so medkit_lint_config() finds them at the same relative
# path from a source tree, a plain install and a --symlink-install. The paths
# every editor and pre-commit hook uses - .clang-format, .clang-tidy and .flake8
# at the repository root - are symlinks to these files.
#
# Unconditional on purpose. The previous shape copied .clang-tidy from the
# repository root if it happened to be there, which made a package built from
# its own exported directory ship without it and lint against nothing.
install(
  FILES
    cmake/.clang-format
    cmake/.clang-tidy
    cmake/.flake8
  DESTINATION share/${PROJECT_NAME}/cmake
)

if(BUILD_TESTING)
  # Lints the domain-allocation scripts. Without this they sit outside
  # `colcon test -L linter` even though every package in the tree runs them.
  find_package(ament_cmake_flake8 REQUIRED)
  # Without CONFIG_FILE, ament_flake8 reads the ini bundled inside the
  # ament_flake8 package and the shared .flake8 never applies here. This package
  # owns the config, so it resolves it from its own source tree rather than from
  # an installed copy - the same reason it includes ROS2MedkitTestDomain.cmake
  # by path below.
  include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/ROS2MedkitLinting.cmake)
  medkit_lint_config(.flake8 _medkit_flake8_config)
  ament_flake8(CONFIG_FILE "${_medkit_flake8_config}")

  find_package(ament_cmake_pytest REQUIRED)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)

  # This package owns the domain machinery, so it includes it from its own
  # source tree rather than from an installed copy. The per-package gate arms
  # itself from the extras hook; the workspace sweep is registered once, here,
  # because this package is a dependency of every other one and colcon builds
  # and tests it first.
  include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/ROS2MedkitTestDomain.cmake)
  medkit_add_domain_coverage_test()
  # The extras hook cannot arm this package: it is what the hook belongs to, so
  # nothing here ever calls find_package(ros2_medkit_cmake). Same deferral, so
  # the gate still registers after this package's own tests exist.
  cmake_language(DEFER CALL _medkit_register_domain_gate)
  cmake_language(DEFER CALL _medkit_set_domain_wait_budgets)

  # These three drive the allocator directly: they hold coordination ports across
  # the whole band to reach the degenerate cases (band exhausted, every domain
  # busy), so they must not hold a domain of their own, and they must not run
  # while a test in this package is holding one. The lock buys the second part;
  # this package sits at the root of the dependency graph, so colcon tests it
  # before any package that allocates domains.
  ament_add_pytest_test(test_domain_band test/test_domain_band.py TIMEOUT 300)
  set_tests_properties(test_domain_band PROPERTIES LABELS "unit")
  medkit_test_needs_no_domain(test_domain_band)

  ament_add_pytest_test(test_domain_wrapper test/test_domain_wrapper.py TIMEOUT 900)
  set_tests_properties(test_domain_wrapper PROPERTIES LABELS "integration")
  medkit_test_needs_no_domain(test_domain_wrapper)

  ament_add_pytest_test(test_domain_check test/test_domain_check.py TIMEOUT 300)
  set_tests_properties(test_domain_check PROPERTIES LABELS "unit")
  medkit_test_needs_no_domain(test_domain_check)

  # The shared lint configs, checked from the package rather than from the
  # workspace: this is the one test that has to keep working when the package is
  # built alone, because that is the tree where the configs used to go missing.
  ament_add_pytest_test(test_lint_config test/test_lint_config.py TIMEOUT 300)
  set_tests_properties(test_lint_config PROPERTIES LABELS "unit")
  medkit_test_needs_no_domain(test_lint_config)

  set_property(TEST test_domain_band APPEND PROPERTY RESOURCE_LOCK medkit_domain_band)
  set_property(TEST test_domain_wrapper APPEND PROPERTY RESOURCE_LOCK medkit_domain_band)

  # Real ROS nodes: the control that proves the probe can see cross-talk at all,
  # and the isolated pair that must then hear nothing.
  medkit_add_pytest_test(test_domain_crosstalk test/test_domain_crosstalk.py TIMEOUT 600)
  set_tests_properties(test_domain_crosstalk PROPERTIES LABELS "integration")
  set_property(TEST test_domain_crosstalk APPEND PROPERTY RESOURCE_LOCK medkit_domain_band)

  # This project is declared NONE, so no language is enabled and CMake leaves
  # CMAKE_LIBRARY_ARCHITECTURE and CMAKE_SIZEOF_VOID_P empty. Anything that
  # reaches the deprecated FindPythonLibs then fails to find libpython even when
  # the development headers are installed, because find_library never searches
  # the multiarch directory. launch_testing_ament_cmake takes that route on
  # Humble through python_cmake_module, which is why the launch test that proves
  # a launch test's children inherit its domain lives in
  # ros2_medkit_fault_reporter instead: that package compiles, so the search
  # works there. Every package in the tree depends on this one, so a
  # configure-time failure here stops the whole workspace. Keep test-only
  # find_package calls out of it.

  # The two halves of the cross-package probe live in ros2_medkit_log_bridge and
  # ros2_medkit_action_status_bridge, which colcon tests concurrently.
endif()

ament_package(
  CONFIG_EXTRAS cmake/ros2_medkit_cmake-extras.cmake
)
