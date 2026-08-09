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

# ROS_DOMAIN_ID allocation for tests.
#
# ---------------------------------------------------------------------------
# What this does
# ---------------------------------------------------------------------------
#
# Every test registered through the macros below runs behind a wrapper that
# takes a DDS domain when the test starts, holds it through an open socket for
# exactly as long as the test runs, and gives it back when the test process
# ends - including when it is killed, because the kernel closes the socket.
#
# Nothing here decides which domain a test gets, and nothing here has to know
# how many tests a package has. There is no table to extend, no per-package
# pool to size and no disjointness to maintain: the socket is the allocation.
#
# That socket is also why the isolation reaches across packages. colcon runs one
# ctest per package, in parallel, and a CTest RESOURCE_LOCK only binds inside a
# single ctest run - which is what used to force every package onto its own
# range. Two processes contending for the same port do not care which ctest run
# started them.
#
# The band the wrapper draws from, and the reasoning behind it, live in
# scripts/medkit_domain.py next to the code that uses them.
#
# ---------------------------------------------------------------------------
# Registering a test
# ---------------------------------------------------------------------------
#
#   medkit_add_gtest(test_foo test/test_foo.cpp)          # instead of ament_add_gtest
#   medkit_add_gmock(test_bar test/test_bar.cpp)          # instead of ament_add_gmock
#   medkit_add_launch_test(test_e2e test/test_e2e.test.py TIMEOUT 120)
#   medkit_add_pytest_test(test_py test/test_py.py)
#   medkit_add_wrapped_test(test_script COMMAND "${Python3_EXECUTABLE}" "${_script}")
#
# All of them take an optional DOMAINS <n> for a test that needs more than one
# domain at a time. The first is exported as ROS_DOMAIN_ID and the rest as
# MEDKIT_SECONDARY_DOMAINS, which ros2_medkit_test_utils.constants reads.
#
# A test that creates no ROS entities at all can say so with
# medkit_test_needs_no_domain(<test>), and test_dds_domain_allocation reports
# every test that does neither.

# Where this module lives, captured at include time. Inside a macro,
# CMAKE_CURRENT_LIST_DIR would point at the calling CMakeLists.txt instead.
set(_MEDKIT_TEST_DOMAIN_MODULE_DIR "${CMAKE_CURRENT_LIST_DIR}")

# The scripts are installed next to the modules, so an installed module finds
# them beside itself. ros2_medkit_cmake includes this module out of its own
# source tree, where they are one directory over.
if(EXISTS "${_MEDKIT_TEST_DOMAIN_MODULE_DIR}/../scripts/medkit_domain.py")
  get_filename_component(MEDKIT_TEST_DOMAIN_SCRIPT_DIR
    "${_MEDKIT_TEST_DOMAIN_MODULE_DIR}/../scripts" ABSOLUTE)
else()
  set(MEDKIT_TEST_DOMAIN_SCRIPT_DIR "${_MEDKIT_TEST_DOMAIN_MODULE_DIR}")
endif()

# Replaces ament's run_test.py, so the domain is held by the process that
# already supervises the test command rather than by one added in front of it.
set(MEDKIT_TEST_DOMAIN_RUNNER "${MEDKIT_TEST_DOMAIN_SCRIPT_DIR}/medkit_domain_runner.py")
# For tests registered with a plain add_test, which build their own command.
set(MEDKIT_TEST_DOMAIN_EXEC "${MEDKIT_TEST_DOMAIN_SCRIPT_DIR}/medkit_run_with_domain.py")
set(_MEDKIT_TEST_DOMAIN_CHECK "${MEDKIT_TEST_DOMAIN_SCRIPT_DIR}/check_test_domains.py")
# Fixture behind the cross-talk probes; see scripts/domain_crosstalk_peer.py.
set(MEDKIT_TEST_DOMAIN_CROSSTALK_PEER
  "${MEDKIT_TEST_DOMAIN_SCRIPT_DIR}/domain_crosstalk_peer.py")

foreach(_medkit_required_script
  "${MEDKIT_TEST_DOMAIN_RUNNER}"
  "${MEDKIT_TEST_DOMAIN_EXEC}"
  "${_MEDKIT_TEST_DOMAIN_CHECK}")
  if(NOT EXISTS "${_medkit_required_script}")
    message(FATAL_ERROR
      "ROS2MedkitTestDomain: ${_medkit_required_script} is missing. Without it a test "
      "would silently run on the default domain 0 and see every node on the machine.")
  endif()
endforeach()

# Record how many domains a registered test needs, for the wrapper to read and
# for test_dds_domain_allocation to check.
#
# APPEND, so a caller may add its own environment entries. Use
# set_property(TEST ... APPEND PROPERTY ENVIRONMENT ...) for those: a plain
# set_tests_properties(... PROPERTIES ENVIRONMENT ...) after this call replaces
# the count instead of adding to it, and the domain check says so.
macro(_medkit_declare_domains TEST_NAME COUNT)
  if(NOT "${COUNT}" MATCHES "^[1-9][0-9]*$")
    message(FATAL_ERROR
      "medkit test domains: DOMAINS must be a positive integer, got '${COUNT}' for "
      "${TEST_NAME}. A test that needs no domain uses medkit_test_needs_no_domain().")
  endif()
  set_property(TEST ${TEST_NAME} APPEND PROPERTY ENVIRONMENT "MEDKIT_TEST_DOMAINS=${COUNT}")
endmacro()

# ament_add_gtest with a domain. Same arguments, plus DOMAINS <n>.
#
# Usage:
#   medkit_add_gtest(test_foo test/test_foo.cpp)
#   medkit_add_gtest(test_foo test/test_foo.cpp TIMEOUT 180 DOMAINS 2)
macro(medkit_add_gtest TARGET)
  cmake_parse_arguments(_MAGT "" "DOMAINS" "" ${ARGN})
  if(NOT DEFINED _MAGT_DOMAINS)
    set(_MAGT_DOMAINS 1)
  endif()
  ament_add_gtest(${TARGET} ${_MAGT_UNPARSED_ARGUMENTS}
    RUNNER "${MEDKIT_TEST_DOMAIN_RUNNER}")
  if(TARGET ${TARGET})
    _medkit_declare_domains(${TARGET} ${_MAGT_DOMAINS})
  endif()
endmacro()

# ament_add_gmock with a domain. Same arguments, plus DOMAINS <n>.
macro(medkit_add_gmock TARGET)
  cmake_parse_arguments(_MAGM "" "DOMAINS" "" ${ARGN})
  if(NOT DEFINED _MAGM_DOMAINS)
    set(_MAGM_DOMAINS 1)
  endif()
  ament_add_gmock(${TARGET} ${_MAGM_UNPARSED_ARGUMENTS}
    RUNNER "${MEDKIT_TEST_DOMAIN_RUNNER}")
  if(TARGET ${TARGET})
    _medkit_declare_domains(${TARGET} ${_MAGM_DOMAINS})
  endif()
endmacro()

# ament_add_pytest_test with a domain. Same arguments, plus DOMAINS <n>.
macro(medkit_add_pytest_test TEST_NAME TEST_PATH)
  cmake_parse_arguments(_MAPT "" "DOMAINS" "" ${ARGN})
  if(NOT DEFINED _MAPT_DOMAINS)
    set(_MAPT_DOMAINS 1)
  endif()
  ament_add_pytest_test(${TEST_NAME} ${TEST_PATH} ${_MAPT_UNPARSED_ARGUMENTS}
    RUNNER "${MEDKIT_TEST_DOMAIN_RUNNER}")
  _medkit_declare_domains(${TEST_NAME} ${_MAPT_DOMAINS})
endmacro()

# Register a launch_testing test AND give it a domain in one call.
# This is the required way to add a launch test: it makes it impossible to add
# one without isolation, and a launch test left on the default domain 0 sees
# every other node on the machine. Do not call add_launch_test directly.
#
# The domain is exported by the runner, so every process the launch description
# starts inherits it.
#
# Usage:
#   medkit_add_launch_test(test_integration test/test_integration.test.py)
#   medkit_add_launch_test(test_multi test/test_multi.test.py TIMEOUT 300 DOMAINS 4)
macro(medkit_add_launch_test TEST_NAME TEST_FILE)
  cmake_parse_arguments(_MALT "" "TIMEOUT;DOMAINS" "" ${ARGN})
  if(NOT DEFINED _MALT_DOMAINS)
    set(_MALT_DOMAINS 1)
  endif()
  set(_MALT_EXTRA_ARGS "")
  if(DEFINED _MALT_TIMEOUT)
    list(APPEND _MALT_EXTRA_ARGS TIMEOUT ${_MALT_TIMEOUT})
  endif()
  # RUNNER is not a parameter add_launch_test knows; it forwards what it does
  # not recognise to ament_add_test, which does. If a future launch_testing
  # stopped forwarding it, the test would quietly run on domain 0 - which is
  # exactly what test_dds_domain_allocation reads the generated command for.
  add_launch_test(${TEST_FILE}
    TARGET ${TEST_NAME}
    ${_MALT_EXTRA_ARGS}
    ${_MALT_UNPARSED_ARGUMENTS}
    RUNNER "${MEDKIT_TEST_DOMAIN_RUNNER}")
  _medkit_declare_domains(${TEST_NAME} ${_MALT_DOMAINS})
endmacro()

# Register an arbitrary command as a test that holds a domain.
# For tests that build their own command line and so cannot take an ament test
# runner. Everything else should use one of the macros above.
#
# Usage:
#   medkit_add_wrapped_test(test_script
#     COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/test/script.py")
macro(medkit_add_wrapped_test TEST_NAME)
  cmake_parse_arguments(_MAWT "" "DOMAINS;WORKING_DIRECTORY" "COMMAND" ${ARGN})
  if(NOT _MAWT_COMMAND)
    message(FATAL_ERROR "medkit_add_wrapped_test(${TEST_NAME}) needs a COMMAND")
  endif()
  if(NOT DEFINED _MAWT_DOMAINS)
    set(_MAWT_DOMAINS 1)
  endif()
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  set(_MAWT_WD_ARGS "")
  if(DEFINED _MAWT_WORKING_DIRECTORY)
    set(_MAWT_WD_ARGS WORKING_DIRECTORY "${_MAWT_WORKING_DIRECTORY}")
  endif()
  add_test(
    NAME ${TEST_NAME}
    COMMAND "${Python3_EXECUTABLE}" "${MEDKIT_TEST_DOMAIN_EXEC}"
      "--domains" "${_MAWT_DOMAINS}" "--" ${_MAWT_COMMAND}
    ${_MAWT_WD_ARGS}
  )
  _medkit_declare_domains(${TEST_NAME} ${_MAWT_DOMAINS})
endmacro()

# Declare that a test creates no ROS entities and therefore needs no domain.
#
# The only way past test_dds_domain_allocation for a test that does not go
# through the wrapper. Written at the call site so the claim is greppable and
# ages with the test, rather than sitting in a list somewhere else.
#
# A CTest label rather than an environment entry, because a test's environment
# is inherited by everything it starts: a test that runs a wrapped command of
# its own would hand the child its own "needs no domain" declaration.
#
# Call it AFTER any set_tests_properties(... LABELS ...) on the same test - that
# form replaces labels rather than adding to them. Getting the order wrong loses
# the declaration, and test_dds_domain_allocation then reports the test, which is
# the safe direction to fail in.
#
# Usage:
#   add_test(NAME core_only COMMAND $<TARGET_FILE:test_core_only>)
#   set_tests_properties(core_only PROPERTIES LABELS "unit")
#   medkit_test_needs_no_domain(core_only)
macro(medkit_test_needs_no_domain TEST_NAME)
  set_property(TEST ${TEST_NAME} APPEND PROPERTY LABELS "no_ros_domain")
endmacro()

# Register the per-package guard that re-reads the generated CTest properties on
# the machine that RUNS the tests.
#
# Nothing at configure time can see whether a test ended up behind the wrapper:
# the command is assembled by ament, and a later set_tests_properties can drop
# the count that goes with it. This reads what was actually generated.
#
# NOT called by packages. ros2_medkit_cmake-extras.cmake defers a call to this
# for every package that does find_package(ros2_medkit_cmake), which is the
# point: a gate a package has to ask for is a gate the next package will not
# have. The deferral is what makes it possible to register from the extras hook
# at all - the hook runs before a package's tests exist, and in several packages
# before BUILD_TESTING is even defined, so the work has to happen at the end of
# the directory instead of where the hook fires.
# Give every wrapped test a wait budget that fits inside its own CTest TIMEOUT.
#
# The allocator used to wait a fixed 180s for a domain. ament_add_test defaults
# to TIMEOUT 60, so on almost every test in the tree CTest killed the process
# while the allocator was still waiting, and the message explaining that no
# domain could be had never printed. Deferred to the end of the directory
# because several call sites raise TIMEOUT with a set_tests_properties AFTER
# registering the test, and the budget has to be read after that has happened.
function(_medkit_set_domain_wait_budgets)
  if(NOT BUILD_TESTING)
    return()
  endif()
  get_property(_mswb_tests DIRECTORY PROPERTY TESTS)
  foreach(_mswb_test IN LISTS _mswb_tests)
    get_property(_mswb_env TEST ${_mswb_test} PROPERTY ENVIRONMENT)
    string(FIND "${_mswb_env}" "MEDKIT_TEST_DOMAINS=" _mswb_wrapped)
    if(_mswb_wrapped LESS 0)
      continue()
    endif()
    string(FIND "${_mswb_env}" "MEDKIT_TEST_DOMAIN_WAIT=" _mswb_explicit)
    if(NOT _mswb_explicit LESS 0)
      continue()  # a caller said what it wants; leave it alone
    endif()
    get_property(_mswb_timeout TEST ${_mswb_test} PROPERTY TIMEOUT)
    if(NOT _mswb_timeout)
      # No TIMEOUT property means CTest's own default of 1500 seconds.
      set(_mswb_timeout 1500)
    endif()
    string(REGEX REPLACE "\\..*$" "" _mswb_timeout "${_mswb_timeout}")
    # Half the budget: long enough to ride out a busy band, short enough that
    # the wrapper reports in its own words instead of being killed mid-wait.
    math(EXPR _mswb_wait "${_mswb_timeout} / 2")
    if(_mswb_wait LESS 5)
      set(_mswb_wait 5)
    endif()
    set_property(TEST ${_mswb_test} APPEND PROPERTY
      ENVIRONMENT "MEDKIT_TEST_DOMAIN_WAIT=${_mswb_wait}")
  endforeach()
endfunction()

function(_medkit_register_domain_gate)
  if(NOT BUILD_TESTING)
    return()
  endif()
  get_property(_mrdg_tests DIRECTORY PROPERTY TESTS)
  if(NOT _mrdg_tests)
    return()
  endif()
  if("test_dds_domain_allocation" IN_LIST _mrdg_tests)
    return()
  endif()
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  add_test(
    NAME test_dds_domain_allocation
    COMMAND "${Python3_EXECUTABLE}" "${_MEDKIT_TEST_DOMAIN_CHECK}"
      --build-dir "${CMAKE_CURRENT_BINARY_DIR}"
      --package "${PROJECT_NAME}"
  )
  # Labelled "gate" so a developer preset can select it. Deliberately not
  # "linter": CI runs with -LE linter, and a gate excluded from CI is not a
  # gate. scripts/test.sh selects "linter|gate" and "integration|gate", since
  # ctest -L takes a regex.
  set_tests_properties(test_dds_domain_allocation PROPERTIES LABELS "gate")
endfunction()

# Register the workspace-wide sweep, which is the half of the gate that reaches
# packages the per-package guard cannot.
#
# The per-package guard rides on find_package(ros2_medkit_cmake). A package that
# never finds it - a new one, or an interface-only one - would carry no gate and
# report nothing, which is the same opt-in hole in a different place. This walks
# every sibling package build directory in the workspace and applies the same
# rule to all of them, and additionally reports a package that has tests of its
# own but no per-package guard, because that is exactly how a package leaves the
# scheme.
#
# Registered once, by ros2_medkit_cmake, which every other package depends on
# and which colcon therefore builds first.
macro(medkit_add_domain_coverage_test)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  get_filename_component(_MADCT_WORKSPACE_BUILD "${CMAKE_BINARY_DIR}/.." ABSOLUTE)
  # The sweep must only judge packages from this repository. An overlay
  # workspace that source-builds a dependency next to us would otherwise be
  # told to rewrite a package nobody here maintains. Every package build
  # directory records its source directory, so the sweep filters on that
  # against the repository root rather than on a name.
  get_filename_component(_MADCT_REPO_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)
  add_test(
    NAME test_dds_domain_coverage
    COMMAND "${Python3_EXECUTABLE}" "${_MEDKIT_TEST_DOMAIN_CHECK}"
      --workspace-build-dir "${_MADCT_WORKSPACE_BUILD}"
      --repo-root "${_MADCT_REPO_ROOT}"
  )
  set_tests_properties(test_dds_domain_coverage PROPERTIES LABELS "gate")
endmacro()
