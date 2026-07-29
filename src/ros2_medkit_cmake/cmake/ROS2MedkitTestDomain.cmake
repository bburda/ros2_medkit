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
# DDS domain IDs are limited to 0-232 (UDP port formula: 7400 + 250 * domain_id).
# colcon test runs CTest per package in parallel, so different packages MUST use
# non-overlapping domain ID ranges to prevent DDS cross-contamination.
#
# Allocated ranges:
#   ros2_medkit_sovd_service_interface: 1 -   9 (9 slots)
#   ros2_medkit_fault_manager:         10 -  29 (20 slots)
#   ros2_medkit_gateway:               30 -  74 (45 slots)
#   ros2_medkit_graph_watchdog:        75 -  89 (15 slots; launch tests share 89)
#   ros2_medkit_diagnostic_bridge:     90 -  99 (10 slots)
#   ros2_medkit_param_beacon:         100 - 104 (5 slots)
#   ros2_medkit_topic_beacon:         105 - 109 (5 slots)
#   ros2_medkit_fault_reporter:       110 - 112 (3 slots)
#   ros2_medkit_log_bridge:           113 - 115 (3 slots)
#   ros2_medkit_action_status_bridge: 116 - 119 (4 slots)
#   ros2_medkit_graph_provider:       120 - 129 (10 slots)
#   ros2_medkit_integration_tests:    130 - 219 (90 slots)
#   ros2_medkit_opcua:                220 - 229 (10 slots)
#   multi-domain tests (secondary):   230 - 232 (3 slots, reserved for peer_aggregation etc.)
#
# Launch_testing tests (add_launch_test) create real ROS nodes and MUST consume
# a domain, exactly like a gtest. Two concurrently scheduled tests on the same
# domain (the default, 0) discover each other's nodes and cross-contaminate.
# Use medkit_add_launch_test (below), which registers the test and assigns its
# domain in one call, so a launch test can never be added without isolation.
#
# ros2_medkit_linux_introspection reads /proc and cgroups only (no ROS nodes),
# so it needs no domain isolation and reserves no range.
#
# integration_tests (it grows by glob) and opcua are the tightest pools. When
# either fills, reclaim slack from the fixed 5-slot beacon pools above (each
# uses one of five) and re-carve - the 1-232 DDS space is otherwise fully
# allocated.
#
# To add a new package: pick the next free range and update this comment.

# Initialize the domain ID counter for a package.
# Must be called once per CMakeLists.txt before using medkit_set_test_domain.
#
# Usage:
#   medkit_init_test_domains(START 30 END 89)
#
macro(medkit_init_test_domains)
  cmake_parse_arguments(_MTID "" "START;END" "" ${ARGN})
  if(NOT DEFINED _MTID_START OR NOT DEFINED _MTID_END)
    message(FATAL_ERROR "medkit_init_test_domains requires START and END arguments")
  endif()
  set(_MEDKIT_DOMAIN_COUNTER ${_MTID_START})
  set(_MEDKIT_DOMAIN_END ${_MTID_END})
  set(_MEDKIT_DOMAIN_START ${_MTID_START})
endmacro()

# Assign the next available ROS_DOMAIN_ID to a test target.
# The test must already be defined via ament_add_gtest or add_launch_test.
# Uses macro (not function) so the counter persists between calls.
#
# Usage:
#   ament_add_gtest(test_foo test_foo.cpp)
#   medkit_set_test_domain(test_foo)
#
macro(medkit_set_test_domain TEST_NAME)
  if(NOT DEFINED _MEDKIT_DOMAIN_COUNTER)
    message(FATAL_ERROR
      "medkit_set_test_domain called before medkit_init_test_domains")
  endif()
  if(${_MEDKIT_DOMAIN_COUNTER} GREATER ${_MEDKIT_DOMAIN_END})
    message(FATAL_ERROR
      "Domain ID range exhausted (${_MEDKIT_DOMAIN_START}-${_MEDKIT_DOMAIN_END}). "
      "Increase END or allocate a new range in ROS2MedkitTestDomain.cmake.")
  endif()
  set_tests_properties(${TEST_NAME} PROPERTIES
    ENVIRONMENT "ROS_DOMAIN_ID=${_MEDKIT_DOMAIN_COUNTER}"
  )
  math(EXPR _MEDKIT_DOMAIN_COUNTER "${_MEDKIT_DOMAIN_COUNTER} + 1")
endmacro()

# Register a launch_testing test AND assign it a unique DDS domain in one call.
# This is the required way to add a launch test: it makes it impossible to add
# one without domain isolation (see the note above about domain-0 collisions).
# Do not call add_launch_test directly.
#
# The caller may still set extra properties afterwards (e.g. LABELS), since this
# only sets the ENVIRONMENT and TIMEOUT properties.
#
# Usage:
#   medkit_add_launch_test(test_integration test/test_integration.test.py)
#   medkit_add_launch_test(test_integration test/test_integration.test.py TIMEOUT 90)
macro(medkit_add_launch_test TEST_NAME TEST_FILE)
  cmake_parse_arguments(_MALT "" "TIMEOUT" "" ${ARGN})
  if(DEFINED _MALT_TIMEOUT)
    add_launch_test(${TEST_FILE} TARGET ${TEST_NAME} TIMEOUT ${_MALT_TIMEOUT})
  else()
    add_launch_test(${TEST_FILE} TARGET ${TEST_NAME})
  endif()
  medkit_set_test_domain(${TEST_NAME})
endmacro()
