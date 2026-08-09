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
project(ros2_medkit_graph_watchdog)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ros2_medkit_cmake REQUIRED)
include(ROS2MedkitCompat)
include(ROS2MedkitCcache)
include(ROS2MedkitSanitizers)
include(ROS2MedkitCoverage)
include(ROS2MedkitLinting)
include(ROS2MedkitWarnings)

find_package(ament_cmake REQUIRED)
find_package(ros2_medkit_gateway REQUIRED)
find_package(ros2_medkit_msgs REQUIRED)
find_package(rclcpp REQUIRED)
# The reused gateway parameter transport includes rcl_interfaces message headers directly;
# it built only through rclcpp's transitive exports.
find_package(rcl_interfaces REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# ros2_medkit_gateway only exports its include directories via ament (no
# installed gateway_core/gateway_ros2 library - see its CMakeLists.txt), so
# ros2_medkit_gateway exports its include dirs but installs no library target,
# so these non-header-only helpers must be compiled in. Their
# headers must come from the SAME tree: the installed package can be older than
# the workspace, and then a reused source fails to see its own members.
set(GATEWAY_SRC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../ros2_medkit_gateway"
    CACHE PATH "Path to ros2_medkit_gateway source (for reused non-header-only helpers)")
set(GATEWAY_SRC_INCLUDE_DIR "${GATEWAY_SRC_DIR}/include")
set(LIFECYCLE_STATE_READER_SOURCES
  ${GATEWAY_SRC_DIR}/src/core/status/lifecycle_status_helpers.cpp
  ${GATEWAY_SRC_DIR}/src/ros2/status/ros2_lifecycle_state_reader.cpp)
# Fail early with a clear message if the reused gateway sources are missing
# (e.g. building the package outside the workspace),
# instead of a later non-obvious compile error.
foreach(_gw_src ${LIFECYCLE_STATE_READER_SOURCES})
  if(NOT EXISTS "${_gw_src}")
    message(FATAL_ERROR
      "ros2_medkit_graph_watchdog: required gateway source not found: ${_gw_src}. "
      "Set -DGATEWAY_SRC_DIR to the ros2_medkit_gateway source tree "
      "(default assumes the standard workspace layout).")
  endif()
endforeach()

set(PARAMETER_TRANSPORT_SOURCES
  ${GATEWAY_SRC_DIR}/src/ros2/transports/ros2_parameter_transport.cpp)
foreach(_gw_src ${PARAMETER_TRANSPORT_SOURCES})
  if(NOT EXISTS "${_gw_src}")
    message(FATAL_ERROR
      "ros2_medkit_graph_watchdog: required gateway source not found: ${_gw_src}. "
      "Set -DGATEWAY_SRC_DIR to the ros2_medkit_gateway source tree.")
  endif()
endforeach()

install(DIRECTORY include/ DESTINATION include)

file(GLOB DETECTOR_SRCS CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/src/detectors/*.cpp)

add_library(ros2_medkit_graph_watchdog MODULE
  src/graph_watchdog_plugin.cpp
  src/graph_watchdog_plugin_exports.cpp
  src/detector_registry.cpp
  src/lifecycle_watcher.cpp
  src/reliability_gate.cpp
  ${LIFECYCLE_STATE_READER_SOURCES}
  ${PARAMETER_TRANSPORT_SOURCES}
  ${DETECTOR_SRCS})
target_include_directories(ros2_medkit_graph_watchdog PRIVATE include ${GATEWAY_SRC_INCLUDE_DIR})
medkit_target_dependencies(ros2_medkit_graph_watchdog
  ros2_medkit_gateway rclcpp rcl_interfaces ros2_medkit_msgs lifecycle_msgs tf2_msgs geometry_msgs)
target_link_libraries(ros2_medkit_graph_watchdog nlohmann_json::nlohmann_json)
install(TARGETS ros2_medkit_graph_watchdog LIBRARY DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  include(ROS2MedkitTestDomain)

  find_package(ament_lint_auto REQUIRED)
  # uncrustify/cpplint conflict with project-wide clang-format (120 cols vs 100).
  # flake8 now runs here too. Only pep257 stays excluded, deferred together
  # with the same change in ros2_medkit_integration_tests - see that
  # package's CMakeLists.txt for the docstring backlog behind it. copyright,
  # lint_cmake and xmllint stay on.
  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_uncrustify
    ament_cmake_cpplint
    ament_cmake_clang_format
    ament_cmake_pep257
  )
  ament_lint_auto_find_test_dependencies()

  # ci.yml skips the ament_cmake_clang_format rosdep key in the build-and-test
  # job (linters only run in the Jazzy quality job), so QUIET + FOUND check
  # avoids hard-failing when the package is not installed.
  find_package(ament_cmake_clang_format QUIET)
  if(ament_cmake_clang_format_FOUND)
    file(GLOB_RECURSE _format_files "include/*.hpp" "src/*.cpp" "test/*.cpp")
    medkit_lint_config(.clang-format _clang_format_config)
    ament_clang_format(${_format_files}
      CONFIG_FILE "${_clang_format_config}")
  endif()

  find_package(ament_cmake_gtest REQUIRED)
  find_package(rosgraph_msgs REQUIRED)
  # test_param_drift_integration.cpp includes rcutils/logging.h directly.
  find_package(rcutils REQUIRED)

  medkit_add_gtest(test_warmup_tracker test/test_warmup_tracker.cpp)
  target_include_directories(test_warmup_tracker PRIVATE include ${GATEWAY_SRC_INCLUDE_DIR})

  medkit_add_gtest(test_fault_request test/test_fault_request.cpp)
  target_include_directories(test_fault_request PRIVATE include ${GATEWAY_SRC_INCLUDE_DIR})
  medkit_target_dependencies(test_fault_request ros2_medkit_msgs)

  medkit_add_gtest(test_detector
    test/test_detector.cpp
    src/reliability_gate.cpp
    src/lifecycle_watcher.cpp
    ${LIFECYCLE_STATE_READER_SOURCES})
  target_include_directories(test_detector PRIVATE include ${GATEWAY_SRC_INCLUDE_DIR})
  medkit_target_dependencies(test_detector rclcpp ros2_medkit_msgs ros2_medkit_gateway lifecycle_msgs)
  target_link_libraries(test_detector nlohmann_json::nlohmann_json)

  medkit_add_gtest(test_detector_registry
    test/test_detector_registry.cpp
    src/detector_registry.cpp)
  target_include_directories(test_detector_registry PRIVATE include ${GATEWAY_SRC_INCLUDE_DIR})
  medkit_target_dependencies(test_detector_registry rclcpp ros2_medkit_msgs)
  target_link_libraries(test_detector_registry nlohmann_json::nlohmann_json)

  medkit_add_gtest(test_detector_config test/test_detector_config.cpp)
  target_include_directories(test_detector_config PRIVATE include ${GATEWAY_SRC_INCLUDE_DIR})
  medkit_target_dependencies(test_detector_config rclcpp ros2_medkit_msgs)
  target_link_libraries(test_detector_config nlohmann_json::nlohmann_json)

  medkit_add_gtest(test_watchdog_clock test/test_watchdog_clock.cpp)
  target_include_directories(test_watchdog_clock PRIVATE include ${GATEWAY_SRC_INCLUDE_DIR})
  medkit_target_dependencies(test_watchdog_clock rclcpp rosgraph_msgs)

  medkit_add_gtest(test_graph_watchdog_plugin
    test/test_graph_watchdog_plugin.cpp
    src/graph_watchdog_plugin.cpp
    src/graph_watchdog_plugin_exports.cpp
    src/detector_registry.cpp
    src/reliability_gate.cpp
    src/lifecycle_watcher.cpp
    ${LIFECYCLE_STATE_READER_SOURCES})
  target_include_directories(test_graph_watchdog_plugin PRIVATE include ${GATEWAY_SRC_INCLUDE_DIR})
  medkit_target_dependencies(test_graph_watchdog_plugin
    ros2_medkit_gateway rclcpp ros2_medkit_msgs lifecycle_msgs)
  target_link_libraries(test_graph_watchdog_plugin nlohmann_json::nlohmann_json)

  medkit_add_gtest(test_lifecycle_watcher
    test/test_lifecycle_watcher.cpp
    src/lifecycle_watcher.cpp
    ${LIFECYCLE_STATE_READER_SOURCES})
  target_include_directories(test_lifecycle_watcher PRIVATE include ${GATEWAY_SRC_INCLUDE_DIR})
  medkit_target_dependencies(test_lifecycle_watcher ros2_medkit_gateway rclcpp lifecycle_msgs)

  medkit_add_gtest(test_reliability_gate
    test/test_reliability_gate.cpp
    src/reliability_gate.cpp
    src/lifecycle_watcher.cpp
    ${LIFECYCLE_STATE_READER_SOURCES})
  target_include_directories(test_reliability_gate PRIVATE include ${GATEWAY_SRC_INCLUDE_DIR})
  medkit_target_dependencies(test_reliability_gate ros2_medkit_gateway rclcpp ros2_medkit_msgs lifecycle_msgs)
  target_link_libraries(test_reliability_gate nlohmann_json::nlohmann_json)

  # Pure matching logic over hand-built inputs: no rclcpp::Node anywhere in it.
  medkit_add_gtest(test_orphan_policy test/test_orphan_policy.cpp)
  target_include_directories(test_orphan_policy PRIVATE include)

  medkit_add_gtest(test_param_drift_policy test/test_param_drift_policy.cpp)
  target_include_directories(test_param_drift_policy PRIVATE include)
  target_link_libraries(test_param_drift_policy nlohmann_json::nlohmann_json)

  medkit_add_gtest(test_qos_policy test/test_qos_policy.cpp)
  target_include_directories(test_qos_policy PRIVATE include)
  medkit_target_dependencies(test_qos_policy rmw)   # rmw is transitively found via rclcpp; no find_package needed

  medkit_add_gtest(test_aggregated_fault test/test_aggregated_fault.cpp)
  target_include_directories(test_aggregated_fault PRIVATE include)
  medkit_target_dependencies(test_aggregated_fault ros2_medkit_gateway rclcpp ros2_medkit_msgs)
  target_link_libraries(test_aggregated_fault nlohmann_json::nlohmann_json)

  # Real DDS endpoints read through the live graph API. The two that ALSO register a fake
  # /fault_manager/report_fault service share a RESOURCE_LOCK, because they spin real nodes for
  # many seconds and running them concurrently on this box starves the discovery each one waits
  # on - a domain of their own does not help with that.
  medkit_add_gtest(test_qos_mismatch_integration
    test/test_qos_mismatch_integration.cpp
    src/detectors/qos_mismatch_detector.cpp
    src/detector_registry.cpp
    src/reliability_gate.cpp
    src/lifecycle_watcher.cpp
    ${LIFECYCLE_STATE_READER_SOURCES})
  target_include_directories(test_qos_mismatch_integration PRIVATE include)
  medkit_target_dependencies(test_qos_mismatch_integration
    ros2_medkit_gateway rclcpp ros2_medkit_msgs lifecycle_msgs std_msgs)
  target_link_libraries(test_qos_mismatch_integration nlohmann_json::nlohmann_json)

  medkit_add_gtest(test_orphan_integration
    test/test_orphan_integration.cpp
    src/detectors/orphan_detector.cpp
    src/detector_registry.cpp
    src/reliability_gate.cpp
    src/lifecycle_watcher.cpp
    ${LIFECYCLE_STATE_READER_SOURCES})
  target_include_directories(test_orphan_integration PRIVATE include)
  medkit_target_dependencies(test_orphan_integration
    ros2_medkit_gateway rclcpp ros2_medkit_msgs lifecycle_msgs std_msgs sensor_msgs)
  target_link_libraries(test_orphan_integration nlohmann_json::nlohmann_json)
  set_tests_properties(test_orphan_integration PROPERTIES TIMEOUT 120)
  set_property(TEST test_orphan_integration APPEND PROPERTY
    RESOURCE_LOCK graph_watchdog_integration_domain)

  medkit_add_gtest(test_param_drift_integration
    test/test_param_drift_integration.cpp
    src/detectors/param_drift_detector.cpp
    src/detector_registry.cpp
    src/reliability_gate.cpp
    src/lifecycle_watcher.cpp
    ${LIFECYCLE_STATE_READER_SOURCES}
    ${PARAMETER_TRANSPORT_SOURCES})
  target_include_directories(test_param_drift_integration PRIVATE include ${GATEWAY_SRC_INCLUDE_DIR})
  medkit_target_dependencies(test_param_drift_integration
    ros2_medkit_gateway rclcpp rcl_interfaces rcutils ros2_medkit_msgs lifecycle_msgs)
  target_link_libraries(test_param_drift_integration nlohmann_json::nlohmann_json)
  # 300 s, not 120: several cases here are paced off the detector's own attempt and tick horizons
  # (sixty failed reads, sixty unarmed ticks, a read that answers past the per-read bound) and each
  # of those costs real seconds that no budget knob can compress away. A timeout the suite can
  # outgrow kills the process before gtest prints which case failed, which is the one thing a
  # timeout must never do to a diagnostic suite.
  set_tests_properties(test_param_drift_integration PROPERTIES TIMEOUT 300)
  set_property(TEST test_param_drift_integration APPEND PROPERTY
    RESOURCE_LOCK graph_watchdog_integration_domain)

  # === QoS-mismatch e2e (launch_testing) ===
  #
  # Acceptance gate: proves GRAPH_QOS_MISMATCH raises and clears through the
  # REAL gateway + qos_mismatch_detector + fault_manager stack, not just against a
  # fake ReportFault service (test_qos_mismatch_integration.cpp's job). See
  # test/e2e/test_qos_e2e.test.py for the full story.
  # The plugin .so path is injected straight from the build output via
  # GRAPH_WATCHDOG_PLUGIN_PATH (see harness.py's graph_watchdog_plugin_path()).
  find_package(launch_testing_ament_cmake REQUIRED)
  set(_WATCHDOG_E2E_SO "$<TARGET_FILE:ros2_medkit_graph_watchdog>")
  # The part of the environment every e2e launch below shares. Each launch takes a domain of
  # its own from the wrapper, so nothing about the domain appears here.
  set(_WATCHDOG_E2E_ENV "GRAPH_WATCHDOG_PLUGIN_PATH=${_WATCHDOG_E2E_SO}")

  medkit_add_launch_test(test_e2e_test_qos_e2e.test.py test/e2e/test_qos_e2e.test.py
    TIMEOUT 180)
  set_tests_properties(test_e2e_test_qos_e2e.test.py PROPERTIES LABELS "integration;e2e")
  # APPEND: medkit_add_launch_test put this test's domain count in ENVIRONMENT and a plain
  # PROPERTIES assignment would drop it, leaving the test on the default domain.
  set_property(TEST test_e2e_test_qos_e2e.test.py APPEND PROPERTY
    ENVIRONMENT "GATEWAY_TEST_PORT=19120;${_WATCHDOG_E2E_ENV}")

  medkit_add_launch_test(test_e2e_test_orphan_e2e.test.py test/e2e/test_orphan_e2e.test.py
    TIMEOUT 180)
  set_tests_properties(test_e2e_test_orphan_e2e.test.py PROPERTIES LABELS "integration;e2e")
  set_property(TEST test_e2e_test_orphan_e2e.test.py APPEND PROPERTY
    ENVIRONMENT "GATEWAY_TEST_PORT=19130;${_WATCHDOG_E2E_ENV}")

  # param_drift e2e: the detector's DEFAULT mode. The config-plumbing scenarios below all set
  # baseline: false and none of them asserts a clear, so without this the self-capturing behaviour
  # operators actually get, and the whole healing story the README documents, had no e2e at all.
  # 300 s, not 240: the file's own deadlines are now the arming gate (60), the silent-capture
  # window (15), the raise poll (60), the still-raised check (30) and the heal poll (60), and 240
  # no longer covers their sum. A budget shorter than the internal deadlines is killed by ctest
  # before any of the messages those deadlines exist to print reaches the log, which turns every
  # one of them into "timed out".
  medkit_add_launch_test(test_e2e_test_param_drift_e2e.test.py
    test/e2e/test_param_drift_e2e.test.py TIMEOUT 300)
  set_tests_properties(test_e2e_test_param_drift_e2e.test.py PROPERTIES LABELS "integration;e2e")
  set_property(TEST test_e2e_test_param_drift_e2e.test.py APPEND PROPERTY
    ENVIRONMENT "GATEWAY_TEST_PORT=19170;${_WATCHDOG_E2E_ENV}")

  # Round-trip cost e2e: what ONE parameter read really costs the node that serves it, counted
  # at a node that answers the parameter services by hand. param_drift's read budget is charged
  # in round trips (2 / 3 / 2), every other test in this package proves the arithmetic GIVEN
  # those numbers, and the stand-in transport they use counts CALLS - so nothing else here can
  # notice a round trip being added to or removed from Ros2ParameterTransport. Deliberately runs
  # with param_drift OFF: the reads are driven over the configurations endpoint so each count
  # belongs to exactly one transport call.
  # 300 s, not 180: this test's own retry deadlines (discovery, first contact against each of the
  # two probes) plus the single-shot measurements between them can legitimately add up to more
  # than 180 on a slow-discovery run - which is the run it exists to diagnose. A timeout shorter
  # than the internal budgets kills the process before any of those diagnostics print, so the
  # failure reads as "test timed out" and says nothing about round trips.
  medkit_add_launch_test(test_e2e_test_param_roundtrip_e2e.test.py
    test/e2e/test_param_roundtrip_e2e.test.py TIMEOUT 300)
  set_tests_properties(test_e2e_test_param_roundtrip_e2e.test.py PROPERTIES
    LABELS "integration;e2e")
  set_property(TEST test_e2e_test_param_roundtrip_e2e.test.py APPEND PROPERTY
    ENVIRONMENT "GATEWAY_TEST_PORT=19180;${_WATCHDOG_E2E_ENV}")

  # === Config-plumbing e2e (launch_testing) ===
  #
  # Proves nested per-detector config reaches a live detector through the REAL gateway
  # (extract_plugin_config -> detector_config.hpp -> ParamDriftConfig::from_json). No C++ test
  # above can give that proof: they all call configure() directly with hand-built JSON and
  # never touch the delivery path.
  #
  # One source file, three CTest targets. The plugin reads its config once, at set_context()
  # time, so proving the nested `expect` reader and the nested `mode` reader needs separate
  # gateway launches rather than separate test methods in one. WATCHDOG_E2E_SCENARIO selects
  # which launch and which assertions run in each process.
  medkit_add_launch_test(test_config_plumbing_e2e_positive
    test/e2e/test_config_plumbing_e2e.test.py TIMEOUT 180)
  set_tests_properties(test_config_plumbing_e2e_positive PROPERTIES LABELS "integration;e2e")
  set_property(TEST test_config_plumbing_e2e_positive APPEND PROPERTY
    ENVIRONMENT "GATEWAY_TEST_PORT=19140;WATCHDOG_E2E_SCENARIO=positive;${_WATCHDOG_E2E_ENV}")

  medkit_add_launch_test(test_config_plumbing_e2e_mode_off
    test/e2e/test_config_plumbing_e2e.test.py TIMEOUT 180)
  set_tests_properties(test_config_plumbing_e2e_mode_off PROPERTIES LABELS "integration;e2e")
  set_property(TEST test_config_plumbing_e2e_mode_off APPEND PROPERTY
    ENVIRONMENT "GATEWAY_TEST_PORT=19150;WATCHDOG_E2E_SCENARIO=mode_off;${_WATCHDOG_E2E_ENV}")

  # Third launch: `mode: off` written BARE, which the ROS parser types as a YAML 1.1 boolean
  # rather than a string. That is the form operators write, and a string-only reader silently
  # keeps the detector raising.
  medkit_add_launch_test(test_config_plumbing_e2e_mode_off_yaml_bool
    test/e2e/test_config_plumbing_e2e.test.py TIMEOUT 180)
  set_tests_properties(test_config_plumbing_e2e_mode_off_yaml_bool PROPERTIES
    LABELS "integration;e2e")
  set_property(TEST test_config_plumbing_e2e_mode_off_yaml_bool APPEND PROPERTY
    ENVIRONMENT
      "GATEWAY_TEST_PORT=19160;WATCHDOG_E2E_SCENARIO=mode_off_yaml_bool;${_WATCHDOG_E2E_ENV}")

  ros2_medkit_relax_vendor_warnings()

endif()

ament_package()
