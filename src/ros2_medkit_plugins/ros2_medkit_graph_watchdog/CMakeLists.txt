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
    src/detectors/node_death_detector.cpp
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

  # Pure matching logic over hand-built inputs: no rclcpp::Node anywhere in it. Depends on
  # ros2_medkit_gateway/nlohmann_json only because it assembles a description through the
  # real AggregatedFault::describe (R13/C1/C3's cap-filling test), the same reason
  # test_aggregated_fault below needs it - not because anything here builds a live graph.
  medkit_add_gtest(test_lifecycle_expectation_tracker test/test_lifecycle_expectation_tracker.cpp)
  target_include_directories(test_lifecycle_expectation_tracker PRIVATE include)
  medkit_target_dependencies(test_lifecycle_expectation_tracker ros2_medkit_gateway rclcpp ros2_medkit_msgs)
  target_link_libraries(test_lifecycle_expectation_tracker nlohmann_json::nlohmann_json)

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

  # Pure interface/logic: DetectorContext is only ever default-constructed (every pointer
  # null), so no rclcpp::init() is needed anywhere in this one.
  medkit_add_gtest(test_suppressor test/test_suppressor.cpp)
  target_include_directories(test_suppressor PRIVATE include)
  medkit_target_dependencies(test_suppressor rclcpp ros2_medkit_msgs)
  target_link_libraries(test_suppressor nlohmann_json::nlohmann_json)

  # AllowlistSuppressor::suppresses() ignores ctx entirely - same "no rclcpp::init()" shape
  # as test_suppressor above.
  medkit_add_gtest(test_allowlist_suppressor test/test_allowlist_suppressor.cpp)
  target_include_directories(test_allowlist_suppressor PRIVATE include)
  medkit_target_dependencies(test_allowlist_suppressor rclcpp ros2_medkit_msgs)
  target_link_libraries(test_allowlist_suppressor nlohmann_json::nlohmann_json)

  # Reads a REAL ReliabilityGate (needs a real rclcpp::Node for its LifecycleWatcher),
  # driven through set_departed_lifecycle_state_for_test() rather than a live managed node.
  medkit_add_gtest(test_lifecycle_shutdown_suppressor
    test/test_lifecycle_shutdown_suppressor.cpp
    src/reliability_gate.cpp
    src/lifecycle_watcher.cpp
    ${LIFECYCLE_STATE_READER_SOURCES})
  target_include_directories(test_lifecycle_shutdown_suppressor PRIVATE include ${GATEWAY_SRC_INCLUDE_DIR})
  medkit_target_dependencies(test_lifecycle_shutdown_suppressor ros2_medkit_gateway rclcpp ros2_medkit_msgs lifecycle_msgs)
  target_link_libraries(test_lifecycle_shutdown_suppressor nlohmann_json::nlohmann_json)

  # Pure logic over hand-built present/armed sets: no rclcpp::Node anywhere in it. Depends
  # on ros2_medkit_gateway/nlohmann_json only because the D1 case assembles a description
  # through the real AggregatedFault::describe_ordered(), the same reason
  # test_lifecycle_expectation_tracker above needs it.
  medkit_add_gtest(test_node_liveness_tracker test/test_node_liveness_tracker.cpp)
  target_include_directories(test_node_liveness_tracker PRIVATE include)
  medkit_target_dependencies(test_node_liveness_tracker ros2_medkit_gateway rclcpp ros2_medkit_msgs)
  target_link_libraries(test_node_liveness_tracker nlohmann_json::nlohmann_json)

  # Registers a fake /fault_manager/report_fault service like the QoS/orphan/param_drift/
  # lifecycle_expectation integration tests above, so it shares their RESOURCE_LOCK.
  medkit_add_gtest(test_node_death_integration
    test/test_node_death_integration.cpp
    src/detectors/node_death_detector.cpp
    src/detector_registry.cpp
    src/reliability_gate.cpp
    src/lifecycle_watcher.cpp
    ${LIFECYCLE_STATE_READER_SOURCES})
  target_include_directories(test_node_death_integration PRIVATE include ${GATEWAY_SRC_INCLUDE_DIR})
  medkit_target_dependencies(test_node_death_integration
    ros2_medkit_gateway rclcpp rcutils ros2_medkit_msgs lifecycle_msgs)
  target_link_libraries(test_node_death_integration nlohmann_json::nlohmann_json)
  # N11's churn sweep and the two retention-boundary cases each drive dozens of ticks; none
  # needs real DDS endpoint discovery (every snapshot is hand-built), so this is generous
  # against CI slowness rather than against any real per-tick cost.
  set_tests_properties(test_node_death_integration PROPERTIES TIMEOUT 120)
  set_property(TEST test_node_death_integration APPEND PROPERTY
    RESOURCE_LOCK graph_watchdog_integration_domain)

  # Real DDS endpoints read through the live graph API. The three that ALSO register a fake
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

  # Registers a fake /fault_manager/report_fault service like orphan/param_drift above, so it
  # shares the same RESOURCE_LOCK (see the comment on the integration block).
  medkit_add_gtest(test_lifecycle_expectation_integration
    test/test_lifecycle_expectation_integration.cpp
    src/detectors/lifecycle_expectation_detector.cpp
    src/detector_registry.cpp
    src/reliability_gate.cpp
    src/lifecycle_watcher.cpp
    ${LIFECYCLE_STATE_READER_SOURCES})
  target_include_directories(test_lifecycle_expectation_integration PRIVATE include ${GATEWAY_SRC_INCLUDE_DIR})
  medkit_target_dependencies(test_lifecycle_expectation_integration
    ros2_medkit_gateway rclcpp rcutils ros2_medkit_msgs lifecycle_msgs)
  target_link_libraries(test_lifecycle_expectation_integration nlohmann_json::nlohmann_json)
  set_tests_properties(test_lifecycle_expectation_integration PROPERTIES TIMEOUT 120)
  set_property(TEST test_lifecycle_expectation_integration APPEND PROPERTY
    RESOURCE_LOCK graph_watchdog_integration_domain)

  # Fixture node for the lifecycle_expectation e2e scenarios that need a node which looks
  # managed, answers a chosen lifecycle label, and can stop advertising its lifecycle
  # services on command - see test/e2e/droppable_lifecycle_node.cpp's file doc. It lives
  # here rather than with the shared demo nodes because it exists purely for this
  # package's own detector claims. Installed into lib/${PROJECT_NAME} so
  # launch_ros.actions.Node(package=..., executable=...) can resolve it.
  add_executable(droppable_lifecycle_node test/e2e/droppable_lifecycle_node.cpp)
  medkit_target_dependencies(droppable_lifecycle_node rclcpp lifecycle_msgs)
  install(TARGETS droppable_lifecycle_node RUNTIME DESTINATION lib/${PROJECT_NAME})

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
    TIMEOUT 180
    LABELS "integration;e2e"
    ENV "GATEWAY_TEST_PORT=19120" "${_WATCHDOG_E2E_ENV}")

  medkit_add_launch_test(test_e2e_test_orphan_e2e.test.py test/e2e/test_orphan_e2e.test.py
    TIMEOUT 180
    LABELS "integration;e2e"
    ENV "GATEWAY_TEST_PORT=19130" "${_WATCHDOG_E2E_ENV}")

  # param_drift e2e: the detector's DEFAULT mode. The config-plumbing scenarios below all set
  # baseline: false and none of them asserts a clear, so without this the self-capturing behaviour
  # operators actually get, and the whole healing story the README documents, had no e2e at all.
  # 300 s, not 240: the file's own deadlines are now the arming gate (60), the silent-capture
  # window (15), the raise poll (60), the still-raised check (30) and the heal poll (60), and 240
  # no longer covers their sum. A budget shorter than the internal deadlines is killed by ctest
  # before any of the messages those deadlines exist to print reaches the log, which turns every
  # one of them into "timed out".
  medkit_add_launch_test(test_e2e_test_param_drift_e2e.test.py
    test/e2e/test_param_drift_e2e.test.py TIMEOUT 300
    LABELS "integration;e2e"
    ENV "GATEWAY_TEST_PORT=19170" "${_WATCHDOG_E2E_ENV}")

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
    test/e2e/test_param_roundtrip_e2e.test.py TIMEOUT 300
    LABELS "integration;e2e"
    ENV "GATEWAY_TEST_PORT=19180" "${_WATCHDOG_E2E_ENV}")

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
    test/e2e/test_config_plumbing_e2e.test.py TIMEOUT 180
    LABELS "integration;e2e"
    ENV "GATEWAY_TEST_PORT=19140" "WATCHDOG_E2E_SCENARIO=positive" "${_WATCHDOG_E2E_ENV}")

  medkit_add_launch_test(test_config_plumbing_e2e_mode_off
    test/e2e/test_config_plumbing_e2e.test.py TIMEOUT 180
    LABELS "integration;e2e"
    ENV "GATEWAY_TEST_PORT=19150" "WATCHDOG_E2E_SCENARIO=mode_off" "${_WATCHDOG_E2E_ENV}")

  # Third launch: `mode: off` written BARE, which the ROS parser types as a YAML 1.1 boolean
  # rather than a string. That is the form operators write, and a string-only reader silently
  # keeps the detector raising.
  medkit_add_launch_test(test_config_plumbing_e2e_mode_off_yaml_bool
    test/e2e/test_config_plumbing_e2e.test.py TIMEOUT 180
    LABELS "integration;e2e"
    ENV "GATEWAY_TEST_PORT=19160" "WATCHDOG_E2E_SCENARIO=mode_off_yaml_bool" "${_WATCHDOG_E2E_ENV}")

  # Real-cmake-configure proof of the launch-test guard for this package -
  # this package's own half of the pair described in
  # ros2_medkit_integration_tests/test/test_launch_test_guard_system_build.py's
  # module docstring. Registers unconditionally: it configures only this
  # package's own source directory, present in every build context including
  # the ROS build farm's binarydeb chroot, unlike the sibling test's need for
  # a repository root above it. ROS-free: it spawns its own cmake
  # subprocesses rather than creating ROS entities.
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_launch_test_guard_system_build
    test/test_launch_test_guard_system_build.py TIMEOUT 180)
  set_tests_properties(test_launch_test_guard_system_build PROPERTIES LABELS "integration")
  medkit_test_needs_no_domain(test_launch_test_guard_system_build)

  # === Lifecycle-expectation e2e (launch_testing) ===
  #
  # Acceptance gate for the lifecycle_expectation detector: proves GRAPH_NODE_INACTIVE
  # raises and heals through the REAL gateway + plugin .so + fault_manager stack against
  # a REAL managed rclcpp_lifecycle demo node driven through real ChangeState
  # transitions - the C++ integration test can only puppet the lifecycle label.
  #
  # One source file, eight CTest targets (the config_plumbing pattern): the plugin reads
  # its config once at set_context() time, so each distinct config/fixture combination
  # (main, unconfigured-by-default, negative-control, unreadable, healing_threshold,
  # departure_keeps, not_managed, restart_loop) needs its own gateway process.
  # WATCHDOG_E2E_SCENARIO selects which launch and which assertions run in each.
  #
  # 780 s. The budget is never shorter than the sum of the file's own internal deadlines
  # plus the teardown the launch itself configures, because a budget under that sum is
  # killed by ctest before the message a failing deadline exists to print reaches the log -
  # turning every one of those diagnostics into a bare "timed out with no output". The sum,
  # per test case:
  #   test_01: arming gate (60) + raise poll (60) + entity-scoped poll (30)          = 150
  #   test_02: CONFIGURE (30 service wait + 30 across the call attempts + 10 for the
  #            already-applied check a retry can need) + still-raised poll (30)       = 100
  #   test_02b: pre-restart record (30) + port-down wait (60) + post-restart arming
  #             gate (90) + heal-sequence settle (4) + post record (30) + poll (30)   = 244
  #   test_03: heal precondition (30) + ACTIVATE (30 + 30 + 10) + per-entity gate (60)
  #            + heal poll (60)                                                       = 220
  # 714 in total, and the run that exhausts it is the slow-but-PASSING run - every poll
  # succeeding near its deadline. On top of that, teardown runs inside the same CTest
  # budget: every launched process carries sigterm_timeout 30 + sigkill_timeout 15
  # (launch_helpers.py), so a process that sits out SIGTERM costs up to ~45 s AFTER the last
  # assertion. 714 + 45 = 759, and a passing run that reached the teardown would otherwise
  # be flipped into a timeout with no output.
  medkit_add_launch_test(test_lifecycle_expectation_e2e_main
    test/e2e/test_lifecycle_expectation_e2e.test.py TIMEOUT 780
    LABELS "integration;e2e"
    ENV "GATEWAY_TEST_PORT=19190" "WATCHDOG_E2E_SCENARIO=main" "${_WATCHDOG_E2E_ENV}")

  # Shipped-default silence: no detectors.lifecycle_expectation config at all, with the
  # unconfigured lifecycle node present - the README's zero-false-positive default claim,
  # falsifiable nowhere else. 240 s: arming gate (60) + fault-surface gate (30) +
  # label-read poll (30) + sustained silence window (20) + the post-window re-pin of the
  # trigger (15) = 155 internal, plus the launch's own teardown (sigterm 30 + sigkill 15).
  medkit_add_launch_test(test_lifecycle_expectation_e2e_default_config
    test/e2e/test_lifecycle_expectation_e2e.test.py TIMEOUT 240
    LABELS "integration;e2e"
    ENV "GATEWAY_TEST_PORT=19200" "WATCHDOG_E2E_SCENARIO=default_config" "${_WATCHDOG_E2E_ENV}")

  # Negative control: the same grace and cadence as the main scenario, with require_active
  # naming the self-activating variant of the same executable instead - an active required
  # node must never raise, so the discriminating variable is the node's real lifecycle
  # state. 240 s: per-entity arming gate (60) + fault-surface gate (30) + label-read poll
  # (30) + sustained silence window (20) + the post-window re-pin (15) = 155 internal, plus
  # the launch's own teardown (sigterm 30 + sigkill 15).
  medkit_add_launch_test(test_lifecycle_expectation_e2e_negative_control
    test/e2e/test_lifecycle_expectation_e2e.test.py TIMEOUT 240
    LABELS "integration;e2e"
    ENV "GATEWAY_TEST_PORT=19210" "WATCHDOG_E2E_SCENARIO=negative_control" "${_WATCHDOG_E2E_ENV}")

  # Proof that GRAPH_NODE_UNREADABLE itself raises and clears against a REAL, sustained
  # GetState failure - launches unreadable_lifecycle_node.cpp (advertises get_state /
  # change_state like a managed lifecycle node, but get_state never answers until told
  # to) instead of the real rclcpp_lifecycle node every other scenario here uses. Needs
  # its own, much faster tick cadence: kUnmeasuredHoldTicks (60, fixed - not
  # configurable) is far more ticks than any `grace` window above ever waits out.
  # 450 s: arming gate (60) + faults-live gate (30) + unread-label poll (30) = 120
  # (test_01); silence poll (10) + end-of-window label poll (15) = 25 (test_02); raise
  # poll (60, see UNREADABLE_RAISE_TIMEOUT_SEC's own arithmetic comment in the .test.py
  # for why 60 s covers an expected ~8 s of tick-driven work) + INACTIVE re-check poll
  # (3) + entity-scoped poll (30) = 93 (test_03); heal-precondition poll (30) +
  # SetParameters call (30) + active-label poll (30) + heal poll (30) = 120 (test_04) -
  # 358 internal, plus the launch's own teardown (sigterm 30 + sigkill 15).
  medkit_add_launch_test(test_lifecycle_expectation_e2e_unreadable
    test/e2e/test_lifecycle_expectation_e2e.test.py TIMEOUT 450
    LABELS "integration;e2e"
    ENV "GATEWAY_TEST_PORT=19230" "WATCHDOG_E2E_SCENARIO=unreadable" "${_WATCHDOG_E2E_ENV}")

  # The withheld-clear guard's pending leg, at the tier that runs the REAL fault_manager
  # debounce state machine: a required node already reported stuck is SIGTERM'd and
  # respawned twice (a real snapshot blink, not a lifecycle transition), each blink well
  # inside the tracker's absence grace, against a sensitive healing_threshold of 1 - the
  # value this package's own README recommends. 360 s: arming gate (60) + label poll (30)
  # + raise poll (60) + two blinks (SIGTERM + up to 15 s rediscovery poll each, ~35 total)
  # + settle (3) + post-blink record poll (30) + final label poll (15) = 233 internal,
  # plus the launch's own teardown for TWO processes (sigterm 30 + sigkill 15 each, not
  # fully serial) - the same generous margin the other scenarios budget.
  medkit_add_launch_test(test_lifecycle_expectation_e2e_healing_threshold
    test/e2e/test_lifecycle_expectation_e2e.test.py TIMEOUT 360
    LABELS "integration;e2e"
    ENV "GATEWAY_TEST_PORT=19220" "WATCHDOG_E2E_SCENARIO=healing_threshold" "${_WATCHDOG_E2E_ENV}")

  # A sibling to the "unreadable" scenario above, not an extension of it: it needs a
  # PERMANENT kill (no respawn) of the same fixture process, which cannot share a
  # gateway process with test_04's start_answering-driven clear without racing it -
  # see TestLifecycleExpectationDepartureKeeps's own docstring. Proves that an
  # already-reported node LEAVING the graph does not heal its fault: a lifecycle
  # promise that was never verified is no more verified once the node is gone.
  # Previously proven only by the integration tier's
  # UnreadableNodeAlreadyReportedThatVanishesKeepsItsOwnRecord.
  # 420 s: arming gate (60) + faults-live gate (30) + unread-label poll (30) = 120
  # (test_01); raise poll (60, same UNREADABLE_RAISE_TIMEOUT_SEC arithmetic as the
  # "unreadable" scenario above) + mutual-exclusion window (3) = 63 (test_02); departure
  # poll (30) + settle (10) + record poll (30) + still-active poll (30) + description
  # poll (10) + mutual-exclusion window (3) = 113 (test_03) - 296 internal, plus the
  # launch's own teardown for TWO processes (sigterm 30 + sigkill 15 each, not fully
  # serial) - the same generous margin healing_threshold budgets for the same launch shape.
  medkit_add_launch_test(test_lifecycle_expectation_e2e_departure_keeps
    test/e2e/test_lifecycle_expectation_e2e.test.py TIMEOUT 420
    LABELS "integration;e2e"
    ENV "GATEWAY_TEST_PORT=19240" "WATCHDOG_E2E_SCENARIO=departure_keeps" "${_WATCHDOG_E2E_ENV}")

  # The not-managed sibling of "unreadable"/"departure_keeps": proves GRAPH_NODE_NOT_MANAGED's
  # own raise (naming the node, WARN severity, mutually exclusive with the other two codes)
  # and the same evidence retention across a departure, against a PLAIN demo node with no
  # lifecycle interface at all (`calibration`) rather than a purpose-built fixture - none
  # was needed for this cause. 300 s: arming gate (60) + faults-live gate (30) + presence
  # poll (5) = 95 (test_01); raise poll (60, same NOT_MANAGED_RAISE_TIMEOUT_SEC arithmetic
  # as the "unreadable" scenario's UNREADABLE_RAISE_TIMEOUT_SEC - if anything cheaper, since
  # there is no blocking GetState round trip to pay for) + two mutual-exclusion windows (6) +
  # entity-scoped poll (30) = 96 (test_02); raise precondition (60) + departure poll (30) +
  # settle (10) + record poll (30) + still-active poll (30) + description poll (10) +
  # mutual-exclusion window (3) = 173 (test_03) - 364 internal is already over 300, so this
  # budgets from the same internal-deadline-sum discipline as the sibling scenarios but
  # rounds the target up to 480 s to also cover the launch's own teardown for TWO processes
  # (sigterm 30 + sigkill 15 each, not fully serial).
  medkit_add_launch_test(test_lifecycle_expectation_e2e_not_managed
    test/e2e/test_lifecycle_expectation_e2e.test.py TIMEOUT 480
    LABELS "integration;e2e"
    ENV "GATEWAY_TEST_PORT=19250" "WATCHDOG_E2E_SCENARIO=not_managed" "${_WATCHDOG_E2E_ENV}")

  # The scenario the evidence-retention model exists for: a required node in a CRASH LOOP,
  # killed and respawned on a cadence that never lets it accumulate the 60 consecutive
  # PRESENT ticks kUnmeasuredHoldTicks needs. A node that keeps dying touches absence by
  # construction, so a detector that discarded its clock there would stay silent forever -
  # which is exactly the node this detector most exists to catch. Uses the same plain
  # `calibration` fixture as "not_managed", with launch's own respawn.
  # 300 s: arming gate (60) + faults-live gate (30) + presence poll (30) = 120; the kill
  # loop's own wall-clock budget (45, RESTART_LOOP_WINDOW_SEC) plus the per-cycle
  # departure/return polls it contains (30 + 30 on the worst single cycle) = 105 - 225
  # internal, plus the launch's own teardown for TWO processes (sigterm 30 + sigkill 15
  # each, not fully serial).
  medkit_add_launch_test(test_lifecycle_expectation_e2e_restart_loop
    test/e2e/test_lifecycle_expectation_e2e.test.py TIMEOUT 300
    LABELS "integration;e2e"
    ENV "GATEWAY_TEST_PORT=19260" "WATCHDOG_E2E_SCENARIO=restart_loop" "${_WATCHDOG_E2E_ENV}")

  # A FULL tracked-node cap: two required nodes and `tracked_node_cap: 1`, so one of them is
  # refused on every tick. Reachable only because the cap is a config key. Proves the refusal
  # is visible on the status route, that it WITHHOLDS GRAPH_NODE_INACTIVE's clear, and that a
  # departed entry is collapsed so the present broken node is checked instead.
  # 420 s: arming gate (60) + fault-surface gate (30) + two presence polls (60) + label poll
  # (30) + raise poll (60) + saturation poll (30) = 270 (test_01); record poll (30) +
  # SetParameters (30) + not-managed raise poll (60) + saturation re-check (15) + withhold
  # window (10) + record poll (30) = 175 (test_02); departure poll (30) + presence poll (30)
  # + admission poll (60) + saturation poll (30) + two record polls (60) = 210 (test_03).
  # The sum is the SLOW-but-passing run, in which every poll succeeds near its deadline;
  # the expected run is a small fraction of it, and CI slowness is what the margin is for.
  # Rounded to 900 to also cover the launch's own teardown for THREE processes (sigterm 30 +
  # sigkill 15 each, not fully serial).
  medkit_add_launch_test(test_lifecycle_expectation_e2e_cap_pressure
    test/e2e/test_lifecycle_expectation_e2e.test.py TIMEOUT 900
    LABELS "integration;e2e"
    ENV "GATEWAY_TEST_PORT=19270" "WATCHDOG_E2E_SCENARIO=cap_pressure" "${_WATCHDOG_E2E_ENV}")

  # One missed sweep before a clean shutdown must raise nothing, while the same drop held
  # long enough to be corroborated must still be reported - two legs of one fixture whose
  # only difference is the hold, so the pair discriminates instead of merely proving silence.
  # Runs at a deliberately slow 500 ms cadence so the test can place its actions between
  # ticks rather than race them. 480 s: arming gate (60) + fault-surface gate (30) + two
  # label polls (60) = 150 (test_01); null-label poll (15) + departure poll (30) = 45
  # (test_02); null-label poll (15) + the deliberate settled hold (5) + departure poll (30) +
  # raise poll (90) = 140 (test_03); three record polls (15) = 15 (test_04) - 350 internal,
  # plus the launch's own teardown for THREE processes.
  medkit_add_launch_test(test_lifecycle_expectation_e2e_unsettled_departure
    test/e2e/test_lifecycle_expectation_e2e.test.py TIMEOUT 480
    LABELS "integration;e2e"
    ENV "GATEWAY_TEST_PORT=19280" "WATCHDOG_E2E_SCENARIO=unsettled_departure" "${_WATCHDOG_E2E_ENV}")

  # `grace` at the value that used to be the accepted maximum. Under it the detector can
  # neither raise nor heal GRAPH_NODE_INACTIVE for days, so the value must be refused and the
  # documented default applied - measured by the fault actually appearing.
  # 300 s: arming gate (60) + fault-surface gate (30) + label poll (30) + raise poll (60) =
  # 180 (test_01); departure poll (30) + description poll (30) + record poll (10) = 70
  # (test_02) - 250 internal, plus the launch's own teardown for TWO processes.
  medkit_add_launch_test(test_lifecycle_expectation_e2e_wide_grace
    test/e2e/test_lifecycle_expectation_e2e.test.py TIMEOUT 300
    LABELS "integration;e2e"
    ENV "GATEWAY_TEST_PORT=19290" "WATCHDOG_E2E_SCENARIO=wide_grace" "${_WATCHDOG_E2E_ENV}")

  # Records where "a departure never heals a fault" ends: it holds within one gateway
  # lifetime, and a restart re-baselines because the restarted detector cannot tell an entry
  # for a departed node from a misspelt one. Restarts the gateway by PID, like the "main"
  # scenario. 420 s: arming gate (60) + label poll (30) + raise poll (60) + departure poll
  # (30) + settle (4) + record poll (30) = 214 (test_01); port-down wait (60) + arming gate
  # (90) + heal poll (90) + record poll (30) = 270 (test_02) - 484 internal is already over
  # 420, so this budgets 600 and also covers the launch's own teardown for TWO processes.
  medkit_add_launch_test(test_lifecycle_expectation_e2e_restart_departed
    test/e2e/test_lifecycle_expectation_e2e.test.py TIMEOUT 600
    LABELS "integration;e2e"
    ENV "GATEWAY_TEST_PORT=19300" "WATCHDOG_E2E_SCENARIO=restart_departed" "${_WATCHDOG_E2E_ENV}")

  # === node_death e2e (launch_testing) ===
  #
  # Exercises the node_death detector end to end against a real gateway + fault_manager +
  # demo-node stack. See test/e2e/test_node_death_e2e.test.py's own module docstring for the
  # full scenario list and what each row proves.

  # 60 (arming gate, app_id-scoped - already implies presence, so this is not a separate
  # budget) + 30 (departure poll) + 60 (raise poll) + 30 (entity-scoped fault poll) = 180
  # internal, plus the two harness self-tests (a handful of seconds against a local
  # stand-in server, no ROS graph involved). Rounded to 300 to also cover the launch's own
  # teardown for TWO processes (sigterm 30 + sigkill 15 each, not fully serial).
  medkit_add_launch_test(test_node_death_e2e_raise
    test/e2e/test_node_death_e2e.test.py TIMEOUT 300
    LABELS "integration;e2e"
    ENV "GATEWAY_TEST_PORT=19310" "WATCHDOG_E2E_SCENARIO=raise" "${_WATCHDOG_E2E_ENV}")

  # 60 (arming, app_id-scoped - already implies presence) + 30 (departure) + 60 (raise
  # poll) + 42.5 (presence poll after the respawn: DEPARTURE_TIMEOUT_SEC + RESPAWN_DELAY_SEC)
  # + 60 (heal poll) = 252.5 internal. Rounded to 420 to also cover the launch's own teardown
  # for TWO processes.
  medkit_add_launch_test(test_node_death_e2e_clear_on_return
    test/e2e/test_node_death_e2e.test.py TIMEOUT 420
    LABELS "integration;e2e"
    ENV "GATEWAY_TEST_PORT=19320" "WATCHDOG_E2E_SCENARIO=clear_on_return" "${_WATCHDOG_E2E_ENV}")

  # Same drive as clear_on_return (60 arming, app_id-scoped + 30 faults-live + 30
  # departure + 60 raise poll + 42.5 presence-after-respawn = 222.5), plus the persistence
  # window itself (20, SUSTAINED_WINDOW_SEC) instead of a heal poll = 242.5 internal.
  # Rounded to 420 to also cover the launch's own teardown for TWO processes - same
  # budget as clear_on_return since the internal sum is close and both drive one
  # kill-then-return cycle.
  medkit_add_launch_test(test_node_death_e2e_no_heal_standalone
    test/e2e/test_node_death_e2e.test.py TIMEOUT 420
    LABELS "integration;e2e"
    ENV "GATEWAY_TEST_PORT=19330" "WATCHDOG_E2E_SCENARIO=no_heal_standalone" "${_WATCHDOG_E2E_ENV}")

  # 60 (arming) + 30 (faults-live) + 30 (label poll to "active") + 30 (the ChangeState
  # call itself) + 30 (separate label poll to "inactive") + 20 (silence window) + 15
  # (end-of-window label check) = 215 internal, 260 with the launch's own teardown for
  # TWO processes - three independent 30 s budgets, not one combined 30 s.
  medkit_add_launch_test(test_node_death_e2e_deactivated_not_dead
    test/e2e/test_node_death_e2e.test.py TIMEOUT 300
    LABELS "integration;e2e"
    ENV "GATEWAY_TEST_PORT=19340" "WATCHDOG_E2E_SCENARIO=deactivated_not_dead" "${_WATCHDOG_E2E_ENV}")

  # 60 (arming - proves the hybrid-mode manifest actually loaded rather than silently
  # degrading to runtime_only) + 30 (faults-live) + 30 (presence poll for the one online
  # app) + 30 (a SEPARATE presence poll for the never-online app's manifest-derived
  # snapshot entry) + 20 (silence window) = 170 internal, 215 with the launch's own
  # teardown for TWO processes - two independent 30 s budgets, not one.
  medkit_add_launch_test(test_node_death_e2e_manifest_never_online
    test/e2e/test_node_death_e2e.test.py TIMEOUT 240
    LABELS "integration;e2e"
    ENV "GATEWAY_TEST_PORT=19350" "WATCHDOG_E2E_SCENARIO=manifest_never_online" "${_WATCHDOG_E2E_ENV}")

  # test_01 (load-bearing): 60 (arming) + 30 (faults-live) + three renamed-node arm/kill
  # cycles (30 presence poll on GET /apps, visible only because this scenario's own
  # discovery.runtime.filter_internal_nodes is off + 60 app_id-scoped arming poll + 30
  # departure poll + 15 process-reap budget each = 135 worst case, *3 = 405) + 20 (final
  # silence window, SUSTAINED_WINDOW_SEC) = 515 internal.
  # test_02: a constant comparison against the installed ros2cli package - no I/O, no
  # polling, negligible budget.
  # Rounded to 660 to also cover the launch's own teardown for the ONE launch-managed
  # process (the gateway; this scenario declares no demo_nodes), plus however many
  # renamed `demo_engine_temp_sensor` processes test_01 spawns and reaps itself across
  # its three cycles (not launch-managed).
  medkit_add_launch_test(test_node_death_e2e_ros2cli_ignored
    test/e2e/test_node_death_e2e.test.py TIMEOUT 660
    LABELS "integration;e2e"
    ENV "GATEWAY_TEST_PORT=19360" "WATCHDOG_E2E_SCENARIO=ros2cli_ignored" "${_WATCHDOG_E2E_ENV}")

  # 120 (two app_id-scoped arming gates, 60 each - one per colliding node, each already
  # implying presence) + 30 (departure poll for the killed one) + 0 (the survivor's
  # liveness check is a single non-blocking signal-0 probe, not a poll) + 60 (raise poll)
  # + 5 (the describes-only window, MUTUAL_NAMING_WINDOW_SEC) = 215 internal. Rounded to
  # 360 to also cover the launch's own teardown for THREE processes.
  medkit_add_launch_test(test_node_death_e2e_bare_name_collision
    test/e2e/test_node_death_e2e.test.py TIMEOUT 360
    LABELS "integration;e2e"
    ENV "GATEWAY_TEST_PORT=19370" "WATCHDOG_E2E_SCENARIO=bare_name_collision" "${_WATCHDOG_E2E_ENV}")

  # 60 (arming) + 30 (faults-live) + 30 (presence poll) + 20 (silence window) + 5
  # (end-of-window presence re-check) = 145 internal. Rounded to 240 to also cover the
  # launch's own teardown for TWO processes.
  medkit_add_launch_test(test_node_death_e2e_fast_tick_floor
    test/e2e/test_node_death_e2e.test.py TIMEOUT 240
    LABELS "integration;e2e"
    ENV "GATEWAY_TEST_PORT=19380" "WATCHDOG_E2E_SCENARIO=fast_tick_floor" "${_WATCHDOG_E2E_ENV}")

  # 60 (arming, app_id-scoped - already implies presence) + THREE kill cycles (see
  # RESTART_LOOP_OCCURRENCES_TARGET's own comment for why three), two of which also
  # acknowledge and re-arm on the respawned instance (30 departure + 60 occurrence-count
  # poll + 42.5 app_id-scoped re-arm after the respawn [DEPARTURE_TIMEOUT_SEC +
  # RESPAWN_DELAY_SEC] = 132.5 each, *2 = 265) plus the third cycle's kill-and-confirm alone
  # (30 + 60 = 90) + 30 (final record poll) = 445 internal. Rounded to 900, matching
  # cap_pressure's own budget for a similarly multi-cycle scenario, to also cover the
  # launch's own teardown for TWO processes plus CI-slowness margin.
  medkit_add_launch_test(test_node_death_e2e_restart_loop_occurrences
    test/e2e/test_node_death_e2e.test.py TIMEOUT 900
    LABELS "integration;e2e"
    ENV "GATEWAY_TEST_PORT=19390" "WATCHDOG_E2E_SCENARIO=restart_loop_occurrences" "${_WATCHDOG_E2E_ENV}")

  # 60 (arming, app_id-scoped - already implies presence) + 30 (departure) + 60 (raise
  # poll) + 30 (record poll) = 180 (test_01); 60 (port-down wait) + 90 (global arming gate
  # after the restart - TARGET_NODE is gone by design here, so this cannot be app_id-scoped
  # the way test_01's is) + 30 (faults-live, proving the restarted gateway reconnected to
  # the fault_manager before the persistence window below could mean anything) + 5
  # (not-present check) + 20 (persistence window) + 30 (record poll) = 235 (test_02) - 415
  # internal. Rounded to 600, the same budget as the sibling detector's restart_departed
  # scenario (an identically-shaped gateway-restart test), to also cover the launch's own
  # teardown for TWO processes.
  medkit_add_launch_test(test_node_death_e2e_restart_rebaseline
    test/e2e/test_node_death_e2e.test.py TIMEOUT 600
    LABELS "integration;e2e"
    ENV "GATEWAY_TEST_PORT=19400" "WATCHDOG_E2E_SCENARIO=restart_rebaseline" "${_WATCHDOG_E2E_ENV}")

  # === node_death suppression e2e (launch_testing) ===
  #
  # Exercises the node_death detector's suppression framework (allowlist, self-suppression,
  # pruning) against a real gateway + fault_manager + demo-node stack - see
  # test/e2e/test_node_death_suppression_e2e.test.py's own module docstring for the full
  # scenario list and what each row proves.

  # 60 (arming, app_id-scoped) + 30 (faults-live) + 30 (departure poll) + 20 (absence window,
  # SUSTAINED_WINDOW_SEC) = 140 internal, plus the harness self-test for assert_fault_never_names
  # (a handful of seconds against a local stand-in server, no ROS graph involved). Rounded to
  # 240 to also cover the launch's own teardown for TWO processes.
  medkit_add_launch_test(test_node_death_suppression_e2e_allowlist_suppresses
    test/e2e/test_node_death_suppression_e2e.test.py TIMEOUT 240
    LABELS "integration;e2e"
    ENV "GATEWAY_TEST_PORT=19410" "WATCHDOG_E2E_SCENARIO=allowlist_suppresses"
      "${_WATCHDOG_E2E_ENV}")

  # 60 (arming, app_id-scoped) + 30 (departure poll) + 60 (raise poll) + 30 (startup-warning
  # wait on the gateway's own stderr via proc_output, WARNING_TIMEOUT_SEC - unreachable today
  # since the raise poll above fails first, but written and counted on its own budget) = 180
  # internal. Rounded to 300 to also cover the launch's own teardown for TWO processes.
  medkit_add_launch_test(test_node_death_suppression_e2e_allowlist_not_named_is_inert
    test/e2e/test_node_death_suppression_e2e.test.py TIMEOUT 300
    LABELS "integration;e2e"
    ENV "GATEWAY_TEST_PORT=19420" "WATCHDOG_E2E_SCENARIO=allowlist_not_named_is_inert"
      "${_WATCHDOG_E2E_ENV}")

  # 60 (global arming) + 30 (faults-live) + 60 (watchdog-entity poll, CLEAN_NODE "unconfigured")
  # + 60 (watchdog-entity poll, KILLED_NODE "active") + 60 (the ChangeState call itself: up to
  # 30 waiting for the service plus another 30 for the response - _call_change_state_once pays
  # both, not one) + 30 (watchdog-entity poll, CLEAN_NODE "finalized") + 30 (departure poll,
  # CLEAN_NODE) + 30 (departure poll, KILLED_NODE) + 60 (raise poll) + 20 (describes-only window,
  # SUSTAINED_WINDOW_SEC) = 440 internal - ten independent budgets, not merged. Rounded to 720
  # to also cover the launch's own teardown for THREE processes (gateway plus both lifecycle
  # fixtures).
  medkit_add_launch_test(test_node_death_suppression_e2e_lifecycle_clean_shutdown
    test/e2e/test_node_death_suppression_e2e.test.py TIMEOUT 720
    LABELS "integration;e2e"
    ENV "GATEWAY_TEST_PORT=19430" "WATCHDOG_E2E_SCENARIO=lifecycle_clean_shutdown"
      "${_WATCHDOG_E2E_ENV}")

  # 60 (arming, app_id-scoped) + 30 (departure poll) + 60 (raise poll) = 150 internal. Rounded
  # to 240 to also cover the launch's own teardown for TWO processes.
  medkit_add_launch_test(test_node_death_suppression_e2e_suppression_is_opt_in
    test/e2e/test_node_death_suppression_e2e.test.py TIMEOUT 240
    LABELS "integration;e2e"
    ENV "GATEWAY_TEST_PORT=19440" "WATCHDOG_E2E_SCENARIO=suppression_is_opt_in"
      "${_WATCHDOG_E2E_ENV}")

  # Two independent nodes (TARGET_NODE, allowlisted; SECOND_NODE, not) - see the module
  # docstring's own note on why a single unsuppressed node cannot exercise pruning at all.
  # 60+60 (arming, app_id-scoped, one per node) + 30 (faults-live) + 40 (baseline settle,
  # STABLE_TRACKED_COUNT_TIMEOUT_SEC - node_death's own tick has to catch up to the gate
  # before the reclaim delta below means anything, see _poll_stable_tracked_count) + 30+30
  # (departure poll, one per node) + 60 (raise poll, SECOND_NODE) + 60 (reclaim poll on
  # detectors.node_death.tracked_count via GET /x-medkit-watchdog - PRUNE_GRACE ticks elapse
  # comfortably inside it) + 20 (describes-only window, SUSTAINED_WINDOW_SEC) = 390 internal.
  # Rounded to 600 to also cover the launch's own teardown for THREE processes.
  medkit_add_launch_test(test_node_death_suppression_e2e_prune_no_false_heal
    test/e2e/test_node_death_suppression_e2e.test.py TIMEOUT 600
    LABELS "integration;e2e"
    ENV "GATEWAY_TEST_PORT=19450" "WATCHDOG_E2E_SCENARIO=prune_no_false_heal"
      "${_WATCHDOG_E2E_ENV}")

  # === node_death boundary, config and instrument e2e (launch_testing) ===
  #
  # Exercises the seam between node_death and lifecycle_expectation, plus config/instrument
  # boundary cases, against a real gateway + fault_manager + demo-node stack - see
  # test/e2e/test_node_death_boundary_e2e.test.py's own module docstring for the full scenario
  # list and which rows (B1, B3) are satisfied by lifecycle_expectation alone.

  # 60 (global arming) + 30 (faults-live) + 30 (presence poll) + 60 (raise poll, INACTIVE) + 20
  # (absence window, DISAPPEARED, SUSTAINED_WINDOW_SEC) + 5 (end-of-window presence recheck) =
  # 205 internal. Rounded to 300 to also cover the launch's own teardown for TWO processes.
  medkit_add_launch_test(test_node_death_boundary_e2e_b1_inactive_present
    test/e2e/test_node_death_boundary_e2e.test.py TIMEOUT 300
    LABELS "integration;e2e"
    ENV "GATEWAY_TEST_PORT=19460" "WATCHDOG_E2E_SCENARIO=b1_inactive_present"
      "${_WATCHDOG_E2E_ENV}")

  # 60 (global arming) + 30 (faults-live) + 60 (tracked_nodes==1 poll) + 30 (departure poll) +
  # 20 (absence window, INACTIVE, SUSTAINED_WINDOW_SEC) + 60 (raise poll, DISAPPEARED -
  # unreachable today since the absence window above fails first, but written and counted on
  # its own budget) = 260 internal. Rounded to 360 to also cover the launch's own teardown for
  # TWO processes.
  medkit_add_launch_test(test_node_death_boundary_e2e_b2_inactive_below_grace_then_gone
    test/e2e/test_node_death_boundary_e2e.test.py TIMEOUT 360
    LABELS "integration;e2e"
    ENV "GATEWAY_TEST_PORT=19470" "WATCHDOG_E2E_SCENARIO=b2_inactive_below_grace_then_gone"
      "${_WATCHDOG_E2E_ENV}")

  # 60 (global arming) + 30 (faults-live) + 60 (raise poll, INACTIVE) + 30 (departure poll) + 20
  # (persistence window, INACTIVE, SUSTAINED_WINDOW_SEC) + 60 (raise poll, DISAPPEARED) = 260
  # internal. Rounded to 360 to also cover the launch's own teardown for TWO processes.
  medkit_add_launch_test(test_node_death_boundary_e2e_b3_matured_then_gone
    test/e2e/test_node_death_boundary_e2e.test.py TIMEOUT 360
    LABELS "integration;e2e"
    ENV "GATEWAY_TEST_PORT=19480" "WATCHDOG_E2E_SCENARIO=b3_matured_then_gone"
      "${_WATCHDOG_E2E_ENV}")

  # 60 (arming, app_id-scoped) + 30 (faults-live) + 30 (departure poll) + 20 (absence window,
  # INACTIVE, SUSTAINED_WINDOW_SEC) + 60 (raise poll, DISAPPEARED) = 200 internal. Rounded to
  # 300 to also cover the launch's own teardown for TWO processes.
  medkit_add_launch_test(test_node_death_boundary_e2e_b4_healthy_then_gone
    test/e2e/test_node_death_boundary_e2e.test.py TIMEOUT 300
    LABELS "integration;e2e"
    ENV "GATEWAY_TEST_PORT=19490" "WATCHDOG_E2E_SCENARIO=b4_healthy_then_gone"
      "${_WATCHDOG_E2E_ENV}")

  # 60 (global arming) + 30 (faults-live) + 30 (presence poll) + THREE restart cycles (see
  # B5_CYCLES's own comment for why three), each (30 departure poll + 60 raise poll + 42.5
  # presence-after-respawn poll [DEPARTURE_TIMEOUT_SEC + B5_RESPAWN_DELAY_SEC] + 60 clear
  # poll = 192.5), *3 = 577.5, plus the 120 preamble = 697.5 internal - four independent
  # per-cycle budgets, not one merged figure, times three cycles. B5_RESPAWN_DELAY_SEC
  # (12.5 s) replaces the launch-wide RESPAWN_DELAY_SEC (1.5 s) for this scenario alone: the
  # outage has to stay safely above the detector's own wall-clock floor for every cycle to be
  # reportable by a correct implementation - see that constant's own comment for the margin
  # arithmetic. Rounded to 1500, well past the 900 this package's own restart-loop-shaped
  # scenarios already use for a smaller sum, to also cover the launch's own teardown for TWO
  # processes plus CI-slowness margin.
  medkit_add_launch_test(test_node_death_boundary_e2e_b5_restart_loop_still_caught
    test/e2e/test_node_death_boundary_e2e.test.py TIMEOUT 1500
    LABELS "integration;e2e"
    ENV "GATEWAY_TEST_PORT=19500" "WATCHDOG_E2E_SCENARIO=b5_restart_loop_still_caught"
      "${_WATCHDOG_E2E_ENV}")

  # 60 (global arming) + 30 (faults-live) + 30 (presence poll) + 60 (tracked_nodes==1 poll) +
  # 30 (departure poll) + 60 (raise poll, INACTIVE) + 20 (absence window, DISAPPEARED,
  # SUSTAINED_WINDOW_SEC) = 290 internal. Rounded to 360 to also cover the launch's own
  # teardown for TWO processes.
  medkit_add_launch_test(test_node_death_boundary_e2e_b6_never_armed_below_grace_then_gone
    test/e2e/test_node_death_boundary_e2e.test.py TIMEOUT 360
    LABELS "integration;e2e"
    ENV "GATEWAY_TEST_PORT=19530" "WATCHDOG_E2E_SCENARIO=b6_never_armed_below_grace_then_gone"
      "${_WATCHDOG_E2E_ENV}")

  # 60 (arming, app_id-scoped) + 30 (faults-live) + 30 (departure poll) + 25 (early silence
  # window, C4_EARLY_WINDOW_SEC, needle-scoped) + 100 (late raise poll,
  # C4_LATE_RAISE_TIMEOUT_SEC) = 245 internal - five independent budgets, not merged. One
  # gateway, one target, one domain: rounded to 360 to also cover the launch's own teardown for
  # TWO processes.
  medkit_add_launch_test(test_node_death_boundary_e2e_c4_config_endpoint_e2e
    test/e2e/test_node_death_boundary_e2e.test.py TIMEOUT 360
    LABELS "integration;e2e"
    ENV "GATEWAY_TEST_PORT=19510" "WATCHDOG_E2E_SCENARIO=c4_config_endpoint_e2e"
      "${_WATCHDOG_E2E_ENV}")

  # test_01: 60 (arming, app_id-scoped) + 30 (departure poll) + 60 (raise poll) + 30 (record
  # poll) = 180. test_02: 60 (port-down wait) + 30 (faults-live gate BEFORE the ungated window
  # opens - a restarted gateway's HTTP server is not up the instant the old port goes down,
  # confirmed live) + 3.5 (ungated pre-arm window, D2_UNGATED_WATCH_SEC) + 90 (global arming
  # after the restart - TARGET_NODE is gone by design here, so this cannot be app_id-scoped) +
  # 30 (faults-live, again, gating the persistence window below) + 5 (not-present check) + 20
  # (persistence window, SUSTAINED_WINDOW_SEC) + 30 (record poll) = 268.5 - 448.5 internal, nine
  # independent budgets across both test methods, not merged. Rounded to 660 to also cover the
  # launch's own teardown for TWO processes.
  medkit_add_launch_test(test_node_death_boundary_e2e_d2_ungated_clear
    test/e2e/test_node_death_boundary_e2e.test.py TIMEOUT 660
    LABELS "integration;e2e"
    ENV "GATEWAY_TEST_PORT=19520" "WATCHDOG_E2E_SCENARIO=d2_ungated_clear"
      "${_WATCHDOG_E2E_ENV}")

  # === presence-ownership e2e (launch_testing) ===
  #
  # Which detector owns the departure of a managed node whose lifecycle state was never
  # measured. Uses unreadable_lifecycle_node.cpp (advertises get_state / change_state like a
  # managed node, and never answers GetState until told to), so the unmeasured window is
  # permanent rather than a race between the GetState seed and the warmup - see
  # test/e2e/test_presence_ownership_e2e.test.py's own module docstring for the three rows.

  # 60 (arming, app_id-scoped) + 30 (faults-live) + 30 (empty-label poll) + 30 (departure poll)
  # + 60 (DISAPPEARED raise poll) + 90 (UNREADABLE raise poll, kUnmeasuredHoldTicks is 60 fixed
  # ticks plus the absence grace) = 300 internal - six independent budgets, not merged. Rounded
  # to 420 to also cover the launch's own teardown for TWO processes (sigterm 30 + sigkill 15
  # each, not fully serial).
  medkit_add_launch_test(test_presence_ownership_e2e_budget_spent_then_owned
    test/e2e/test_presence_ownership_e2e.test.py TIMEOUT 420
    LABELS "integration;e2e"
    ENV "GATEWAY_TEST_PORT=19540" "WATCHDOG_E2E_SCENARIO=budget_spent_then_owned"
      "${_WATCHDOG_E2E_ENV}")

  # A crash loop whose uptime is shorter than the warmup: killed, reported, back for less than
  # one re-warm, killed again. The other restart rows in this package all wait for the returned
  # node to arm before killing it, so none of them can see a key handed back while it is merely
  # re-warming. 60 (arming, app_id-scoped, at RESTART_WARMUP_CYCLES) + 30 (faults-live) + 30
  # (first departure poll) + 60 (first raise poll) + 60 (return poll,
  # RESTART_RESPAWN_DELAY_SEC + 30) + 30 (un-armed state poll) + 30 (record poll) + 30 (second
  # departure poll) + 60 (second raise poll) + 10 (no-PASSED window, SILENCE_WINDOW_SEC) = 400
  # internal - ten independent budgets, not merged. Rounded to 540 to also cover the launch's
  # own teardown for TWO processes (sigterm 30 + sigkill 15 each, not fully serial).
  medkit_add_launch_test(test_presence_ownership_e2e_restart_inside_warmup
    test/e2e/test_presence_ownership_e2e.test.py TIMEOUT 540
    LABELS "integration;e2e"
    ENV "GATEWAY_TEST_PORT=19600" "WATCHDOG_E2E_SCENARIO=restart_inside_warmup"
      "${_WATCHDOG_E2E_ENV}")

  # The two GROUNDS for ownership, and the only row here that asserts GRAPH_NODE_DISAPPEARED
  # ABSENT: the fixture is left until its GetState budget is spent (provisional ownership),
  # then told to announce "inactive", then killed. 60 (arming, app_id-scoped) + 30
  # (faults-live) + 30 (unread-and-settled poll) + 30 (SetParameters call) + 30 (late-label
  # poll) + 30 (release poll, node_death's own tracked_count) + 30 (departure poll) + 60
  # (INACTIVE raise poll) + 10 (silence window, DISAPPEARED, SILENCE_WINDOW_SEC) = 310 internal
  # - nine independent budgets, not merged. Rounded to 420 to also cover the launch's own
  # teardown for TWO processes.
  medkit_add_launch_test(test_presence_ownership_e2e_provisional_yields_to_a_real_label
    test/e2e/test_presence_ownership_e2e.test.py TIMEOUT 420
    LABELS "integration;e2e"
    ENV "GATEWAY_TEST_PORT=19590" "WATCHDOG_E2E_SCENARIO=provisional_yields_to_a_real_label"
      "${_WATCHDOG_E2E_ENV}")

  # The same shape in the SHIPPED configuration (no require_active at all), where node_death is
  # the only detector that can report the death. 60 (arming, app_id-scoped) + 30 (faults-live) +
  # 30 (empty-label poll) + 30 (departure poll) + 60 (DISAPPEARED raise poll) + 10 (silence
  # window, UNREADABLE, SILENCE_WINDOW_SEC) = 220 internal. Rounded to 360 to also cover the
  # launch's own teardown for TWO processes.
  medkit_add_launch_test(test_presence_ownership_e2e_no_require_active_still_owned
    test/e2e/test_presence_ownership_e2e.test.py TIMEOUT 360
    LABELS "integration;e2e"
    ENV "GATEWAY_TEST_PORT=19570" "WATCHDOG_E2E_SCENARIO=no_require_active_still_owned"
      "${_WATCHDOG_E2E_ENV}")

  # The reverse transition on a live node: measured "active", then its lifecycle services are
  # dropped, then it is killed. Uses droppable_lifecycle_node.cpp, the only fixture that can
  # take a node from measured to unmeasured without killing it. 60 (arming, app_id-scoped) + 30
  # (faults-live) + 30 (active-label poll) + 30 (SetParameters call) + 30 (null-label poll) + 30
  # (departure poll) + 60 (DISAPPEARED raise poll) = 270 internal. Rounded to 360 to also cover
  # the launch's own teardown for TWO processes.
  medkit_add_launch_test(test_presence_ownership_e2e_measured_then_unmeasured
    test/e2e/test_presence_ownership_e2e.test.py TIMEOUT 360
    LABELS "integration;e2e"
    ENV "GATEWAY_TEST_PORT=19580" "WATCHDOG_E2E_SCENARIO=measured_then_unmeasured"
      "${_WATCHDOG_E2E_ENV}")

  # 60 (arming, app_id-scoped) + 30 (faults-live) + 30 (empty-label poll) + 30 (SetParameters
  # call) + 30 (active-label poll) + 30 (departure poll) + 60 (DISAPPEARED raise poll) = 270
  # internal. Rounded to 360 to also cover the launch's own teardown for TWO processes.
  medkit_add_launch_test(test_presence_ownership_e2e_answered_then_owned
    test/e2e/test_presence_ownership_e2e.test.py TIMEOUT 360
    LABELS "integration;e2e"
    ENV "GATEWAY_TEST_PORT=19550" "WATCHDOG_E2E_SCENARIO=answered_then_owned"
      "${_WATCHDOG_E2E_ENV}")

  # test_01: 60 (arming, app_id-scoped) + 30 (empty-label poll) + 60 (PARAM_DRIFT raise poll) =
  # 150. test_02: 60 (QOS_MISMATCH raise poll). test_03: 60 (ORPHAN raise poll) - 270 internal
  # across the three cases, not merged. Nothing is killed here, so the only teardown is the
  # launch's own for TWO processes; rounded to 360 for the same margin as its siblings.
  medkit_add_launch_test(test_presence_ownership_e2e_gate_unaffected
    test/e2e/test_presence_ownership_e2e.test.py TIMEOUT 360
    LABELS "integration;e2e"
    ENV "GATEWAY_TEST_PORT=19560" "WATCHDOG_E2E_SCENARIO=gate_unaffected"
      "${_WATCHDOG_E2E_ENV}")

  ros2_medkit_relax_vendor_warnings()

endif()

ament_package()
