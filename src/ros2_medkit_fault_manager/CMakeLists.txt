# Copyright 2025-2026 mfaferek93, bburda
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
project(ros2_medkit_fault_manager)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Shared cmake modules (multi-distro compat, ccache, linting)
find_package(ros2_medkit_cmake REQUIRED)
include(ROS2MedkitCompat)
include(ROS2MedkitCcache)
include(ROS2MedkitSanitizers)
include(ROS2MedkitCoverage)
include(ROS2MedkitLinting)
include(ROS2MedkitWarnings)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ros2_medkit_msgs REQUIRED)
find_package(ros2_medkit_serialization REQUIRED)
find_package(SQLite3 REQUIRED)
find_package(nlohmann_json REQUIRED)
# OpenSSL EVP SHA-256 for the tamper-evident audit log hash chain
find_package(OpenSSL REQUIRED)
# yaml-cpp is required as transitive dependency from ros2_medkit_serialization
medkit_find_yaml_cpp()
# rosbag2 for time-window snapshot recording
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)

medkit_detect_compat_defs()

# Library target (shared between executable and tests)
add_library(fault_manager_lib STATIC
  src/capture_thread_pool.cpp
  src/fault_manager_node.cpp
  src/fault_storage.cpp
  src/sqlite_fault_storage.cpp
  src/fault_audit_log.cpp
  src/snapshot_capture.cpp
  src/rosbag_capture.cpp
  src/correlation/types.cpp
  src/correlation/config_parser.cpp
  src/correlation/pattern_matcher.cpp
  src/correlation/correlation_engine.cpp
  src/entity_threshold_resolver.cpp
)

target_include_directories(fault_manager_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

medkit_target_dependencies(fault_manager_lib PUBLIC
  rclcpp
  ros2_medkit_msgs
  ros2_medkit_serialization
  rosbag2_cpp
  rosbag2_storage
)

target_link_libraries(fault_manager_lib PUBLIC
  SQLite::SQLite3
  nlohmann_json::nlohmann_json
  yaml-cpp::yaml-cpp
  OpenSSL::Crypto
)

medkit_apply_compat_defs(fault_manager_lib)

# Executable target
add_executable(fault_manager_node src/main.cpp)
target_link_libraries(fault_manager_node fault_manager_lib)

# Install targets
install(TARGETS fault_manager_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  find_package(std_msgs REQUIRED)

  medkit_lint_config(.clang-format ament_cmake_clang_format_CONFIG_FILE)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_uncrustify ament_cmake_cpplint ament_cmake_clang_tidy)
  ament_lint_auto_find_test_dependencies()

  ros2_medkit_clang_tidy()

  include(ROS2MedkitTestDomain)

  # Manifest test: the storage plugins declared as exec/test dependencies.
  # Creates no ROS node, so it needs no domain.
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_rosbag_storage_dependency test/test_rosbag_storage_dependency.py
    TIMEOUT 60)
  set_tests_properties(test_rosbag_storage_dependency PROPERTIES LABELS "unit")
  medkit_test_needs_no_domain(test_rosbag_storage_dependency)

  # Unit tests
  # Each GTest target that instantiates ROS 2 nodes takes a ROS_DOMAIN_ID from this
  # package's pool so it cannot interfere with launch_testing integration tests that run
  # concurrently in the same CTest invocation and launch their own fault_manager_node
  # subprocess.
  medkit_add_gtest(test_fault_manager test/test_fault_manager.cpp)
  target_link_libraries(test_fault_manager fault_manager_lib)
  medkit_target_dependencies(test_fault_manager rclcpp ros2_medkit_msgs std_msgs)

  # SQLite storage tests
  medkit_add_gtest(test_sqlite_storage test/test_sqlite_storage.cpp)
  target_link_libraries(test_sqlite_storage fault_manager_lib)
  medkit_target_dependencies(test_sqlite_storage rclcpp ros2_medkit_msgs)

  # Rosbag retention parity: every assertion runs against both storage backends.
  medkit_add_gtest(test_rosbag_storage_parity test/test_rosbag_storage_parity.cpp)
  target_link_libraries(test_rosbag_storage_parity fault_manager_lib)
  medkit_target_dependencies(test_rosbag_storage_parity rclcpp ros2_medkit_msgs)

  # Fault audit log tests (hash chain, verify, rotation, reopen)
  medkit_add_gtest(test_fault_audit_log test/test_fault_audit_log.cpp)
  target_link_libraries(test_fault_audit_log fault_manager_lib)

  # Snapshot capture tests
  medkit_add_gtest(test_snapshot_capture test/test_snapshot_capture.cpp)
  target_link_libraries(test_snapshot_capture fault_manager_lib)
  medkit_target_dependencies(test_snapshot_capture rclcpp ros2_medkit_msgs std_msgs)

  # Rosbag capture tests. The suite drives real post-roll windows (multi-second
  # buffer fills and timer waits), so it needs more than the 60s default.
  medkit_add_gtest(test_rosbag_capture test/test_rosbag_capture.cpp TIMEOUT 180)
  target_link_libraries(test_rosbag_capture fault_manager_lib)
  medkit_target_dependencies(test_rosbag_capture rclcpp ros2_medkit_msgs std_msgs)

  # Correlation config parser tests
  medkit_add_gtest(test_correlation_config_parser test/test_correlation_config_parser.cpp)
  target_link_libraries(test_correlation_config_parser fault_manager_lib)

  # Pattern matcher tests
  medkit_add_gtest(test_pattern_matcher test/test_pattern_matcher.cpp)
  target_link_libraries(test_pattern_matcher fault_manager_lib)

  # Correlation engine tests
  medkit_add_gtest(test_correlation_engine test/test_correlation_engine.cpp)
  target_link_libraries(test_correlation_engine fault_manager_lib)

  # Entity threshold resolver tests
  medkit_add_gtest(test_entity_thresholds test/test_entity_thresholds.cpp)
  target_link_libraries(test_entity_thresholds fault_manager_lib)
  medkit_target_dependencies(test_entity_thresholds rclcpp ros2_medkit_msgs)

  # Integration tests
  install(DIRECTORY test
    DESTINATION share/${PROJECT_NAME}
  )

  medkit_add_launch_test(test_integration test/integration/test_integration.test.py TIMEOUT 60
    LABELS "integration")

  medkit_add_launch_test(test_rosbag_integration test/integration/test_rosbag_integration.test.py TIMEOUT 90
    LABELS "integration")

  # Launches without snapshots.rosbag.format, so the parameter default (mcap)
  # is what is under test rather than an operator's explicit choice.
  medkit_add_launch_test(test_rosbag_default_format test/integration/test_rosbag_default_format.test.py TIMEOUT 90
    LABELS "integration")

  medkit_add_launch_test(test_entity_thresholds_integration test/integration/test_entity_thresholds_integration.test.py
    TIMEOUT 60 LABELS "integration")

  # Drives the debounce and healing pair with the event counts an edge-triggered
  # reporter actually sends: one FAILED per raise, one PASSED per clear. Two
  # cases wait out the auto-confirm window, hence the timeout.
  medkit_add_launch_test(test_debounce_and_healing test/integration/test_debounce_and_healing.test.py
    TIMEOUT 120 LABELS "integration")

  medkit_add_launch_test(test_rosbag_entity_scope test/integration/test_rosbag_entity_scope.test.py TIMEOUT 120
    LABELS "integration")

  # The only suite running above max_bags_per_fault=1: drives confirm / clear /
  # confirm on one code and asserts both recordings survive and stay separately
  # addressable. Six occurrences plus their post-roll windows, hence the timeout.
  medkit_add_launch_test(test_rosbag_history test/integration/test_rosbag_history.test.py TIMEOUT 240
    LABELS "integration")

  # Parametrized over both storage formats (sqlite3 + mcap), so the launch
  # runs twice inside one ctest invocation.
  medkit_add_launch_test(test_rosbag_boundary test/integration/test_rosbag_boundary.test.py TIMEOUT 240
    LABELS "integration")

  # Drives the DEFAULT "entity" topic mode, where a boundary recording carries a
  # filter, over two full post-fault windows plus discovery.
  medkit_add_launch_test(test_rosbag_boundary_scope test/integration/test_rosbag_boundary_scope.test.py TIMEOUT 180
    LABELS "integration")

  # CaptureThreadPool unit tests
  medkit_add_gtest(test_capture_thread_pool test/test_capture_thread_pool.cpp)
  target_link_libraries(test_capture_thread_pool fault_manager_lib)
  medkit_target_dependencies(test_capture_thread_pool rclcpp)

  ros2_medkit_relax_vendor_warnings()
endif()

ament_package()
