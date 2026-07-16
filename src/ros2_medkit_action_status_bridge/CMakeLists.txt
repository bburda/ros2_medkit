# Copyright 2026 mfaferek93, bburda
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
project(ros2_medkit_action_status_bridge)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ros2_medkit_cmake REQUIRED)
include(ROS2MedkitCcache)
include(ROS2MedkitSanitizers)
include(ROS2MedkitLinting)
include(ROS2MedkitWarnings)

option(ENABLE_COVERAGE "Enable code coverage reporting" OFF)
if(ENABLE_COVERAGE)
  message(STATUS "Code coverage enabled")
  add_compile_options(--coverage -O0 -g)
  add_link_options(--coverage)
endif()

find_package(ament_cmake REQUIRED)

include(ROS2MedkitCompat)

find_package(rclcpp REQUIRED)
find_package(action_msgs REQUIRED)
find_package(ros2_medkit_msgs REQUIRED)
find_package(ros2_medkit_fault_reporter REQUIRED)

add_library(action_status_bridge_lib SHARED
  src/action_status_bridge_node.cpp
)

target_include_directories(action_status_bridge_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

medkit_target_dependencies(action_status_bridge_lib
  rclcpp
  action_msgs
  ros2_medkit_msgs
  ros2_medkit_fault_reporter
)

add_executable(action_status_bridge_node src/main.cpp)
target_link_libraries(action_status_bridge_node action_status_bridge_lib)
medkit_target_dependencies(action_status_bridge_node rclcpp)

install(TARGETS action_status_bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS action_status_bridge_lib
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp action_msgs ros2_medkit_msgs ros2_medkit_fault_reporter)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)

  set(ament_cmake_clang_format_CONFIG_FILE "${CMAKE_CURRENT_SOURCE_DIR}/../../.clang-format")
  list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_uncrustify ament_cmake_cpplint ament_cmake_clang_tidy)
  ament_lint_auto_find_test_dependencies()

  ros2_medkit_clang_tidy()

  include(ROS2MedkitTestDomain)
  medkit_init_test_domains(START 116 END 119)

  ament_add_gtest(test_action_status_bridge test/test_action_status_bridge.cpp)
  target_link_libraries(test_action_status_bridge action_status_bridge_lib)
  medkit_target_dependencies(test_action_status_bridge rclcpp action_msgs ros2_medkit_msgs)
  medkit_set_test_domain(test_action_status_bridge)

  if(ENABLE_COVERAGE)
    target_compile_options(test_action_status_bridge PRIVATE --coverage -O0 -g)
    target_link_options(test_action_status_bridge PRIVATE --coverage)
  endif()

  # Integration test (launch_testing): fault_manager + bridge + synthetic status
  install(DIRECTORY test
    DESTINATION share/${PROJECT_NAME}
  )
  medkit_add_launch_test(test_integration test/test_integration.test.py TIMEOUT 120)

  # Deterministic end-to-end regression for #467 provisional re-attribution.
  # slow_discovery_bridge is the real ActionStatusBridgeNode with only the racy
  # DDS-FQN resolution forced to "" for the first few rescans, so the real
  # provisional -> supersede path runs against a real FaultManager.
  add_executable(slow_discovery_bridge test/slow_discovery_bridge_main.cpp)
  target_link_libraries(slow_discovery_bridge action_status_bridge_lib)
  medkit_target_dependencies(slow_discovery_bridge rclcpp)
  install(TARGETS slow_discovery_bridge DESTINATION lib/${PROJECT_NAME})

  medkit_add_launch_test(test_reattribution_e2e test/test_reattribution_e2e.test.py TIMEOUT 120)

  ros2_medkit_relax_vendor_warnings()
endif()

ament_package()
