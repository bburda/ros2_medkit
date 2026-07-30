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
project(ros2_medkit_diagnostic_bridge)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ros2_medkit_cmake REQUIRED)
include(ROS2MedkitCcache)
include(ROS2MedkitSanitizers)
include(ROS2MedkitCoverage)
include(ROS2MedkitLinting)
include(ROS2MedkitWarnings)

find_package(ament_cmake REQUIRED)

include(ROS2MedkitCompat)

find_package(rclcpp REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(ros2_medkit_msgs REQUIRED)
find_package(ros2_medkit_fault_reporter REQUIRED)

# Library target (for testing)
add_library(diagnostic_bridge_lib SHARED
  src/diagnostic_bridge_node.cpp
)

target_include_directories(diagnostic_bridge_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

medkit_target_dependencies(diagnostic_bridge_lib
  rclcpp
  diagnostic_msgs
  ros2_medkit_msgs
  ros2_medkit_fault_reporter
)

# Executable
add_executable(diagnostic_bridge_node src/main.cpp)
target_link_libraries(diagnostic_bridge_node diagnostic_bridge_lib)
medkit_target_dependencies(diagnostic_bridge_node rclcpp)

# Install executable
install(TARGETS diagnostic_bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install library
install(TARGETS diagnostic_bridge_lib
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install headers
install(DIRECTORY include/
  DESTINATION include
)

# Install launch and config
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

# Export for downstream packages
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp diagnostic_msgs ros2_medkit_msgs ros2_medkit_fault_reporter)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)

  set(ament_cmake_clang_format_CONFIG_FILE "${CMAKE_CURRENT_SOURCE_DIR}/../../.clang-format")
  list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_uncrustify ament_cmake_cpplint ament_cmake_clang_tidy)
  ament_lint_auto_find_test_dependencies()

  ros2_medkit_clang_tidy()

  include(ROS2MedkitTestDomain)
  medkit_init_test_domains(START 90 END 99)

  # Unit tests
  ament_add_gtest(test_diagnostic_bridge test/test_diagnostic_bridge.cpp)
  target_link_libraries(test_diagnostic_bridge diagnostic_bridge_lib)
  medkit_target_dependencies(test_diagnostic_bridge rclcpp diagnostic_msgs ros2_medkit_msgs)
  medkit_set_test_domain(test_diagnostic_bridge)

  # Integration tests
  install(DIRECTORY test
    DESTINATION share/${PROJECT_NAME}
  )

  # add_launch_test() defaults the test's ctest LABELS to "launch_test", which
  # the unit selector (colcon test -LE 'linter|integration') does not exclude,
  # so the launch test ran in the unit phase together with all other unit tests
  # in one colcon invocation. Under that load the node could miss the launch
  # SIGINT shutdown window and get killed with SIGTERM, failing the
  # post-shutdown exit-code check. Override the label to "integration" via
  # set_tests_properties (the repo convention) so it runs only in the
  # integration phase.
  medkit_add_launch_test(test_integration test/test_integration.test.py TIMEOUT 60)
  set_tests_properties(test_integration PROPERTIES LABELS "integration")
  ros2_medkit_relax_vendor_warnings()
endif()

ament_package()
