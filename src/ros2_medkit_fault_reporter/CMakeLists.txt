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
project(ros2_medkit_fault_reporter)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ros2_medkit_cmake REQUIRED)
include(ROS2MedkitCcache)
include(ROS2MedkitSanitizers)
include(ROS2MedkitLinting)
include(ROS2MedkitWarnings)

# Code coverage option
option(ENABLE_COVERAGE "Enable code coverage reporting" OFF)
if(ENABLE_COVERAGE)
  message(STATUS "Code coverage enabled")
  add_compile_options(--coverage -O0 -g)
  add_link_options(--coverage)
endif()

find_package(ament_cmake REQUIRED)

include(ROS2MedkitCompat)

find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(ros2_medkit_msgs REQUIRED)

# Library target
add_library(fault_reporter_lib SHARED
  src/fault_reporter.cpp
  src/local_filter.cpp
)

target_include_directories(fault_reporter_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

medkit_target_dependencies(fault_reporter_lib
  rclcpp
  rclcpp_lifecycle
  ros2_medkit_msgs
)

if(ENABLE_COVERAGE)
  target_compile_options(fault_reporter_lib PRIVATE --coverage -O0 -g)
  target_link_options(fault_reporter_lib PRIVATE --coverage)
endif()

# Install library
install(TARGETS fault_reporter_lib
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install headers
install(DIRECTORY include/
  DESTINATION include
)

# Export for downstream packages
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp rclcpp_lifecycle ros2_medkit_msgs)

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
  medkit_init_test_domains(START 110 END 112)

  # Unit tests
  ament_add_gtest(test_local_filter test/test_local_filter.cpp)
  target_link_libraries(test_local_filter fault_reporter_lib)
  medkit_target_dependencies(test_local_filter rclcpp rclcpp_lifecycle ros2_medkit_msgs)

  if(ENABLE_COVERAGE)
    target_compile_options(test_local_filter PRIVATE --coverage -O0 -g)
    target_link_options(test_local_filter PRIVATE --coverage)
  endif()

  # Integration tests
  install(DIRECTORY test
    DESTINATION share/${PROJECT_NAME}
  )

  medkit_add_launch_test(test_integration test/test_integration.test.py TIMEOUT 60)
  ros2_medkit_relax_vendor_warnings()
endif()

ament_package()
