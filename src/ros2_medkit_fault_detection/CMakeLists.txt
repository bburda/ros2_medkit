# Copyright 2026 mfaferek93
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
project(ros2_medkit_fault_detection)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ros2_medkit_cmake REQUIRED)
include(ROS2MedkitCoverage)
include(ROS2MedkitLinting)
include(ROS2MedkitWarnings)

find_package(ament_cmake REQUIRED)

# Header-only INTERFACE library: the detection logic is compiled directly into
# each consuming plugin (see header rationale).
add_library(ros2_medkit_fault_detection INTERFACE)
target_include_directories(ros2_medkit_fault_detection INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(ros2_medkit_fault_detection INTERFACE cxx_std_17)

install(TARGETS ros2_medkit_fault_detection
  EXPORT export_${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include
)

ament_export_targets(export_${PROJECT_NAME})
ament_export_include_directories(include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # clang_format is registered explicitly below; excluding it here prevents
  # ament_lint_auto from registering a second clang_format test.
  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_uncrustify
    ament_cmake_cpplint
    ament_cmake_clang_tidy
    ament_cmake_clang_format)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_clang_format QUIET)
  if(ament_cmake_clang_format_FOUND)
    file(GLOB_RECURSE _format_files "include/*.hpp" "test/*.cpp")
    medkit_lint_config(.clang-format _clang_format_config)
    ament_clang_format(${_format_files}
      CONFIG_FILE "${_clang_format_config}")
  endif()

  find_package(ament_cmake_gtest REQUIRED)

  include(ROS2MedkitTestDomain)
  ament_add_gtest(test_fault_detection test/test_fault_detection.cpp)
  target_link_libraries(test_fault_detection ros2_medkit_fault_detection)

  ros2_medkit_relax_vendor_warnings()
  # No ROS middleware on the link line and none loaded at run time (these pass
  # with RMW_IMPLEMENTATION pointing at a library that does not exist), so there
  # is no DDS participant to isolate and a wrapper would be cost without cover.
  medkit_test_needs_no_domain(test_fault_detection)

endif()

ament_package()
