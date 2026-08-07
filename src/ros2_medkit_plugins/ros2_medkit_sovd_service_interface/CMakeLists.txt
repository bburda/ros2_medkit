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
project(ros2_medkit_sovd_service_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wshadow -Wconversion)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Shared cmake modules (multi-distro compat)
find_package(ros2_medkit_cmake REQUIRED)
include(ROS2MedkitCompat)
include(ROS2MedkitCoverage)

find_package(ament_cmake REQUIRED)
find_package(ros2_medkit_gateway REQUIRED)
find_package(ros2_medkit_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nlohmann_json REQUIRED)

# MODULE target: loaded via dlopen at runtime by PluginManager.
# Symbols from gateway_ros2 (and gateway_core transitively) are resolved
# from the host process at runtime.
add_library(sovd_service_interface MODULE
  src/sovd_service_interface.cpp
  src/service_exports.cpp
)

target_include_directories(sovd_service_interface PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/src
)

medkit_target_dependencies(sovd_service_interface
  ros2_medkit_msgs
  ros2_medkit_gateway
  rclcpp
)

# Allow unresolved symbols - they resolve from the host process at runtime
target_link_options(sovd_service_interface PRIVATE
  -Wl,--unresolved-symbols=ignore-all
)

target_link_libraries(sovd_service_interface
  nlohmann_json::nlohmann_json
)

install(TARGETS sovd_service_interface
  LIBRARY DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # uncrustify/cpplint conflict with project-wide clang-format (120 cols vs 100)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_uncrustify
    ament_cmake_cpplint
    ament_cmake_clang_format
  )
  ament_lint_auto_find_test_dependencies()

  # ci.yml skips the ament_cmake_clang_format rosdep key in the build-and-test
  # job (linters only run in the Jazzy quality job), so QUIET + FOUND check
  # avoids hard-failing when the package is not installed.
  find_package(ament_cmake_clang_format QUIET)
  if(ament_cmake_clang_format_FOUND)
    file(GLOB_RECURSE _format_files
      "src/*.hpp" "src/*.cpp" "test/*.cpp"
    )
    ament_clang_format(${_format_files}
      CONFIG_FILE "${CMAKE_CURRENT_SOURCE_DIR}/../../../.clang-format")
  endif()

  find_package(ament_cmake_gtest REQUIRED)

  include(ROS2MedkitTestDomain)
  medkit_init_test_domains(PACKAGE ros2_medkit_sovd_service_interface)
  medkit_add_domain_allocation_test()

  ament_add_gtest(test_sovd_service_interface
    test/test_sovd_service_interface.cpp
    src/sovd_service_interface.cpp
  )
  target_include_directories(test_sovd_service_interface PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
  )
  medkit_target_dependencies(test_sovd_service_interface
    ros2_medkit_gateway
    rclcpp
    ros2_medkit_msgs
  )
  target_link_libraries(test_sovd_service_interface
    nlohmann_json::nlohmann_json
  )
  medkit_set_test_domain(test_sovd_service_interface)
endif()

ament_package()
