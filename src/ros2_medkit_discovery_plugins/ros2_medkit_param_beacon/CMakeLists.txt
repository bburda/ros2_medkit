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
project(ros2_medkit_param_beacon)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Shared cmake modules (multi-distro compat, ccache, linting)
find_package(ros2_medkit_cmake REQUIRED)
include(ROS2MedkitCompat)
include(ROS2MedkitCcache)
include(ROS2MedkitSanitizers)
include(ROS2MedkitCoverage)
include(ROS2MedkitLinting)
include(ROS2MedkitWarnings)

find_package(ament_cmake REQUIRED)
find_package(ros2_medkit_beacon_common REQUIRED)
find_package(ros2_medkit_gateway REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nlohmann_json REQUIRED)

# MODULE target: loaded via dlopen at runtime by PluginManager.
add_library(param_beacon_plugin MODULE
  src/param_beacon_plugin.cpp
)

target_include_directories(param_beacon_plugin PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

medkit_target_dependencies(param_beacon_plugin
  ros2_medkit_gateway
  ros2_medkit_beacon_common
  rclcpp
)

target_link_libraries(param_beacon_plugin
  nlohmann_json::nlohmann_json
  ros2_medkit_beacon_common::beacon_common_lib
)

install(TARGETS param_beacon_plugin
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
      "include/*.hpp" "src/*.cpp" "test/*.cpp"
    )
    medkit_lint_config(.clang-format _clang_format_config)
    ament_clang_format(${_format_files}
      CONFIG_FILE "${_clang_format_config}")
  endif()

  find_package(ament_cmake_gmock REQUIRED)

  include(ROS2MedkitTestDomain)

  # Include plugin source in test (plugin is MODULE/dlopen, test instantiates directly)
  # TIMEOUT 180: the 12 test cases each spin a node for ~200-400 ms. Under
  # plain gcc that's ~4 s total, well under the 30 s ctest default, but TSan
  # instrumentation inflates that 5-10x and occasionally trips the 30 s cap
  # (CI saw timeouts at test 4/12 around 27 s). 180 s covers the worst-case
  # sanitizer slowdown without masking real hangs.
  medkit_add_gmock(test_param_beacon_plugin
    test/test_param_beacon_plugin.cpp
    src/param_beacon_plugin.cpp
    TIMEOUT 180)
  target_include_directories(test_param_beacon_plugin PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include)
  medkit_target_dependencies(test_param_beacon_plugin
    ros2_medkit_gateway ros2_medkit_beacon_common rclcpp)
  target_link_libraries(test_param_beacon_plugin
    nlohmann_json::nlohmann_json
    ros2_medkit_beacon_common::beacon_common_lib)
  ros2_medkit_relax_vendor_warnings()
endif()

ament_package()
