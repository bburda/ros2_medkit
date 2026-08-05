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
project(ros2_medkit_graph_provider)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Without this the package exports no compile_commands.json, so clang-tidy run
# standalone - as the pre-push hook runs it - cannot resolve the gateway headers
# this plugin includes and reports them as missing on files that compile clean.
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Shared cmake modules (multi-distro compat)
find_package(ros2_medkit_cmake REQUIRED)
include(ROS2MedkitCompat)
include(ROS2MedkitCcache)
include(ROS2MedkitSanitizers)
include(ROS2MedkitCoverage)
include(ROS2MedkitLinting)
include(ROS2MedkitWarnings)

find_package(ament_cmake REQUIRED)
find_package(ros2_medkit_gateway REQUIRED)
find_package(rclcpp REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)
# MODULE target: loaded via dlopen at runtime by PluginManager.
# Symbols from gateway_ros2 (and gateway_core transitively) are resolved
# from the host process at runtime.
# We only need headers at compile time (via medkit_target_dependencies).
add_library(ros2_medkit_graph_provider_plugin MODULE
  src/graph_provider_plugin.cpp
  src/graph_provider_plugin_exports.cpp
)

target_include_directories(ros2_medkit_graph_provider_plugin PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

medkit_target_dependencies(ros2_medkit_graph_provider_plugin
  ros2_medkit_gateway
  rclcpp
  diagnostic_msgs
)

# Allow unresolved symbols - they resolve from the host process at runtime
target_link_options(ros2_medkit_graph_provider_plugin PRIVATE
  -Wl,--unresolved-symbols=ignore-all
)

target_link_libraries(ros2_medkit_graph_provider_plugin
  nlohmann_json::nlohmann_json
)

install(TARGETS ros2_medkit_graph_provider_plugin
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

  find_package(ament_cmake_gtest REQUIRED)

  # The route test uses httplib::Server/Client directly for end-to-end HTTP testing.
  # On Humble/Jammy the system package is too old (0.10.x); fall back to gateway's
  # vendored copy which is installed alongside the gateway package.
  find_package(OpenSSL REQUIRED)
  set(_gw_vendored "${ros2_medkit_gateway_DIR}/../vendored/cpp_httplib")
  medkit_find_cpp_httplib(VENDORED_DIR "${_gw_vendored}")
  unset(_gw_vendored)

  include(ROS2MedkitTestDomain)

  medkit_add_gtest(test_graph_provider_plugin
    test/test_graph_provider_plugin.cpp
    src/graph_provider_plugin.cpp
  )
  target_include_directories(test_graph_provider_plugin PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )
  medkit_target_dependencies(test_graph_provider_plugin
    ros2_medkit_gateway
    rclcpp
    diagnostic_msgs
  )
  target_compile_definitions(test_graph_provider_plugin PRIVATE CPPHTTPLIB_OPENSSL_SUPPORT)
  target_link_libraries(test_graph_provider_plugin
    nlohmann_json::nlohmann_json
    cpp_httplib_target
    OpenSSL::SSL OpenSSL::Crypto
  )
  ros2_medkit_relax_vendor_warnings()
endif()

ament_package()
