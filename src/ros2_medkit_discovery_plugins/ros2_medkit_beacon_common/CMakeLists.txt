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
project(ros2_medkit_beacon_common)

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
find_package(ros2_medkit_msgs REQUIRED)
find_package(ros2_medkit_gateway REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nlohmann_json REQUIRED)

add_library(beacon_common_lib STATIC
  src/beacon_validator.cpp
  src/beacon_hint_store.cpp
  src/beacon_entity_mapper.cpp
  src/beacon_response_builder.cpp
)

set_target_properties(beacon_common_lib PROPERTIES
  POSITION_INDEPENDENT_CODE ON  # Required: linked into MODULE .so targets
)

target_include_directories(beacon_common_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

medkit_target_dependencies(beacon_common_lib
  ros2_medkit_msgs
  ros2_medkit_gateway
  rclcpp
)

target_link_libraries(beacon_common_lib nlohmann_json::nlohmann_json)

# Install headers
install(DIRECTORY include/ DESTINATION include)

# Install library and export for downstream packages (topic_beacon)
install(TARGETS beacon_common_lib
  EXPORT beacon_common_libTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

ament_export_targets(beacon_common_libTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(ros2_medkit_msgs ros2_medkit_gateway rclcpp nlohmann_json)
ament_export_include_directories(include)

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

  include(ROS2MedkitTestDomain)

  ament_add_gtest(test_beacon_validator test/test_beacon_validator.cpp)
  target_link_libraries(test_beacon_validator beacon_common_lib)
  medkit_target_dependencies(test_beacon_validator ros2_medkit_gateway)

  ament_add_gtest(test_beacon_hint_store test/test_beacon_hint_store.cpp)
  target_link_libraries(test_beacon_hint_store beacon_common_lib)
  medkit_target_dependencies(test_beacon_hint_store ros2_medkit_gateway)

  ament_add_gtest(test_beacon_entity_mapper test/test_beacon_entity_mapper.cpp)
  target_link_libraries(test_beacon_entity_mapper beacon_common_lib)
  medkit_target_dependencies(test_beacon_entity_mapper ros2_medkit_gateway)
  ros2_medkit_relax_vendor_warnings()
  # No ROS middleware on the link line and none loaded at run time (these pass
  # with RMW_IMPLEMENTATION pointing at a library that does not exist), so there
  # is no DDS participant to isolate and a wrapper would be cost without cover.
  medkit_test_needs_no_domain(test_beacon_validator)
  medkit_test_needs_no_domain(test_beacon_hint_store)
  medkit_test_needs_no_domain(test_beacon_entity_mapper)

endif()

ament_package()
