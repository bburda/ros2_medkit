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
project(ros2_medkit_cmake NONE)

find_package(ament_cmake REQUIRED)

# Install cmake modules to the ament share directory.
# The *-extras.cmake hook adds this path to CMAKE_MODULE_PATH automatically.
install(
  FILES
    cmake/ROS2MedkitCompat.cmake
    cmake/ROS2MedkitCcache.cmake
    cmake/ROS2MedkitCoverage.cmake
    cmake/ROS2MedkitLinting.cmake
    cmake/ROS2MedkitSanitizers.cmake
    cmake/ROS2MedkitTestDomain.cmake
    cmake/ROS2MedkitWarnings.cmake
  DESTINATION share/${PROJECT_NAME}/cmake
)

# Install .clang-tidy config alongside the cmake modules so
# ROS2MedkitLinting.cmake can find it via CMAKE_CURRENT_LIST_DIR.
# Conditional: in colcon workspace the file exists at ../../.clang-tidy;
# on the buildfarm it doesn't, but BUILD_TESTING=OFF so linting is disabled.
set(_CLANG_TIDY_SRC "${CMAKE_CURRENT_SOURCE_DIR}/../../.clang-tidy")
if(EXISTS "${_CLANG_TIDY_SRC}")
  install(FILES "${_CLANG_TIDY_SRC}" DESTINATION share/${PROJECT_NAME}/cmake)
endif()

ament_package(
  CONFIG_EXTRAS cmake/ros2_medkit_cmake-extras.cmake
)
