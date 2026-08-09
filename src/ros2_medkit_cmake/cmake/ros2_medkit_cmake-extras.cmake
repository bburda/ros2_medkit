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

# Automatically sourced by ament after find_package(ros2_medkit_cmake).
# Adds the installed cmake module directory to CMAKE_MODULE_PATH so that
# include(ROS2MedkitCompat), include(ROS2MedkitCcache),
# include(ROS2MedkitCoverage), include(ROS2MedkitLinting),
# include(ROS2MedkitSanitizers), include(ROS2MedkitTestDomain),
# and include(ROS2MedkitWarnings) work transparently.

list(APPEND CMAKE_MODULE_PATH "${ros2_medkit_cmake_DIR}")

# Arm the DDS domain gate for this package, without the package asking for it.
#
# A test registered without the domain wrapper does not fail, it runs on the
# default domain 0 and shares it with every process on the machine. A gate that
# only runs where somebody remembered to register it cannot catch that: the next
# package added would be outside it and nothing would say so. So the gate rides
# on find_package(ros2_medkit_cmake) instead, which every package already does.
#
# Deferred rather than registered here: this hook runs before any of the
# package's tests exist, and in several packages before ament_cmake has defined
# BUILD_TESTING. cmake_language(DEFER) runs the registration at the end of the
# directory, by which time both are settled. It needs CMake 3.19; the oldest
# distro we build for ships 3.22.
include("${ros2_medkit_cmake_DIR}/ROS2MedkitTestDomain.cmake")
cmake_language(DEFER CALL _medkit_register_domain_gate)
cmake_language(DEFER CALL _medkit_set_domain_wait_budgets)
