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

include_guard(GLOBAL)

# Shared linting configuration for ros2_medkit packages.
# Include this in CMakeLists.txt BEFORE the if(BUILD_TESTING) block
# (alongside include(ROS2MedkitCcache) - CMAKE_MODULE_PATH is already set).
#
# Provides:
#   option ENABLE_CLANG_TIDY (default OFF)
#   cache var ROS2_MEDKIT_CLANG_TIDY_JOBS (default: host core count)
#   function ros2_medkit_clang_tidy([HEADER_FILTER <regex>] [TIMEOUT <seconds>] [JOBS <n>])

option(ENABLE_CLANG_TIDY "Register clang-tidy as a CTest target" OFF)

# ament_clang_tidy defaults to --jobs 1, so one CTest test analyses a whole
# package one translation unit at a time while the rest of the machine idles.
# CTest-level parallelism does not help: a participating package registers a
# single clang_tidy test, so --ctest-args -j has nothing to overlap.
#
# Analysing in parallel WITHIN the test is the lever. What still runs packages
# concurrently is colcon's --parallel-workers, so that is what has to come down
# when clang-tidy runs (scripts/test.sh pins --parallel-workers 1 in the tidy
# preset) or peak memory becomes packages x jobs x the per-process footprint.
#
# That footprint is why the default is capped rather than set to the core count.
# Measured on the gateway package, memory scales linearly at ~1.2 GiB per job
# while wall clock does not:
#
#   jobs=2   1078 s    2.6 GiB      jobs=8    359 s    9.4 GiB
#   jobs=4    595 s    4.7 GiB      jobs=16   272 s   17.4 GiB
#
# The default is sized so one package fits an 8 GB machine, which puts it at 2.
# That is the slow end of the curve on purpose: the alternative is a default
# that works only on the biggest machine anyone here has, and 17.4 GiB for a
# single package is not something we can assume. Raise it where the memory is
# there: ./scripts/test.sh tidy --jobs <n> reconfigures and runs in one step.
#
# Over-subscribing is guarded only through that script. ament_clang_tidy derives
# its exit status from the warnings it parsed, so a clang-tidy killed by the OOM
# reaper contributes nothing and the test passes as if the package were clean.
# The script scans the per-test logs for the failure marker and fails the run; a
# bare colcon test -R clang_tidy does not.
include(ProcessorCount)
# Invoked lower-case: CMake command names are case-insensitive, and the
# repository's cmake-lint gate rejects mixed case.
processorcount(_ros2_medkit_host_cores)
if(_ros2_medkit_host_cores EQUAL 0)
  set(_ros2_medkit_host_cores 1)
endif()
set(_ros2_medkit_clang_tidy_jobs 2)
if(_ros2_medkit_host_cores LESS _ros2_medkit_clang_tidy_jobs)
  set(_ros2_medkit_clang_tidy_jobs "${_ros2_medkit_host_cores}")
endif()
set(ROS2_MEDKIT_CLANG_TIDY_JOBS "${_ros2_medkit_clang_tidy_jobs}"
    CACHE STRING "Number of clang-tidy processes per package (default: min(host cores, 2))")

# Capture at include-time: inside a function CMAKE_CURRENT_LIST_DIR resolves to the caller.
set(_ROS2_MEDKIT_CLANG_TIDY_CONFIG "${CMAKE_CURRENT_LIST_DIR}/.clang-tidy")

function(ros2_medkit_clang_tidy)
  if(NOT ENABLE_CLANG_TIDY)
    return()
  endif()

  cmake_parse_arguments(ARG "" "HEADER_FILTER;TIMEOUT;JOBS" "" ${ARGN})

  find_package(ament_cmake_clang_tidy REQUIRED)

  set(_args "${CMAKE_CURRENT_BINARY_DIR}" CONFIG_FILE "${_ROS2_MEDKIT_CLANG_TIDY_CONFIG}")

  if(ARG_HEADER_FILTER)
    list(APPEND _args HEADER_FILTER "${ARG_HEADER_FILTER}")
  endif()

  if(ARG_TIMEOUT)
    list(APPEND _args TIMEOUT "${ARG_TIMEOUT}")
  endif()

  if(ARG_JOBS)
    list(APPEND _args JOBS "${ARG_JOBS}")
  elseif(ROS2_MEDKIT_CLANG_TIDY_JOBS)
    list(APPEND _args JOBS "${ROS2_MEDKIT_CLANG_TIDY_JOBS}")
  endif()

  ament_clang_tidy(${_args})
endfunction()
