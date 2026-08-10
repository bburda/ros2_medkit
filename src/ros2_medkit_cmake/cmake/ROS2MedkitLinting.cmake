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
#   CMAKE_EXPORT_COMPILE_COMMANDS ON
#   function medkit_lint_config(<file name> <output variable>)
#   option ENABLE_CLANG_TIDY (default OFF)
#   cache var ROS2_MEDKIT_CLANG_TIDY_JOBS (default: host core count)
#   function ros2_medkit_clang_tidy([HEADER_FILTER <regex>] [TIMEOUT <seconds>] [JOBS <n>])

# Every package that lints needs a compile database, because that is where
# clang-tidy reads a file's include paths and flags from. Setting it per package
# meant a new package started life invisible to the analysis: scripts/
# clang-tidy-diff.sh merges the per-package databases, and a file absent from
# the merged database is analysed with no flags at all. It then fails to find
# its own headers, and the pre-push hook reports that parse error instead of
# the checks it was meant to run. Eight of the packages set this and the rest
# did not, so most of the workspace was never really analysed. Setting it here
# means including this module is what makes a package lintable, with nothing
# else to remember.
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# The shared lint configurations - .clang-format, .clang-tidy, .flake8 - are
# files of this package, kept next to the cmake modules and installed next to
# them, so the same relative lookup works from a source tree, from a plain
# install, and from a --symlink-install. The repository root paths that editors,
# pre-commit and scripts/clang-tidy-diff.sh use are symlinks to these files, so
# there is one copy of each and nothing to keep in sync.
#
# What this replaces is a path out of the package into the repository root,
# "${CMAKE_CURRENT_SOURCE_DIR}/../../.flake8". That resolves in a workspace
# checkout and nowhere else. A binary package is built from an export of one
# package directory with nothing above it, so the linter was handed a path that
# does not exist and failed on every such build. Because bloom's debian/rules
# runs the test step as `dh_auto_test || true`, the failure could not stop the
# build and nobody had to see it.
#
# Captured at include time: inside a function CMAKE_CURRENT_LIST_DIR resolves to
# the caller's list file, not to this one.
set(_ROS2_MEDKIT_LINT_CONFIG_DIR "${CMAKE_CURRENT_LIST_DIR}")

# Resolve a shared lint configuration by file name to an absolute path.
#
#   medkit_lint_config(.clang-format _config)
#   ament_clang_format(CONFIG_FILE "${_config}")
#
# Aborts the configure step when the file is missing. Falling back to "run the
# linter without a config" or to "skip the linter" would turn a build that says
# it is wrong into a build that lints against the wrong rules or does not lint
# at all, which is the failure worth having least.
function(medkit_lint_config _name _output_var)
  set(_path "${_ROS2_MEDKIT_LINT_CONFIG_DIR}/${_name}")
  if(NOT EXISTS "${_path}")
    message(FATAL_ERROR
      "ros2_medkit_cmake: shared lint config '${_name}' not found at '${_path}'. "
      "These configs ship with ros2_medkit_cmake alongside its cmake modules; "
      "an install or package of ros2_medkit_cmake that lacks them is incomplete.")
  endif()
  set(${_output_var} "${_path}" PARENT_SCOPE)
endfunction()

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

function(ros2_medkit_clang_tidy)
  if(NOT ENABLE_CLANG_TIDY)
    return()
  endif()

  cmake_parse_arguments(ARG "" "HEADER_FILTER;TIMEOUT;JOBS" "" ${ARGN})

  find_package(ament_cmake_clang_tidy REQUIRED)

  medkit_lint_config(.clang-tidy _clang_tidy_config)
  set(_args "${CMAKE_CURRENT_BINARY_DIR}" CONFIG_FILE "${_clang_tidy_config}")

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
