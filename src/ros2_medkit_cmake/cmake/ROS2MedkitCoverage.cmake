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

# Code coverage support for ros2_medkit packages.
# Include this module early in CMakeLists.txt, before the first target is
# declared (alongside ROS2MedkitCcache and ROS2MedkitSanitizers).
#
# Activate from the command line:
#   colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug -DENABLE_COVERAGE=ON
#
# When ENABLE_COVERAGE is OFF (the default), this module is a no-op.
#
# The flags are applied at directory scope, so every target declared after the
# include() is instrumented: libraries, executables and test binaries alike.
# There is no per-target opt-in, which is deliberate - a package that forgets
# to instrument a target does not fail the build, it silently drops out of the
# coverage report, leaving both the numerator and the denominator. Whether that
# moves the workspace percentage up or down depends on how well the vanished
# package was covered, which is exactly why it must not be left to chance.
#
# Every package that compiles production C++ must include this module; packages
# that compile only test scaffolding are exempt and are named, with a reason, in
# EXCLUDED_PACKAGES in scripts/check_coverage_packages.sh. That script is the
# guard: it derives the packages that must be instrumented from the source tree,
# not from this include, so a package that never opts in is still caught. It
# runs in the Quality workflow (--static-only) and in the CI coverage job.

option(ENABLE_COVERAGE "Enable code coverage reporting" OFF)

if(NOT ENABLE_COVERAGE)
  return()
endif()

# -O0 keeps the line mapping faithful to the source: inlining and other
# optimisations make gcov attribute lines to the wrong place. -g is not required
# by gcov (the line map lives in the .gcno) but keeps the build debuggable, and
# coverage builds are debug builds.
add_compile_options(--coverage -O0 -g)
add_link_options(--coverage)

message(STATUS "Code coverage enabled for ${PROJECT_NAME}")
