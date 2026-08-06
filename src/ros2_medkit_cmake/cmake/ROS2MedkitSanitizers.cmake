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

# Sanitizer support for ros2_medkit packages.
# Include this module early in CMakeLists.txt (alongside ROS2MedkitCcache).
#
# Activate from the command line:
#   colcon build --cmake-args -DSANITIZER=asan,ubsan
#   colcon build --cmake-args -DSANITIZER=tsan
#
# Supported sanitizers: asan, tsan, ubsan (comma-separated).
# ASan and TSan cannot be combined (incompatible runtimes).
# When SANITIZER is empty or unset, this module is a no-op.

set(SANITIZER "" CACHE STRING "Comma-separated list of sanitizers: asan, tsan, ubsan")

if(SANITIZER STREQUAL "")
  return()
endif()

# Parse comma-separated list
string(REPLACE "," ";" _SANITIZER_LIST "${SANITIZER}")

# Validate: asan + tsan is not allowed
list(FIND _SANITIZER_LIST "asan" _HAS_ASAN)
list(FIND _SANITIZER_LIST "tsan" _HAS_TSAN)
if(NOT _HAS_ASAN EQUAL -1 AND NOT _HAS_TSAN EQUAL -1)
  message(FATAL_ERROR "Cannot combine ASan and TSan - they use incompatible runtimes")
endif()

# Validate: only known sanitizers
foreach(_SAN IN LISTS _SANITIZER_LIST)
  if(NOT _SAN MATCHES "^(asan|tsan|ubsan)$")
    message(FATAL_ERROR "Unknown sanitizer '${_SAN}'. Supported: asan, tsan, ubsan")
  endif()
endforeach()

# Build the -fsanitize= flag value (asan -> address, tsan -> thread, ubsan -> undefined)
set(_FSANITIZE_FLAGS "")
foreach(_SAN IN LISTS _SANITIZER_LIST)
  if(_SAN STREQUAL "asan")
    list(APPEND _FSANITIZE_FLAGS "address")
  elseif(_SAN STREQUAL "tsan")
    list(APPEND _FSANITIZE_FLAGS "thread")
  elseif(_SAN STREQUAL "ubsan")
    list(APPEND _FSANITIZE_FLAGS "undefined")
  endif()
endforeach()
list(JOIN _FSANITIZE_FLAGS "," _FSANITIZE_VALUE)

# -O1: sanitizers produce fewer false positives and run faster than -O0.
# This intentionally overrides Debug's -O0 for sanitizer builds.
#
# -g1: line tables only, no variable/type DWARF. Until this was set the module
# overrode the build type's -O but never its -g, so RelWithDebInfo's full -g
# stayed in force and the instrumented ASan tree reached ~27 GB, the large
# majority of it debug info. That is not just disk: every one of those bytes is
# written by the compiler, read by the linker, and stored in ccache, and at the
# CI cache ceiling it meant ccache evicted the objects it was still producing.
#
# Nothing needed to diagnose a sanitizer finding is lost. Symbolication of the
# stack frames in an ASan/UBSan report needs the line table, which -g1 keeps, so
# reports still name file:line. The variable named in a stack-buffer-overflow
# report ("in frame ... at offset N ... 'buf'") comes from the frame descriptor
# ASan embeds at instrumentation time, not from DWARF, so it survives too. What
# -g1 does drop is the ability to inspect locals in a debugger on a core file.
#
# -UNDEBUG: keep assert() live whatever build type the caller picked. Release
# and RelWithDebInfo both carry -DNDEBUG, which compiles every assert away, and
# the CI sanitizer jobs build RelWithDebInfo. A sanitizer build exists to abort
# on a broken invariant, so the one build where asserts must fire was the one
# silently compiling them out. Debug is not a substitute here: it would bring
# -O0 with it, and -O1 above is deliberate.
#
# This is directory-scoped like the rest, so it also enables the assertions in
# headers compiled into a participating package - nlohmann/json routes
# JSON_ASSERT to assert, and the vendored cpp-httplib and dynmsg carry their
# own. That is far more than the three assertions the repository writes itself,
# and it is the intent: a sanitizer build should be running those checks.
#
# All three flags come from add_compile_options, which CMake places after
# CMAKE_CXX_FLAGS_<CONFIG> on the command line, so these win over the build
# type's -O2 -g -DNDEBUG. Do not move them into CMAKE_CXX_FLAGS, where they
# would lose.
add_compile_options(-fsanitize=${_FSANITIZE_VALUE} -fno-omit-frame-pointer -O1 -g1 -UNDEBUG)
add_link_options(-fsanitize=${_FSANITIZE_VALUE})

message(STATUS "Sanitizers enabled: ${SANITIZER} (-fsanitize=${_FSANITIZE_VALUE})")
