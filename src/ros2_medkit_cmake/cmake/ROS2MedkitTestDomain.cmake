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

# ROS_DOMAIN_ID allocation for tests.
#
# ---------------------------------------------------------------------------
# Why the allocation is not simply 1-232
# ---------------------------------------------------------------------------
#
# RTPS gives every DDS domain a 250-port slice of the UDP space:
#
#   slice(d) = [7400 + 250 * d, 7400 + 250 * d + 249]
#
# Cyclone DDS 0.10.5, what both Humble and Jazzy ship, binds the multicast
# sockets at offsets 0 and 1 unconditionally, on every configuration -
# ddsi_portmapping.c forces the participant index to 0 for both multicast
# cases, so the index never enters into it. Those sockets do set both
# SO_REUSEPORT and SO_REUSEADDR, but a collision there is still fatal:
# Linux only grants the reuse exemption when every socket that ever held
# the port opted in, so an ordinary socket already sitting on it blocks
# the bind regardless. A failed multicast bind aborts domain creation, so
# this is the failure that actually kills a node:
#
#   ddsi_udp_create_conn: failed to bind to ANY:53150: address in use
#   rmw_create_node: failed to create domain, error Error
#
# 53150 is 7400 + 250 * 183, the multicast base port of domain 183 - the
# failure above is exactly the case this scheme protects against.
#
# The unicast sockets, at offsets 10 + 2p and 11 + 2p for participant
# index p, depend on Discovery/ParticipantIndex. On Humble,
# rmw_cyclonedds_cpp emits no <Discovery> element, so the default "none"
# applies and the single unicast socket gets a kernel-assigned port with
# no relation to the domain. On Jazzy, rmw_cyclonedds_cpp injects
# <ParticipantIndex>auto</ParticipantIndex> with MaxAutoParticipantIndex
# 32, so unicast does land on offsets 10 through 75 - but a collision
# there is not fatal, it just advances to the next participant index.
#
# Fast DDS 3.6.2 (Lyrical) derives every listening port from the formula
# and never falls back to a kernel-assigned one; its participant id
# mutates by +2 for up to 100 tries on collision. A failed multicast bind
# there is only a warning, so discovery degrades instead of the process
# dying.
#
# What keeping a domain's slice out of the ephemeral range buys, then: on
# every distro it protects offsets 0 and 1, the one collision that is
# fatal. On Jazzy it additionally covers offsets 10 through 75. On Humble
# the unicast socket sits outside the scheme entirely and needs no
# protection, since a kernel-assigned port is free by construction.
#
# The protection is deterministic rather than statistical: once a socket
# with the reuse options holds a port, the kernel will not also hand that
# port out as an ephemeral one. The race runs one way - the unrelated
# process has to grab the port before the participant starts, never the
# other way around.
#
# The kernel hands out ephemeral ports from net.ipv4.ip_local_port_range, which
# defaults to 32768-60999. Any unrelated process - a browser, a package manager,
# a second test run - can be given a port in that range, and if it is one of
# ours the whole domain stops working. Mapped back through the formula, the
# default range covers domains 101 to 214 - domain 101's slice, 32650-32899,
# already overlaps the range starting at 32768.
#
# The usable domains are the ones whose whole slice sits outside that range:
# 1-100 and 215-231. Domain 0 stays free because it is the ROS 2 default a
# developer shell uses, and 232 is dropped because its slice runs past 65535.
#
# That is 117 domains for more test slots than that, so packages draw from a
# pool and reuse domains inside it. CTest never runs two tests that share a
# domain at the same time, because every domain also installs a RESOURCE_LOCK -
# which matters because scripts/test.sh runs `ctest -j $(nproc)`, so tests
# inside one package really do run concurrently.
#
# ---------------------------------------------------------------------------
# The allocation table
# ---------------------------------------------------------------------------
#
# colcon runs one CTest per package, in parallel across packages, and a
# RESOURCE_LOCK only binds inside a single CTest run. Cross-package isolation
# therefore comes from the pools being disjoint, which is checked below rather
# than trusted.
#
# The table is the single source of truth. A package names itself and gets its
# pool from here, so a range cannot drift between this file and a CMakeLists.txt.
#
# Pools are sized to the number of tests that hold a domain plus headroom; the
# two largest pools are deliberately smaller than their test count and rely on
# reuse. The safe band is fully allocated, so a new package takes its slots from
# a pool that has headroom.
set(MEDKIT_DOMAIN_TABLE
  "ros2_medkit_sovd_service_interface:1-2"
  "ros2_medkit_param_beacon:3-4"
  "ros2_medkit_topic_beacon:5-6"
  "ros2_medkit_graph_provider:7-8"
  "ros2_medkit_fault_reporter:9-11"
  "ros2_medkit_log_bridge:12-14"
  "ros2_medkit_action_status_bridge:15-17"
  "ros2_medkit_diagnostic_bridge:18-20"
  "ros2_medkit_fault_manager:21-32"
  "ros2_medkit_opcua:33-44"
  "ros2_medkit_graph_watchdog:45-54"
  "ros2_medkit_gateway:55-79"
  "ros2_medkit_integration_tests:80-100,215-228"
)

# Secondary pool for tests that need MORE than one domain at a time (a
# multi-gateway test runs its second and third gateway on their own domains).
# Shared by every such test and serialised with a RESOURCE_LOCK, so it does not
# grow with the number of them. Exported to Python through
# MEDKIT_SECONDARY_DOMAINS so the test utils do not repeat the numbers.
set(MEDKIT_SECONDARY_DOMAIN_RANGE "229-231")

set(_MEDKIT_RTPS_PORT_BASE 7400)
set(_MEDKIT_RTPS_DOMAIN_GAIN 250)

# Where this module lives, captured at include time. Inside a macro,
# CMAKE_CURRENT_LIST_DIR would point at the calling CMakeLists.txt instead.
set(_MEDKIT_TEST_DOMAIN_MODULE_DIR "${CMAKE_CURRENT_LIST_DIR}")

# ---------------------------------------------------------------------------
# Ephemeral port range
# ---------------------------------------------------------------------------
#
# Read from the running kernel, then widened to at least the Linux default. The
# widening matters: without it, configuring on a host with a narrow range would
# accept an allocation that breaks on a stock machine, and the check would only
# fail for whoever happened to run it last.
set(_MEDKIT_EPHEMERAL_LOW 32768)
set(_MEDKIT_EPHEMERAL_HIGH 60999)
set(_MEDKIT_EPHEMERAL_SOURCE "Linux default; kernel range not readable")
if(EXISTS "/proc/sys/net/ipv4/ip_local_port_range")
  file(READ "/proc/sys/net/ipv4/ip_local_port_range" _medkit_range_raw)
  if(_medkit_range_raw MATCHES "([0-9]+)[ \t]+([0-9]+)")
    set(_medkit_sys_low ${CMAKE_MATCH_1})
    set(_medkit_sys_high ${CMAKE_MATCH_2})
    set(_MEDKIT_EPHEMERAL_SOURCE
      "net.ipv4.ip_local_port_range=${_medkit_sys_low}-${_medkit_sys_high}, widened to the Linux default")
    if(_medkit_sys_low LESS _MEDKIT_EPHEMERAL_LOW)
      set(_MEDKIT_EPHEMERAL_LOW ${_medkit_sys_low})
    endif()
    if(_medkit_sys_high GREATER _MEDKIT_EPHEMERAL_HIGH)
      set(_MEDKIT_EPHEMERAL_HIGH ${_medkit_sys_high})
    endif()
  endif()
endif()

# Expand "1-2,215-228" into a flat list of domain IDs.
function(_medkit_expand_domain_ranges RANGE_SPEC OUT_VAR)
  set(_domains "")
  string(REPLACE "," ";" _ranges "${RANGE_SPEC}")
  foreach(_range IN LISTS _ranges)
    if(NOT _range MATCHES "^([0-9]+)-([0-9]+)$")
      message(FATAL_ERROR
        "ROS2MedkitTestDomain: malformed domain range '${_range}' in '${RANGE_SPEC}' "
        "(expected '<start>-<end>')")
    endif()
    set(_start ${CMAKE_MATCH_1})
    set(_end ${CMAKE_MATCH_2})
    if(_start GREATER _end)
      message(FATAL_ERROR
        "ROS2MedkitTestDomain: inverted domain range '${_range}' in '${RANGE_SPEC}'")
    endif()
    foreach(_d RANGE ${_start} ${_end})
      list(APPEND _domains ${_d})
    endforeach()
  endforeach()
  set(${OUT_VAR} "${_domains}" PARENT_SCOPE)
endfunction()

# Fail if any domain in the list would bind a port the kernel can hand to
# somebody else, or a port outside the legal UDP space.
function(_medkit_assert_domains_safe OWNER DOMAINS)
  foreach(_d IN LISTS DOMAINS)
    if(_d EQUAL 0)
      message(FATAL_ERROR
        "ROS2MedkitTestDomain: ${OWNER} claims domain 0, which is the ROS 2 default and "
        "belongs to whoever is using the machine. Pick another domain.")
    endif()
    math(EXPR _slice_low "${_MEDKIT_RTPS_PORT_BASE} + ${_MEDKIT_RTPS_DOMAIN_GAIN} * ${_d}")
    math(EXPR _slice_high "${_slice_low} + ${_MEDKIT_RTPS_DOMAIN_GAIN} - 1")
    if(_slice_high GREATER 65535)
      message(FATAL_ERROR
        "ROS2MedkitTestDomain: ${OWNER} claims domain ${_d}, whose UDP slice "
        "${_slice_low}-${_slice_high} runs past 65535. The highest usable domain is 231.")
    endif()
    if(NOT (_slice_high LESS _MEDKIT_EPHEMERAL_LOW OR _slice_low GREATER _MEDKIT_EPHEMERAL_HIGH))
      message(FATAL_ERROR
        "ROS2MedkitTestDomain: ${OWNER} claims domain ${_d}, whose UDP slice "
        "${_slice_low}-${_slice_high} overlaps the kernel ephemeral port range "
        "${_MEDKIT_EPHEMERAL_LOW}-${_MEDKIT_EPHEMERAL_HIGH} (${_MEDKIT_EPHEMERAL_SOURCE}). "
        "Any process on the machine can be given one of those ports, and the test then dies "
        "with 'failed to bind to ANY:<port>: address in use'. Use a domain in 1-100 or 215-231.")
    endif()
  endforeach()
endfunction()

# Validate the whole table on every include, not only the pool being asked for.
# A package that is not currently being built still cannot hold an unsafe or
# overlapping range without the next build of any other package saying so.
function(_medkit_validate_domain_table)
  set(_seen "")
  set(_all_specs ${MEDKIT_DOMAIN_TABLE} "secondary pool:${MEDKIT_SECONDARY_DOMAIN_RANGE}")
  foreach(_entry IN LISTS _all_specs)
    if(NOT _entry MATCHES "^([^:]+):(.+)$")
      message(FATAL_ERROR "ROS2MedkitTestDomain: malformed table entry '${_entry}'")
    endif()
    set(_owner "${CMAKE_MATCH_1}")
    _medkit_expand_domain_ranges("${CMAKE_MATCH_2}" _domains)
    _medkit_assert_domains_safe("${_owner}" "${_domains}")
    foreach(_d IN LISTS _domains)
      if(_d IN_LIST _seen)
        message(FATAL_ERROR
          "ROS2MedkitTestDomain: domain ${_d} is allocated twice; '${_owner}' overlaps an "
          "earlier entry. Pools must be disjoint - colcon runs packages in parallel and a "
          "RESOURCE_LOCK does not reach across CTest runs.")
      endif()
      list(APPEND _seen ${_d})
    endforeach()
  endforeach()
endfunction()

_medkit_validate_domain_table()

# Initialize the domain pool for a package.
# Must be called once per CMakeLists.txt, before any other macro here.
#
# Usage:
#   medkit_init_test_domains(PACKAGE ros2_medkit_gateway)
#
macro(medkit_init_test_domains)
  cmake_parse_arguments(_MTID "" "PACKAGE" "" ${ARGN})
  if(NOT DEFINED _MTID_PACKAGE)
    message(FATAL_ERROR "medkit_init_test_domains requires a PACKAGE argument")
  endif()
  set(_MEDKIT_DOMAIN_POOL "")
  set(_MTID_MATCHES 0)
  foreach(_mtid_entry IN LISTS MEDKIT_DOMAIN_TABLE)
    if(_mtid_entry MATCHES "^${_MTID_PACKAGE}:(.+)$")
      _medkit_expand_domain_ranges("${CMAKE_MATCH_1}" _MEDKIT_DOMAIN_POOL)
      math(EXPR _MTID_MATCHES "${_MTID_MATCHES} + 1")
    endif()
  endforeach()
  if(_MTID_MATCHES GREATER 1)
    message(FATAL_ERROR
      "medkit_init_test_domains: '${_MTID_PACKAGE}' appears ${_MTID_MATCHES} times in "
      "MEDKIT_DOMAIN_TABLE, and only the last entry would be used. Give the package one "
      "entry with comma-separated ranges instead, e.g. '80-100,215-228'.")
  endif()
  if(NOT _MEDKIT_DOMAIN_POOL)
    message(FATAL_ERROR
      "medkit_init_test_domains: no domain pool for '${_MTID_PACKAGE}'. Add one to "
      "MEDKIT_DOMAIN_TABLE in ROS2MedkitTestDomain.cmake - the safe band is fully "
      "allocated, so take the slots from a pool that has headroom.")
  endif()
  set(_MEDKIT_DOMAIN_OWNER ${_MTID_PACKAGE})
  set(_MEDKIT_DOMAIN_NEXT 0)
  list(LENGTH _MEDKIT_DOMAIN_POOL _MEDKIT_DOMAIN_POOL_SIZE)
endmacro()

# Take the next domain from the pool, with its lock name.
#
# Wraps around when the pool is exhausted, so a package may hold more tests than
# it has domains. Reuse stays safe only if the caller puts the returned lock on
# the test: CTest will not schedule two tests holding the same lock at once.
# Prefer medkit_set_test_domain, which does both.
#
# Usage:
#   medkit_reserve_test_domain(_my_domain _my_lock)
#
macro(medkit_reserve_test_domain DOMAIN_VAR LOCK_VAR)
  if(NOT DEFINED _MEDKIT_DOMAIN_POOL)
    message(FATAL_ERROR "medkit_reserve_test_domain called before medkit_init_test_domains")
  endif()
  list(GET _MEDKIT_DOMAIN_POOL ${_MEDKIT_DOMAIN_NEXT} ${DOMAIN_VAR})
  set(${LOCK_VAR} "medkit_dds_domain_${${DOMAIN_VAR}}")
  math(EXPR _MEDKIT_DOMAIN_NEXT "(${_MEDKIT_DOMAIN_NEXT} + 1) % ${_MEDKIT_DOMAIN_POOL_SIZE}")
endmacro()

# Assign the next domain from the pool to a test target.
# The test must already be defined via ament_add_gtest or add_launch_test.
#
# Both properties are APPENDed, so a caller may add its own environment entries
# and its own resource locks. Use set_property(TEST ... APPEND PROPERTY ...) for
# those: a plain set_tests_properties(... PROPERTIES ENVIRONMENT ...) after this
# call would drop the domain assignment on the floor. test_dds_domain_allocation
# catches that at test time.
#
# Usage:
#   ament_add_gtest(test_foo test_foo.cpp)
#   medkit_set_test_domain(test_foo)
#
macro(medkit_set_test_domain TEST_NAME)
  medkit_reserve_test_domain(_MSTD_DOMAIN _MSTD_LOCK)
  set_property(TEST ${TEST_NAME} APPEND PROPERTY ENVIRONMENT "ROS_DOMAIN_ID=${_MSTD_DOMAIN}")
  set_property(TEST ${TEST_NAME} APPEND PROPERTY RESOURCE_LOCK "${_MSTD_LOCK}")
endmacro()

# Register a launch_testing test AND assign it a domain in one call.
# This is the required way to add a launch test: it makes it impossible to add
# one without domain isolation (a launch test left on the default domain 0 sees
# every other node on the machine).
# Do not call add_launch_test directly.
#
# Usage:
#   medkit_add_launch_test(test_integration test/test_integration.test.py)
#   medkit_add_launch_test(test_integration test/test_integration.test.py TIMEOUT 90)
macro(medkit_add_launch_test TEST_NAME TEST_FILE)
  cmake_parse_arguments(_MALT "" "TIMEOUT" "" ${ARGN})
  if(DEFINED _MALT_TIMEOUT)
    add_launch_test(${TEST_FILE} TARGET ${TEST_NAME} TIMEOUT ${_MALT_TIMEOUT})
  else()
    add_launch_test(${TEST_FILE} TARGET ${TEST_NAME})
  endif()
  medkit_set_test_domain(${TEST_NAME})
endmacro()

# Comma-separated secondary domains, for tests that hold several domains at
# once. Pass it through the environment so Python does not repeat the numbers.
#
# Usage:
#   medkit_secondary_test_domains(_secondary)
#   ... ENVIRONMENT "MEDKIT_SECONDARY_DOMAINS=${_secondary}"
macro(medkit_secondary_test_domains OUT_VAR)
  _medkit_expand_domain_ranges("${MEDKIT_SECONDARY_DOMAIN_RANGE}" _MSEC_DOMAINS)
  string(REPLACE ";" "," ${OUT_VAR} "${_MSEC_DOMAINS}")
endmacro()

# Register the per-package guard that re-reads the generated CTest properties on
# the machine that RUNS the tests. The checks above run at configure time, on the
# machine that builds them, against the table; this one runs against what was
# actually generated and against that machine's live kernel range.
#
# May be called anywhere after medkit_init_test_domains: it asks ctest for the
# test list when it runs, by which time every test in the package is registered.
#
# Usage:
#   medkit_add_domain_allocation_test()
macro(medkit_add_domain_allocation_test)
  if(NOT DEFINED _MEDKIT_DOMAIN_POOL)
    message(FATAL_ERROR
      "medkit_add_domain_allocation_test called before medkit_init_test_domains")
  endif()
  string(REPLACE ";" "," _MADAT_POOL "${_MEDKIT_DOMAIN_POOL}")
  medkit_secondary_test_domains(_MADAT_SECONDARY)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  add_test(
    NAME test_dds_domain_allocation
    COMMAND "${Python3_EXECUTABLE}"
      "${_MEDKIT_TEST_DOMAIN_MODULE_DIR}/check_test_domains.py"
      --build-dir "${CMAKE_CURRENT_BINARY_DIR}"
      --package "${_MEDKIT_DOMAIN_OWNER}"
      --pool "${_MADAT_POOL}"
      --secondary "${_MADAT_SECONDARY}"
  )
endmacro()
