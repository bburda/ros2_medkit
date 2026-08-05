cmake_minimum_required(VERSION 3.14)
project(ros2_medkit_integration_tests)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)

# Shared cmake modules (multi-distro compat, ccache, linting)
find_package(ros2_medkit_cmake REQUIRED)
include(ROS2MedkitCompat)
include(ROS2MedkitWarnings)

# The shared coverage module is deliberately NOT included here: everything this
# package compiles is test scaffolding - the demo_nodes/ fixtures driven by the
# launch_testing suites - not production code, so it stays out of the coverage
# report by design rather than by oversight.
#
# What keeps it out is the EXCLUDED_PACKAGES array in
# scripts/check_coverage_packages.sh, NOT the absence of the include. That gate
# derives the packages it expects from the source tree precisely so that a
# missing include cannot excuse itself. Delete the array entry and this package
# starts failing the gate.

find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(ros2_medkit_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(ros2_medkit_gateway REQUIRED)

# === Demo node executables ===

# Shared include directory for the demo_node_main.hpp helper. Keeps the
# graceful-shutdown boilerplate out of every demo binary.
set(_demo_include_dir ${CMAKE_CURRENT_SOURCE_DIR}/include)

add_executable(demo_engine_temp_sensor demo_nodes/engine_temp_sensor.cpp)
target_include_directories(demo_engine_temp_sensor PRIVATE ${_demo_include_dir})
medkit_target_dependencies(demo_engine_temp_sensor rclcpp rcl_interfaces sensor_msgs)

add_executable(demo_engine_temp_monitor demo_nodes/engine_temp_monitor.cpp)
target_include_directories(demo_engine_temp_monitor PRIVATE ${_demo_include_dir})
medkit_target_dependencies(demo_engine_temp_monitor rclcpp rcl_interfaces sensor_msgs)

add_executable(demo_rpm_sensor demo_nodes/rpm_sensor.cpp)
target_include_directories(demo_rpm_sensor PRIVATE ${_demo_include_dir})
medkit_target_dependencies(demo_rpm_sensor rclcpp std_msgs)

add_executable(demo_brake_pressure_sensor demo_nodes/brake_pressure_sensor.cpp)
target_include_directories(demo_brake_pressure_sensor PRIVATE ${_demo_include_dir})
medkit_target_dependencies(demo_brake_pressure_sensor rclcpp std_msgs)

add_executable(demo_door_status_sensor demo_nodes/door_status_sensor.cpp)
target_include_directories(demo_door_status_sensor PRIVATE ${_demo_include_dir})
medkit_target_dependencies(demo_door_status_sensor rclcpp std_msgs)

add_executable(demo_brake_actuator demo_nodes/brake_actuator.cpp)
target_include_directories(demo_brake_actuator PRIVATE ${_demo_include_dir})
medkit_target_dependencies(demo_brake_actuator rclcpp std_msgs)

add_executable(demo_light_controller demo_nodes/light_controller.cpp)
target_include_directories(demo_light_controller PRIVATE ${_demo_include_dir})
medkit_target_dependencies(demo_light_controller rclcpp std_msgs)

add_executable(demo_calibration_service demo_nodes/calibration_service.cpp)
target_include_directories(demo_calibration_service PRIVATE ${_demo_include_dir})
medkit_target_dependencies(demo_calibration_service rclcpp std_srvs)

add_executable(demo_long_calibration_action demo_nodes/long_calibration_action.cpp)
target_include_directories(demo_long_calibration_action PRIVATE ${_demo_include_dir})
medkit_target_dependencies(demo_long_calibration_action rclcpp rclcpp_action example_interfaces)

add_executable(demo_lidar_sensor demo_nodes/lidar_sensor.cpp)
target_include_directories(demo_lidar_sensor PRIVATE ${_demo_include_dir})
medkit_target_dependencies(demo_lidar_sensor rclcpp rcl_interfaces sensor_msgs std_srvs ros2_medkit_msgs)

add_executable(demo_beacon_publisher demo_nodes/beacon_publisher.cpp)
target_include_directories(demo_beacon_publisher PRIVATE ${_demo_include_dir})
medkit_target_dependencies(demo_beacon_publisher rclcpp ros2_medkit_msgs diagnostic_msgs)

add_executable(demo_param_beacon_node demo_nodes/param_beacon_node.cpp)
target_include_directories(demo_param_beacon_node PRIVATE ${_demo_include_dir})
medkit_target_dependencies(demo_param_beacon_node rclcpp)

add_executable(managed_lifecycle demo_nodes/managed_lifecycle_node.cpp)
medkit_target_dependencies(managed_lifecycle rclcpp rclcpp_lifecycle)

add_executable(demo_unresponsive_param_node demo_nodes/unresponsive_param_node.cpp)
target_include_directories(demo_unresponsive_param_node PRIVATE ${_demo_include_dir})
medkit_target_dependencies(demo_unresponsive_param_node rclcpp rcl_interfaces)

install(TARGETS
  demo_engine_temp_sensor
  demo_engine_temp_monitor
  demo_rpm_sensor
  demo_brake_pressure_sensor
  demo_door_status_sensor
  demo_brake_actuator
  demo_light_controller
  demo_calibration_service
  demo_long_calibration_action
  demo_lidar_sensor
  demo_beacon_publisher
  demo_param_beacon_node
  managed_lifecycle
  demo_unresponsive_param_node
  DESTINATION lib/${PROJECT_NAME}
)

# === Python test utilities ===

ament_python_install_package(ros2_medkit_test_utils)

# === Install launch files ===

install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)

# === Integration tests ===

# Topic-backed test plugin for the issue #584 integration test: dlopen-loaded
# by the gateway under test, so it mirrors the production plugin link model
# (MODULE, unresolved symbols provided by the host process).
add_library(topics_test_plugin MODULE test_plugins/topics_test_plugin.cpp)
medkit_target_dependencies(topics_test_plugin ros2_medkit_gateway rclcpp std_msgs)
target_link_options(topics_test_plugin PRIVATE -Wl,--unresolved-symbols=ignore-all)
set_target_properties(topics_test_plugin PROPERTIES OUTPUT_NAME "topics_test_plugin")
install(TARGETS topics_test_plugin LIBRARY DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(launch_testing_ament_cmake REQUIRED)

  # Each integration test gets a unique HTTP port via GATEWAY_TEST_PORT env var,
  # allowing parallel CTest execution without port conflicts.
  # Stride of 10 per test so multi-gateway tests can use get_test_port(offset).
  # Each test also gets a unique ROS_DOMAIN_ID to prevent DDS cross-contamination
  # between tests (e.g., stale DDS participants from previous tests leaking into
  # subsequent test's graph discovery). Multi-gateway tests (e.g. peer aggregation)
  # use get_test_domain_id(offset) for additional domains in the 230-232 range.
  set(_INTEG_PORT 9100)

  include(ROS2MedkitTestDomain)
  # Primary per-test domain pool 130-219, with 220-229 owned by opcua and
  # 230-232 reserved for the multi-gateway secondary pool (see
  # get_test_domain_id below). One domain is consumed per feature + scenario
  # test. This pool grows by glob, so it is the first to fill; when it does,
  # reclaim slack from the fixed beacon pools (see ROS2MedkitTestDomain.cmake).
  # The tests below call add_launch_test directly instead of the
  # medkit_add_launch_test wrapper because each needs a combined
  # GATEWAY_TEST_PORT + ROS_DOMAIN_ID environment, assigned inline in the loop.
  medkit_init_test_domains(START 130 END 219)

  # Feature tests (atomic, independent, 120s timeout)
  file(GLOB FEATURE_TESTS test/features/*.test.py)
  foreach(test_file ${FEATURE_TESTS})
    get_filename_component(test_name ${test_file} NAME_WE)
    add_launch_test(${test_file}
      TARGET ${test_name}
      TIMEOUT 120
    )
    set_tests_properties(${test_name} PROPERTIES
      LABELS "integration;feature"
      ENVIRONMENT "GATEWAY_TEST_PORT=${_INTEG_PORT};ROS_DOMAIN_ID=${_MEDKIT_DOMAIN_COUNTER}"
    )
    math(EXPR _MEDKIT_DOMAIN_COUNTER "${_MEDKIT_DOMAIN_COUNTER} + 1")
    math(EXPR _INTEG_PORT "${_INTEG_PORT} + 10")
  endforeach()

  # Scenario tests (E2E stories, 300s timeout)
  file(GLOB SCENARIO_TESTS test/scenarios/*.test.py)
  foreach(test_file ${SCENARIO_TESTS})
    get_filename_component(test_name ${test_file} NAME_WE)
    add_launch_test(${test_file}
      TARGET ${test_name}
      TIMEOUT 300
    )
    set_tests_properties(${test_name} PROPERTIES
      LABELS "integration;scenario"
      ENVIRONMENT "GATEWAY_TEST_PORT=${_INTEG_PORT};ROS_DOMAIN_ID=${_MEDKIT_DOMAIN_COUNTER}"
    )
    math(EXPR _MEDKIT_DOMAIN_COUNTER "${_MEDKIT_DOMAIN_COUNTER} + 1")
    math(EXPR _INTEG_PORT "${_INTEG_PORT} + 10")
  endforeach()

  # Verify domain range not exhausted
  if(_MEDKIT_DOMAIN_COUNTER GREATER _MEDKIT_DOMAIN_END)
    message(FATAL_ERROR "Integration test domain range exhausted (${_MEDKIT_DOMAIN_START}-${_MEDKIT_DOMAIN_END})")
  endif()

  # Serialise multi-gateway tests that share the secondary DDS domain pool
  # (230-232, see ros2_medkit_test_utils.constants.get_test_domain_id). CTest
  # will not run two tests holding this resource lock concurrently, so they
  # cannot collide on those shared domains. Every integration test that
  # calls get_test_domain_id(offset>0) must be listed here.
  set_tests_properties(
    test_peer_aggregation
    test_cross_ecu_fanout
    test_daisy_chain_aggregation
    test_leaf_collision_aggregation
    PROPERTIES RESOURCE_LOCK medkit_secondary_dds_domains
  )

  # test_graph_provider_greenwave drives a real greenwave_monitor node,
  # which only publishes /diagnostics at ~1 Hz with the first useful
  # reading a few seconds in, plus an 8s launch-ordering delay so
  # greenwave starts strictly after the demo nodes it monitors are already
  # publishing (see the test's module docstring). The default 120s feature
  # timeout does not leave enough headroom for that startup delay plus the
  # 30-90s polling budgets its cases need, so widen it to match the
  # scenario-test timeout.
  set_tests_properties(test_graph_provider_greenwave PROPERTIES TIMEOUT 300)

  # test_graph_provider_stale and test_graph_provider_sse also drive a real
  # greenwave_monitor node through the same 10s launch-ordering delay plus
  # warmup as test_graph_provider_greenwave above, and their staleness/SSE
  # polling budgets are similarly generous - widen both to match.
  set_tests_properties(test_graph_provider_stale PROPERTIES TIMEOUT 300)
  set_tests_properties(test_graph_provider_sse PROPERTIES TIMEOUT 300)
endif()

ament_package()
