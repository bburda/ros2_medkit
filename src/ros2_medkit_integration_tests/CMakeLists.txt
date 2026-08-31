cmake_minimum_required(VERSION 3.14)
project(ros2_medkit_integration_tests)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)

# Shared cmake modules (multi-distro compat, ccache, linting)
find_package(ros2_medkit_cmake REQUIRED)
include(ROS2MedkitCompat)
include(ROS2MedkitLinting)
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
find_package(lifecycle_msgs REQUIRED)
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
medkit_target_dependencies(demo_calibration_service rclcpp rcl_interfaces std_srvs)

add_executable(demo_slow_calibration_service demo_nodes/slow_calibration_service.cpp)
target_include_directories(demo_slow_calibration_service PRIVATE ${_demo_include_dir})
medkit_target_dependencies(demo_slow_calibration_service rclcpp rcl_interfaces std_srvs)

add_executable(demo_dual_calibration_service demo_nodes/dual_calibration_service.cpp)
target_include_directories(demo_dual_calibration_service PRIVATE ${_demo_include_dir})
medkit_target_dependencies(demo_dual_calibration_service rclcpp rclcpp_action example_interfaces std_srvs)

add_executable(demo_long_calibration_action demo_nodes/long_calibration_action.cpp)
target_include_directories(demo_long_calibration_action PRIVATE ${_demo_include_dir})
medkit_target_dependencies(demo_long_calibration_action rclcpp rclcpp_action example_interfaces)

add_executable(demo_shared_leaf demo_nodes/shared_leaf.cpp)
target_include_directories(demo_shared_leaf PRIVATE ${_demo_include_dir})
medkit_target_dependencies(demo_shared_leaf rclcpp std_msgs std_srvs)

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

# Proof fixture for GRAPH_NODE_UNREADABLE (see the file doc): a plain rclcpp::Node that
# advertises get_state/change_state like managed_lifecycle above, but whose get_state
# never answers until its start_answering parameter is flipped at runtime.
add_executable(unreadable_lifecycle demo_nodes/unreadable_lifecycle_node.cpp)
target_include_directories(unreadable_lifecycle PRIVATE ${_demo_include_dir})
medkit_target_dependencies(unreadable_lifecycle rclcpp lifecycle_msgs)

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
  demo_slow_calibration_service
  demo_dual_calibration_service
  demo_long_calibration_action
  demo_shared_leaf
  demo_lidar_sensor
  demo_beacon_publisher
  demo_param_beacon_node
  managed_lifecycle
  unreadable_lifecycle
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

  # Lint the Python that makes up this package. Without this the whole package
  # sits outside `colcon test -L linter`, so the documented lint command passes
  # while none of these files are checked, and the pre-commit hooks are the
  # first thing to see a problem.
  #
  # Both gates read the same .flake8 (see CONFIG_FILE below) and the hook
  # installs the same plugins ament_flake8 depends on, though CI installs
  # those from apt while the hook's additional_dependencies are unpinned and
  # resolve to whatever is current on PyPI - plugin coverage matches, exact
  # plugin versions are not guaranteed to. The hook additionally runs
  # flake8-blind-except and flake8-class-newline, which ament_flake8 does not
  # depend on - it is a superset, not a match.
  #
  # Only flake8 for now. ament_pep257 also belongs here - every package in
  # the tree runs it except ros2_medkit_graph_watchdog, which excludes
  # pep257 - but this package drifted while it was unchecked: 559
  # multi-line docstrings put the summary on the opening line, which the
  # convention the rest of the tree follows does not. Adding the checker and
  # correcting those docstrings is its own change.
  #
  # ament_lint_common is deliberately not used either: it would also apply the
  # C++ style checks to the demo nodes, which is a separate decision.
  find_package(ament_cmake_flake8 REQUIRED)
  # Without CONFIG_FILE, ament_flake8 reads the ini bundled inside the
  # ament_flake8 package and the shared .flake8 never applies here. The two
  # would then be hand-synced copies that drift in silence. Point it at the
  # shared file so there is one configuration.
  medkit_lint_config(.flake8 _flake8_config)
  ament_flake8(CONFIG_FILE "${_flake8_config}")

  # Each integration test gets a unique HTTP port via GATEWAY_TEST_PORT env var,
  # allowing parallel CTest execution without port conflicts.
  # Stride of 10 per test so multi-gateway tests can use get_test_port(offset).
  set(_INTEG_PORT 9100)

  include(ROS2MedkitTestDomain)

  # The shared test helpers, checked without a gateway. Declared ROS-free: it
  # manipulates ROS_DOMAIN_ID itself to prove the refusal, so a domain of its
  # own would be the thing under test rather than a fixture.
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_test_utils_constants test/utils TIMEOUT 60)
  set_tests_properties(test_test_utils_constants PROPERTIES LABELS "unit")
  medkit_test_needs_no_domain(test_test_utils_constants)

  # The crash reporter every demo binary installs. Death tests, because the
  # behaviour under test only exists in a process that is being killed - a
  # handler that is merely installed proves nothing about what a crash leaves
  # behind. ROS-free: it creates no node, only signals.
  find_package(ament_cmake_gmock REQUIRED)
  medkit_add_gmock(test_crash_backtrace test/test_crash_backtrace.cpp)
  target_include_directories(test_crash_backtrace PRIVATE ${_demo_include_dir})
  medkit_test_needs_no_domain(test_crash_backtrace)

  # Configures every launch-test-registering package this file can reach for
  # real, twice each (ordinary workspace conditions and system-package
  # conditions), and compares what each run actually registered. Lives here
  # rather than in ros2_medkit_cmake because that package cannot depend on
  # the packages it is shared by; this package's own dependency chain (via
  # ros2_medkit_gateway) already reaches all six. ros2_medkit_graph_watchdog
  # calls medkit_add_launch_test too but is covered by its own copy of this
  # test instead - see the module docstring. ROS-free for the same reason as
  # the fixture above - it spawns its own cmake subprocesses rather than
  # creating ROS entities.
  #
  # Registered only when this package can reach a repository root above
  # itself - reusing _medkit_launch_tests_can_register()'s own detection of a
  # system-package build (ROS2MedkitTestDomain.cmake), because the two
  # conditions are the same fact: the ROS build farm's binarydeb chroot
  # builds this package alone, with no sibling packages beside it, which is
  # exactly when this test's own repository_root fixture would find nothing.
  # A test that can prove nothing about its own subject must not run to
  # report a pass on that account; every other test in this file registers
  # unconditionally.
  _medkit_launch_tests_can_register(_malt_system_build_test_can_register)
  if(_malt_system_build_test_can_register)
    ament_add_pytest_test(test_launch_test_guard_system_build
      test/test_launch_test_guard_system_build.py TIMEOUT 900)
    set_tests_properties(test_launch_test_guard_system_build PROPERTIES LABELS "integration")
    medkit_test_needs_no_domain(test_launch_test_guard_system_build)
  endif()

  # Multi-gateway tests run a second and a third gateway, each on a domain of its
  # own, and reach them through ros2_medkit_test_utils.constants.get_test_domain_id.
  # DOMAINS below is one for the test itself plus one per extra gateway; the
  # extras arrive as MEDKIT_SECONDARY_DOMAINS. The highest offset any of them
  # asks for is 3, so four domains covers all of them.
  # test_startup_param_clamp_warnings is not an aggregation test, but it does
  # launch four gateways. They have no reason to see each other, and sharing one
  # domain makes each of them discover the other three's nodes and leaves four
  # sets of participants dying on that domain at teardown.
  set(_MULTI_GATEWAY_TESTS
    test_peer_aggregation
    test_cross_ecu_fanout
    test_daisy_chain_aggregation
    test_grouping_entity_aggregation
    test_aggregate_lock_identity
    test_aggregator_only_configurations
    test_leaf_collision_aggregation
    test_startup_param_clamp_warnings
    test_aggregation_time_budgets
    test_peer_failure_reasons
    test_relay_peer_credential)
  set(_MULTI_GATEWAY_DOMAINS 4)

  # Two-gateway tests that need the second gateway isolated from the first and
  # ask for offset 1 only. Listed apart from the four-domain set above because
  # a test holds every domain it is given for as long as it runs: asking for
  # four where two are used takes two away from whatever the runner could
  # otherwise have started alongside it.
  set(_TWO_GATEWAY_TESTS
    test_peer_recovery
    test_triggers_restore_before_discovery
    test_aggregator_fault_stream)
  set(_TWO_GATEWAY_DOMAINS 2)

  # Tests whose polling budgets do not fit the glob's default timeout. Keyed by
  # test name; anything not listed takes the default for its glob.
  #
  # test_rosbag_boundary_download drives three real rosbag recordings end to end
  # (the demo node's own fault, then a burst across the post-fault window
  # boundary), each with its own post-roll and finalize. Its polling deadlines
  # add up past the 120s the feature glob assigns, so it gets its own budget
  # rather than have a slow runner kill it with no test name in the output.
  #
  # test_graph_provider_greenwave drives a real greenwave_monitor node, which
  # only publishes /diagnostics at ~1 Hz with the first useful reading a few
  # seconds in, plus an 8s launch-ordering delay so greenwave starts strictly
  # after the demo nodes it monitors are already publishing (see the test's
  # module docstring). The default 120s feature timeout does not leave enough
  # headroom for that startup delay plus the 30-90s polling budgets its cases
  # need, so it is widened to match the scenario-test timeout.
  #
  # test_graph_provider_stale and test_graph_provider_sse also drive a real
  # greenwave_monitor node through the same 10s launch-ordering delay plus
  # warmup as test_graph_provider_greenwave above, and their staleness/SSE
  # polling budgets are similarly generous - both widened to match.
  # test_rosbag_history_download drives two full occurrences of one real fault -
  # break the range, wait for the confirmation, wait for the post-roll to finalise
  # into a listed descriptor, repair, acknowledge - and then downloads every bag.
  # Two of those cycles do not fit the feature glob's 120s.
  #
  # test_triggers_restore_before_discovery keeps its entity provably absent for
  # longer than the 60s budget a trigger subscription attempt gets, because a
  # shorter window is one a gateway that abandons the attempt would also pass.
  # That window plus two gateway startups and the SSE wait after it exceed the
  # 120s the feature glob assigns, so it gets its own budget rather than have
  # the file killed with no test name in the output.
  #
  # test_aggregation_time_budgets holds a peer-owned operation open for four
  # seconds twice on purpose - a budget cannot be measured against a service
  # that answers instantly - and reads a lidar scan large enough to break the
  # tight budget three times over. Three gateway startups on top of that do not
  # fit the feature glob's 120s.
  #
  # test_peer_failure_reasons waits out the peer's own six-second parameter
  # round trip on the patient aggregator, twice, and then polls after killing
  # the peer. Two of those plus three gateway startups exceed the default.
  #
  # test_aggregator_fault_stream spends most of its budget waiting rather than
  # working: a closed SSE connection is noticed at the peer's next keepalive,
  # 30s away, and the test settles the peer's occupancy before each measurement
  # so an earlier case's closed stream is not attributed to the relay. That is
  # deliberate - a shorter wait would pass for a gateway that never released
  # the slot at all.
  #
  # test_auth_policy_contract launches three full-feature gateways, one per
  # auth.require_auth_for value, and sends a real unauthenticated request at
  # every operation each of them documents. That is the measurement - the
  # published requirement is compared against what the middleware actually
  # does - and it costs three startups plus several hundred requests, two of
  # which wait out a fault-service timeout. It does not fit the 120s the
  # feature glob assigns.
  set(_MEDKIT_TEST_TIMEOUT_OVERRIDES
    test_aggregation_time_budgets 300
    test_peer_failure_reasons 300
    test_aggregator_fault_stream 420
    test_auth_policy_contract 300
    test_rosbag_boundary_download 180
    test_rosbag_history_download 180
    test_grouping_entity_aggregation 300
    test_graph_provider_greenwave 300
    test_graph_provider_stale 300
    test_graph_provider_sse 300
    test_peer_recovery 300
    test_triggers_restore_before_discovery 300
    # Three discovery budgets in setUpClass before the first case runs, and a
    # broken merge spends all of them. The glob default cuts that short and
    # reports a timeout with no test name, which is the one answer that says
    # nothing about which rule broke.
    test_aggregator_only_configurations 300)
  # Names actually matched against a discovered test_name in the two loops
  # below. Checked against _MEDKIT_TEST_TIMEOUT_OVERRIDES itself after both
  # loops finish - see the FATAL_ERROR check at the end of this block - so a
  # renamed or deleted test file leaves a stale entry here that fails the
  # configure instead of silently reverting that test's budget to the glob
  # default.
  unset(_MEDKIT_TEST_TIMEOUT_OVERRIDES_SEEN)

  # Feature tests (atomic, independent, 120s timeout unless overridden above)
  # CONFIGURE_DEPENDS is load-bearing, not tidiness: a glob is expanded at
  # configure time, so without it a test file added after the build tree was
  # configured is never registered. The suite then reports a clean pass while
  # silently not running it, and the absence of a whole test file looks
  # exactly like success.
  file(GLOB FEATURE_TESTS CONFIGURE_DEPENDS test/features/*.test.py)
  foreach(test_file ${FEATURE_TESTS})
    get_filename_component(test_name ${test_file} NAME_WE)
    set(_test_domains 1)
    if(test_name IN_LIST _MULTI_GATEWAY_TESTS)
      set(_test_domains ${_MULTI_GATEWAY_DOMAINS})
    elseif(test_name IN_LIST _TWO_GATEWAY_TESTS)
      set(_test_domains ${_TWO_GATEWAY_DOMAINS})
    endif()
    set(_test_timeout 120)
    list(FIND _MEDKIT_TEST_TIMEOUT_OVERRIDES ${test_name} _test_timeout_idx)
    if(NOT _test_timeout_idx EQUAL -1)
      math(EXPR _test_timeout_value_idx "${_test_timeout_idx} + 1")
      list(GET _MEDKIT_TEST_TIMEOUT_OVERRIDES ${_test_timeout_value_idx} _test_timeout)
      list(APPEND _MEDKIT_TEST_TIMEOUT_OVERRIDES_SEEN ${test_name})
    endif()
    medkit_add_launch_test(${test_name} ${test_file}
      TIMEOUT ${_test_timeout}
      DOMAINS ${_test_domains}
      LABELS "integration;feature"
      ENV "GATEWAY_TEST_PORT=${_INTEG_PORT}"
    )
    math(EXPR _INTEG_PORT "${_INTEG_PORT} + 10")
  endforeach()

  # Scenario tests (E2E stories, 300s timeout unless overridden above)
  # CONFIGURE_DEPENDS is load-bearing, not tidiness: a glob is expanded at
  # configure time, so without it a test file added after the build tree was
  # configured is never registered. The suite then reports a clean pass while
  # silently not running it, and the absence of a whole test file looks
  # exactly like success.
  file(GLOB SCENARIO_TESTS CONFIGURE_DEPENDS test/scenarios/*.test.py)
  foreach(test_file ${SCENARIO_TESTS})
    get_filename_component(test_name ${test_file} NAME_WE)
    set(_test_domains 1)
    if(test_name IN_LIST _MULTI_GATEWAY_TESTS)
      set(_test_domains ${_MULTI_GATEWAY_DOMAINS})
    elseif(test_name IN_LIST _TWO_GATEWAY_TESTS)
      set(_test_domains ${_TWO_GATEWAY_DOMAINS})
    endif()
    set(_test_timeout 300)
    list(FIND _MEDKIT_TEST_TIMEOUT_OVERRIDES ${test_name} _test_timeout_idx)
    if(NOT _test_timeout_idx EQUAL -1)
      math(EXPR _test_timeout_value_idx "${_test_timeout_idx} + 1")
      list(GET _MEDKIT_TEST_TIMEOUT_OVERRIDES ${_test_timeout_value_idx} _test_timeout)
      list(APPEND _MEDKIT_TEST_TIMEOUT_OVERRIDES_SEEN ${test_name})
    endif()
    medkit_add_launch_test(${test_name} ${test_file}
      TIMEOUT ${_test_timeout}
      DOMAINS ${_test_domains}
      LABELS "integration;scenario"
      ENV "GATEWAY_TEST_PORT=${_INTEG_PORT}"
    )
    math(EXPR _INTEG_PORT "${_INTEG_PORT} + 10")
  endforeach()

  # A stale entry in _MEDKIT_TEST_TIMEOUT_OVERRIDES used to be a silent
  # no-op: list(FIND ...) above returns -1 for a name that matches no
  # discovered test_name, the if() is skipped, and the test simply keeps
  # whichever glob default it already had - nothing in either loop fails,
  # and a test's budget quietly reverts from (say) 300 to 120 with no line
  # in the configure log to say so. Every even-indexed name in the override
  # list must therefore have matched some test_name in one of the two loops
  # above, or this fails the configure instead of a test silently losing
  # its budget.
  list(LENGTH _MEDKIT_TEST_TIMEOUT_OVERRIDES _medkit_timeout_overrides_length)
  if(_medkit_timeout_overrides_length GREATER 0)
    math(EXPR _medkit_timeout_overrides_last_index "${_medkit_timeout_overrides_length} - 1")
    foreach(_medkit_timeout_override_idx RANGE 0 ${_medkit_timeout_overrides_last_index} 2)
      list(GET _MEDKIT_TEST_TIMEOUT_OVERRIDES ${_medkit_timeout_override_idx}
        _medkit_timeout_override_name)
      if(NOT _medkit_timeout_override_name IN_LIST _MEDKIT_TEST_TIMEOUT_OVERRIDES_SEEN)
        message(FATAL_ERROR
          "ros2_medkit_integration_tests: _MEDKIT_TEST_TIMEOUT_OVERRIDES names "
          "'${_medkit_timeout_override_name}', which matched no test_name discovered "
          "under test/features/*.test.py or test/scenarios/*.test.py. A stale entry "
          "here used to be a silent no-op: the test it once named would quietly keep "
          "the glob's default timeout instead of the override. Remove the entry, or "
          "fix the name to match the renamed or moved test file.")
      endif()
    endforeach()
  endif()

  # gtest/gmock vendor sources are built as subdirectory targets and inherit
  # this package's promoted warnings, which they do not compile clean under.
  ros2_medkit_relax_vendor_warnings()
endif()

ament_package()
