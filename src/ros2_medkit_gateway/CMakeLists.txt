cmake_minimum_required(VERSION 3.8)
project(ros2_medkit_gateway)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Shared cmake modules (multi-distro compat, ccache, linting)
find_package(ros2_medkit_cmake REQUIRED)
include(ROS2MedkitCompat)
include(ROS2MedkitCcache)
include(ROS2MedkitSanitizers)
include(ROS2MedkitCoverage)
include(ROS2MedkitLinting)
include(ROS2MedkitWarnings)

# Swagger UI embedding option
option(ENABLE_SWAGGER_UI "Embed Swagger UI assets in gateway binary" OFF)

# Extract version from package.xml so version.hpp stays in sync automatically
file(READ "${CMAKE_CURRENT_SOURCE_DIR}/package.xml" _package_xml)
string(REGEX MATCH "<version>([0-9]+\\.[0-9]+\\.[0-9]+)</version>" _ "${_package_xml}")
set(GATEWAY_VERSION "${CMAKE_MATCH_1}")

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
medkit_find_yaml_cpp()
find_package(ament_index_cpp REQUIRED)
find_package(action_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(ros2_medkit_msgs REQUIRED)
find_package(ros2_medkit_serialization REQUIRED)
find_package(SQLite3 REQUIRED)

# GenericClient compat shim dependencies (needed explicitly for Humble,
# transitively available on Iron+/Jazzy but harmless to declare unconditionally)
find_package(rosidl_typesupport_cpp REQUIRED)
find_package(rosidl_typesupport_introspection_cpp REQUIRED)

medkit_find_cpp_httplib(VENDORED_DIR "${CMAKE_CURRENT_SOURCE_DIR}/src/vendored/cpp_httplib")

# Find OpenSSL (required by jwt-cpp for RS256 and optional TLS support)
find_package(OpenSSL REQUIRED)

# Enable OpenSSL support for cpp-httplib SSLServer
# This enables the httplib::SSLServer class for HTTPS support
add_compile_definitions(CPPHTTPLIB_OPENSSL_SUPPORT)

# Emitted-status recorder (include/ros2_medkit_gateway/http/detail/status_recorder.hpp).
# Instruments make_error() and every registry-mounted route so
# test_openapi_error_coverage can assert the OpenAPI document declares every
# status the gateway actually serves. Directory scope and set before any
# target is created, because make_error() is an inline function shared by 13
# translation units - a per-target definition would give it two bodies and
# break ODR. A shipped gateway compiles none of it: the Dockerfile builds with
# -DBUILD_TESTING=OFF.
if(BUILD_TESTING)
  add_compile_definitions(MEDKIT_STATUS_RECORDER)
endif()

# Vendored header-only libraries (no network access on ROS build farm)
# tl::expected v1.3.1 (CC0) - https://github.com/TartanLlama/expected
add_library(tl_expected_iface INTERFACE)
target_include_directories(tl_expected_iface SYSTEM INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/vendored/tl_expected/include>
  $<INSTALL_INTERFACE:include/ros2_medkit_gateway/vendored>
)
add_library(tl::expected ALIAS tl_expected_iface)

# jwt-cpp v0.7.0 (MIT) - https://github.com/Thalhammer/jwt-cpp
add_library(jwt_cpp_iface INTERFACE)
target_include_directories(jwt_cpp_iface SYSTEM INTERFACE
  ${CMAKE_CURRENT_SOURCE_DIR}/src/vendored/jwt_cpp/include
)
add_library(jwt-cpp::jwt-cpp ALIAS jwt_cpp_iface)

# mjansson/mdns (Public Domain) - https://github.com/mjansson/mdns
# Header-only C library for mDNS/DNS-SD, used for peer gateway discovery
include_directories(SYSTEM src/vendored/mdns)

# Include directories
include_directories(include)

# ─── Neutral core static library ───────────────────────────────────────────
# Middleware-neutral business logic. No ROS dependencies. Compiled from
# sources under src/core/ which were relocated from the flat src/ layout
# during the layered-architecture refactor.
file(GLOB_RECURSE GATEWAY_CORE_SOURCES
  CONFIGURE_DEPENDS
  "src/core/*.cpp"
)

add_library(gateway_core STATIC ${GATEWAY_CORE_SOURCES})

target_include_directories(gateway_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  PRIVATE
    # Private implementation headers for the openapi module (not installed)
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/openapi>
)

target_link_libraries(gateway_core
  PUBLIC
    cpp_httplib_target
    nlohmann_json::nlohmann_json
    yaml-cpp::yaml-cpp
    OpenSSL::SSL
    OpenSSL::Crypto
    tl::expected
    jwt-cpp::jwt-cpp
    SQLite::SQLite3
    ${CMAKE_DL_LIBS}
)

set_target_properties(gateway_core PROPERTIES
  POSITION_INDEPENDENT_CODE ON
  CXX_STANDARD 17
  CXX_STANDARD_REQUIRED ON
  CXX_EXTENSIONS OFF
)

# Pass version from package.xml to version.hpp
target_compile_definitions(gateway_core PUBLIC GATEWAY_VERSION_STRING="${GATEWAY_VERSION}")

# Core PCH - heavy headers used across the neutral layer, no ROS includes.
# Speeds up full builds by ~30-40%. Disable with -DCMAKE_DISABLE_PRECOMPILE_HEADERS=ON.
target_precompile_headers(gateway_core PRIVATE
  <string>
  <vector>
  <memory>
  <optional>
  <functional>
  <unordered_map>
  <algorithm>
  <sstream>
  <chrono>
  <mutex>
  <nlohmann/json.hpp>
  <httplib.h>
  <tl/expected.hpp>
)

# ─── ROS adapter static library ────────────────────────────────────────────
# Files that depend on rclcpp / rcl_interfaces / message-package interfaces.
# Layered atop gateway_core via PUBLIC link so the executable and tests
# transitively get both layers from a single dependency.
add_library(gateway_ros2 STATIC
  src/aggregation/aggregation_manager.cpp
  src/data/ros2_topic_data_provider.cpp
  src/ros2/conversions/fault_msg_conversions.cpp
  src/ros2/providers/ros2_runtime_introspection.cpp
  src/ros2/transports/ros2_action_transport.cpp
  src/ros2/transports/ros2_fault_service_transport.cpp
  src/ros2/transports/ros2_log_source.cpp
  src/ros2/transports/ros2_parameter_transport.cpp
  src/ros2/transports/ros2_service_transport.cpp
  src/ros2/transports/ros2_topic_subscription_transport.cpp
  src/ros2/transports/ros2_topic_transport.cpp
  src/ros2/status/ros2_lifecycle_state_reader.cpp
  src/ros2/trigger_topic_subscriber.cpp
  src/default_script_provider.cpp
  src/discovery/discovery_manager.cpp
  src/discovery/layers/manifest_layer.cpp
  src/discovery/layers/plugin_layer.cpp
  src/discovery/layers/runtime_layer.cpp
  src/discovery/manifest/manifest_manager.cpp
  src/discovery/manifest/manifest_parser.cpp
  src/discovery/manifest/runtime_linker.cpp
  src/discovery/merge_pipeline.cpp
  src/entity_freeze_frame_capture.cpp
  src/fault_manager_paths.cpp
  src/gateway_node.cpp
  src/http/fan_out_helpers.cpp
  src/http/handlers/auth_handlers.cpp
  src/http/handlers/bulkdata_handlers.cpp
  src/http/handlers/config_handlers.cpp
  src/http/handlers/cyclic_subscription_handlers.cpp
  src/http/handlers/data_handlers.cpp
  src/http/handlers/discovery_handlers.cpp
  src/http/handlers/docs_handlers.cpp
  src/http/handlers/fault_handlers.cpp
  src/http/handlers/handler_context.cpp
  src/http/handlers/health_handlers.cpp
  src/http/handlers/lifecycle_handlers.cpp
  src/http/handlers/lock_handlers.cpp
  src/http/handlers/log_handlers.cpp
  src/http/handlers/operation_handlers.cpp
  src/http/handlers/script_handlers.cpp
  src/http/handlers/sse_fault_handler.cpp
  src/http/handlers/sse_transport_provider.cpp
  src/http/handlers/trigger_handlers.cpp
  src/http/handlers/update_handlers.cpp
  src/http/http_server.cpp
  src/http/rest_server.cpp
  src/openapi/capability_generator.cpp
  src/openapi/openapi_spec_builder.cpp
  src/openapi/path_builder.cpp
  src/openapi/schema_builder.cpp
  src/plugins/plugin_context.cpp
  src/plugins/plugin_http_types.cpp
  src/plugins/plugin_loader.cpp
  src/plugins/plugin_manager.cpp
  src/ros2_common/callback_groups.cpp
  src/ros2_common/ros2_subscription_executor.cpp
  src/ros2_common/ros2_subscription_slot.cpp
  src/script_manager.cpp
  src/trigger_fault_subscriber.cpp
)

# Private implementation headers for openapi module (shared by both layers)
target_include_directories(gateway_ros2 PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/openapi>
)

# Link gateway_core first using plain signature (before medkit_target_dependencies
# calls ament_target_dependencies which also uses the plain signature). This avoids
# the CMake "mixed plain/keyword target_link_libraries" error.
target_link_libraries(gateway_ros2 gateway_core)

medkit_target_dependencies(gateway_ros2
  rclcpp
  std_msgs
  std_srvs
  rcl_interfaces
  ament_index_cpp
  action_msgs
  lifecycle_msgs
  ros2_medkit_msgs
  ros2_medkit_serialization
  rosidl_typesupport_cpp
  rosidl_typesupport_introspection_cpp
)

set_target_properties(gateway_ros2 PROPERTIES
  POSITION_INDEPENDENT_CODE ON
  CXX_STANDARD 17
  CXX_STANDARD_REQUIRED ON
)

# ROS adapter PCH - extends core PCH with rclcpp on top.
target_precompile_headers(gateway_ros2 PRIVATE
  <rclcpp/rclcpp.hpp>
)

# Gateway node executable
add_executable(gateway_node src/main.cpp)
target_link_libraries(gateway_node gateway_ros2)
# Export symbols to dlopen'd plugins (PluginContext::send_json, send_error, etc.)
set_target_properties(gateway_node PROPERTIES ENABLE_EXPORTS ON)

# Apply Swagger UI compile definition and embed assets
if(ENABLE_SWAGGER_UI)
  target_compile_definitions(gateway_ros2 PRIVATE ENABLE_SWAGGER_UI)
  target_compile_definitions(gateway_node PRIVATE ENABLE_SWAGGER_UI)
  include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/SwaggerUI.cmake)
  target_compile_definitions(gateway_ros2 PRIVATE SWAGGER_UI_VERSION="${SWAGGER_UI_VERSION}")
endif()

# Install
install(TARGETS gateway_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

# Install scripts with execute permissions explicitly set
install(PROGRAMS scripts/get_type_schema.py scripts/generate_dev_certs.sh
  DESTINATION share/${PROJECT_NAME}/scripts
)

# Testing
if(BUILD_TESTING)
  # Linting and code quality checks
  find_package(ament_lint_auto REQUIRED)
  # Use the shared clang-format config that ships with ros2_medkit_cmake
  medkit_lint_config(.clang-format ament_cmake_clang_format_CONFIG_FILE)

  # Exclude linters that don't work well with vendored dependencies:
  # - uncrustify/cpplint: conflicts with clang-format
  # - copyright/clang_format: we configure manually to skip src/vendored/
  # - clang_tidy: we configure it manually below with increased timeout
  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_uncrustify
    ament_cmake_cpplint
    ament_cmake_clang_tidy
    ament_cmake_copyright
    ament_cmake_clang_format
  )
  ament_lint_auto_find_test_dependencies()

  # Configure copyright check to exclude vendored dependencies
  find_package(ament_cmake_copyright REQUIRED)
  set(VENDORED_FILES
    "src/vendored/jwt_cpp/include/jwt-cpp/base.h"
    "src/vendored/jwt_cpp/include/jwt-cpp/jwt.h"
    "src/vendored/jwt_cpp/include/jwt-cpp/traits/boost-json/defaults.h"
    "src/vendored/jwt_cpp/include/jwt-cpp/traits/boost-json/traits.h"
    "src/vendored/jwt_cpp/include/jwt-cpp/traits/danielaparker-jsoncons/defaults.h"
    "src/vendored/jwt_cpp/include/jwt-cpp/traits/danielaparker-jsoncons/traits.h"
    "src/vendored/jwt_cpp/include/jwt-cpp/traits/kazuho-picojson/defaults.h"
    "src/vendored/jwt_cpp/include/jwt-cpp/traits/kazuho-picojson/traits.h"
    "src/vendored/jwt_cpp/include/jwt-cpp/traits/nlohmann-json/defaults.h"
    "src/vendored/jwt_cpp/include/jwt-cpp/traits/nlohmann-json/traits.h"
    "src/vendored/jwt_cpp/include/picojson/picojson.h"
    "src/vendored/tl_expected/include/tl/expected.hpp"
    "src/vendored/mdns/mdns.h"
    "src/vendored/cpp_httplib/httplib.h"
  )
  ament_copyright(EXCLUDE ${VENDORED_FILES})

  # Configure clang-format to only check our source files (not vendored).
  # ament_cmake_clang_format is intentionally only installed in the Jazzy
  # quality job (ci.yml skips its rosdep key in the build-and-test job), so use
  # QUIET + FOUND check instead of REQUIRED to gracefully degrade on the
  # build-and-test runners without breaking the build.
  find_package(ament_cmake_clang_format QUIET)
  if(ament_cmake_clang_format_FOUND)
    file(GLOB_RECURSE _format_files
      "include/*.h" "include/*.hpp"
      "src/*.cpp" "src/*.h" "src/*.hpp"
      "test/*.cpp" "test/*.h" "test/*.hpp"
    )
    list(FILTER _format_files EXCLUDE REGEX ".*/vendored/.*")
    ament_clang_format(${_format_files}
      CONFIG_FILE "${ament_cmake_clang_format_CONFIG_FILE}"
    )
  endif()

  ros2_medkit_clang_tidy(
    HEADER_FILTER "^${CMAKE_CURRENT_SOURCE_DIR}/(include|src|test)/"
    TIMEOUT 3000
  )

  include(ROS2MedkitTestDomain)

  # ─── Core purity check ────────────────────────────────────────────────────
  # Fails the build if any file under include/ros2_medkit_gateway/core/ or
  # src/core/ directly includes a ROS header. The neutral layer's contract
  # is enforced both by the link-time smoke test (test_gateway_core_smoke)
  # and by this grep-based scan that catches misuse before linkage.
  add_test(
    NAME gateway_core_purity
    COMMAND bash "${CMAKE_CURRENT_SOURCE_DIR}/scripts/check_core_purity.sh"
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
  )
  set_tests_properties(gateway_core_purity PROPERTIES LABELS "linter")

  add_test(
    NAME gateway_plugin_header_purity
    COMMAND bash "${CMAKE_CURRENT_SOURCE_DIR}/scripts/check_headers_httplib_free.sh"
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
  )
  set_tests_properties(gateway_plugin_header_purity PROPERTIES LABELS "linter")

  # ─── Error-code documentation coverage ────────────────────────────────────
  # Fails if any ERR_* with a non-test emitter is missing from the REST
  # guide's error table. The table carries completeness claims that nothing
  # else checks, and prose is the tier this branch found least durable.
  #
  # Interpreter found explicitly rather than relying on Python3_EXECUTABLE
  # being set: it is a cache variable our build passes on the command line, so
  # a configure without it would silently register an empty COMMAND.
  find_package(Python3 COMPONENTS Interpreter REQUIRED)
  add_test(
    NAME gateway_error_codes_documented
    COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/scripts/check_error_codes_documented.py"
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
  )
  set_tests_properties(gateway_error_codes_documented PROPERTIES LABELS "linter")

  # ─── gateway_core link-time smoke test ────────────────────────────────────
  # Compiles a translation unit including a sampling of core/ headers and
  # links exclusively against gateway_core + GTest. No ament_target_dependencies
  # are applied, so any rclcpp transitive include reaching this TU surfaces
  # as a missing-symbol link error.
  find_package(GTest REQUIRED)
  add_executable(test_gateway_core_smoke test/test_gateway_core_smoke.cpp)
  target_link_libraries(test_gateway_core_smoke
    PRIVATE
      gateway_core
      GTest::gtest
      GTest::gtest_main
  )
  add_test(
    NAME gateway_core_smoke
    COMMAND $<TARGET_FILE:test_gateway_core_smoke>
  )
  set_tests_properties(gateway_core_smoke PROPERTIES LABELS "unit")
  # gateway_core only, no rclcpp on the link line, so no ROS entity to isolate.
  medkit_test_needs_no_domain(gateway_core_smoke)

  # ─── DataAccessManager routing test (gateway_core only) ────────────────────
  # Mock-transport coverage proving the manager body compiles + links against
  # gateway_core alone, with no rclcpp on the link line.
  add_executable(test_data_access_manager_routing test/test_data_access_manager_routing.cpp)
  target_link_libraries(test_data_access_manager_routing
    PRIVATE
      gateway_core
      GTest::gtest
      GTest::gtest_main
  )
  add_test(
    NAME data_access_manager_routing
    COMMAND $<TARGET_FILE:test_data_access_manager_routing>
  )
  set_tests_properties(data_access_manager_routing PROPERTIES LABELS "unit")
  # gateway_core only, no rclcpp on the link line, so no ROS entity to isolate.
  medkit_test_needs_no_domain(data_access_manager_routing)

  # ─── OperationManager routing test (gateway_core only) ─────────────────────
  # Mock-transport coverage proving the manager body (service / action call
  # routing, goal tracking, status callback wiring, cleanup) compiles and
  # links against gateway_core alone, with no rclcpp on the link line.
  add_executable(test_operation_manager_routing test/test_operation_manager_routing.cpp)
  target_link_libraries(test_operation_manager_routing
    PRIVATE
      gateway_core
      GTest::gtest
      GTest::gtest_main
  )
  add_test(
    NAME operation_manager_routing
    COMMAND $<TARGET_FILE:test_operation_manager_routing>
  )
  set_tests_properties(operation_manager_routing PROPERTIES LABELS "unit")
  # gateway_core only, no rclcpp on the link line, so no ROS entity to isolate.
  medkit_test_needs_no_domain(operation_manager_routing)

  # ─── ConfigurationManager routing test (gateway_core only) ─────────────────
  # Mock-transport coverage proving the manager body (CRUD delegation,
  # self-node short circuit, reset orchestration via cached defaults,
  # shutdown idempotency, error code propagation) compiles and links against
  # gateway_core alone, with no rclcpp on the link line.
  add_executable(test_configuration_manager_routing test/test_configuration_manager_routing.cpp)
  target_link_libraries(test_configuration_manager_routing
    PRIVATE
      gateway_core
      GTest::gtest
      GTest::gtest_main
  )
  add_test(
    NAME configuration_manager_routing
    COMMAND $<TARGET_FILE:test_configuration_manager_routing>
  )
  set_tests_properties(configuration_manager_routing PROPERTIES LABELS "unit")
  # gateway_core only, no rclcpp on the link line, so no ROS entity to isolate.
  medkit_test_needs_no_domain(configuration_manager_routing)

  # ─── FaultManager routing test (gateway_core only) ─────────────────────────
  # Mock-transport coverage proving the manager body (eight SOVD operations,
  # error_message propagation, JSON-shape contract for get_fault_with_env)
  # compiles and links against gateway_core alone, with no rclcpp on the link
  # line.
  add_executable(test_fault_manager_routing test/test_fault_manager_routing.cpp)
  target_link_libraries(test_fault_manager_routing
    PRIVATE
      gateway_core
      GTest::gtest
      GTest::gtest_main
  )
  add_test(
    NAME fault_manager_routing
    COMMAND $<TARGET_FILE:test_fault_manager_routing>
  )
  set_tests_properties(fault_manager_routing PROPERTIES LABELS "unit")
  # gateway_core only, no rclcpp on the link line, so no ROS entity to isolate.
  medkit_test_needs_no_domain(fault_manager_routing)

  # ─── LogManager routing test (gateway_core only) ───────────────────────────
  # Mock-source coverage proving the manager body (ring-buffer storage,
  # plugin observer pattern, manages_ingestion short-circuit, severity
  # filter, source teardown) compiles and links against gateway_core alone,
  # with no rclcpp on the link line.
  add_executable(test_log_manager_routing test/test_log_manager_routing.cpp)
  target_link_libraries(test_log_manager_routing
    PRIVATE
      gateway_core
      GTest::gtest
      GTest::gtest_main
  )
  add_test(
    NAME log_manager_routing
    COMMAND $<TARGET_FILE:test_log_manager_routing>
  )
  set_tests_properties(log_manager_routing PROPERTIES LABELS "unit")
  # gateway_core only, no rclcpp on the link line, so no ROS entity to isolate.
  medkit_test_needs_no_domain(log_manager_routing)

  # ─── TriggerManager routing test (gateway_core only) ───────────────────────
  # Mock-transport coverage proving the manager body (data-trigger subscribe
  # routing, handle-destruction unsubscribe, sample callback dispatch into
  # the resource-change notifier, orphan sweep, multi-trigger isolation,
  # shutdown teardown) compiles and links against gateway_core alone, with
  # no rclcpp on the link line.
  add_executable(test_trigger_manager_routing test/test_trigger_manager_routing.cpp)
  target_link_libraries(test_trigger_manager_routing
    PRIVATE
      gateway_core
      GTest::gtest
      GTest::gtest_main
  )
  add_test(
    NAME trigger_manager_routing
    COMMAND $<TARGET_FILE:test_trigger_manager_routing>
  )
  set_tests_properties(trigger_manager_routing PROPERTIES LABELS "unit")
  # gateway_core only, no rclcpp on the link line, so no ROS entity to isolate.
  medkit_test_needs_no_domain(trigger_manager_routing)

  # ─── ServiceActionResolver abstract contract test (gateway_core only) ──────
  # Standalone pin of the ServiceActionResolver interface contract: nullopt
  # on miss, populated ServiceInfo/ActionInfo on hit, and orthogonal service
  # vs. action namespaces. Until now the interface was only exercised
  # transitively through OperationManager routing - this gives future
  # implementors (e.g. an aggregation-aware resolver) an explicit, transport-
  # neutral specification. Links against gateway_core alone, with no rclcpp.
  add_executable(test_service_action_resolver_contract
    test/test_service_action_resolver_contract.cpp)
  target_link_libraries(test_service_action_resolver_contract
    PRIVATE
      gateway_core
      GTest::gtest
      GTest::gtest_main
  )
  add_test(
    NAME service_action_resolver_contract
    COMMAND $<TARGET_FILE:test_service_action_resolver_contract>
  )
  set_tests_properties(service_action_resolver_contract PROPERTIES LABELS "unit")
  # gateway_core only, no rclcpp on the link line, so no ROS entity to isolate.
  medkit_test_needs_no_domain(service_action_resolver_contract)

  # Add GTest
  find_package(ament_cmake_gtest REQUIRED)
  find_package(rclcpp_action REQUIRED)
  find_package(example_interfaces REQUIRED)

  medkit_add_gtest(test_gateway_node test/test_gateway_node.cpp)
  target_link_libraries(test_gateway_node gateway_ros2)

  medkit_add_gtest(test_entity_change_scope test/test_entity_change_scope.cpp)
  target_link_libraries(test_entity_change_scope gateway_ros2)

  medkit_add_gtest(test_manifest_fragments test/test_manifest_fragments.cpp)
  target_link_libraries(test_manifest_fragments gateway_ros2)

  medkit_add_gtest(test_plugin_notify_integration test/test_plugin_notify_integration.cpp)
  target_link_libraries(test_plugin_notify_integration gateway_ros2)

  medkit_add_gtest(test_plugin_context_aggregation test/test_plugin_context_aggregation.cpp)
  target_link_libraries(test_plugin_context_aggregation gateway_ros2)

  medkit_add_gtest(test_auth_manager test/test_auth_manager.cpp)
  target_link_libraries(test_auth_manager gateway_ros2)

  medkit_add_gtest(test_generic_client_compat test/test_generic_client_compat.cpp)
  target_link_libraries(test_generic_client_compat gateway_ros2)
  medkit_target_dependencies(test_generic_client_compat rclcpp std_srvs)

  medkit_add_gtest(test_operation_manager test/test_operation_manager.cpp)
  target_link_libraries(test_operation_manager gateway_ros2)

  medkit_add_gtest(test_configuration_manager test/test_configuration_manager.cpp)
  target_link_libraries(test_configuration_manager gateway_ros2)

  # Ros2ParameterTransport round-trip timeout coverage (#531): a
  # discoverable-but-unresponsive node's parameter service must not hang the
  # transport forever.
  medkit_add_gtest(test_ros2_parameter_transport test/test_ros2_parameter_transport.cpp)
  target_link_libraries(test_ros2_parameter_transport gateway_ros2)

  # Separate binary: these tests own the whole rclcpp init/shutdown lifecycle (they
  # destroy a transport AFTER rclcpp::shutdown()), so they cannot share the
  # process-wide context bracket the tests above use.
  medkit_add_gtest(test_ros2_parameter_transport_shutdown test/test_ros2_parameter_transport_shutdown.cpp)
  target_link_libraries(test_ros2_parameter_transport_shutdown gateway_ros2)

  # BoundedLruCache: pure-C++ header, no ROS - bounds the parameter transport's
  # per-node caches.
  medkit_add_gtest(test_bounded_lru_cache test/test_bounded_lru_cache.cpp)

  find_package(std_msgs REQUIRED)

  # Add DiscoveryManager tests
  medkit_add_gtest(test_discovery_manager test/test_discovery_manager.cpp)
  target_link_libraries(test_discovery_manager gateway_ros2)
  medkit_target_dependencies(test_discovery_manager std_msgs)

  # Add Ros2RuntimeIntrospection tests
  medkit_add_gtest(test_runtime_discovery test/test_runtime_discovery.cpp)
  target_link_libraries(test_runtime_discovery gateway_ros2)

  # Add TLS configuration tests
  medkit_add_gtest(test_tls_config test/test_tls_config.cpp)
  target_link_libraries(test_tls_config gateway_ros2)

  # Add HTTP server thread-pool bound tests (issue #440)
  medkit_add_gtest(test_http_server_thread_pool test/test_http_server_thread_pool.cpp)
  target_link_libraries(test_http_server_thread_pool gateway_ros2)

  # Add FaultManager tests
  medkit_add_gtest(test_fault_manager test/test_fault_manager.cpp)
  target_link_libraries(test_fault_manager gateway_ros2)
  medkit_target_dependencies(test_fault_manager rclcpp ros2_medkit_msgs)

  # Add error info tests
  medkit_add_gtest(test_error_info test/test_error_info.cpp)
  target_link_libraries(test_error_info gateway_ros2)

  # Parameter error classification unit tests (pure C++, no rclcpp)
  medkit_add_gtest(test_parameter_error_classification test/test_parameter_error_classification.cpp)
  target_link_libraries(test_parameter_error_classification gateway_ros2)

  # Parameter error precedence tests (multi-node GET aggregation, live GatewayNode)
  medkit_add_gtest(test_parameter_error_precedence test/test_parameter_error_precedence.cpp)
  target_link_libraries(test_parameter_error_precedence gateway_ros2)
  medkit_target_dependencies(test_parameter_error_precedence rclcpp)

  # Add ROS 2 subscription executor tests
  medkit_add_gtest(test_ros2_subscription_executor test/test_ros2_subscription_executor.cpp)
  target_link_libraries(test_ros2_subscription_executor gateway_ros2)
  medkit_target_dependencies(test_ros2_subscription_executor rclcpp std_msgs)

  # Add ROS 2 subscription slot tests
  medkit_add_gtest(test_ros2_subscription_slot test/test_ros2_subscription_slot.cpp)
  target_link_libraries(test_ros2_subscription_slot gateway_ros2)
  medkit_target_dependencies(test_ros2_subscription_slot rclcpp std_msgs)

  # Add TopicDataProvider interface tests (pure C++, no rclcpp init required)
  medkit_add_gtest(test_topic_data_provider_interface test/test_topic_data_provider_interface.cpp)
  target_link_libraries(test_topic_data_provider_interface gateway_ros2)

  # Add Ros2TopicDataProvider tests (rclcpp required)
  medkit_add_gtest(test_ros2_topic_data_provider test/test_ros2_topic_data_provider.cpp)
  target_link_libraries(test_ros2_topic_data_provider gateway_ros2)
  medkit_target_dependencies(test_ros2_topic_data_provider rclcpp std_msgs)

  # TriggerTopicSubscriber adapter tests (rclcpp + a real subscription executor)
  medkit_add_gtest(test_trigger_topic_subscriber test/test_trigger_topic_subscriber.cpp)
  target_link_libraries(test_trigger_topic_subscriber gateway_ros2)
  medkit_target_dependencies(test_trigger_topic_subscriber rclcpp std_msgs)

  # Add Ros2LogSource tests (rclcpp required - publishes to /rosout)
  medkit_add_gtest(test_ros2_log_source test/test_ros2_log_source.cpp)
  target_link_libraries(test_ros2_log_source gateway_ros2)
  medkit_target_dependencies(test_ros2_log_source rclcpp rcl_interfaces)

  # Add discovery models tests
  medkit_add_gtest(test_discovery_models test/test_discovery_models.cpp)
  target_link_libraries(test_discovery_models gateway_ros2)

  # Add host info provider tests
  medkit_add_gtest(test_host_info_provider test/test_host_info_provider.cpp)
  target_link_libraries(test_host_info_provider gateway_ros2)

  # Add discovery handlers tests
  medkit_add_gtest(test_discovery_handlers test/test_discovery_handlers.cpp)
  target_link_libraries(test_discovery_handlers gateway_ros2)

  # Add manifest parser tests
  medkit_add_gtest(test_manifest_parser test/test_manifest_parser.cpp)
  target_link_libraries(test_manifest_parser gateway_ros2)

  # Add manifest validator tests
  medkit_add_gtest(test_manifest_validator test/test_manifest_validator.cpp)
  target_link_libraries(test_manifest_validator gateway_ros2)

  # Add manifest manager tests
  medkit_add_gtest(test_manifest_manager test/test_manifest_manager.cpp)
  target_link_libraries(test_manifest_manager gateway_ros2)

  # Add runtime linker tests
  medkit_add_gtest(test_runtime_linker test/test_runtime_linker.cpp)
  target_link_libraries(test_runtime_linker gateway_ros2)

  # Add merge pipeline tests
  medkit_add_gtest(test_merge_pipeline test/test_merge_pipeline.cpp)
  target_link_libraries(test_merge_pipeline gateway_ros2)

  # Add asset-identity model + merge tests
  medkit_add_gtest(test_asset_identity test/test_asset_identity.cpp)
  target_link_libraries(test_asset_identity gateway_ros2)

  # Asset inventory: CSV / manifest asset-list parse + merge-by-id
  medkit_add_gtest(test_asset_inventory test/test_asset_inventory.cpp)
  target_link_libraries(test_asset_inventory gateway_ros2)

  # Add capability builder tests
  medkit_add_gtest(test_capability_builder test/test_capability_builder.cpp)
  target_link_libraries(test_capability_builder gateway_ros2)

  # Add handler context tests
  medkit_add_gtest(test_handler_context test/test_handler_context.cpp)
  target_link_libraries(test_handler_context gateway_ros2)

  # FlatHashMap open-addressed container (pure C++17, no ROS node)
  medkit_add_gtest(test_flat_hash_map test/test_flat_hash_map.cpp)
  if(TARGET test_flat_hash_map)
    target_link_libraries(test_flat_hash_map gateway_core)
  endif()

  # Graph-event refresh debounce decision (pure C++17, no ROS node)
  medkit_add_gtest(test_refresh_debounce test/test_refresh_debounce.cpp)
  if(TARGET test_refresh_debounce)
    target_link_libraries(test_refresh_debounce gateway_core)
  endif()

  # SlotStore object pool (pure C++17, no ROS node)
  medkit_add_gtest(test_slot_store test/test_slot_store.cpp)
  if(TARGET test_slot_store)
    target_link_libraries(test_slot_store gateway_core)
  endif()

  # ThreadSafeEntityCache incremental reconcile (pure C++17, no ROS node)
  medkit_add_gtest(test_thread_safe_entity_cache
    test/test_thread_safe_entity_cache.cpp)
  if(TARGET test_thread_safe_entity_cache)
    target_link_libraries(test_thread_safe_entity_cache gateway_core)
  endif()

  # DTO contract core (pure C++17, no ROS node)
  medkit_add_gtest(test_dto_contract test/test_dto_contract.cpp)
  target_link_libraries(test_dto_contract gateway_core)

  # Lifecycle DTO (pure C++17, no ROS node)
  medkit_add_gtest(test_lifecycle_dto test/test_lifecycle_dto.cpp)
  target_link_libraries(test_lifecycle_dto gateway_core)

  # DTO Collection<T, XMedkit> template + fan-out observability fields
  medkit_add_gtest(test_collection test/test_collection.cpp)
  target_link_libraries(test_collection gateway_core)

  # DTO opaque_object machinery (pure C++17, no ROS node)
  medkit_add_gtest(test_opaque_object test/test_opaque_object.cpp)
  target_link_libraries(test_opaque_object gateway_core)

  # Entity value equality (operator==) - pure C++17, no ROS node
  medkit_add_gtest(test_entity_equality test/test_entity_equality.cpp)
  target_link_libraries(test_entity_equality gateway_core)

  # Typed router foundational types (pure C++17, no ROS node)
  medkit_add_gtest(test_typed_router test/test_typed_router.cpp)
  target_link_libraries(test_typed_router gateway_core)

  # Framework response-writing primitives (pure C++17, no ROS node)
  medkit_add_gtest(test_primitives test/test_primitives.cpp)
  target_link_libraries(test_primitives gateway_core)

  # Add rate limiter tests
  medkit_add_gtest(test_rate_limiter test/test_rate_limiter.cpp)
  target_link_libraries(test_rate_limiter gateway_ros2)

  # Add auth config tests
  medkit_add_gtest(test_auth_config test/test_auth_config.cpp)
  target_link_libraries(test_auth_config gateway_ros2)

  # Add data access manager tests
  medkit_add_gtest(test_data_access_manager test/test_data_access_manager.cpp)
  target_link_libraries(test_data_access_manager gateway_ros2)
  medkit_target_dependencies(test_data_access_manager rclcpp std_msgs)

  # Add entity resource model tests
  medkit_add_gtest(test_entity_resource_model test/test_entity_resource_model.cpp)
  target_link_libraries(test_entity_resource_model gateway_ros2)

  # Function/Area resource collection aggregation tests
  medkit_add_gtest(test_function_resource_collections test/test_function_resource_collections.cpp)
  target_link_libraries(test_function_resource_collections gateway_ros2)

  # Add entity path utils tests
  medkit_add_gtest(test_entity_path_utils test/test_entity_path_utils.cpp)
  target_link_libraries(test_entity_path_utils gateway_ros2)

  # Add fault handlers tests (SOVD response building)
  medkit_add_gtest(test_fault_handlers test/test_fault_handlers.cpp)
  target_link_libraries(test_fault_handlers gateway_ros2)

  # Add bulk data store tests
  medkit_add_gtest(test_bulk_data_store test/test_bulk_data_store.cpp)
  target_link_libraries(test_bulk_data_store gateway_ros2)

  # Add bulk data handlers tests
  medkit_add_gtest(test_bulkdata_handlers test/test_bulkdata_handlers.cpp)
  target_link_libraries(test_bulkdata_handlers gateway_ros2)

  # Add subscription manager tests
  medkit_add_gtest(test_subscription_manager test/test_subscription_manager.cpp)
  target_link_libraries(test_subscription_manager gateway_ros2)

  # Add resource sampler registry tests
  medkit_add_gtest(test_resource_sampler_registry test/test_resource_sampler_registry.cpp)
  target_link_libraries(test_resource_sampler_registry gateway_ros2)

  # Add transport registry tests
  medkit_add_gtest(test_transport_registry test/test_transport_registry.cpp)
  target_link_libraries(test_transport_registry gateway_ros2)

  # Add cyclic subscription handler tests
  medkit_add_gtest(test_cyclic_subscription_handlers test/test_cyclic_subscription_handlers.cpp)
  target_link_libraries(test_cyclic_subscription_handlers gateway_ros2)

  # Add SSE transport provider tests
  medkit_add_gtest(test_sse_transport_provider test/test_sse_transport_provider.cpp)
  target_link_libraries(test_sse_transport_provider gateway_ros2)

  # Add SSE fault handler tests
  medkit_add_gtest(test_sse_fault_handler test/test_sse_fault_handler.cpp)
  target_link_libraries(test_sse_fault_handler gateway_ros2)
  medkit_target_dependencies(test_sse_fault_handler rclcpp ros2_medkit_msgs)

  # Zero-config entity freeze-frame capture tests
  medkit_add_gtest(test_entity_freeze_frame_capture test/test_entity_freeze_frame_capture.cpp)
  target_link_libraries(test_entity_freeze_frame_capture gateway_ros2)
  medkit_target_dependencies(test_entity_freeze_frame_capture rclcpp ros2_medkit_msgs)

  # Add update manager tests
  medkit_add_gtest(test_update_manager test/test_update_manager.cpp)
  target_link_libraries(test_update_manager gateway_ros2)

  # Add script manager tests
  medkit_add_gtest(test_script_manager test/test_script_manager.cpp)
  target_link_libraries(test_script_manager gateway_ros2)

  # Add default script provider tests
  medkit_add_gtest(test_default_script_provider test/test_default_script_provider.cpp)
  target_link_libraries(test_default_script_provider gateway_ros2)

  # Add script handler tests
  medkit_add_gtest(test_script_handlers test/test_script_handlers.cpp)
  target_link_libraries(test_script_handlers gateway_ros2)

  # Add data handler tests
  medkit_add_gtest(test_data_handlers test/test_data_handlers.cpp)
  target_link_libraries(test_data_handlers gateway_ros2)

  # Add auth handler tests
  medkit_add_gtest(test_auth_handlers test/test_auth_handlers.cpp)
  target_link_libraries(test_auth_handlers gateway_ros2)

  # Add health handler tests
  medkit_add_gtest(test_health_handlers test/test_health_handlers.cpp)
  target_link_libraries(test_health_handlers gateway_ros2)

  # Add lifecycle handler tests
  medkit_add_gtest(test_lifecycle_handlers test/test_lifecycle_handlers.cpp)
  target_link_libraries(test_lifecycle_handlers gateway_ros2)
  medkit_target_dependencies(test_lifecycle_handlers rclcpp)

  # Lifecycle status helpers (pure C++17, no ROS node)
  medkit_add_gtest(test_lifecycle_status_helpers test/test_lifecycle_status_helpers.cpp)
  target_link_libraries(test_lifecycle_status_helpers gateway_core)

  # GetState-backed lifecycle state reader (ROS 2 in-process service)
  medkit_add_gtest(test_ros2_lifecycle_state_reader test/test_ros2_lifecycle_state_reader.cpp)
  target_link_libraries(test_ros2_lifecycle_state_reader gateway_ros2)
  medkit_target_dependencies(test_ros2_lifecycle_state_reader rclcpp lifecycle_msgs)

  # Add operation handler tests
  medkit_add_gtest(test_operation_handlers test/test_operation_handlers.cpp)
  target_link_libraries(test_operation_handlers gateway_ros2)
  medkit_target_dependencies(test_operation_handlers rclcpp rclcpp_action std_srvs example_interfaces)

  # Cancel-outcome falsifiers (issue #576). The fixture's CancelGoal service
  # deliberately parks requests past the configured budget, so the suite
  # spends multiple seconds inside blocking waits by design - give it
  # headroom over the default gtest timeout.
  medkit_add_gtest(test_cancel_outcomes test/test_cancel_outcomes.cpp TIMEOUT 180)
  target_link_libraries(test_cancel_outcomes gateway_ros2)
  medkit_target_dependencies(test_cancel_outcomes rclcpp action_msgs)

  # Callback-group wiring contract (issue #575)
  medkit_add_gtest(test_callback_groups test/test_callback_groups.cpp)
  target_link_libraries(test_callback_groups gateway_ros2)
  medkit_target_dependencies(test_callback_groups rclcpp action_msgs)

  # Demo update backend plugin (.so for integration tests)
  add_library(test_update_backend MODULE
    test/demo_nodes/test_update_backend.cpp
  )
  target_include_directories(test_update_backend PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )
  target_link_libraries(test_update_backend
    nlohmann_json::nlohmann_json
    tl::expected
    cpp_httplib_target
    OpenSSL::SSL
    OpenSSL::Crypto
  )
  install(TARGETS test_update_backend
    LIBRARY DESTINATION lib/${PROJECT_NAME}
  )

  # Test gateway plugin (.so for plugin loader tests)
  add_library(test_gateway_plugin MODULE
    test/demo_nodes/test_gateway_plugin.cpp
  )
  target_link_libraries(test_gateway_plugin gateway_ros2)
  install(TARGETS test_gateway_plugin
    LIBRARY DESTINATION lib/${PROJECT_NAME}
  )

  # Minimal test plugins for error-path coverage
  foreach(_plugin test_bad_version_plugin test_no_symbols_plugin test_null_factory_plugin test_version_only_plugin)
    add_library(${_plugin} MODULE test/demo_nodes/${_plugin}.cpp)
    target_include_directories(${_plugin} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
    target_link_libraries(${_plugin} nlohmann_json::nlohmann_json)
    install(TARGETS ${_plugin} LIBRARY DESTINATION lib/${PROJECT_NAME})
  endforeach()

  # Demo PLC-like plugin: IntrospectionProvider + DataProvider + FaultProvider
  # (.so for the zero-config entity freeze-frame integration test)
  add_library(test_data_provider_plugin MODULE test/demo_nodes/test_data_provider_plugin.cpp)
  target_include_directories(test_data_provider_plugin PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  target_link_libraries(test_data_provider_plugin
    nlohmann_json::nlohmann_json
    tl::expected
    cpp_httplib_target
    OpenSSL::SSL
    OpenSSL::Crypto
  )
  install(TARGETS test_data_provider_plugin LIBRARY DESTINATION lib/${PROJECT_NAME})

  # Demo commercial-bridge-shaped plugin: IntrospectionProvider ONLY, live
  # values served exclusively via a registered x-plc-data route (no
  # get_data_provider / get_fault_provider exports, like the S7comm bridge).
  # Proves the freeze-frame route-dispatch fallback and the fault_manager
  # fall-through for FaultProvider-less plugin entities.
  add_library(test_route_data_plugin MODULE test/demo_nodes/test_route_data_plugin.cpp)
  target_include_directories(test_route_data_plugin PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  target_link_libraries(test_route_data_plugin
    nlohmann_json::nlohmann_json
    tl::expected
    cpp_httplib_target
    OpenSSL::SSL
    OpenSSL::Crypto
  )
  install(TARGETS test_route_data_plugin LIBRARY DESTINATION lib/${PROJECT_NAME})

  # Minimal plugin with no provider query functions (only required exports)
  add_library(test_minimal_plugin MODULE test/demo_nodes/test_minimal_plugin.cpp)
  target_include_directories(test_minimal_plugin PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  target_link_libraries(test_minimal_plugin nlohmann_json::nlohmann_json cpp_httplib_target OpenSSL::SSL OpenSSL::Crypto)
  install(TARGETS test_minimal_plugin LIBRARY DESTINATION lib/${PROJECT_NAME})

  # Add plugin loader tests
  medkit_add_gtest(test_plugin_loader test/test_plugin_loader.cpp)
  target_link_libraries(test_plugin_loader gateway_ros2)
  medkit_target_dependencies(test_plugin_loader ament_index_cpp)

  # Plugin config from YAML tests
  medkit_add_gtest(test_plugin_config test/test_plugin_config.cpp)
  target_link_libraries(test_plugin_config gateway_ros2)

  # Plugin manager tests
  medkit_add_gtest(test_plugin_manager test/test_plugin_manager.cpp)
  target_link_libraries(test_plugin_manager gateway_ros2)

  # Plugin entity routing tests
  medkit_add_gtest(test_plugin_entity_routing test/test_plugin_entity_routing.cpp)
  target_link_libraries(test_plugin_entity_routing gateway_ros2)

  # Lifecycle provider routing tests
  medkit_add_gtest(test_lifecycle_provider_routing test/test_lifecycle_provider_routing.cpp)
  target_link_libraries(test_lifecycle_provider_routing gateway_ros2)

  # Plugin HTTP types tests
  medkit_add_gtest(test_plugin_http_types test/test_plugin_http_types.cpp)
  target_link_libraries(test_plugin_http_types gateway_ros2)

  # Plugin ABI conformance tests
  # Pins the public plugin ABI (PluginRoute, PluginRequest, PluginResponse,
  # PLUGIN_API_VERSION) via static_assert + loads test_gateway_plugin.so via
  # the production PluginLoader entry point. Locks in the wire-format contract
  # so refactors that would break commercial out-of-tree plugins (UDS, OPC-UA,
  # Uptane OTA, Mender OTA) fail loudly here.
  medkit_add_gtest(test_plugin_abi_conformance test/test_plugin_abi_conformance.cpp)
  target_link_libraries(test_plugin_abi_conformance gateway_ros2)
  medkit_target_dependencies(test_plugin_abi_conformance ament_index_cpp)

  # Log manager tests
  medkit_add_gtest(test_log_manager test/test_log_manager.cpp)
  target_link_libraries(test_log_manager gateway_ros2)

  # Log handlers tests
  medkit_add_gtest(test_log_handlers test/test_log_handlers.cpp)
  target_link_libraries(test_log_handlers gateway_ros2)

  # Log scope tests (entity -> log-source resolution shared by handlers and samplers)
  medkit_add_gtest(test_log_scope test/test_log_scope.cpp)
  target_link_libraries(test_log_scope gateway_ros2)

  # Route descriptions tests (OpenAPI doc builder)
  medkit_add_gtest(test_route_descriptions test/test_route_descriptions.cpp)
  target_link_libraries(test_route_descriptions gateway_ros2)

  # Schema builder tests (OpenAPI schema generation)
  medkit_add_gtest(test_schema_builder test/test_schema_builder.cpp)
  target_link_libraries(test_schema_builder gateway_ros2)

  # Path builder tests (OpenAPI path item generation)
  medkit_add_gtest(test_path_builder test/test_path_builder.cpp)
  target_link_libraries(test_path_builder gateway_ros2)

  # OpenAPI spec builder tests (full document assembly)
  medkit_add_gtest(test_openapi_spec_builder test/test_openapi_spec_builder.cpp)
  target_link_libraries(test_openapi_spec_builder gateway_ros2)

  # Path resolver tests (OpenAPI path classification)
  medkit_add_gtest(test_path_resolver test/test_path_resolver.cpp)
  target_link_libraries(test_path_resolver gateway_ros2)

  # Capability generator tests (OpenAPI spec generation engine)
  medkit_add_gtest(test_capability_generator test/test_capability_generator.cpp)
  target_link_libraries(test_capability_generator gateway_ros2)

  # Schema reachability tests (orphan detection over a finished document).
  # gateway_core, not gateway_ros2: unreachable_schemas() takes a nlohmann::json
  # and touches no ROS type, so it lives in the neutral layer.
  ament_add_gtest(test_schema_reachability test/test_schema_reachability.cpp)
  target_link_libraries(test_schema_reachability gateway_core)
  medkit_test_needs_no_domain(test_schema_reachability)

  # Route registry tests (OpenAPI route registration and path conversion)
  medkit_add_gtest(test_route_registry test/test_route_registry.cpp)
  target_link_libraries(test_route_registry gateway_ros2)

  # Typed RouteRegistry overload tests (PR-403 commit 4 typed router wrappers)
  medkit_add_gtest(test_typed_route_registry test/test_typed_route_registry.cpp)
  target_link_libraries(test_typed_route_registry gateway_core)

  # Docs handlers tests (OpenAPI /docs endpoint handlers)
  medkit_add_gtest(test_docs_handlers test/test_docs_handlers.cpp)
  target_link_libraries(test_docs_handlers gateway_ros2)

  # Lock manager tests
  medkit_add_gtest(test_lock_manager test/test_lock_manager.cpp)
  target_link_libraries(test_lock_manager gateway_ros2)

  # Lock handlers tests
  medkit_add_gtest(test_lock_handlers test/test_lock_handlers.cpp)
  target_link_libraries(test_lock_handlers gateway_ros2)

  # Condition evaluator tests (trigger condition interface + built-in evaluators)
  medkit_add_gtest(test_condition_evaluator test/test_condition_evaluator.cpp)
  target_link_libraries(test_condition_evaluator gateway_ros2)

  # Resource change notifier tests (async notification hub for triggers)
  medkit_add_gtest(test_resource_change_notifier test/test_resource_change_notifier.cpp TIMEOUT 180)
  target_link_libraries(test_resource_change_notifier gateway_ros2)

  # Trigger store tests (SQLite persistence for triggers)
  medkit_add_gtest(test_trigger_store test/test_trigger_store.cpp)
  target_link_libraries(test_trigger_store gateway_ros2)

  # Trigger manager tests (CRUD, condition eval, hierarchy matching)
  medkit_add_gtest(test_trigger_manager test/test_trigger_manager.cpp)
  target_link_libraries(test_trigger_manager gateway_ros2)

  # Trigger handler tests (REST endpoints for triggers)
  medkit_add_gtest(test_trigger_handlers test/test_trigger_handlers.cpp)
  target_link_libraries(test_trigger_handlers gateway_ros2)

  # Fault-trigger (threshold-rule) engine tests (issue #235): CRUD, validation,
  # edge-detection, persistence. Links gateway_core only (ROS-neutral).
  medkit_add_gtest(test_fault_trigger_engine test/test_fault_trigger_engine.cpp)
  target_link_libraries(test_fault_trigger_engine gateway_core)

  # Data-point suggestion helpers (issue #584): header-only, ROS-neutral.
  medkit_add_gtest(test_data_point_suggest test/test_data_point_suggest.cpp)
  target_link_libraries(test_data_point_suggest gateway_core)

  # Peer client tests (aggregation module)
  medkit_add_gtest(test_peer_client test/test_peer_client.cpp)
  target_link_libraries(test_peer_client gateway_ros2)

  # Entity merger tests (aggregation module)
  medkit_add_gtest(test_entity_merger test/test_entity_merger.cpp)
  target_link_libraries(test_entity_merger gateway_ros2)

  # Fan-out helper tests (shared URL encoding and peer merge utilities)
  medkit_add_gtest(test_fan_out_helpers test/test_fan_out_helpers.cpp)
  target_link_libraries(test_fan_out_helpers gateway_ros2)

  # Aggregation manager tests (aggregation module)
  medkit_add_gtest(test_aggregation_manager test/test_aggregation_manager.cpp)
  target_link_libraries(test_aggregation_manager gateway_ros2)

  # Aggregation classification tests (hierarchical vs leaf Component routing)
  medkit_add_gtest(test_aggregation_classification test/test_aggregation_classification.cpp)
  target_link_libraries(test_aggregation_classification gateway_ros2)

  # Stream proxy tests (SSE parsing, aggregation module)
  medkit_add_gtest(test_stream_proxy test/test_stream_proxy.cpp)
  target_link_libraries(test_stream_proxy gateway_ros2)

  medkit_add_gtest(test_peer_fault_relay test/test_peer_fault_relay.cpp)
  target_link_libraries(test_peer_fault_relay gateway_ros2)
  medkit_test_needs_no_domain(test_peer_fault_relay)

  # mDNS discovery tests (aggregation module)
  medkit_add_gtest(test_mdns_discovery test/test_mdns_discovery.cpp)
  target_link_libraries(test_mdns_discovery gateway_ros2)

  # Network utility tests (URL parsing, local address collection)
  medkit_add_gtest(test_network_utils test/test_network_utils.cpp)
  target_link_libraries(test_network_utils gateway_ros2)

  # ─── Allocation-counter proxy gate (gateway_core only) ────────────────────
  # Standalone TU with its own global operator new/delete override (gated by
  # g_armed). Proves update_all performs ~zero net structural allocations in
  # steady-state churn over a bounded id-set. Must be its OWN executable so
  # the operator new override does not bleed into other test binaries.
  # gateway_core only, so nothing here creates a ROS entity - but it still goes
  # through medkit_add_gtest, because "this one does not need a domain" is a
  # judgement that ages badly the moment a case adds a node.
  medkit_add_gtest(test_entity_cache_alloc test/test_entity_cache_alloc.cpp)
  target_link_libraries(test_entity_cache_alloc gateway_core)

  ros2_medkit_relax_vendor_warnings()
endif()

# Export include directories for downstream packages (plugins)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY src/vendored/tl_expected/include/ DESTINATION include/ros2_medkit_gateway/vendored)
# Vendored httplib.h installed to share/ (NOT include/) so it does not pollute
# downstream include paths. Used as VENDORED_DIR fallback by packages whose tests
# need httplib (e.g. graph_provider) on platforms where the system package is too old.
install(FILES src/vendored/cpp_httplib/httplib.h
  DESTINATION share/${PROJECT_NAME}/vendored/cpp_httplib)
ament_export_include_directories(include include/ros2_medkit_gateway/vendored)

ament_package()
