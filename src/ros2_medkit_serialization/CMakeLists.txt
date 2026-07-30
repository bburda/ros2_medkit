cmake_minimum_required(VERSION 3.8)
project(ros2_medkit_serialization)

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

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcutils REQUIRED)
find_package(rosidl_runtime_c REQUIRED)
find_package(rosidl_typesupport_introspection_c REQUIRED)
find_package(rosidl_typesupport_introspection_cpp REQUIRED)
find_package(rosidl_runtime_cpp REQUIRED)
find_package(rcpputils REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
medkit_find_yaml_cpp()
find_package(nlohmann_json REQUIRED)

# Library
add_library(${PROJECT_NAME}
  src/json_serializer.cpp
  src/type_cache.cpp
  src/type_introspection.cpp
  src/service_action_types.cpp
  src/message_cleanup.cpp
  # Vendored dynmsg sources (C++ API only)
  src/vendored/dynmsg/typesupport.cpp
  src/vendored/dynmsg/message_reading_cpp.cpp
  src/vendored/dynmsg/msg_parser_cpp.cpp
  src/vendored/dynmsg/string_utils.cpp
  src/vendored/dynmsg/vector_utils.cpp
  src/vendored/dynmsg/yaml_utils.cpp
)

# Vendored dynmsg is third-party code - suppress our strict warnings
set_source_files_properties(
  src/vendored/dynmsg/typesupport.cpp
  src/vendored/dynmsg/message_reading_cpp.cpp
  src/vendored/dynmsg/msg_parser_cpp.cpp
  src/vendored/dynmsg/string_utils.cpp
  src/vendored/dynmsg/vector_utils.cpp
  src/vendored/dynmsg/yaml_utils.cpp
  PROPERTIES COMPILE_OPTIONS "-w"
)

# Enable PIC so this static library can be linked into shared objects
# (e.g. test_gateway_plugin.so via gateway_core.a / gateway_ros2.a)
set_target_properties(${PROJECT_NAME} PROPERTIES POSITION_INDEPENDENT_CODE ON)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

medkit_target_dependencies(${PROJECT_NAME}
  rclcpp
  rcutils
  rosidl_runtime_c
  rosidl_typesupport_introspection_c
  rosidl_typesupport_introspection_cpp
  rosidl_runtime_cpp
  rcpputils
  yaml_cpp_vendor
)

target_link_libraries(${PROJECT_NAME}
  nlohmann_json::nlohmann_json
  yaml-cpp::yaml-cpp
)

# Export the library
ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  rcutils
  rosidl_runtime_c
  rosidl_typesupport_introspection_c
  rosidl_typesupport_introspection_cpp
  rosidl_runtime_cpp
  rcpputils
  yaml_cpp_vendor
  nlohmann_json
)

install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  find_package(std_msgs REQUIRED)
  find_package(std_srvs REQUIRED)
  find_package(geometry_msgs REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(test_msgs REQUIRED)

  # Exclude conflicting linters and those we configure manually
  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_uncrustify
    ament_cmake_cpplint
    ament_cmake_clang_tidy
    ament_cmake_clang_format
  )
  ament_lint_auto_find_test_dependencies()

  # Configure clang-format manually for non-vendored files only.
  # ament_cmake_clang_format is intentionally only installed in the Jazzy
  # quality job (ci.yml skips its rosdep key in the build-and-test job), so use
  # QUIET + FOUND check instead of REQUIRED to gracefully degrade on the
  # build-and-test runners without breaking the build.
  find_package(ament_cmake_clang_format QUIET)
  if(ament_cmake_clang_format_FOUND)
    set(_clang_format_config "${CMAKE_CURRENT_SOURCE_DIR}/../../.clang-format")
    ament_clang_format(
      CONFIG_FILE "${_clang_format_config}"
      "include/ros2_medkit_serialization/json_serializer.hpp"
      "include/ros2_medkit_serialization/message_cleanup.hpp"
      "include/ros2_medkit_serialization/serialization_error.hpp"
      "include/ros2_medkit_serialization/service_action_types.hpp"
      "include/ros2_medkit_serialization/type_cache.hpp"
      "include/ros2_medkit_serialization/type_introspection.hpp"
      "src/json_serializer.cpp"
      "src/message_cleanup.cpp"
      "src/service_action_types.cpp"
      "src/type_cache.cpp"
      "src/type_introspection.cpp"
      "test/test_json_serializer.cpp"
      "test/test_service_action_types.cpp"
      "test/test_type_cache.cpp"
      "test/test_type_introspection.cpp"
    )
  endif()

  ros2_medkit_clang_tidy(
    HEADER_FILTER "^${CMAKE_CURRENT_SOURCE_DIR}/include/ros2_medkit_serialization/[^v].*"
    TIMEOUT 300
  )

  # Unit tests
  ament_add_gtest(test_type_cache test/test_type_cache.cpp)
  target_link_libraries(test_type_cache ${PROJECT_NAME})
  medkit_target_dependencies(test_type_cache std_msgs std_srvs)

  ament_add_gtest(test_json_serializer test/test_json_serializer.cpp)
  target_link_libraries(test_json_serializer ${PROJECT_NAME})
  medkit_target_dependencies(test_json_serializer std_msgs std_srvs geometry_msgs sensor_msgs test_msgs)

  ament_add_gtest(test_service_action_types test/test_service_action_types.cpp)
  target_link_libraries(test_service_action_types ${PROJECT_NAME})
  medkit_target_dependencies(test_service_action_types std_srvs)

  ament_add_gtest(test_type_introspection test/test_type_introspection.cpp)
  target_link_libraries(test_type_introspection ${PROJECT_NAME})
  medkit_target_dependencies(test_type_introspection std_msgs std_srvs test_msgs)

  ros2_medkit_relax_vendor_warnings()
endif()

ament_package()
