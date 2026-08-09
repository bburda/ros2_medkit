cmake_minimum_required(VERSION 3.8)
project(ros2_medkit_linux_introspection)

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

find_package(ament_cmake REQUIRED)
find_package(ros2_medkit_gateway REQUIRED)
find_package(nlohmann_json REQUIRED)

# --- Static utility library ---
add_library(medkit_linux_utils STATIC
  src/linux_utils/proc_reader.cpp
  src/linux_utils/cgroup_reader.cpp
)
set_target_properties(medkit_linux_utils PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(medkit_linux_utils PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_include_directories(medkit_linux_utils PUBLIC ${ros2_medkit_gateway_INCLUDE_DIRS})

# --- procfs plugin ---
add_library(procfs_introspection MODULE
  src/procfs_plugin.cpp
)
target_link_libraries(procfs_introspection medkit_linux_utils)
target_include_directories(procfs_introspection PRIVATE ${ros2_medkit_gateway_INCLUDE_DIRS})
medkit_target_dependencies(procfs_introspection nlohmann_json)
install(TARGETS procfs_introspection LIBRARY DESTINATION lib/${PROJECT_NAME})

# --- systemd plugin ---
find_package(PkgConfig REQUIRED)
pkg_check_modules(SYSTEMD libsystemd)

if(SYSTEMD_FOUND)
  add_library(systemd_introspection MODULE
    src/systemd_plugin.cpp
  )
  target_link_libraries(systemd_introspection medkit_linux_utils ${SYSTEMD_LIBRARIES})
  target_include_directories(systemd_introspection PRIVATE ${SYSTEMD_INCLUDE_DIRS} ${ros2_medkit_gateway_INCLUDE_DIRS})
  medkit_target_dependencies(systemd_introspection nlohmann_json)
  install(TARGETS systemd_introspection LIBRARY DESTINATION lib/${PROJECT_NAME})
else()
  message(WARNING "libsystemd not found - systemd_introspection plugin will not be built")
endif()

# --- container plugin ---
add_library(container_introspection MODULE
  src/container_plugin.cpp
)
target_link_libraries(container_introspection medkit_linux_utils)
target_include_directories(container_introspection PRIVATE ${ros2_medkit_gateway_INCLUDE_DIRS})
medkit_target_dependencies(container_introspection nlohmann_json)
install(TARGETS container_introspection LIBRARY DESTINATION lib/${PROJECT_NAME})

# --- Tests ---
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)

  include(ROS2MedkitTestDomain)

  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_proc_reader test/test_proc_reader.cpp)
  target_link_libraries(test_proc_reader medkit_linux_utils)

  ament_add_gtest(test_cgroup_reader test/test_cgroup_reader.cpp)
  target_link_libraries(test_cgroup_reader medkit_linux_utils)

  ament_add_gtest(test_pid_cache test/test_pid_cache.cpp)
  target_link_libraries(test_pid_cache medkit_linux_utils)

  ament_add_gtest(test_procfs_plugin test/test_procfs_plugin.cpp)
  target_link_libraries(test_procfs_plugin medkit_linux_utils)
  target_include_directories(test_procfs_plugin PRIVATE ${ros2_medkit_gateway_INCLUDE_DIRS})
  medkit_target_dependencies(test_procfs_plugin nlohmann_json)

  ament_add_gtest(test_container_plugin test/test_container_plugin.cpp)
  target_link_libraries(test_container_plugin medkit_linux_utils)
  target_include_directories(test_container_plugin PRIVATE ${ros2_medkit_gateway_INCLUDE_DIRS})
  medkit_target_dependencies(test_container_plugin nlohmann_json)

  ament_add_gtest(test_systemd_plugin test/test_systemd_plugin.cpp)
  target_link_libraries(test_systemd_plugin medkit_linux_utils)
  target_include_directories(test_systemd_plugin PRIVATE ${ros2_medkit_gateway_INCLUDE_DIRS})
  medkit_target_dependencies(test_systemd_plugin nlohmann_json)
  ros2_medkit_relax_vendor_warnings()
  # No ROS middleware on the link line and none loaded at run time (these pass
  # with RMW_IMPLEMENTATION pointing at a library that does not exist), so there
  # is no DDS participant to isolate and a wrapper would be cost without cover.
  medkit_test_needs_no_domain(test_proc_reader)
  medkit_test_needs_no_domain(test_cgroup_reader)
  medkit_test_needs_no_domain(test_pid_cache)
  medkit_test_needs_no_domain(test_procfs_plugin)
  medkit_test_needs_no_domain(test_container_plugin)
  medkit_test_needs_no_domain(test_systemd_plugin)

endif()

ament_package()
