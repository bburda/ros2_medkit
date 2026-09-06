# Copyright 2025 mfaferek93
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

cmake_minimum_required(VERSION 3.8)
project(ros2_medkit_msgs)

# Note: ROS2MedkitWarnings is intentionally NOT included here.
# rosidl-generated code triggers strict warnings we cannot fix.
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(diagnostic_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Fault.msg"
  "msg/FaultEvent.msg"
  "msg/MutedFaultInfo.msg"
  "msg/ClusterInfo.msg"
  "msg/ExtendedDataRecords.msg"
  "msg/Snapshot.msg"
  "msg/EnvironmentData.msg"
  "msg/MedkitDiscoveryHint.msg"
  "msg/EntityInfo.msg"
  "msg/PlannedStop.msg"
  "srv/ReportFault.srv"
  "srv/ListFaults.srv"
  "srv/GetFault.srv"
  "srv/ClearFault.srv"
  "srv/GetSnapshots.srv"
  "srv/GetRosbag.srv"
  "srv/ListRosbags.srv"
  "srv/ListFaultsForEntity.srv"
  "srv/ListEntities.srv"
  "srv/GetEntityData.srv"
  "srv/GetCapabilities.srv"
  "srv/DeclarePlannedStop.srv"
  "srv/EndPlannedStop.srv"
  "srv/ListPlannedStops.srv"
  DEPENDENCIES builtin_interfaces diagnostic_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
