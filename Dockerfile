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

# Multi-stage build for ros2_medkit gateway with all open-core plugins.
# Produces a minimal runtime image suitable for deployment and as a base
# for downstream plugin integration (mount .so + config).
#
# Build:  docker build -t ros2_medkit .
# Run:    docker run -p 8080:8080 ros2_medkit
# Config: docker run -v ./my_params.yaml:/etc/ros2_medkit/params.yaml ros2_medkit

# ============================================================================
# Stage 1: Builder
# ============================================================================
ARG ROS_DISTRO=jazzy
FROM ros:${ROS_DISTRO}-ros-base AS builder

ARG ROS_DISTRO
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=${ROS_DISTRO}
ENV COLCON_WS=/root/ws

# Build dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-ament-lint-auto \
    ros-${ROS_DISTRO}-ament-lint-common \
    ros-${ROS_DISTRO}-ament-cmake-gtest \
    ros-${ROS_DISTRO}-yaml-cpp-vendor \
    ros-${ROS_DISTRO}-example-interfaces \
    python3-colcon-common-extensions \
    nlohmann-json3-dev \
    sqlite3 \
    libsqlite3-dev \
    libsystemd-dev \
    libssl-dev \
    pkg-config \
    cmake \
    g++ \
    git \
    && rm -rf /var/lib/apt/lists/*

WORKDIR ${COLCON_WS}

# Copy shared cmake modules first (depended on by all packages)
COPY src/ros2_medkit_cmake/ ${COLCON_WS}/src/ros2_medkit_cmake/

# Copy core packages
COPY src/ros2_medkit_msgs/ ${COLCON_WS}/src/ros2_medkit_msgs/
COPY src/ros2_medkit_serialization/ ${COLCON_WS}/src/ros2_medkit_serialization/
COPY src/ros2_medkit_fault_detection/ ${COLCON_WS}/src/ros2_medkit_fault_detection/
COPY src/ros2_medkit_gateway/ ${COLCON_WS}/src/ros2_medkit_gateway/
COPY src/ros2_medkit_fault_manager/ ${COLCON_WS}/src/ros2_medkit_fault_manager/
COPY src/ros2_medkit_fault_reporter/ ${COLCON_WS}/src/ros2_medkit_fault_reporter/
COPY src/ros2_medkit_diagnostic_bridge/ ${COLCON_WS}/src/ros2_medkit_diagnostic_bridge/
COPY src/ros2_medkit_log_bridge/ ${COLCON_WS}/src/ros2_medkit_log_bridge/
COPY src/ros2_medkit_action_status_bridge/ ${COLCON_WS}/src/ros2_medkit_action_status_bridge/

# Copy open-core plugins
COPY src/ros2_medkit_discovery_plugins/ ${COLCON_WS}/src/ros2_medkit_discovery_plugins/
COPY src/ros2_medkit_plugins/ ${COLCON_WS}/src/ros2_medkit_plugins/

# Install ROS dependencies and build (skip tests for smaller image).
#
# apt-get update has to be re-run here: the previous RUN cleared
# /var/lib/apt/lists, so without a fresh fetch rosdep's internal
# apt-get install calls fail with "Unable to locate package" for anything
# that wasn't pulled in by the first RUN's apt-get install (notably
# python3-jsonschema, which the integration_tests package declares as a
# test_depend and which rosdep tries to install even with BUILD_TESTING=OFF).
# This was previously masked by Docker layer cache hits on CI; cold builds
# always failed.
#
# rosbag2_storage_mcap stays skip-keyed here even though fault_manager now
# exec_depends on it: this stage compiles with -DBUILD_TESTING=OFF against
# rosbag2_cpp/rosbag2_storage (the plugin interface, already resolvable
# without it) and never runs a bag, so it never needs the plugin itself, only
# its interface. The key exists because the plugin package was not available
# via rosdep on every distro this Dockerfile targets when it was added
# (ce702f61); that constraint is about apt package availability per distro,
# not about whether fault_manager declares the dependency, so becoming an
# exec_depend does not remove the reason to skip it here. It is installed for
# real in the runtime stage below, which is where it is actually loaded.
RUN bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && \
    apt-get update && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y \
      --skip-keys='ament_cmake_clang_format ament_cmake_clang_tidy ament_cmake_flake8 test_msgs sqlite3 libcpp-httplib-dev rosbag2_storage_mcap' && \
    rm -rf /var/lib/apt/lists/* && \
    colcon build --cmake-args -DBUILD_TESTING=OFF"

# ============================================================================
# Stage 2: Runtime
# ============================================================================
ARG ROS_DISTRO=jazzy
FROM ros:${ROS_DISTRO}-ros-base

ARG ROS_DISTRO
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=${ROS_DISTRO}
ENV COLCON_WS=/home/medkit/ws

# Runtime dependencies only (nlohmann-json and cpp-httplib are compiled into
# the binaries at build time, no need to install here).
# CycloneDDS RMW is bundled so the gateway can attach to a stack running
# CycloneDDS (e.g. Autoware) without rebuilding. FastDDS stays the default;
# switch with RMW_IMPLEMENTATION=rmw_cyclonedds_cpp at runtime.
# rosbag2-storage-mcap and rosbag2-storage-default-plugins are the actual
# rosbag2 storage plugins fault_manager loads at runtime through
# rosbag2_storage's plugin interface: the former is always the mcap plugin,
# the latter is the sqlite3 plugin itself on humble but, on jazzy (this
# stage's default) and lyrical, a metapackage that pulls in both sqlite3 and
# mcap - so it is not a clean sqlite3-only counterpart to the first package on
# every distro this image targets, just the name that guarantees sqlite3 is
# present. libsqlite3-0 alone is the C library, not a rosbag2 plugin, and this
# stage never gets /opt/ros from the builder - only the colcon workspace
# install/ - so without these two the image would advertise mcap-by-default
# and silently have no working black-box storage backend at all.
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-yaml-cpp-vendor \
    ros-${ROS_DISTRO}-example-interfaces \
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    ros-${ROS_DISTRO}-rosbag2-storage-mcap \
    ros-${ROS_DISTRO}-rosbag2-storage-default-plugins \
    libsqlite3-0 \
    libsystemd0 \
    libssl3 \
    curl \
    && rm -rf /var/lib/apt/lists/*

# Copy built workspace from builder (builder uses /root/ws, runtime uses /home/medkit/ws)
COPY --from=builder /root/ws/install/ ${COLCON_WS}/install/

# Default config - can be overridden via volume mount
COPY docker/gateway_docker_params.yaml /etc/ros2_medkit/params.yaml

# When running via ros2 run (as this container does), plugin .so paths must be
# configured explicitly via plugins.<name>.path parameters in the params file.
# To add external plugins, mount them at /opt/ros2_medkit/plugins/ and reference
# their full paths in your custom params file.
RUN mkdir -p /opt/ros2_medkit/plugins

# Non-root user for production safety. /var/lib/ros2_medkit is the default
# fault_manager database_path parent; create and chown it so the fault manager
# can persist faults.db without running as root.
RUN groupadd -r medkit && useradd -r -g medkit -d /home/medkit -s /bin/bash medkit && \
    mkdir -p /home/medkit /var/lib/ros2_medkit && chown medkit:medkit /home/medkit && \
    chown -R medkit:medkit ${COLCON_WS}/install /etc/ros2_medkit /opt/ros2_medkit /var/lib/ros2_medkit

# Entrypoint that sources ROS and starts the gateway
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

USER medkit
EXPOSE 8080

ENTRYPOINT ["/entrypoint.sh"]
CMD ["--ros-args", "--params-file", "/etc/ros2_medkit/params.yaml"]
