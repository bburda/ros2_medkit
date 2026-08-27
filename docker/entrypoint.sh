#!/bin/bash
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

set -e

# Source ROS 2 and the built workspace
# shellcheck disable=SC1090,SC1091
source "/opt/ros/${ROS_DISTRO}/setup.bash"
# shellcheck disable=SC1091
source "${COLCON_WS}/install/setup.bash"

# Default to FastDDS (can be overridden via RMW_IMPLEMENTATION env var)
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"

# Bootstrap credentials so the image can ship closed AND still start.
#
# The packaged params file turns authentication on, matching the gateway's own
# default, and the gateway refuses to start without a signing secret. An image
# that therefore needed a secret before `docker run` would do anything is a
# quickstart nobody completes, and the usual reaction is to turn auth off and
# leave it off. So: if the operator supplied nothing, generate a secret and an
# admin client for this container and print the credential once.
#
# Supply MEDKIT_JWT_SECRET and MEDKIT_CLIENTS to pin your own, or
# MEDKIT_AUTH_DISABLED=1 to run open on a host nothing else can reach.
AUTH_ARGS=()
if [ "${MEDKIT_AUTH_DISABLED:-0}" = "1" ]; then
  AUTH_ARGS+=(-p auth.enabled:=false)
  echo "ros2_medkit: MEDKIT_AUTH_DISABLED=1 - starting WITHOUT authentication." >&2
  echo "             Every route is readable by anyone who can reach this port." >&2
else
  if [ -z "${MEDKIT_JWT_SECRET:-}" ]; then
    # Per container, and not persisted: a restart issues a new one, which is
    # correct for a credential nobody chose and nobody stored.
    MEDKIT_JWT_SECRET="$(head -c 32 /dev/urandom | base64 | tr -d '\n' | tr '+/' '-_' | tr -d '=')"
    MEDKIT_CLIENT_SECRET="$(head -c 24 /dev/urandom | base64 | tr -d '\n' | tr '+/' '-_' | tr -d '=')"
    MEDKIT_CLIENTS="${MEDKIT_CLIENTS:-medkit:${MEDKIT_CLIENT_SECRET}:admin}"
    echo "=============================================================" >&2
    echo "ros2_medkit: generated a one-time admin credential for this" >&2
    echo "             container. It changes on every restart." >&2
    echo "" >&2
    echo "  client_id:     medkit" >&2
    echo "  client_secret: ${MEDKIT_CLIENT_SECRET}" >&2
    echo "" >&2
    echo "  curl -s http://localhost:8080/api/v1/auth/authorize \\" >&2
    echo "    -H 'Content-Type: application/json' \\" >&2
    echo "    -d '{\"grant_type\":\"client_credentials\",\"client_id\":\"medkit\",\"client_secret\":\"${MEDKIT_CLIENT_SECRET}\"}'" >&2
    echo "" >&2
    echo "  Set MEDKIT_JWT_SECRET and MEDKIT_CLIENTS to pin your own." >&2
    echo "=============================================================" >&2
  fi
  AUTH_ARGS+=(-p "auth.jwt_secret:=${MEDKIT_JWT_SECRET}")
  [ -n "${MEDKIT_CLIENTS:-}" ] && AUTH_ARGS+=(-p "auth.clients:=[${MEDKIT_CLIENTS}]")
fi
# Exported so the other dispatch branch works too: `docker run <img> ros2 launch
# ... bringup.launch.py` execs a command instead of the node, so it never sees
# AUTH_ARGS. gateway.launch.py falls back to these variables.
export MEDKIT_JWT_SECRET MEDKIT_CLIENTS MEDKIT_AUTH_DISABLED
# The image serves plain HTTP; see gateway_docker_params.yaml for why.
export MEDKIT_TLS_DISABLED=1

# Dispatch on the first argument:
#   - empty, or starts with "-" (the default CMD "--ros-args --params-file ..."
#     or an override like --ros-args -p server.port:=9090): run the gateway node
#     directly, so `docker run <img>` and arg-only overrides keep working.
#   - a full command (e.g. `ros2 launch ros2_medkit_gateway bringup.launch.py`
#     or `bash`): exec it as-is, so the image can launch the whole bringup stack.
if [ -z "$1" ] || [ "${1#-}" != "$1" ]; then
  exec ros2 run ros2_medkit_gateway gateway_node "$@" "${AUTH_ARGS[@]}"
fi
exec "$@"
