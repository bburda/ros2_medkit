#!/bin/bash
# Copyright 2026 selfpatch contributors
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

# Smoke test: verify the gateway starts and responds to health checks.
# Intended to run inside a pixi environment after build.
set -euo pipefail

# Ensure colcon workspace is sourced (needed when run outside pixi)
# Temporarily allow unset variables - colcon's setup.bash references COLCON_TRACE
if [ -f install/setup.bash ]; then
  set +u
  # shellcheck source=/dev/null
  source install/setup.bash
  set -u
fi

PORT="${GATEWAY_SMOKE_PORT:-8080}"
TIMEOUT=30

# The shipped config has TLS and authentication on, so a bare launch refuses to
# start. This is a smoke test for "does the gateway come up and answer", not for
# the security posture, so it runs the way a developer on a local machine would:
# plain HTTP, no credential. test_shipped_defaults covers the shipped posture.
ros2 launch ros2_medkit_gateway gateway.launch.py \
  server_port:="$PORT" tls_enabled:=false auth_enabled:=false &
GW_PID=$!

# shellcheck disable=SC2317  # cleanup is invoked indirectly via trap
cleanup() { kill "$GW_PID" 2>/dev/null || true; wait "$GW_PID" 2>/dev/null || true; }
trap cleanup EXIT

for i in $(seq 1 "$TIMEOUT"); do
  if ! kill -0 "$GW_PID" 2>/dev/null; then
    echo "ERROR: Gateway process died during startup"
    exit 1
  fi
  if curl -sf "http://localhost:${PORT}/api/v1/health" > /dev/null 2>&1; then
    echo "Gateway healthy after ${i}s"
    exit 0
  fi
  sleep 1
done

echo "ERROR: Gateway failed to respond within ${TIMEOUT}s"
exit 1
