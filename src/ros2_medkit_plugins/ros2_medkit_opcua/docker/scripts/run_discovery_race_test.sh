#!/usr/bin/env bash
# Copyright 2026 mfaferek93
#
# Integration test for the config-less discovery start-up race.
#
# The field failure this reproduces: the gateway and the PLC power on
# together, the gateway's start-up scan runs while the PLC is still booting
# and finds nothing, and without a re-scan the plugin retries its fallback
# endpoint for as long as it runs. Only a restart ever found the PLC.
#
# So this scenario starts the gateway FIRST, with discovery on and no
# OPCUA_ENDPOINT_URL, asserts it settled on the fallback endpoint with no
# session, then starts an OPC-UA server and asserts the gateway adopts it
# within two re-scan intervals without being restarted.
#
# Every other opcua docker scenario pins OPCUA_ENDPOINT_URL, which
# short-circuits discovery, so none of them can see this.

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "$0")/../../../../.." && pwd)"
NET_NAME=discovery-race-net
NET_SUBNET=172.31.77.0/24
SERVER_NAME=discovery-race-server
GATEWAY_NAME=discovery-race-gateway
SERVER_PORT=4840
GATEWAY_PORT=8089
RESCAN_INTERVAL_S=5
CONFIG_DIR=/tmp/discovery_race_config

# The fallback the plugin keeps when a scan selects nothing (OpcuaClientConfig).
FALLBACK_ENDPOINT="opc.tcp://localhost:4840"

cleanup() {
  local rc=$?
  if [[ ${rc} -ne 0 ]]; then
    for c in "${SERVER_NAME}" "${GATEWAY_NAME}"; do
      echo "=== ${c} logs (cleanup trap) ===" >&2
      docker logs "${c}" >&2 2>&1 || true
    done
  fi
  docker rm -f "${SERVER_NAME}" "${GATEWAY_NAME}" >/dev/null 2>&1 || true
  docker network rm "${NET_NAME}" >/dev/null 2>&1 || true
  rm -rf "${CONFIG_DIR}"
}
trap cleanup EXIT

fail() {
  echo "  FAIL: $*" >&2
  exit 1
}

status_json() {
  curl -sf "http://localhost:${GATEWAY_PORT}/api/v1/components/discovery_race_runtime/x-plc-status" || echo '{}'
}

status_field() {
  status_json | python3 -c "import json,sys; print(json.load(sys.stdin).get('$1', ''))"
}

cd "${REPO_ROOT}"

# Idempotent teardown of anything a hard-killed earlier run left behind.
docker rm -f "${SERVER_NAME}" "${GATEWAY_NAME}" >/dev/null 2>&1 || true
docker network rm "${NET_NAME}" >/dev/null 2>&1 || true

echo "[1/6] Build images"
docker build --network=host \
  -f src/ros2_medkit_plugins/ros2_medkit_opcua/docker/test_alarm_server/Dockerfile \
  -t ros2_medkit_alarm_test_server:dev . >/dev/null
docker build --network=host \
  -f src/ros2_medkit_plugins/ros2_medkit_opcua/docker/Dockerfile.gateway \
  -t gateway-opcua:discovery-race . >/dev/null

# A /24 keeps the read-only sweep to 254 hosts, so a scan finishes in seconds.
# Discovery rejects anything wider than /16 outright.
docker network create --subnet "${NET_SUBNET}" "${NET_NAME}" >/dev/null

echo "[2/6] Start the gateway BEFORE any server, discovery on, no endpoint pinned"
mkdir -p "${CONFIG_DIR}"
cat >"${CONFIG_DIR}/discovery_nodes.yaml" <<'EOF'
area_id: plc_systems
component_id: discovery_race_runtime
# One node is enough: the assertion is on the session, not on any value. The
# node id need not resolve on the server, since a failed read does not drop
# the connection.
nodes:
  - node_id: "ns=2;s=StatusWord"
    entity_id: tank_process
    data_name: status_word
    data_type: int
EOF
cat >"${CONFIG_DIR}/manifest.yaml" <<'EOF'
manifest_version: "1.0"
EOF
cp src/ros2_medkit_plugins/ros2_medkit_opcua/docker/gateway_params.yaml \
   "${CONFIG_DIR}/gateway_params.yaml"

docker run -d --name "${GATEWAY_NAME}" --network "${NET_NAME}" \
  -p "${GATEWAY_PORT}:8080" \
  -v "${CONFIG_DIR}:/config:ro" \
  -e ROS_DOMAIN_ID=67 \
  -e OPCUA_DISCOVERY_ENABLED=1 \
  -e OPCUA_DISCOVERY_SUBNETS="${NET_SUBNET}" \
  -e OPCUA_DISCOVERY_INTERVAL_S="${RESCAN_INTERVAL_S}" \
  -e OPCUA_NODE_MAP_PATH=/config/discovery_nodes.yaml \
  gateway-opcua:discovery-race \
  bash -c '
    set -e
    mkdir -p /var/lib/ros2_medkit/rosbags
    source /opt/ros/jazzy/setup.bash
    source /root/ws/install/setup.bash
    ros2 run ros2_medkit_fault_manager fault_manager_node \
      > /var/lib/ros2_medkit/fault_manager.log 2>&1 &
    PLUGIN_PATH=$(find /root/ws/install -name "libros2_medkit_opcua_plugin.so" | head -1)
    exec ros2 run ros2_medkit_gateway gateway_node \
      --ros-args --params-file /config/gateway_params.yaml \
      -p plugins.opcua.path:="${PLUGIN_PATH}" \
      -p discovery.mode:=hybrid \
      -p discovery.manifest_path:=/config/manifest.yaml \
      -p discovery.manifest_strict_validation:=false' >/dev/null

echo "[3/6] Wait for the REST API"
for _ in $(seq 1 60); do
  if curl -sf "http://localhost:${GATEWAY_PORT}/api/v1/components" >/dev/null 2>&1; then
    break
  fi
  sleep 1
done
curl -sf "http://localhost:${GATEWAY_PORT}/api/v1/components" >/dev/null \
  || fail "gateway REST API never came up"

echo "[4/6] Assert the start-up scan found nothing and left the fallback endpoint"
# The scan runs during set_context(), so by the time the API answers it has
# already completed against an empty network.
endpoint="$(status_field endpoint_url)"
connected="$(status_field connected)"
[[ "${endpoint}" == "${FALLBACK_ENDPOINT}" ]] \
  || fail "expected the fallback endpoint before the server exists, got '${endpoint}'"
[[ "${connected}" == "False" ]] \
  || fail "expected no session before the server exists, got connected='${connected}'"
echo "  OK no server found at start-up, endpoint left at ${FALLBACK_ENDPOINT}"

echo "[5/6] Start the OPC-UA server (the PLC finishing its boot)"
docker run -d --name "${SERVER_NAME}" --network "${NET_NAME}" \
  ros2_medkit_alarm_test_server:dev --port "${SERVER_PORT}" >/dev/null
for _ in $(seq 1 30); do
  if docker logs "${SERVER_NAME}" 2>&1 | grep -q '^READY '; then
    break
  fi
  sleep 1
done
docker logs "${SERVER_NAME}" 2>&1 | grep -q '^READY ' || fail "test server never became ready"
SERVER_IP="$(docker inspect -f "{{(index .NetworkSettings.Networks \"${NET_NAME}\").IPAddress}}" "${SERVER_NAME}")"
echo "  server up at ${SERVER_IP}:${SERVER_PORT}"

echo "[6/6] Assert the gateway adopts it within two re-scan intervals, unrestarted"
# Budget: two intervals for the re-scan to come round, plus the sweep and
# connect themselves. Generous enough not to flake, far short of "never",
# which is what the bug did.
DEADLINE=$((SECONDS + 2 * RESCAN_INTERVAL_S + 40))
adopted=""
while [[ ${SECONDS} -lt ${DEADLINE} ]]; do
  endpoint="$(status_field endpoint_url)"
  connected="$(status_field connected)"
  if [[ "${endpoint}" == "opc.tcp://${SERVER_IP}:${SERVER_PORT}" && "${connected}" == "True" ]]; then
    adopted="${endpoint}"
    break
  fi
  sleep 2
done
[[ -n "${adopted}" ]] \
  || fail "endpoint still '${endpoint}' (connected='${connected}') after $((2 * RESCAN_INTERVAL_S + 40))s"
echo "  OK re-scan adopted ${adopted} without a gateway restart"

# The gateway must have adopted the server in the process that started before
# it, not in a fresh one: a restarted container would pass the check above
# while proving nothing.
restarts="$(docker inspect -f '{{.RestartCount}}' "${GATEWAY_NAME}")"
[[ "${restarts}" == "0" ]] || fail "gateway restarted ${restarts} time(s) during the run"
echo "  OK gateway never restarted"

echo "Discovery race scenario passed."
