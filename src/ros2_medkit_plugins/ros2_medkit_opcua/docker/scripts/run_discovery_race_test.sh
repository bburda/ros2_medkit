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
# It then repeats the race with NO node map at all (the config-less deployment).
# There the component identity is derived from the device, and a gateway that
# scanned before its PLC existed can only name it after the fallback endpoint -
# so the second pass asserts the SOVD component stops being served under that
# provisional name once the PLC is adopted.
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
# Config-less naming: with no node map the component id is derived from the
# device. Before any server exists that can only be the fallback endpoint's
# host. After adoption it is the test server's DI nameplate (Manufacturer "SelfPatch
# Devices" + Model "SPX-1000"), slugified.
FALLBACK_COMPONENT_ID="opcua-localhost"
DEVICE_COMPONENT_ID="selfpatch_devices_spx_1000"

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

# x-plc-status of a named component. The node-map pass pins the id, the
# config-less pass has to look it up first.
status_json_for() {
  curl -sf "http://localhost:${GATEWAY_PORT}/api/v1/components/$1/x-plc-status" || echo '{}'
}

status_json() {
  status_json_for discovery_race_runtime
}

status_field() {
  status_json | python3 -c "import json,sys; print(json.load(sys.stdin).get('$1', ''))"
}

status_field_for() {
  status_json_for "$1" | python3 -c "import json,sys; print(json.load(sys.stdin).get('$2', ''))"
}

component_ids() {
  curl -sf "http://localhost:${GATEWAY_PORT}/api/v1/components" 2>/dev/null |
    python3 -c "
import json,sys
try:
    print(' '.join(c.get('id','') for c in json.load(sys.stdin).get('items', [])))
except Exception:
    print('')
"
}

wait_for_rest_api() {
  for _ in $(seq 1 60); do
    if curl -sf "http://localhost:${GATEWAY_PORT}/api/v1/components" >/dev/null 2>&1; then
      return 0
    fi
    sleep 1
  done
  curl -sf "http://localhost:${GATEWAY_PORT}/api/v1/components" >/dev/null \
    || fail "gateway REST API never came up"
}

cd "${REPO_ROOT}"

# Idempotent teardown of anything a hard-killed earlier run left behind.
docker rm -f "${SERVER_NAME}" "${GATEWAY_NAME}" >/dev/null 2>&1 || true
docker network rm "${NET_NAME}" >/dev/null 2>&1 || true

echo "[1/9] Build images"
docker build --network=host \
  -f src/ros2_medkit_plugins/ros2_medkit_opcua/docker/test_alarm_server/Dockerfile \
  -t ros2_medkit_alarm_test_server:dev . >/dev/null
docker build --network=host \
  -f src/ros2_medkit_plugins/ros2_medkit_opcua/docker/Dockerfile.gateway \
  -t gateway-opcua:discovery-race . >/dev/null

# A /24 keeps the read-only sweep to 254 hosts, so a scan finishes in seconds.
# Discovery rejects anything wider than /16 outright.
docker network create --subnet "${NET_SUBNET}" "${NET_NAME}" >/dev/null

echo "[2/9] Start the gateway BEFORE any server, discovery on, no endpoint pinned"
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

echo "[3/9] Wait for the REST API"
wait_for_rest_api

echo "[4/9] Assert the start-up scan found nothing and left the fallback endpoint"
# The scan runs during set_context(), so by the time the API answers it has
# already completed against an empty network.
endpoint="$(status_field endpoint_url)"
connected="$(status_field connected)"
[[ "${endpoint}" == "${FALLBACK_ENDPOINT}" ]] \
  || fail "expected the fallback endpoint before the server exists, got '${endpoint}'"
[[ "${connected}" == "False" ]] \
  || fail "expected no session before the server exists, got connected='${connected}'"
echo "  OK no server found at start-up, endpoint left at ${FALLBACK_ENDPOINT}"

echo "[5/9] Start the OPC-UA server (the PLC finishing its boot)"
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

echo "[6/9] Assert the gateway adopts it within two re-scan intervals, unrestarted"
# Budget arithmetic. The re-scan is consulted once per reconnect attempt, and
# those are spaced by the reconnect backoff, so the adoption cadence is
# max(interval_s, backoff). The backoff ceiling is capped at interval_s while
# discovery is re-scanning, which is what keeps this budget in terms of
# interval_s alone: worst case is one full interval before the sweep is due plus
# one for the attempt that follows it, and the +40 s covers the sweep of a /24,
# the connect and the REST refresh. Generous enough not to flake, far short of
# "never", which is what the bug did.
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

# ---------------------------------------------------------------------------
# Config-less variant: the same race with NO node map.
#
# Without a node map the SOVD component is named from the device itself. A
# gateway that scanned before its PLC existed has no device to ask, so it can
# only name the component after the fallback endpoint. The defect this covers is
# that identity being pinned once and never revisited: the component kept
# serving opcua-localhost for the life of the process while the plugin was
# happily polling the adopted PLC.
# ---------------------------------------------------------------------------

echo "[7/9] Config-less pass: stop everything, start the gateway with no node map"
docker rm -f "${SERVER_NAME}" "${GATEWAY_NAME}" >/dev/null 2>&1 || true

docker run -d --name "${GATEWAY_NAME}" --network "${NET_NAME}" \
  -p "${GATEWAY_PORT}:8080" \
  -v "${CONFIG_DIR}:/config:ro" \
  -e ROS_DOMAIN_ID=67 \
  -e OPCUA_DISCOVERY_ENABLED=1 \
  -e OPCUA_DISCOVERY_SUBNETS="${NET_SUBNET}" \
  -e OPCUA_DISCOVERY_INTERVAL_S="${RESCAN_INTERVAL_S}" \
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

wait_for_rest_api

echo "[8/9] Assert the component is named after the fallback while nothing answers"
ids=""
DEADLINE=$((SECONDS + 30))
while [[ ${SECONDS} -lt ${DEADLINE} ]]; do
  ids="$(component_ids)"
  if [[ " ${ids} " == *" ${FALLBACK_COMPONENT_ID} "* ]]; then
    break
  fi
  sleep 2
done
[[ " ${ids} " == *" ${FALLBACK_COMPONENT_ID} "* ]] \
  || fail "expected the provisional component '${FALLBACK_COMPONENT_ID}' before any server, got '${ids}'"
echo "  OK config-less component provisionally named ${FALLBACK_COMPONENT_ID}"

echo "[9/9] Start the server and assert the component is renamed from the device"
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

# Same budget as step 6, plus the discovery refresh that republishes entities.
DEADLINE=$((SECONDS + 2 * RESCAN_INTERVAL_S + 60))
renamed=""
while [[ ${SECONDS} -lt ${DEADLINE} ]]; do
  ids="$(component_ids)"
  if [[ " ${ids} " == *" ${DEVICE_COMPONENT_ID} "* ]]; then
    renamed="${DEVICE_COMPONENT_ID}"
    break
  fi
  sleep 2
done
[[ -n "${renamed}" ]] \
  || fail "component still '${ids}' after adoption - expected the device-derived '${DEVICE_COMPONENT_ID}'"

# The renamed component is the one actually polling the adopted PLC, so the id
# the operator sees is not a second, stale entity next to a live opcua-localhost.
endpoint="$(status_field_for "${renamed}" endpoint_url)"
connected="$(status_field_for "${renamed}" connected)"
[[ "${endpoint}" == "opc.tcp://${SERVER_IP}:${SERVER_PORT}" ]] \
  || fail "renamed component reports endpoint '${endpoint}', expected the adopted server"
[[ "${connected}" == "True" ]] \
  || fail "renamed component reports connected='${connected}', expected a live session"
[[ "${renamed}" != "${FALLBACK_COMPONENT_ID}" ]] \
  || fail "component id never moved off ${FALLBACK_COMPONENT_ID}"
echo "  OK component renamed to ${renamed}, connected at ${endpoint}"

restarts="$(docker inspect -f '{{.RestartCount}}' "${GATEWAY_NAME}")"
[[ "${restarts}" == "0" ]] || fail "gateway restarted ${restarts} time(s) during the config-less run"
echo "  OK gateway never restarted"

echo "Discovery race scenario passed."
