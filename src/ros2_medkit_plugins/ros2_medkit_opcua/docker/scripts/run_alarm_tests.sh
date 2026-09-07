#!/usr/bin/env bash
# Copyright 2026 mfaferek93
#
# Integration test for the native OPC-UA AlarmConditionType subscription
# bridge (issue #386). Boots test_alarm_server, points the gateway at it,
# fires alarms via the server's stdin CLI, and asserts that the gateway's
# SOVD ``/faults`` endpoint reflects the expected lifecycle.
#
# Acknowledge / Confirm round-trips go through the SOVD HTTP path
# (POST /apps/{entity}/operations/{op}/executions) so that the medkit
# implementation - lookup_condition + EventId tracking + the condition-method
# call on the inherited AcknowledgeableConditionType methods - is exercised
# end-to-end, not bypassed via the server stdin shortcuts.
#
# MEDKIT_OPCUA_VARIANT selects the write surface of the image under test and
# has to match how it was built (docker build --build-arg
# MEDKIT_OPCUA_READ_ONLY=ON|OFF); this script builds the image itself, so it
# derives the build arg from the variable and the two cannot drift. It defaults
# to read-only, which is what the plugin and the Dockerfile default to.
# Acknowledging a condition changes its state on the controller, so a read-only
# image refuses it: that leg asserts the refusal and that the condition is still
# unacknowledged on the server, then drives the same lifecycle through the
# server's own CLI so every downstream expectation still runs.

set -euo pipefail

VARIANT="${MEDKIT_OPCUA_VARIANT:-read-only}"
if [[ "${VARIANT}" == "read-only" ]]; then
  READ_ONLY=ON
elif [[ "${VARIANT}" == "write-capable" ]]; then
  READ_ONLY=OFF
else
  echo "MEDKIT_OPCUA_VARIANT must be read-only or write-capable (got '${VARIANT}')" >&2
  exit 2
fi

REPO_ROOT="$(cd "$(dirname "$0")/../../../../.." && pwd)"
NET_NAME=alarm-test-net
SERVER_NAME=alarm-test-server
GATEWAY_NAME=alarm-test-gateway
SERVER_PORT=4842
GATEWAY_PORT=8088

cleanup() {
  local rc=$?
  if [[ ${rc} -ne 0 ]]; then
    # Dump container logs to stderr BEFORE removing them so the CI workflow's
    # "Dump container logs on failure" step (which runs after this trap fires
    # and the script exits) is not the only place to look. Without this, an
    # aggressive cleanup hides whatever made gateway / server crash. Use the
    # full log (no tail) so the OPC-UA event subscription / on_event traces -
    # which fire before the diagnostic intropect() polling that floods the
    # last 120 lines - are visible.
    for c in "${SERVER_NAME}" "${GATEWAY_NAME}"; do
      echo "=== ${c} logs (cleanup trap) ===" >&2
      docker logs "${c}" 2>&1 >&2 || true
    done
  fi
  docker rm -f "${SERVER_NAME}" "${GATEWAY_NAME}" 2>/dev/null || true
  docker network rm "${NET_NAME}" 2>/dev/null || true
  # Reap the foreground docker run process for the test_alarm_server (kept
  # alive as a shell background job so the FIFO stays connected). Sending
  # SIGTERM is enough; the container is already removed via ``docker rm -f``.
  if [[ -n "${SERVER_DOCKER_PID:-}" ]]; then
    kill "${SERVER_DOCKER_PID}" 2>/dev/null || true
    wait "${SERVER_DOCKER_PID}" 2>/dev/null || true
  fi
}
trap cleanup EXIT

# Poll <url> until <jq-expr> evaluates true, with a timeout. Mirrors the
# existing run_integration_tests.sh convention; never sleeps blindly.
wait_for() {
  local url="$1" expr="$2" deadline="${3:-60}"
  for _ in $(seq 1 "${deadline}"); do
    if curl -sf "${url}" 2>/dev/null | jq -e "${expr}" >/dev/null 2>&1; then
      return 0
    fi
    sleep 2
  done
  echo "wait_for timed out after ${deadline} polls: ${url} ${expr}" >&2
  curl -sf "${url}" 2>/dev/null | jq . >&2 || true
  return 1
}

# Poll the gateway until the named fault disappears or transitions to a
# quiescent status. Uses the global ``/api/v1/faults`` list (filtered by
# ``fault_code``) because the per-entity endpoint only mirrors faults stored
# directly on the entity, while AlarmCondition events flow through the
# ROS-level fault_manager and surface there.
wait_no_fault() {
  local fault_code="$1" deadline="${2:-30}"
  local url="http://localhost:${GATEWAY_PORT}/api/v1/faults"
  for _ in $(seq 1 "${deadline}"); do
    local status
    status=$(curl -sf "${url}" 2>/dev/null \
             | jq -r --arg code "${fault_code}" \
               '.items[] | select(.fault_code == $code) | .status' \
             | head -1)
    if [[ -z "${status}" || "${status}" == "CLEARED" ]]; then
      return 0
    fi
    sleep 2
  done
  echo "wait_no_fault timed out: ${fault_code} still present" >&2
  curl -sf "${url}" 2>/dev/null | jq . >&2 || true
  return 1
}

assert_status() {
  local fault_code="$1" expected="$2"
  local actual
  actual=$(curl -sf "http://localhost:${GATEWAY_PORT}/api/v1/faults" \
             | jq -r --arg code "${fault_code}" \
               '.items[] | select(.fault_code == $code) | .status' \
             | head -1)
  if [[ "${actual}" != "${expected}" ]]; then
    echo "ASSERT FAILED: ${fault_code} status=${actual}, expected=${expected}" >&2
    return 1
  fi
  echo "  OK ${fault_code}: ${actual}"
}

assert_fault_present() {
  local fault_code="$1"
  local status
  status=$(curl -sf "http://localhost:${GATEWAY_PORT}/api/v1/faults" \
             | jq -r --arg code "${fault_code}" \
               '.items[] | select(.fault_code == $code) | .status' \
             | head -1)
  if [[ -z "${status}" || "${status}" == "CLEARED" ]]; then
    echo "ASSERT FAILED: ${fault_code} expected still raised, got '${status:-absent}'" >&2
    return 1
  fi
  echo "  OK ${fault_code} still raised (${status})"
}

# Poll the global ``/api/v1/faults`` list until the named fault has the
# expected status. Mirrors ``wait_for`` but specialized for the fault list
# shape so callers do not need to construct jq filters per scenario.
wait_until_status() {
  local fault_code="$1" expected="$2" deadline="${3:-30}"
  for _ in $(seq 1 "${deadline}"); do
    local actual
    actual=$(curl -sf "http://localhost:${GATEWAY_PORT}/api/v1/faults" 2>/dev/null \
             | jq -r --arg code "${fault_code}" \
               '.items[] | select(.fault_code == $code) | .status' \
             | head -1)
    if [[ "${actual}" == "${expected}" ]]; then
      echo "  OK ${fault_code}: ${actual}"
      return 0
    fi
    sleep 2
  done
  echo "wait_until_status timed out: ${fault_code} expected=${expected} actual=${actual:-<absent>}" >&2
  curl -sf "http://localhost:${GATEWAY_PORT}/api/v1/faults" 2>/dev/null | jq . >&2 || true
  return 1
}

# Poll the test_alarm_server's stdout (via docker logs) for the latest
# ``STATE <name>`` line and assert that <key>=<expected_value> appears in it.
# Used to verify medkit's SOVD ack / confirm POSTs actually flipped the
# corresponding state on the OPC-UA server.
assert_server_state() {
  local condition="$1" key="$2" expected="$3" deadline="${4:-30}"
  for _ in $(seq 1 "${deadline}"); do
    local line
    line=$(docker logs "${SERVER_NAME}" 2>&1 | grep -E "^STATE ${condition} " | tail -1 || true)
    if [[ "${line}" == *"${key}=${expected}"* ]]; then
      echo "  OK server ${condition} ${key}=${expected}"
      return 0
    fi
    sleep 2
  done
  echo "ASSERT FAILED: server ${condition} ${key} != ${expected}" >&2
  docker logs "${SERVER_NAME}" 2>&1 | grep -E "^STATE ${condition} " | tail -3 >&2 || true
  return 1
}

sovd_post_op() {
  local op="$1" body="$2"
  local url="http://localhost:${GATEWAY_PORT}/api/v1/apps/tank_process/operations/${op}/executions"
  local code
  code=$(curl -s -o /tmp/alarm_test_resp.json -w '%{http_code}' \
              -X POST -H 'Content-Type: application/json' -d "${body}" "${url}")
  if [[ "${code}" != "200" && "${code}" != "201" ]]; then
    echo "SOVD POST ${op} failed: HTTP ${code}" >&2
    cat /tmp/alarm_test_resp.json >&2 || true
    return 1
  fi
  echo "  OK POST ${op} -> ${code}"
}

# The read-only counterpart: the same POST has to come back as the plugin's
# refusal, not as a transport error or a 404, and the body has to name the build
# property so an operator reading it knows no credential will help.
sovd_post_op_refused() {
  local op="$1" body="$2"
  local url="http://localhost:${GATEWAY_PORT}/api/v1/apps/tank_process/operations/${op}/executions"
  local code
  code=$(curl -s -o /tmp/alarm_test_resp.json -w '%{http_code}' \
              -X POST -H 'Content-Type: application/json' -d "${body}" "${url}")
  if [[ "${code}" != "403" ]]; then
    echo "ASSERT FAILED: POST ${op} expected HTTP 403, got ${code}" >&2
    cat /tmp/alarm_test_resp.json >&2 || true
    return 1
  fi
  if ! jq -e '.vendor_code == "x-medkit-plugin-error"' /tmp/alarm_test_resp.json >/dev/null; then
    echo "ASSERT FAILED: POST ${op} refusal carries no x-medkit-plugin-error vendor code" >&2
    cat /tmp/alarm_test_resp.json >&2 || true
    return 1
  fi
  if ! jq -e '.message | test("MEDKIT_OPCUA_READ_ONLY")' /tmp/alarm_test_resp.json >/dev/null; then
    echo "ASSERT FAILED: POST ${op} refusal does not name MEDKIT_OPCUA_READ_ONLY" >&2
    cat /tmp/alarm_test_resp.json >&2 || true
    return 1
  fi
  echo "  OK POST ${op} refused 403 x-medkit-plugin-error naming MEDKIT_OPCUA_READ_ONLY"
}

# What the tree advertises: a read-only image must not offer an operation it
# will refuse, and a write-capable one must offer both.
assert_operations_offered() {
  local url="http://localhost:${GATEWAY_PORT}/api/v1/apps/tank_process/operations"
  local ids
  ids=$(curl -s "${url}" | jq -c '[.items[].id]')
  for op in acknowledge_fault confirm_fault; do
    if [[ "${VARIANT}" == "read-only" ]]; then
      if jq -e --arg op "${op}" 'contains([$op])' <<<"${ids}" >/dev/null; then
        echo "ASSERT FAILED: read-only image advertises ${op} (${ids})" >&2
        return 1
      fi
      echo "  OK ${op} not advertised"
    else
      if ! jq -e --arg op "${op}" 'contains([$op])' <<<"${ids}" >/dev/null; then
        echo "ASSERT FAILED: write-capable image does not advertise ${op} (${ids})" >&2
        return 1
      fi
      echo "  OK ${op} advertised"
    fi
  done
}

# Poll the gateway's docker logs until <pattern> appears. Required because the
# AlarmConditionType subscription has a 500 ms server-side publishing interval -
# new events from method calls or stdin commands take up to that long to arrive
# at the gateway, and the SOVD ack/confirm path needs the gateway to have
# captured the freshest EventId before it issues the next call_method (server
# rejects stale IDs with BadEventIdUnknown).
wait_gateway_log() {
  local pattern="$1" deadline="${2:-30}"
  for _ in $(seq 1 "${deadline}"); do
    if docker logs "${GATEWAY_NAME}" 2>&1 | grep -q -- "${pattern}"; then
      return 0
    fi
    sleep 1
  done
  echo "wait_gateway_log timed out: ${pattern}" >&2
  docker logs "${GATEWAY_NAME}" 2>&1 | tail -40 >&2 || true
  return 1
}

cd "${REPO_ROOT}"

# Idempotent teardown of any leftover state from a previous interrupted run.
# The cleanup trap fires on EXIT but not on a hard kill that bypasses bash
# signal handling - in that case ``docker network create`` below would fail
# with "network already exists" under set -e.
docker rm -f "${SERVER_NAME}" "${GATEWAY_NAME}" 2>/dev/null || true
docker network rm "${NET_NAME}" 2>/dev/null || true

echo "[1/5] Build test_alarm_server image"
docker build --network=host \
  -f src/ros2_medkit_plugins/ros2_medkit_opcua/docker/test_alarm_server/Dockerfile \
  -t ros2_medkit_alarm_test_server:dev . >/dev/null

echo "[2/5] Build gateway-opcua image (${VARIANT}, re-uses existing Dockerfile.gateway)"
docker build --network=host \
  --build-arg "MEDKIT_OPCUA_READ_ONLY=${READ_ONLY}" \
  -f src/ros2_medkit_plugins/ros2_medkit_opcua/docker/Dockerfile.gateway \
  -t gateway-opcua:alarm-test . >/dev/null

docker network create "${NET_NAME}" >/dev/null

echo "[3/5] Start test_alarm_server (with stdin pipe for CLI commands)"
SERVER_CTRL=$(mktemp -d)
mkfifo "${SERVER_CTRL}/stdin"
# Open FIFO read+write on FD 3 first so neither side blocks when docker run
# attaches to it via stdin. Using ``< fifo`` alone would deadlock - the shell
# opens fifo for read before exec'ing docker, blocking on the missing writer.
exec 3<>"${SERVER_CTRL}/stdin"
# Run docker without -d in a background shell job. ``-d -i <&3`` daemonizes
# the docker client which closes its inherited FD before the daemon can wire
# the FIFO into the container's stdin, so commands written to FD 3 never
# reach the binary. Keeping the foreground docker run process alive in the
# script's job table preserves the FIFO open for the container's lifetime.
# shellcheck disable=SC2094
docker run --rm --name "${SERVER_NAME}" --network "${NET_NAME}" \
  -i ros2_medkit_alarm_test_server:dev --port "${SERVER_PORT}" \
  <&3 >/dev/null 2>&1 &
SERVER_DOCKER_PID=$!
# Wait for server to bind; the binary prints "READY ..." after listen.
for _ in $(seq 1 30); do
  if docker logs "${SERVER_NAME}" 2>&1 | grep -q '^READY '; then
    break
  fi
  sleep 1
done

echo "[4/5] Start gateway with alarm-mode node_map (3 conditions)"
mkdir -p /tmp/alarm_test_config
cat >/tmp/alarm_test_config/alarm_nodes.yaml <<EOF
area_id: plc_systems
component_id: alarm_test_runtime
# Polled detection points (status_bits + fault_enum) exercising the shared
# fault-detection evaluator end-to-end alongside the event_alarms below. The
# StatusWord / FaultCode Int32 registers are driven via the server's
# ``set <NodeName> <int>`` CLI command. Fault codes are globally unique vs the
# event_alarms codes so the shared fault tracker never flaps.
nodes:
  - node_id: "ns=2;s=StatusWord"
    entity_id: tank_process
    data_name: status_word
    data_type: int
    status_bits:
      - bit: 3
        fault_code: PLC_PUMP_OVERLOAD
        severity: ERROR
        message: "Pump overload"
  - node_id: "ns=2;s=FaultCode"
    entity_id: tank_process
    data_name: fault_code
    data_type: int
    fault_enum:
      ok_value: 0
      codes:
        - code: 10
          fault_code: PLC_VFD_OVERVOLTAGE
          severity: ERROR
        - code: 11
          fault_code: PLC_VFD_OVERCURRENT
          severity: ERROR
event_alarms:
  - alarm_source: "ns=2;s=Alarms.Overpressure"
    entity_id: tank_process
    fault_code: PLC_OVERPRESSURE
  - alarm_source: "ns=2;s=Alarms.Overheat"
    entity_id: tank_process
    fault_code: PLC_OVERHEAT
  - alarm_source: "ns=2;s=Alarms.SensorLost"
    entity_id: tank_process
    fault_code: PLC_SENSOR_LOST
EOF
# Pre-write the discovery manifest on the host so the gateway entrypoint
# does not need a writable /config mount inside the container.
cat >/tmp/alarm_test_config/manifest.yaml <<EOF
manifest_version: "1.0"
EOF
# Stage gateway_params.yaml into the bind mount. The gateway image bakes
# it at /config/gateway_params.yaml at build time, but our :ro bind mount
# at /config shadows that file - we must include every file the gateway
# needs in the bind mount instead.
cp src/ros2_medkit_plugins/ros2_medkit_opcua/docker/gateway_params.yaml \
   /tmp/alarm_test_config/gateway_params.yaml

docker run -d --name "${GATEWAY_NAME}" --network "${NET_NAME}" \
  -p "${GATEWAY_PORT}:8080" \
  -v /tmp/alarm_test_config:/config:ro \
  -e ROS_DOMAIN_ID=66 \
  -e OPCUA_ENDPOINT_URL="opc.tcp://${SERVER_NAME}:${SERVER_PORT}" \
  -e OPCUA_NODE_MAP_PATH="/config/alarm_nodes.yaml" \
  gateway-opcua:alarm-test \
  bash -c '
    set -e
    mkdir -p /var/lib/ros2_medkit/rosbags
    source /opt/ros/jazzy/setup.bash
    source /root/ws/install/setup.bash
    # Start fault_manager_node first so its services are advertised before
    # the gateway opcua plugin tries to call /fault_manager/report_fault.
    ros2 run ros2_medkit_fault_manager fault_manager_node \
      > /var/lib/ros2_medkit/fault_manager.log 2>&1 &
    # Poll for service advertisement instead of a fixed sleep (Copilot
    # review on PR #387). On slow CI runners the previous ``sleep 3`` was
    # sometimes too short, leaving the gateway to come up before the
    # service was discoverable. ``ros2 service list`` is the cheapest
    # ROS-native availability signal.
    for _ in $(seq 1 30); do
      if ros2 service list 2>/dev/null | grep -q "/fault_manager/report_fault"; then
        break
      fi
      sleep 0.2
    done
    PLUGIN_PATH=$(find /root/ws/install -name "libros2_medkit_opcua_plugin.so" | head -1)
    exec ros2 run ros2_medkit_gateway gateway_node \
      --ros-args --params-file /config/gateway_params.yaml \
      -p plugins.opcua.path:="$PLUGIN_PATH" \
      -p discovery.mode:=hybrid \
      -p discovery.manifest_path:=/config/manifest.yaml \
      -p discovery.manifest_strict_validation:=false
  ' >/dev/null

# Wait for entity discovery; tank_process appears once node_map loads.
wait_for "http://localhost:${GATEWAY_PORT}/api/v1/apps" \
         '.items | map(.id) | contains(["tank_process"])' 60

echo "[5/5] Run alarm scenarios"

echo "  [scenario] fire / ack / latch / confirm / clear lifecycle (${VARIANT})"
assert_operations_offered
echo "fire Overpressure 750" >&3
wait_until_status PLC_OVERPRESSURE CONFIRMED 30

# Real SOVD ack - exercises lookup_condition + call_method(i=9111) + EventId.
# Returns HTTP 200 once the gateway has dispatched the OPC-UA Acknowledge call.
# Note: the SOVD bridge keeps the fault at status=CONFIRMED until ClearFault
# fires (alarm_state_machine.hpp: Healed -> ReportHealed action is a no-op in
# OpcuaPlugin::on_alarm_change because ros2_medkit_msgs/ReportFault has no
# HEALED verb - we deliberately do not flip fault_manager into PASSED-debounce
# territory). The lifecycle proof is therefore ``wait_no_fault`` after the
# follow-up SOVD confirm + the OPC-UA event with all three states cleared.
if [[ "${VARIANT}" == "read-only" ]]; then
  # The image carries no way to acknowledge or confirm, so both calls are
  # refused. Proving they never reached the server needs a consequence rather
  # than a STATE line: the fixture prints those only from its own stdin CLI, so
  # a condition acknowledged over OPC-UA looks identical to one that was not.
  # The consequence used here is that the fault does not clear - had both calls
  # landed, the latch below would have driven it to cleared - and the CLI-driven
  # ack + confirm afterwards does clear it, which is what makes the negative
  # meaningful instead of a symptom of a stalled pipeline.
  sovd_post_op_refused acknowledge_fault \
    '{"fault_code":"PLC_OVERPRESSURE","comment":"e2e ack via SOVD"}'
  sovd_post_op_refused confirm_fault \
    '{"fault_code":"PLC_OVERPRESSURE","comment":"e2e confirm via SOVD"}'

  echo "latch Overpressure" >&3
  wait_gateway_log "AlarmCondition HEALED.*PLC_OVERPRESSURE" 20
  assert_fault_present PLC_OVERPRESSURE
  echo "  OK the refused acknowledge never reached the server"

  echo "ack Overpressure" >&3
  echo "confirm Overpressure" >&3
  wait_no_fault PLC_OVERPRESSURE 30
  echo "  OK PLC_OVERPRESSURE cleared once the server itself acked and confirmed"
else
  sovd_post_op acknowledge_fault \
    '{"fault_code":"PLC_OVERPRESSURE","comment":"e2e ack via SOVD"}'

  # Latch flips ActiveState=false on the server. Combined with the AckedState=
  # true set by the SOVD ack above, the next AlarmCondition event payload has
  # active=false, acked=true, confirmed=false -> SovdAlarmStatus::Healed
  # (state machine internal), action=ReportHealed (no-op for fault_manager).
  # /faults still shows CONFIRMED here, by design.
  echo "latch Overpressure" >&3

  # Wait for the gateway to actually receive and process the latch event before
  # issuing SOVD confirm. Without this, the gateway still has the EventId from
  # the original fire payload and the OPC-UA Confirm method on the server
  # returns BadEventIdUnknown (the server's branch->lastEventId has been
  # superseded by the Acknowledge auto-emit and the latch trigger).
  wait_gateway_log "AlarmCondition HEALED.*PLC_OVERPRESSURE" 20

  # Real SOVD confirm - exercises call_method(i=9113) + EventId. After this
  # ConfirmedState=true on the server; the resulting event has all three of
  # Active=false, Acked=true, Confirmed=true and the state machine emits
  # ClearFault, removing the entry from /faults.
  sovd_post_op confirm_fault \
    '{"fault_code":"PLC_OVERPRESSURE","comment":"e2e confirm via SOVD"}'
  wait_no_fault PLC_OVERPRESSURE 30
  echo "  OK PLC_OVERPRESSURE cleared after SOVD ack + latch + SOVD confirm"
fi

echo "  [scenario] shelving suppression"
echo "fire Overheat 600" >&3
wait_until_status PLC_OVERHEAT CONFIRMED 30
echo "shelve Overheat" >&3
wait_no_fault PLC_OVERHEAT 30
echo "  OK PLC_OVERHEAT suppressed by Shelving"
echo "unshelve Overheat" >&3
echo "fire Overheat 700" >&3
wait_until_status PLC_OVERHEAT CONFIRMED 30
echo "  OK PLC_OVERHEAT re-armed after Unshelve"

echo "  [scenario] disabled alarm suppression"
echo "fire SensorLost 800" >&3
wait_until_status PLC_SENSOR_LOST CONFIRMED 30
echo "disable SensorLost" >&3
wait_no_fault PLC_SENSOR_LOST 30
echo "  OK PLC_SENSOR_LOST suppressed by EnabledState=false"
echo "enable SensorLost" >&3
echo "fire SensorLost 900" >&3
wait_until_status PLC_SENSOR_LOST CONFIRMED 30
echo "  OK PLC_SENSOR_LOST re-armed after Enable"

echo "  [scenario] status-word bit decode (poll path)"
# bit 3 set => value 8 raises PLC_PUMP_OVERLOAD; clearing the word clears it.
echo "set StatusWord 8" >&3
wait_until_status PLC_PUMP_OVERLOAD CONFIRMED 30
echo "set StatusWord 0" >&3
wait_no_fault PLC_PUMP_OVERLOAD 30
echo "  OK PLC_PUMP_OVERLOAD raised on bit3 then cleared"

echo "  [scenario] fault-code enum decode (poll path)"
# code 11 => PLC_VFD_OVERCURRENT; transition to code 10 must raise
# PLC_VFD_OVERVOLTAGE and clear PLC_VFD_OVERCURRENT; ok_value clears both.
echo "set FaultCode 11" >&3
wait_until_status PLC_VFD_OVERCURRENT CONFIRMED 30
echo "set FaultCode 10" >&3
wait_until_status PLC_VFD_OVERVOLTAGE CONFIRMED 30
wait_no_fault PLC_VFD_OVERCURRENT 30
echo "  OK enum transition 11->10: PLC_VFD_OVERVOLTAGE raised, PLC_VFD_OVERCURRENT cleared"
echo "set FaultCode 0" >&3
wait_no_fault PLC_VFD_OVERVOLTAGE 30
wait_no_fault PLC_VFD_OVERCURRENT 30
echo "  OK both VFD faults cleared on ok_value"

echo "  [scenario] reconnect re-subscribes after server restart"
# This scenario does NOT verify ConditionRefresh re-emit - the test_alarm_server
# is in-memory and loses condition state on restart, so the natural
# Part 9 §5.5.7 contract (server replays retained conditions on RefreshStartEvent)
# cannot fire here. Issue #389 tracks adding a fixture that supports it.
# What this scenario DOES verify: gateway detects disconnect, retries until the
# server returns, re-runs setup_event_subscriptions(), and a freshly fired
# alarm flows through the bridge end-to-end after the reconnect.
# Pre-clear Overpressure so the next fire is a fresh CONFIRMED event.
echo "clear Overpressure" >&3
wait_no_fault PLC_OVERPRESSURE 30
echo "fire Overpressure 750" >&3
wait_until_status PLC_OVERPRESSURE CONFIRMED 30

# Drop the stdin pipe and stop the server. The gateway should detect the
# disconnect and back off until the server returns.
exec 3>&-
docker stop "${SERVER_NAME}" >/dev/null

# Restart the same server image with the SAME network alias so the gateway's
# OPC-UA endpoint URL still resolves. The fixture starts with the previous
# Overpressure condition still ACTIVE in its in-memory state, but the new
# server process resets its node tree. We instead re-fire the condition
# immediately after RESTART so the test verifies the gateway's reconnect +
# subscribe behavior rather than open62541's lack of persistence.
docker rm -f "${SERVER_NAME}" >/dev/null 2>&1 || true
mkfifo "${SERVER_CTRL}/stdin2"
exec 3<>"${SERVER_CTRL}/stdin2"
docker run --rm --name "${SERVER_NAME}" --network "${NET_NAME}" \
  -i ros2_medkit_alarm_test_server:dev --port "${SERVER_PORT}" \
  <&3 >/dev/null 2>&1 &
SERVER_DOCKER_PID=$!
for _ in $(seq 1 30); do
  if docker logs "${SERVER_NAME}" 2>&1 | grep -q '^READY '; then
    break
  fi
  sleep 1
done

# After the server returns the gateway re-runs ``setup_event_subscriptions``
# (triggered by the OpcuaPoller reconnect path); a fresh fire should make
# its way through the ConditionRefresh-aware bridge into /faults.
echo "fire Overpressure 750" >&3
wait_until_status PLC_OVERPRESSURE CONFIRMED 60
echo "  OK PLC_OVERPRESSURE re-armed after gateway reconnect"

echo "All alarm scenarios passed."
exec 3>&-
