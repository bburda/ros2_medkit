#!/usr/bin/env bash
# Start OpenPLC + medkit gateway for manual testing.
# Usage: from the ros2_medkit repo root, run
#     bash src/ros2_medkit_plugins/ros2_medkit_opcua/docker/scripts/start.sh
#
# MEDKIT_OPCUA_VARIANT=write-capable brings up an image with the OPC-UA write
# path compiled in. The default, read-only, is what ships.
set -eo pipefail

VARIANT="${MEDKIT_OPCUA_VARIANT:-read-only}"
if [ "$VARIANT" = "read-only" ]; then
    READ_ONLY=ON
elif [ "$VARIANT" = "write-capable" ]; then
    READ_ONLY=OFF
else
    echo "MEDKIT_OPCUA_VARIANT must be read-only or write-capable (got '$VARIANT')" >&2
    exit 2
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DOCKER_DIR="$(dirname "$SCRIPT_DIR")"
PLUGIN_DIR="$(dirname "$DOCKER_DIR")"
# Repo root = 4 levels up from script (scripts -> docker -> ros2_medkit_opcua
# -> ros2_medkit_plugins -> src -> repo root)
REPO_ROOT="$(cd "$PLUGIN_DIR/../../.." && pwd)"

echo "=== Building OpenPLC ==="
docker build -t openplc-tank "$DOCKER_DIR/openplc" 2>&1 | tail -3

echo ""
echo "=== Building gateway + OPC-UA plugin ($VARIANT) ==="
cd "$REPO_ROOT"
docker build --build-arg "MEDKIT_OPCUA_READ_ONLY=$READ_ONLY" \
    -f "$DOCKER_DIR/Dockerfile.gateway" -t gateway-opcua . 2>&1 | tail -5

echo ""
echo "=== Starting containers ==="
docker rm -f openplc gateway 2>/dev/null || true
docker network create plc-demo 2>/dev/null || true

docker run -d --name openplc --network plc-demo -p 4840:4840 openplc-tank
echo "OpenPLC starting..."
for _ in $(seq 1 45); do
    if docker logs openplc 2>&1 | grep -q "PLC State: RUNNING"; then
        echo "  OpenPLC running"
        break
    fi
    sleep 2
done

docker run -d --name gateway --network plc-demo -p 8080:8080 \
    -e ROS_DOMAIN_ID=60 \
    -e OPCUA_ENDPOINT_URL="opc.tcp://openplc:4840/openplc/opcua" \
    -e OPCUA_NODE_MAP_PATH="/config/tank_nodes.yaml" \
    gateway-opcua \
    bash -c "
      mkdir -p /var/lib/ros2_medkit/rosbags /config
      echo 'manifest_version: \"1.0\"' > /config/manifest.yaml
      source /opt/ros/jazzy/setup.bash && source /root/ws/install/setup.bash
      PLUGIN_PATH=\$(find /root/ws/install -name 'libros2_medkit_opcua_plugin.so' | head -1)
      ros2 run ros2_medkit_gateway gateway_node \
        --ros-args --params-file /config/gateway_params.yaml \
        -p plugins.opcua.path:=\$PLUGIN_PATH \
        -p discovery.mode:=hybrid \
        -p discovery.manifest_path:=/config/manifest.yaml \
        -p discovery.manifest_strict_validation:=false"

echo "Gateway starting..."

for _ in $(seq 1 30); do
    if curl -sf http://localhost:8080/api/v1/apps 2>/dev/null | jq -e '.items | map(.id) | contains(["tank_process"])' >/dev/null 2>&1; then
        echo ""
        echo "============================================"
        echo "  Ready! Gateway on http://localhost:8080"
        echo "============================================"
        echo ""
        echo "Stop:  bash scripts/stop.sh"
        echo "Tests: bash scripts/run_integration_tests.sh"
        exit 0
    fi
    sleep 2
done

echo "WARNING: PLC entities not yet discovered"
