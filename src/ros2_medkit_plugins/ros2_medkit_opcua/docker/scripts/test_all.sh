#!/usr/bin/env bash
# Full end-to-end integration test: build containers, start them, run the 16
# assertions against the OpenPLC tank demo, then clean up.
# Usage: from the ros2_medkit repo root, run
#     bash src/ros2_medkit_plugins/ros2_medkit_opcua/docker/scripts/test_all.sh
#
# MEDKIT_OPCUA_VARIANT selects the write surface of the image under test
# (read-only, the default and what ships, or write-capable). It drives both the
# image build and the expectations the suite applies, so the two cannot drift.
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
export MEDKIT_OPCUA_VARIANT="$VARIANT"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DOCKER_DIR="$(dirname "$SCRIPT_DIR")"
PLUGIN_DIR="$(dirname "$DOCKER_DIR")"
REPO_ROOT="$(cd "$PLUGIN_DIR/../../.." && pwd)"

YELLOW='\033[1;33m'
NC='\033[0m'

cleanup() {
    echo -e "\n${YELLOW}Cleaning up...${NC}"
    docker rm -f openplc gateway 2>/dev/null || true
    docker network rm plc-demo 2>/dev/null || true
}
trap cleanup EXIT

echo -e "${YELLOW}=== ros2_medkit_opcua Integration Test ===${NC}\n"

# 1. Build OpenPLC tank demo container
echo -e "${YELLOW}Step 1: Build OpenPLC container${NC}"
docker build -t openplc-tank "$DOCKER_DIR/openplc" 2>&1 | tail -3

# 2. Build gateway image (includes ros2_medkit_opcua plugin)
echo -e "\n${YELLOW}Step 2: Build gateway + OPC-UA plugin image (${VARIANT})${NC}"
cd "$REPO_ROOT"
docker build --build-arg "MEDKIT_OPCUA_READ_ONLY=$READ_ONLY" \
    -f "$DOCKER_DIR/Dockerfile.gateway" -t gateway-opcua . 2>&1 | tail -5

# 3. Start containers on isolated network
echo -e "\n${YELLOW}Step 3: Start containers${NC}"
docker network create plc-demo 2>/dev/null || true
docker run -d --name openplc --network plc-demo openplc-tank >/dev/null
echo "  OpenPLC starting..."
for i in $(seq 1 45); do
    if docker logs openplc 2>&1 | grep -q "PLC State: RUNNING"; then
        echo "  OpenPLC running after $((i * 2))s"
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
        -p discovery.manifest_strict_validation:=false" >/dev/null
echo "  Gateway starting..."
sleep 20

# 4. Run integration tests
echo -e "\n${YELLOW}Step 4: Run integration tests${NC}\n"
bash "$SCRIPT_DIR/run_integration_tests.sh"
