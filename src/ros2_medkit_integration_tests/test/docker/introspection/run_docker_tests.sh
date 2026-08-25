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

# Docker-based integration tests for Linux introspection plugins.
#
# These tests require Docker and docker compose. They build container images
# with the gateway + plugins + demo nodes, then run pytest against the
# exposed HTTP API from outside the container.
#
# Usage:
#   ./run_docker_tests.sh           # Run all tests (systemd + container)
#   ./run_docker_tests.sh systemd   # Run only systemd tests
#   ./run_docker_tests.sh container # Run only container tests
#
# Requirements:
#   - docker and docker compose (v2)
#   - pytest and requests (pip install pytest requests)
#
# Port allocations (outside colcon test port range 9100+):
#   - 9200: systemd test gateway
#   - 9210: container test gateway, private cgroup namespace
#   - 9211: container test gateway, host cgroup namespace

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

TARGET="${1:-all}"
FAILED=0

cleanup() {
    echo ""
    echo "=== Cleaning up Docker containers ==="
    docker compose -f docker-compose.systemd.yml down --timeout 10 2>/dev/null || true
    docker compose -f docker-compose.container.yml down --timeout 10 2>/dev/null || true
}
trap cleanup EXIT

wait_for_healthy() {
    local compose_file="$1"
    local service="$2"
    local timeout="${3:-60}"
    local deadline=$((SECONDS + timeout))

    echo "Waiting for $service to be healthy (timeout: ${timeout}s)..."
    while [ $SECONDS -lt $deadline ]; do
        local health
        health=$(docker compose -f "$compose_file" ps --format '{{.Health}}' "$service" 2>/dev/null || echo "")
        if [ "$health" = "healthy" ]; then
            echo "$service is healthy"
            return 0
        fi
        sleep 2
    done
    echo "ERROR: $service did not become healthy within ${timeout}s"
    echo "Container logs:"
    docker compose -f "$compose_file" logs "$service" 2>/dev/null || true
    return 1
}

run_systemd_tests() {
    echo ""
    echo "================================================================"
    echo "=== Systemd Introspection Tests                              ==="
    echo "================================================================"
    echo ""

    echo "--- Building systemd test image ---"
    docker compose -f docker-compose.systemd.yml build

    echo ""
    echo "--- Starting systemd container ---"
    docker compose -f docker-compose.systemd.yml up -d

    if ! wait_for_healthy docker-compose.systemd.yml gateway-systemd 90; then
        echo "FAIL: systemd container did not start properly"
        docker compose -f docker-compose.systemd.yml logs
        docker compose -f docker-compose.systemd.yml down
        return 1
    fi

    echo ""
    echo "--- Running systemd tests ---"
    if pytest test_systemd_introspection.py -v; then
        echo "PASS: systemd tests"
    else
        echo "FAIL: systemd tests"
        FAILED=1
    fi

    docker compose -f docker-compose.systemd.yml down --timeout 10
}

run_container_tests() {
    echo ""
    echo "================================================================"
    echo "=== Container Introspection Tests                            ==="
    echo "================================================================"
    echo ""

    echo "--- Building container test image ---"
    docker compose -f docker-compose.container.yml build

    echo ""
    echo "--- Starting containers (private and host cgroup namespaces) ---"
    docker compose -f docker-compose.container.yml up -d

    for service in gateway-container gateway-container-hostns; do
        if ! wait_for_healthy docker-compose.container.yml "$service" 60; then
            echo "FAIL: $service did not start properly"
            docker compose -f docker-compose.container.yml logs
            docker compose -f docker-compose.container.yml down
            return 1
        fi
    done

    echo ""
    echo "--- Running container tests ---"
    if pytest test_container_introspection.py test_container_cgroup_layouts.py -v; then
        echo "PASS: container tests"
    else
        echo "FAIL: container tests"
        FAILED=1
    fi

    docker compose -f docker-compose.container.yml down --timeout 10
}

case "$TARGET" in
    systemd)
        run_systemd_tests
        ;;
    container)
        run_container_tests
        ;;
    all)
        run_systemd_tests
        run_container_tests
        ;;
    *)
        echo "Usage: $0 [all|systemd|container]"
        exit 1
        ;;
esac

echo ""
if [ $FAILED -ne 0 ]; then
    echo "=== SOME DOCKER INTEGRATION TESTS FAILED ==="
    exit 1
else
    echo "=== All Docker integration tests passed ==="
fi
