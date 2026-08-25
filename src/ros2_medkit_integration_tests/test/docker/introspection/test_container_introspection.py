#!/usr/bin/env python3
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

"""Docker-based integration tests for the container introspection plugin.

These tests run OUTSIDE the Docker container, talking to the gateway
inside it via localhost:9210. The container must be started first with:

    docker compose -f docker-compose.container.yml up -d

The gateway runs inside a Docker container with resource limits (512MB RAM,
1 CPU). The container plugin detects containerization by reading cgroup
paths and extracts container ID, runtime, and resource limits.
"""

import os
import re
import time

import pytest
import requests

BASE_URL = os.environ.get(
    'CONTAINER_TEST_BASE_URL', 'http://localhost:9210/api/v1'
)
STARTUP_TIMEOUT = 60
POLL_INTERVAL = 1
POLL_RETRIES = 20


@pytest.fixture(scope='module', autouse=True)
def wait_for_gateway():
    """Wait for gateway health and app discovery inside Docker container."""
    deadline = time.monotonic() + STARTUP_TIMEOUT
    while time.monotonic() < deadline:
        try:
            r = requests.get(f'{BASE_URL}/health', timeout=2)
            if r.status_code == 200:
                # Also wait for at least 2 apps (temp_sensor + rpm_sensor)
                r2 = requests.get(f'{BASE_URL}/apps', timeout=2)
                if r2.status_code == 200:
                    apps = r2.json().get('items', [])
                    if len(apps) >= 2:
                        return
        except requests.RequestException:
            pass
        time.sleep(1)
    pytest.fail(f'Gateway not ready after {STARTUP_TIMEOUT}s')


def _get_app_ids():
    """Get all discovered app IDs."""
    r = requests.get(f'{BASE_URL}/apps', timeout=5)
    r.raise_for_status()
    items = r.json().get('items', [])
    assert len(items) > 0, 'No apps discovered'
    return [item['id'] for item in items]


_container_app_id = None


def _get_first_app_id():
    """Get an app ID the container endpoint can answer for.

    The gateway node is discovered as an App too, but its own process is not in
    the PID cache, so the first entry of /apps is not usable here. Scan for one
    that answers, and retry while the PID cache is still filling.
    """
    global _container_app_id
    if _container_app_id is not None:
        return _container_app_id

    for _ in range(POLL_RETRIES):
        for app_id in _get_app_ids():
            r = requests.get(
                f'{BASE_URL}/apps/{app_id}/x-medkit-container', timeout=5
            )
            if r.status_code == 200:
                _container_app_id = app_id
                return app_id
        time.sleep(POLL_INTERVAL)
    pytest.fail(f'No app reported container info at {BASE_URL}')


def _get_first_component_id():
    """Get first discovered component ID."""
    r = requests.get(f'{BASE_URL}/components', timeout=5)
    r.raise_for_status()
    items = r.json().get('items', [])
    assert len(items) > 0, 'No components discovered'
    return items[0]['id']


def _poll_endpoint(url, retries=POLL_RETRIES, interval=POLL_INTERVAL):
    """Poll an endpoint until it returns 200. Returns (status_code, json)."""
    for _ in range(retries):
        r = requests.get(url, timeout=5)
        if r.status_code == 200:
            return r.status_code, r.json()
        time.sleep(interval)
    return r.status_code, None


class TestContainerAppEndpoint:
    """Tests for GET /apps/{id}/x-medkit-container."""

    def test_returns_container_info(self):
        """Container endpoint returns info when running inside Docker.

        @verifies REQ_INTEROP_003
        """
        app_id = _get_first_app_id()
        status, data = _poll_endpoint(
            f'{BASE_URL}/apps/{app_id}/x-medkit-container'
        )
        assert status == 200, (
            f'container endpoint not available for {app_id}'
        )
        assert 'container_id' in data
        assert 'runtime' in data

    def test_container_id_reflects_the_namespace_mode(self):
        """Container ID is present only when the cgroup namespace exposes it.

        This gateway runs with cgroup: private, where /proc/self/cgroup reports
        the namespace root and carries no ID, so the field is empty. The
        64-character form is exercised against the host-namespace gateway in
        test_container_cgroup_layouts.py.

        @verifies REQ_INTEROP_003
        """
        app_id = _get_first_app_id()
        status, data = _poll_endpoint(
            f'{BASE_URL}/apps/{app_id}/x-medkit-container'
        )
        assert status == 200
        cid = data['container_id']
        assert cid == '' or re.match(r'^[0-9a-f]{64}$', cid), (
            f'Container ID is neither empty nor valid 64-char hex: {cid!r}'
        )

    def test_runtime_is_docker(self):
        """Runtime should be detected as 'docker' when running in Docker.

        @verifies REQ_INTEROP_003
        """
        app_id = _get_first_app_id()
        status, data = _poll_endpoint(
            f'{BASE_URL}/apps/{app_id}/x-medkit-container'
        )
        assert status == 200
        assert data['runtime'] == 'docker', (
            f"Expected runtime 'docker', got '{data['runtime']}'"
        )

    def test_memory_limit_detected(self):
        """Container memory limit should be detected from cgroup.

        docker-compose.container.yml sets 512MB = 536870912 bytes.

        @verifies REQ_INTEROP_003
        """
        app_id = _get_first_app_id()
        status, data = _poll_endpoint(
            f'{BASE_URL}/apps/{app_id}/x-medkit-container'
        )
        assert status == 200
        assert 'memory_limit_bytes' in data, (
            f'memory_limit_bytes missing from response: {data}'
        )
        # 512MB = 536870912 bytes
        assert data['memory_limit_bytes'] == 536870912, (
            f'Expected 536870912 bytes (512MB), '
            f"got {data['memory_limit_bytes']}"
        )

    def test_cpu_quota_detected(self):
        """Container CPU quota should be detected from cgroup.

        docker-compose.container.yml sets cpus: 1.0.
        For cgroup v2 with cpus=1.0, the quota is typically 100000us
        with a period of 100000us.

        @verifies REQ_INTEROP_003
        """
        app_id = _get_first_app_id()
        status, data = _poll_endpoint(
            f'{BASE_URL}/apps/{app_id}/x-medkit-container'
        )
        assert status == 200
        assert 'cpu_quota_us' in data, (
            f'cpu_quota_us missing from response: {data}'
        )
        assert 'cpu_period_us' in data, (
            f'cpu_period_us missing from response: {data}'
        )
        # cpus: 1.0 means quota/period = 1.0
        ratio = data['cpu_quota_us'] / data['cpu_period_us']
        assert abs(ratio - 1.0) < 0.01, (
            f'Expected CPU ratio ~1.0, got {ratio} '
            f"(quota={data['cpu_quota_us']}, "
            f"period={data['cpu_period_us']})"
        )

    def test_all_apps_same_container(self):
        """All apps in the same container should report the same container ID.

        @verifies REQ_INTEROP_003
        """
        app_ids = _get_app_ids()
        container_ids = set()
        for app_id in app_ids:
            status, data = _poll_endpoint(
                f'{BASE_URL}/apps/{app_id}/x-medkit-container'
            )
            if status == 200 and 'container_id' in data:
                container_ids.add(data['container_id'])
        # All apps run in the same Docker container
        assert len(container_ids) == 1, (
            f'Expected 1 unique container ID, got {len(container_ids)}: '
            f'{container_ids}'
        )


class TestContainerComponentEndpoint:
    """Tests for GET /components/{id}/x-medkit-container."""

    def test_returns_containers_aggregation(self):
        """Component endpoint returns aggregated container info.

        @verifies REQ_INTEROP_003
        """
        comp_id = _get_first_component_id()
        status, data = _poll_endpoint(
            f'{BASE_URL}/components/{comp_id}/x-medkit-container'
        )
        assert status == 200, (
            f'container component endpoint not available for {comp_id}'
        )
        assert 'containers' in data
        assert isinstance(data['containers'], list)

    def test_containers_include_node_ids(self):
        """Each container in the aggregation includes node_ids.

        @verifies REQ_INTEROP_003
        """
        comp_id = _get_first_component_id()
        status, data = _poll_endpoint(
            f'{BASE_URL}/components/{comp_id}/x-medkit-container'
        )
        assert status == 200
        if len(data['containers']) > 0:
            container = data['containers'][0]
            assert 'node_ids' in container
            assert isinstance(container['node_ids'], list)
            assert len(container['node_ids']) > 0

    def test_containers_include_runtime(self):
        """Each container in the aggregation includes runtime info.

        @verifies REQ_INTEROP_003
        """
        comp_id = _get_first_component_id()
        status, data = _poll_endpoint(
            f'{BASE_URL}/components/{comp_id}/x-medkit-container'
        )
        assert status == 200
        for container in data['containers']:
            assert 'runtime' in container
            assert 'container_id' in container


class TestContainerErrorHandling:
    """Tests for error cases on container endpoints."""

    def test_nonexistent_app_returns_404(self):
        """Requesting container info for a non-existent app returns 404.

        @verifies REQ_INTEROP_003
        """
        r = requests.get(
            f'{BASE_URL}/apps/nonexistent_app_xyz/x-medkit-container',
            timeout=5,
        )
        assert r.status_code == 404

    def test_nonexistent_component_returns_404(self):
        """Requesting container info for a non-existent component returns 404.

        @verifies REQ_INTEROP_003
        """
        r = requests.get(
            f'{BASE_URL}/components/nonexistent_comp_xyz/x-medkit-container',
            timeout=5,
        )
        assert r.status_code == 404
