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

"""Container limit reporting under both cgroup namespace modes.

The cgroup namespace mode decides where the limit files sit: `private` mounts
the container's own cgroup at /sys/fs/cgroup and reports a path relative to it,
`host` exposes the whole hierarchy and the reported path leads into it. The
limits configured on the container are the same either way, so the values the
gateway reports have to be the same too.

These tests run OUTSIDE the containers, against the two gateways started by
docker-compose.container.yml:

    docker compose -f docker-compose.container.yml up -d

Both containers are built from the same image and carry the same limits; only
the namespace mode differs.
"""

import os
import re
import time

import pytest
import requests

# Limits set on both services in docker-compose.container.yml.
MEM_LIMIT_BYTES = 536870912  # mem_limit: 512m
CPU_LIMIT_RATIO = 1.0  # cpus: 1.0

GATEWAYS = {
    'private': os.environ.get(
        'CONTAINER_TEST_PRIVATE_URL', 'http://localhost:9210/api/v1'
    ),
    'host': os.environ.get(
        'CONTAINER_TEST_HOSTNS_URL', 'http://localhost:9211/api/v1'
    ),
}

STARTUP_TIMEOUT = 60
POLL_INTERVAL = 1
POLL_RETRIES = 20


def _wait_for_gateway(base_url):
    deadline = time.monotonic() + STARTUP_TIMEOUT
    while time.monotonic() < deadline:
        try:
            r = requests.get(f'{base_url}/health', timeout=2)
            if r.status_code == 200:
                r2 = requests.get(f'{base_url}/apps', timeout=2)
                if r2.status_code == 200 and r2.json().get('items'):
                    return
        except requests.RequestException:
            pass
        time.sleep(1)
    pytest.fail(f'Gateway at {base_url} not ready after {STARTUP_TIMEOUT}s')


def _fetch_container_info(mode, base_url):
    """Container info from an app of one gateway that the endpoint answers for.

    The gateway node is discovered as an App as well, but its own process is
    not in the PID cache, so the first entry of /apps is not usable here.
    """
    _wait_for_gateway(base_url)

    for _ in range(POLL_RETRIES):
        apps = (
            requests.get(f'{base_url}/apps', timeout=5).json().get('items', [])
        )
        assert apps, f'No apps discovered on the {mode} gateway'
        for app in apps:
            r = requests.get(
                f"{base_url}/apps/{app['id']}/x-medkit-container", timeout=5
            )
            if r.status_code == 200:
                return r.json()
        time.sleep(POLL_INTERVAL)

    pytest.fail(f'No app reported container info on the {mode} gateway')


@pytest.fixture(params=sorted(GATEWAYS), scope='module')
def container_info(request):
    """Container info from one of the two gateways."""
    mode = request.param
    return mode, _fetch_container_info(mode, GATEWAYS[mode])


class TestCgroupNamespaceModes:
    """The reported limits must not depend on the cgroup namespace mode."""

    def test_container_is_detected(self, container_info):
        """The container is recognised under either namespace mode.

        The host namespace exposes the container ID in the cgroup path; the
        private namespace reports the namespace root instead, so the ID is
        empty and a runtime marker is what identifies the container.

        @verifies REQ_INTEROP_003
        """
        mode, data = container_info
        if mode == 'host':
            assert re.match(r'^[0-9a-f]{64}$', data['container_id']), (
                f'{mode}: expected a 64-char hex container id, got '
                f"{data['container_id']!r}"
            )
        else:
            assert data['container_id'] == '', (
                f'{mode}: expected an empty container id, got '
                f"{data['container_id']!r}"
            )
        assert data['runtime'] == 'docker', (
            f"{mode}: expected runtime 'docker', got {data['runtime']!r}"
        )

    def test_memory_limit_matches_configured(self, container_info):
        """The reported memory limit is the one set on the container.

        @verifies REQ_INTEROP_003
        """
        mode, data = container_info
        assert data['memory_limit_state'] == 'limited', (
            f'{mode}: memory limit reported as '
            f"{data['memory_limit_state']!r}, expected 'limited' - the "
            f'container is capped at {MEM_LIMIT_BYTES} bytes'
        )
        assert data['memory_limit_bytes'] == MEM_LIMIT_BYTES, (
            f'{mode}: expected {MEM_LIMIT_BYTES} bytes, got '
            f"{data['memory_limit_bytes']}"
        )

    def test_cpu_limit_matches_configured(self, container_info):
        """The reported CPU quota is the one set on the container.

        @verifies REQ_INTEROP_003
        """
        mode, data = container_info
        assert data['cpu_quota_state'] == 'limited', (
            f"{mode}: CPU quota reported as {data['cpu_quota_state']!r}, "
            f"expected 'limited' - the container is capped at "
            f'{CPU_LIMIT_RATIO} CPU'
        )
        ratio = data['cpu_quota_us'] / data['cpu_period_us']
        assert abs(ratio - CPU_LIMIT_RATIO) < 0.01, (
            f'{mode}: expected a CPU ratio of {CPU_LIMIT_RATIO}, got {ratio} '
            f"(quota={data['cpu_quota_us']}, period={data['cpu_period_us']})"
        )

    def test_limit_states_are_always_reported(self, container_info):
        """A client can always tell why a limit value is missing.

        @verifies REQ_INTEROP_003
        """
        mode, data = container_info
        valid = {'limited', 'unlimited', 'unreadable', 'unavailable'}
        for field in ('memory_limit_state', 'cpu_quota_state'):
            assert field in data, f'{mode}: {field} missing from {data}'
            assert data[field] in valid, (
                f'{mode}: {field} is {data[field]!r}, expected one of {valid}'
            )

    def test_value_present_exactly_when_limited(self, container_info):
        """A numeric limit is present if and only if the state says so.

        @verifies REQ_INTEROP_003
        """
        mode, data = container_info
        assert ('memory_limit_bytes' in data) == (
            data['memory_limit_state'] == 'limited'
        ), f'{mode}: memory value and state disagree: {data}'
        assert ('cpu_quota_us' in data) == (
            data['cpu_quota_state'] == 'limited'
        ), f'{mode}: CPU value and state disagree: {data}'


def test_both_namespace_modes_agree():
    """Both namespace modes report the same limits for the same settings.

    This is the regression the joined-path-only reader could not survive: one
    mode reported the limits and the other reported none.

    @verifies REQ_INTEROP_003
    """
    reported = {}
    for mode, base_url in GATEWAYS.items():
        data = _fetch_container_info(mode, base_url)
        reported[mode] = (
            data['memory_limit_state'],
            data.get('memory_limit_bytes'),
            data['cpu_quota_state'],
            data.get('cpu_quota_us'),
            data.get('cpu_period_us'),
        )

    assert reported['private'] == reported['host'], (
        f'Namespace modes disagree: private={reported["private"]}, '
        f'host={reported["host"]}'
    )
