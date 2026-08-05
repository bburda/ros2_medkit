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

"""Docker-based integration tests for the systemd introspection plugin.

These tests run OUTSIDE the Docker container, talking to the gateway
inside it via localhost:9200. The container must be started first with:

    docker compose -f docker-compose.systemd.yml up -d

The gateway runs inside a container with systemd as PID 1, and demo nodes
are managed as systemd services. The systemd plugin queries unit properties
via sd-bus and maps PIDs to units via sd_pid_get_unit().
"""

import os
import time

import pytest
import requests

BASE_URL = os.environ.get(
    'SYSTEMD_TEST_BASE_URL', 'http://localhost:9200/api/v1'
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


def _get_first_app_id():
    """Get first discovered app ID."""
    return _get_app_ids()[0]


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


class TestSystemdAppEndpoint:
    """Tests for GET /apps/{id}/x-medkit-systemd."""

    def test_returns_unit_info(self):
        """Systemd endpoint returns unit info with active_state.

        @verifies REQ_INTEROP_003
        """
        app_id = _get_first_app_id()
        status, data = _poll_endpoint(
            f'{BASE_URL}/apps/{app_id}/x-medkit-systemd'
        )
        assert status == 200, (
            f'systemd endpoint not available for {app_id}'
        )
        assert 'unit' in data
        assert data['unit'].endswith('.service')
        assert data['active_state'] == 'active'
        assert 'sub_state' in data
        assert data['sub_state'] == 'running'

    def test_returns_unit_type(self):
        """Systemd endpoint includes unit_type field.

        @verifies REQ_INTEROP_003
        """
        app_id = _get_first_app_id()
        status, data = _poll_endpoint(
            f'{BASE_URL}/apps/{app_id}/x-medkit-systemd'
        )
        assert status == 200
        assert data['unit_type'] == 'service'

    def test_returns_restart_count(self):
        """Systemd endpoint includes restart_count (NRestarts property).

        @verifies REQ_INTEROP_003
        """
        app_id = _get_first_app_id()
        status, data = _poll_endpoint(
            f'{BASE_URL}/apps/{app_id}/x-medkit-systemd'
        )
        assert status == 200
        assert 'restart_count' in data
        assert isinstance(data['restart_count'], int)
        assert data['restart_count'] >= 0

    def test_returns_watchdog_usec(self):
        """Systemd endpoint includes watchdog_usec field.

        @verifies REQ_INTEROP_003
        """
        app_id = _get_first_app_id()
        status, data = _poll_endpoint(
            f'{BASE_URL}/apps/{app_id}/x-medkit-systemd'
        )
        assert status == 200
        assert 'watchdog_usec' in data
        assert isinstance(data['watchdog_usec'], int)


class TestSystemdComponentEndpoint:
    """Tests for GET /components/{id}/x-medkit-systemd."""

    def test_returns_units_aggregation(self):
        """Component endpoint returns aggregated unit info for child apps.

        @verifies REQ_INTEROP_003
        """
        comp_id = _get_first_component_id()
        status, data = _poll_endpoint(
            f'{BASE_URL}/components/{comp_id}/x-medkit-systemd'
        )
        assert status == 200, (
            f'systemd component endpoint not available for {comp_id}'
        )
        assert 'units' in data
        assert isinstance(data['units'], list)

    def test_units_include_node_ids(self):
        """Each unit in the aggregation includes node_ids listing the apps.

        @verifies REQ_INTEROP_003
        """
        comp_id = _get_first_component_id()
        status, data = _poll_endpoint(
            f'{BASE_URL}/components/{comp_id}/x-medkit-systemd'
        )
        assert status == 200
        if len(data['units']) > 0:
            unit = data['units'][0]
            assert 'node_ids' in unit
            assert isinstance(unit['node_ids'], list)
            assert len(unit['node_ids']) > 0

    def test_units_have_active_state(self):
        """Each unit in the aggregation includes active_state.

        @verifies REQ_INTEROP_003
        """
        comp_id = _get_first_component_id()
        status, data = _poll_endpoint(
            f'{BASE_URL}/components/{comp_id}/x-medkit-systemd'
        )
        assert status == 200
        for unit in data['units']:
            assert 'active_state' in unit
            assert 'unit' in unit


class TestSystemdErrorHandling:
    """Tests for error cases on systemd endpoints."""

    def test_nonexistent_app_returns_404(self):
        """Requesting systemd info for a non-existent app returns 404.

        @verifies REQ_INTEROP_003
        """
        r = requests.get(
            f'{BASE_URL}/apps/nonexistent_app_xyz/x-medkit-systemd',
            timeout=5,
        )
        assert r.status_code == 404

    def test_nonexistent_component_returns_404(self):
        """Requesting systemd info for a non-existent component returns 404.

        @verifies REQ_INTEROP_003
        """
        r = requests.get(
            f'{BASE_URL}/components/nonexistent_comp_xyz/x-medkit-systemd',
            timeout=5,
        )
        assert r.status_code == 404
