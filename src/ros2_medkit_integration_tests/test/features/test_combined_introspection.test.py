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

"""Integration tests for combined procfs + container introspection plugins.

Validates that loading multiple introspection plugins simultaneously works
correctly: procfs returns 200 while container returns 404, verifying route
isolation between plugins.

The container plugin reads a synthetic tree describing a process on a plain
host, so the 404 is a property of the test rather than of the machine. The
tests themselves often run inside a container, where the gateway's own process
really is containerized and the endpoint would correctly answer 200.
"""

import atexit
import os
import shutil
import tempfile
import time
import unittest

import launch_testing
import requests

from ros2_medkit_test_utils.cgroup_fixtures import HOST_MOUNTINFO, v2_cgroup, write_process
from ros2_medkit_test_utils.constants import ALLOWED_EXIT_CODES
from ros2_medkit_test_utils.gateway_test_case import GatewayTestCase
from ros2_medkit_test_utils.launch_helpers import create_test_launch

HOST_PROC_ROOT = tempfile.mkdtemp(prefix='medkit_combined_host_')
atexit.register(shutil.rmtree, HOST_PROC_ROOT, True)

HOST_CGROUP = v2_cgroup('/user.slice/user-1000.slice/session-1.scope')


def _get_plugin_path(plugin_so_name):
    """Get path to a plugin .so file installed by ros2_medkit_linux_introspection."""
    from ament_index_python.packages import get_package_prefix

    pkg_prefix = get_package_prefix('ros2_medkit_linux_introspection')
    return os.path.join(
        pkg_prefix, 'lib', 'ros2_medkit_linux_introspection', plugin_so_name
    )


def generate_test_description():
    """Launch gateway with both procfs and container plugins."""
    procfs_path = _get_plugin_path('libprocfs_introspection.so')
    container_path = _get_plugin_path('libcontainer_introspection.so')

    # Only the container plugin reads the synthetic tree; procfs keeps the real
    # root so it still reports this machine's processes.
    write_process(HOST_PROC_ROOT, 5101, '/powertrain/engine/temp_sensor', HOST_CGROUP,
                  mountinfo=HOST_MOUNTINFO)
    write_process(HOST_PROC_ROOT, 5102, '/powertrain/engine/rpm_sensor', HOST_CGROUP,
                  mountinfo=HOST_MOUNTINFO)

    return create_test_launch(
        demo_nodes=['temp_sensor', 'rpm_sensor'],
        fault_manager=False,
        gateway_params={
            'plugins': ['procfs', 'container'],
            'plugins.procfs.path': procfs_path,
            'plugins.container.path': container_path,
            'plugins.container.proc_root': HOST_PROC_ROOT,
            'plugins.container.pid_cache_ttl_seconds': 1,
        },
    )


class TestCombinedIntrospection(GatewayTestCase):
    """Test procfs + container plugins loaded simultaneously.

    @verifies REQ_INTEROP_003
    """

    MIN_EXPECTED_APPS = 2
    REQUIRED_APPS = {'temp_sensor', 'rpm_sensor'}

    def _get_any_app_id(self):
        """Get the first discovered app ID."""
        data = self.get_json('/apps')
        items = data.get('items', [])
        self.assertGreater(len(items), 0, 'No apps discovered')
        return items[0]['id']

    def _poll_procfs_app(self, app_id, timeout=20.0):
        """Poll procfs endpoint until rss_bytes is populated.

        A freshly forked process can show RssAnon=0 in /proc/<pid>/status
        before any memory is touched, so a 200 response alone is not
        sufficient - wait until rss_bytes > 0 to avoid a timing race
        between process spawn and the first procfs sample.
        """
        start = time.monotonic()
        last_status = None
        last_rss = None
        while time.monotonic() - start < timeout:
            try:
                r = requests.get(
                    f'{self.BASE_URL}/apps/{app_id}/x-medkit-procfs',
                    timeout=2,
                )
                last_status = r.status_code
                if r.status_code == 200:
                    data = r.json()
                    last_rss = data.get('rss_bytes')
                    if last_rss and last_rss > 0:
                        return data
            except requests.exceptions.RequestException:
                pass
            time.sleep(1.0)
        self.fail(
            f'Procfs data not available for app {app_id} after {timeout}s '
            f'(last status: {last_status}, last rss_bytes: {last_rss})'
        )

    def _poll_container_app(self, app_id, timeout=20.0):
        """Poll container endpoint until it returns a definitive response.

        On a host (non-containerized), the endpoint returns 404 once the PID
        cache is populated. Before that, it may return 503 (PID not found).
        We wait for either 200 or 404 (both are valid settled states).
        """
        start = time.monotonic()
        last_status = None
        while time.monotonic() - start < timeout:
            try:
                r = requests.get(
                    f'{self.BASE_URL}/apps/{app_id}/x-medkit-container',
                    timeout=5,
                )
                last_status = r.status_code
                if r.status_code in (200, 404):
                    return r
            except requests.exceptions.RequestException:
                pass
            time.sleep(1.0)
        self.fail(
            f'Container endpoint did not settle for app {app_id} after {timeout}s '
            f'(last status: {last_status})'
        )

    def test_01_procfs_returns_200_on_host(self):
        """Procfs plugin returns 200 with valid process data on host."""
        app_id = self._get_any_app_id()
        data = self._poll_procfs_app(app_id)

        self.assertGreater(data['pid'], 0)
        self.assertGreater(data['rss_bytes'], 0)
        self.assertIsInstance(data['exe'], str)
        self.assertTrue(len(data['exe']) > 0)

    def test_02_container_returns_404_on_host(self):
        """Container plugin returns 404 'not containerized' on a host system."""
        app_id = 'temp_sensor'
        r = self._poll_container_app(app_id)

        self.assertEqual(
            r.status_code,
            404,
            f'Expected 404 on non-containerized host, got {r.status_code}: {r.text}',
        )
        body = r.json()
        # Vendor error codes are wrapped: error_code='vendor-error', vendor_code='x-medkit-*'
        self.assertEqual(body.get('vendor_code'), 'x-medkit-not-containerized')

    def test_03_route_isolation(self):
        """One plugin's error does not affect the other's success.

        Verify procfs still returns 200 even though container returns 404
        for the same entity - routes are isolated between plugins.
        """
        app_id = 'temp_sensor'

        # Container should be 404 (not containerized)
        container_resp = self._poll_container_app(app_id)
        self.assertEqual(container_resp.status_code, 404)

        # Procfs should still return 200 with valid data
        procfs_data = self._poll_procfs_app(app_id)
        self.assertGreater(procfs_data['pid'], 0)


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):
    """Verify gateway exits cleanly."""

    def test_exit_codes(self, proc_info):
        for info in proc_info:
            self.assertIn(
                info.returncode,
                ALLOWED_EXIT_CODES,
                f'Process {info.process_name} exited with {info.returncode}',
            )
