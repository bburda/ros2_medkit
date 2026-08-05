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

"""Integration tests for Software Updates plugin system (Issue #206)."""

import os
import time
import unittest

from launch import LaunchDescription
from launch.actions import TimerAction
import launch_testing
import launch_testing.actions
import requests

from ros2_medkit_test_utils.constants import ALLOWED_EXIT_CODES, API_BASE_PATH, get_test_port
from ros2_medkit_test_utils.gateway_test_case import GatewayTestCase
from ros2_medkit_test_utils.launch_helpers import create_gateway_node


PORT_NO_PLUGIN = get_test_port(0)
PORT_WITH_PLUGIN = get_test_port(1)

HTTP_METHODS = {'get', 'post', 'put', 'delete', 'patch', 'head', 'options'}


def _get_test_plugin_path():
    """Get path to test_update_backend.so demo plugin."""
    from ament_index_python.packages import get_package_prefix

    pkg_prefix = get_package_prefix('ros2_medkit_gateway')
    return os.path.join(
        pkg_prefix, 'lib', 'ros2_medkit_gateway', 'libtest_update_backend.so'
    )


def generate_test_description():
    """Launch two gateways: one without plugin (501 mode), one with demo plugin."""
    plugin_path = _get_test_plugin_path()

    gateway_no_plugin = create_gateway_node(
        name='gateway_no_plugin',
        port=PORT_NO_PLUGIN,
        extra_params={
            'server.host': '127.0.0.1',
            'updates.enabled': True,
        },
    )

    gateway_with_plugin = create_gateway_node(
        name='gateway_with_plugin',
        port=PORT_WITH_PLUGIN,
        extra_params={
            'server.host': '127.0.0.1',
            'updates.enabled': True,
            'plugins': ['test_update_backend'],
            'plugins.test_update_backend.path': plugin_path,
        },
    )

    return LaunchDescription([
        gateway_no_plugin,
        gateway_with_plugin,
        TimerAction(
            period=2.0,
            actions=[launch_testing.actions.ReadyToTest()],
        ),
    ])


class _UpdatesTestMixin:
    """Shared helpers for update tests with plugin gateway.

    Must be mixed with GatewayTestCase (provides BASE_URL, poll_endpoint_until).
    """

    def register_package(self, pkg_id, automated=False, origins=None):
        """Register a test package and schedule cleanup."""
        pkg = {
            'id': pkg_id,
            'update_name': f'Test {pkg_id}',
            'automated': automated,
            'origins': origins or ['proximity'],
        }
        r = requests.post(f'{self.BASE_URL}/updates', json=pkg, timeout=5)
        self.assertEqual(r.status_code, 201)
        self.addCleanup(self._cleanup_package, pkg_id)
        return r

    def _cleanup_package(self, pkg_id):
        """Best-effort cleanup: wait for idle, then delete."""
        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline:
            r = requests.get(
                f'{self.BASE_URL}/updates/{pkg_id}/status', timeout=5
            )
            if r.status_code == 404:
                break
            if r.status_code == 200 and r.json().get('status') in (
                'completed',
                'failed',
            ):
                break
            time.sleep(0.2)
        requests.delete(f'{self.BASE_URL}/updates/{pkg_id}', timeout=5)

    def poll_update_status(self, pkg_id, target_status, timeout=30.0):
        """Poll /updates/{id}/status until target_status reached."""
        return self.poll_endpoint_until(
            f'/updates/{pkg_id}/status',
            lambda d: d if d.get('status') == target_status else None,
            timeout=timeout,
        )


class TestUpdatesNoPlugin(GatewayTestCase):
    """Scenario 1: Without plugin, all /updates endpoints return 501."""

    BASE_URL = f'http://127.0.0.1:{PORT_NO_PLUGIN}{API_BASE_PATH}'
    MIN_EXPECTED_APPS = 0

    # @verifies REQ_INTEROP_082
    def test_01_list_updates_returns_501(self):
        """GET /updates returns 501 when no plugin loaded."""
        r = requests.get(f'{self.BASE_URL}/updates', timeout=5)
        self.assertEqual(r.status_code, 501)
        self.assertEqual(r.json()['error_code'], 'not-implemented')

    # @verifies REQ_INTEROP_085
    def test_02_get_update_returns_501(self):
        """GET /updates/{id} returns 501 when no plugin loaded."""
        r = requests.get(f'{self.BASE_URL}/updates/some-pkg', timeout=5)
        self.assertEqual(r.status_code, 501)
        self.assertEqual(r.json()['error_code'], 'not-implemented')

    # @verifies REQ_INTEROP_083
    def test_03_register_update_returns_501(self):
        """POST /updates returns 501 when no plugin loaded."""
        r = requests.post(
            f'{self.BASE_URL}/updates',
            json={
                'id': 'test',
                'update_name': 'Test',
                'automated': False,
                'origins': ['remote'],
            },
            timeout=5,
        )
        self.assertEqual(r.status_code, 501)
        self.assertEqual(r.json()['error_code'], 'not-implemented')

    # @verifies REQ_INTEROP_091
    def test_04_prepare_returns_501(self):
        """PUT /updates/{id}/prepare returns 501 when no plugin loaded."""
        r = requests.put(
            f'{self.BASE_URL}/updates/some-pkg/prepare', timeout=5
        )
        self.assertEqual(r.status_code, 501)
        self.assertEqual(r.json()['error_code'], 'not-implemented')

    # @verifies REQ_INTEROP_092
    def test_05_execute_returns_501(self):
        """PUT /updates/{id}/execute returns 501 when no plugin loaded."""
        r = requests.put(
            f'{self.BASE_URL}/updates/some-pkg/execute', timeout=5
        )
        self.assertEqual(r.status_code, 501)
        self.assertEqual(r.json()['error_code'], 'not-implemented')

    # @verifies REQ_INTEROP_093
    def test_06_automated_returns_501(self):
        """PUT /updates/{id}/automated returns 501 when no plugin loaded."""
        r = requests.put(
            f'{self.BASE_URL}/updates/some-pkg/automated', timeout=5
        )
        self.assertEqual(r.status_code, 501)
        self.assertEqual(r.json()['error_code'], 'not-implemented')

    # @verifies REQ_INTEROP_094
    def test_07_status_returns_501(self):
        """GET /updates/{id}/status returns 501 when no plugin loaded."""
        r = requests.get(
            f'{self.BASE_URL}/updates/some-pkg/status', timeout=5
        )
        self.assertEqual(r.status_code, 501)
        self.assertEqual(r.json()['error_code'], 'not-implemented')

    # @verifies REQ_INTEROP_084
    def test_08_delete_returns_501(self):
        """DELETE /updates/{id} returns 501 when no plugin loaded."""
        r = requests.delete(f'{self.BASE_URL}/updates/some-pkg', timeout=5)
        self.assertEqual(r.status_code, 501)
        self.assertEqual(r.json()['error_code'], 'not-implemented')

    # @verifies REQ_INTEROP_083
    def test_09_malformed_json_returns_400_not_501(self):
        """A malformed body outranks the feature gate.

        The gate runs inside the typed wrapper, after body parsing, so a broken
        payload is still reported as the client's own error. Moving the guard
        ahead of the parse would answer 501 here and hide the real problem.
        """
        r = requests.post(
            f'{self.BASE_URL}/updates',
            data='not{valid json',
            headers={'Content-Type': 'application/json'},
            timeout=5,
        )
        self.assertEqual(r.status_code, 400, r.text)
        self.assertEqual(r.json()['error_code'], 'invalid-request')

    # @verifies REQ_INTEROP_002
    def test_disabled_feature_returns_501_and_declares_it(self):
        """Every /updates operation documents the 501 the gate returns.

        The eight tests above prove the gateway answers 501 with the feature
        off. This one proves the generated document says so, which is what a
        generated client needs to handle the response instead of treating it
        as an undeclared surprise.
        """
        spec = self.get_json('/docs')
        for path, item in spec['paths'].items():
            if not path.startswith('/updates'):
                continue
            for method, op in item.items():
                if method not in HTTP_METHODS:
                    continue
                self.assertIn('501', op['responses'], f'{method.upper()} {path}')


class TestUpdatesCRUD(_UpdatesTestMixin, GatewayTestCase):
    """Scenario 2: CRUD lifecycle - register, list, get, delete."""

    BASE_URL = f'http://127.0.0.1:{PORT_WITH_PLUGIN}{API_BASE_PATH}'
    MIN_EXPECTED_APPS = 0

    # @verifies REQ_INTEROP_082
    def test_01_list_updates_empty(self):
        """GET /updates returns empty list initially."""
        data = self.get_json('/updates')
        self.assertIn('items', data)
        self.assertIsInstance(data['items'], list)

    # @verifies REQ_INTEROP_083
    def test_02_register_update(self):
        """POST /updates registers a new package."""
        r = self.register_package('test-crud-pkg')
        self.assertIn('Location', r.headers)

    # @verifies REQ_INTEROP_082
    def test_03_list_after_register(self):
        """GET /updates includes registered package."""
        self.register_package('test-list-pkg', origins=['remote'])
        data = self.get_json('/updates')
        self.assertIn('test-list-pkg', data['items'])

    # @verifies REQ_INTEROP_085
    def test_04_get_update_metadata(self):
        """GET /updates/{id} returns full metadata."""
        pkg = {
            'id': 'test-meta-pkg',
            'update_name': 'Metadata Test',
            'automated': True,
            'origins': ['remote', 'proximity'],
            'duration': 300,
            'size': 1024,
            'updated_components': ['comp_a'],
            'affected_components': ['comp_a', 'comp_b'],
        }
        requests.post(f'{self.BASE_URL}/updates', json=pkg, timeout=5)
        self.addCleanup(self._cleanup_package, 'test-meta-pkg')

        data = self.get_json('/updates/test-meta-pkg')
        self.assertEqual(data['id'], 'test-meta-pkg')
        self.assertEqual(data['update_name'], 'Metadata Test')
        self.assertTrue(data['automated'])
        self.assertEqual(data['origins'], ['remote', 'proximity'])
        self.assertEqual(data['duration'], 300)
        self.assertEqual(data['size'], 1024)
        self.assertEqual(data['updated_components'], ['comp_a'])
        self.assertEqual(data['affected_components'], ['comp_a', 'comp_b'])

    # @verifies REQ_INTEROP_085
    def test_05_get_nonexistent_returns_404(self):
        """GET /updates/{id} returns 404 for unknown package."""
        r = requests.get(
            f'{self.BASE_URL}/updates/nonexistent', timeout=5
        )
        self.assertEqual(r.status_code, 404)

    # @verifies REQ_INTEROP_084
    def test_06_delete_update(self):
        """DELETE /updates/{id} removes the package."""
        self.register_package('test-delete-pkg')

        r = requests.delete(
            f'{self.BASE_URL}/updates/test-delete-pkg', timeout=5
        )
        self.assertEqual(r.status_code, 204)

        # Verify deleted
        r = requests.get(
            f'{self.BASE_URL}/updates/test-delete-pkg', timeout=5
        )
        self.assertEqual(r.status_code, 404)

    # @verifies REQ_INTEROP_084
    def test_07_delete_nonexistent_returns_404(self):
        """DELETE /updates/{id} returns 404 for unknown package."""
        r = requests.delete(
            f'{self.BASE_URL}/updates/nonexistent', timeout=5
        )
        self.assertEqual(r.status_code, 404)

    # @verifies REQ_INTEROP_083
    def test_08_register_duplicate_returns_400(self):
        """POST /updates returns 400 when ID already exists."""
        self.register_package('test-dup-pkg', origins=['remote'])
        r = requests.post(
            f'{self.BASE_URL}/updates',
            json={
                'id': 'test-dup-pkg',
                'update_name': 'Dup',
                'automated': False,
                'origins': ['remote'],
            },
            timeout=5,
        )
        self.assertEqual(r.status_code, 400)

    # @verifies REQ_INTEROP_082
    def test_09_list_with_origin_filter(self):
        """GET /updates?origin=remote filters by origin."""
        self.register_package('test-filter-remote', origins=['remote'])
        self.register_package('test-filter-prox', origins=['proximity'])

        r = requests.get(
            f'{self.BASE_URL}/updates?origin=remote', timeout=5
        )
        self.assertEqual(r.status_code, 200)
        items = r.json()['items']
        self.assertIn('test-filter-remote', items)
        self.assertNotIn('test-filter-prox', items)

    # @verifies REQ_INTEROP_082
    def test_10_list_with_target_version_filter(self):
        """GET /updates?target-version=X filters by target version."""
        self.register_package('ver-pkg-1')
        self.register_package('ver-pkg-2')
        r = requests.get(
            f'{self.BASE_URL}/updates?target-version=nonexistent',
            timeout=5,
        )
        self.assertEqual(r.status_code, 200)
        data = r.json()
        self.assertIsInstance(data['items'], list)


class TestUpdatesPrepareExecute(_UpdatesTestMixin, GatewayTestCase):
    """Scenario 3: Prepare + execute async workflow with status polling."""

    BASE_URL = f'http://127.0.0.1:{PORT_WITH_PLUGIN}{API_BASE_PATH}'
    MIN_EXPECTED_APPS = 0

    # @verifies REQ_INTEROP_094
    def test_00_status_pending_right_after_register(self):
        """GET /status returns 200 pending immediately after POST /updates."""
        self.register_package('test-pending-after-register')
        r = requests.get(
            f'{self.BASE_URL}/updates/test-pending-after-register/status',
            timeout=5,
        )
        self.assertEqual(r.status_code, 200)
        data = r.json()
        self.assertEqual(data.get('status'), 'pending')
        self.assertNotIn('progress', data)
        self.assertNotIn('error', data)

    # @verifies REQ_INTEROP_091
    def test_01_prepare_returns_202(self):
        """PUT /updates/{id}/prepare returns 202 Accepted."""
        self.register_package('test-prep-202')
        r = requests.put(
            f'{self.BASE_URL}/updates/test-prep-202/prepare', timeout=5
        )
        self.assertEqual(r.status_code, 202)
        self.assertIn('Location', r.headers)
        self.assertIn('/status', r.headers['Location'])

    # @verifies REQ_INTEROP_091
    def test_02_prepare_completes(self):
        """Prepare workflow completes successfully with status polling."""
        self.register_package('test-prep-complete')
        requests.put(
            f'{self.BASE_URL}/updates/test-prep-complete/prepare', timeout=5
        )
        data = self.poll_update_status('test-prep-complete', 'completed')
        self.assertEqual(data['status'], 'completed')

    # @verifies REQ_INTEROP_092
    def test_03_execute_after_prepare(self):
        """Execute succeeds after prepare completes."""
        self.register_package('test-exec-after-prep')
        requests.put(
            f'{self.BASE_URL}/updates/test-exec-after-prep/prepare',
            timeout=5,
        )
        self.poll_update_status('test-exec-after-prep', 'completed')

        r = requests.put(
            f'{self.BASE_URL}/updates/test-exec-after-prep/execute',
            timeout=5,
        )
        self.assertEqual(r.status_code, 202)
        data = self.poll_update_status('test-exec-after-prep', 'completed')
        self.assertEqual(data['status'], 'completed')

    # @verifies REQ_INTEROP_092
    def test_04_execute_without_prepare_returns_400(self):
        """Execute before prepare returns 400."""
        self.register_package('test-exec-no-prep')
        r = requests.put(
            f'{self.BASE_URL}/updates/test-exec-no-prep/execute', timeout=5
        )
        self.assertEqual(r.status_code, 400)

    # @verifies REQ_INTEROP_094
    def test_05_status_shows_progress(self):
        """Status endpoint shows progress during prepare."""
        self.register_package('test-progress')
        requests.put(
            f'{self.BASE_URL}/updates/test-progress/prepare', timeout=5
        )

        saw_in_progress = False
        deadline = time.monotonic() + 30.0
        while time.monotonic() < deadline:
            r = requests.get(
                f'{self.BASE_URL}/updates/test-progress/status', timeout=5
            )
            if r.status_code == 200:
                data = r.json()
                if data.get('status') == 'inProgress':
                    saw_in_progress = True
                    if 'progress' in data:
                        self.assertIsInstance(data['progress'], int)
                        self.assertGreaterEqual(data['progress'], 0)
                        self.assertLessEqual(data['progress'], 100)
                    if 'sub_progress' in data:
                        self.assertIsInstance(data['sub_progress'], list)
                        for sp in data['sub_progress']:
                            self.assertIn('name', sp)
                            self.assertIn('progress', sp)
                            self.assertIsInstance(sp['progress'], int)
                            self.assertGreaterEqual(sp['progress'], 0)
                            self.assertLessEqual(sp['progress'], 100)
                if data.get('status') == 'completed':
                    break
            time.sleep(0.1)

        self.assertTrue(saw_in_progress, 'Never saw inProgress status')

    # @verifies REQ_INTEROP_094
    def test_06_status_not_found_for_unknown(self):
        """GET /status for unknown package returns 404."""
        r = requests.get(
            f'{self.BASE_URL}/updates/unknown/status', timeout=5
        )
        self.assertEqual(r.status_code, 404)


class TestUpdatesAutomated(_UpdatesTestMixin, GatewayTestCase):
    """Scenario 4: Automated (prepare + execute in one step)."""

    BASE_URL = f'http://127.0.0.1:{PORT_WITH_PLUGIN}{API_BASE_PATH}'
    MIN_EXPECTED_APPS = 0

    # @verifies REQ_INTEROP_093
    def test_01_automated_completes(self):
        """Automated update runs prepare + execute and completes."""
        self.register_package(
            'test-auto-complete', automated=True, origins=['remote']
        )
        r = requests.put(
            f'{self.BASE_URL}/updates/test-auto-complete/automated',
            timeout=5,
        )
        self.assertEqual(r.status_code, 202)
        self.assertIn('Location', r.headers)

        data = self.poll_update_status('test-auto-complete', 'completed')
        self.assertEqual(data['status'], 'completed')

    # @verifies REQ_INTEROP_093
    def test_02_automated_on_non_automated_returns_400(self):
        """Automated on non-automated package returns 400."""
        self.register_package('test-auto-non', automated=False)
        r = requests.put(
            f'{self.BASE_URL}/updates/test-auto-non/automated', timeout=5
        )
        self.assertEqual(r.status_code, 400)


class TestUpdatesErrorCases(_UpdatesTestMixin, GatewayTestCase):
    """Scenario 5: Various error conditions."""

    BASE_URL = f'http://127.0.0.1:{PORT_WITH_PLUGIN}{API_BASE_PATH}'
    MIN_EXPECTED_APPS = 0

    # @verifies REQ_INTEROP_084
    def test_01_delete_during_prepare_returns_409(self):
        """Cannot delete a package while prepare is in progress."""
        self.register_package('test-del-inprog')
        requests.put(
            f'{self.BASE_URL}/updates/test-del-inprog/prepare', timeout=5
        )

        r = requests.delete(
            f'{self.BASE_URL}/updates/test-del-inprog', timeout=5
        )
        self.assertEqual(r.status_code, 409)
        data = r.json()
        self.assertEqual(data['error_code'], 'vendor-error')
        self.assertEqual(data['vendor_code'], 'x-medkit-update-in-progress')

    # @verifies REQ_INTEROP_091
    def test_02_prepare_nonexistent_returns_404(self):
        """Cannot prepare a package that doesn't exist."""
        r = requests.put(
            f'{self.BASE_URL}/updates/ghost-pkg/prepare', timeout=5
        )
        self.assertEqual(r.status_code, 404)
        data = r.json()
        self.assertEqual(data['error_code'], 'vendor-error')
        self.assertEqual(data['vendor_code'], 'x-medkit-update-not-found')

    # @verifies REQ_INTEROP_092
    def test_03_execute_nonexistent_returns_404(self):
        """Cannot execute a package that doesn't exist."""
        r = requests.put(
            f'{self.BASE_URL}/updates/ghost-pkg/execute', timeout=5
        )
        self.assertEqual(r.status_code, 404)
        data = r.json()
        self.assertEqual(data['error_code'], 'vendor-error')
        self.assertEqual(data['vendor_code'], 'x-medkit-update-not-found')

    # @verifies REQ_INTEROP_083
    def test_04_register_missing_required_fields(self):
        """POST /updates with missing required fields returns 400."""
        r = requests.post(
            f'{self.BASE_URL}/updates',
            json={'notes': 'missing id and other required fields'},
            timeout=5,
        )
        self.assertEqual(r.status_code, 400)
        data = r.json()
        self.assertIn('error_code', data)
        self.assertEqual(data['error_code'], 'invalid-request')

    # @verifies REQ_INTEROP_083
    def test_05_register_malformed_json(self):
        """POST /updates with broken JSON returns 400."""
        r = requests.post(
            f'{self.BASE_URL}/updates',
            data='not{valid json',
            headers={'Content-Type': 'application/json'},
            timeout=5,
        )
        self.assertEqual(r.status_code, 400)
        data = r.json()
        self.assertIn('error_code', data)


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):
    """Verify both gateways exit cleanly."""

    def test_exit_codes(self, proc_info):
        for info in proc_info:
            self.assertIn(
                info.returncode,
                ALLOWED_EXIT_CODES,
                f'Process {info.process_name} exited with {info.returncode}',
            )
