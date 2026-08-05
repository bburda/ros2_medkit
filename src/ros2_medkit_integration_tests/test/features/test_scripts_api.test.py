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

"""Integration tests for SOVD Scripts endpoints.

Validates script upload/list/get/delete lifecycle, execution with status polling,
execution termination, and 501 responses when scripts backend is not configured.

Two gateway instances are launched:
  - gateway_with_scripts: scripts.scripts_dir set to a temp directory
  - gateway_no_scripts: no scripts_dir configured (returns 501)

A demo node (temp_sensor) is launched alongside so that entities exist in
discovery for validate_entity_for_route to pass.
"""

import tempfile
import time
import unittest

from launch import LaunchDescription
from launch.actions import TimerAction
import launch_testing
import launch_testing.actions
import requests

from ros2_medkit_test_utils.constants import ALLOWED_EXIT_CODES, API_BASE_PATH, get_test_port
from ros2_medkit_test_utils.gateway_test_case import GatewayTestCase
from ros2_medkit_test_utils.launch_helpers import create_demo_nodes, create_gateway_node


PORT_WITH_SCRIPTS = get_test_port(0)
PORT_NO_SCRIPTS = get_test_port(1)

_SCRIPTS_TMPDIR = tempfile.TemporaryDirectory(prefix='medkit_scripts_test_')
SCRIPTS_DIR = _SCRIPTS_TMPDIR.name

# Simple inline scripts for testing
PYTHON_SCRIPT = '#!/usr/bin/env python3\nimport json\nprint(json.dumps({"result": "ok"}))\n'
PYTHON_SLEEP_SCRIPT = (
    '#!/usr/bin/env python3\nimport time\ntime.sleep(60)\n'
)
SHELL_SCRIPT = '#!/bin/sh\necho "hello"\n'


def generate_test_description():
    """Launch two gateways and a demo node for entity discovery."""
    gateway_with_scripts = create_gateway_node(
        name='gateway_with_scripts',
        port=PORT_WITH_SCRIPTS,
        extra_params={
            'server.host': '127.0.0.1',
            'scripts.scripts_dir': SCRIPTS_DIR,
            'scripts.max_concurrent_executions': 3,
            'scripts.default_timeout_sec': 30,
        },
    )

    gateway_no_scripts = create_gateway_node(
        name='gateway_no_scripts',
        port=PORT_NO_SCRIPTS,
        extra_params={'server.host': '127.0.0.1'},
    )

    # Launch a demo node so entities are available in discovery.
    demo_nodes = create_demo_nodes(['temp_sensor'], lidar_faulty=False)

    return LaunchDescription([
        gateway_with_scripts,
        gateway_no_scripts,
        TimerAction(
            period=2.0,
            actions=demo_nodes + [launch_testing.actions.ReadyToTest()],
        ),
    ])


# ---------------------------------------------------------------------------
# Scenario 1: Scripts backend not configured (501)
# ---------------------------------------------------------------------------

class TestScriptsNotConfigured(GatewayTestCase):
    """When scripts_dir is not set, all script endpoints return 501."""

    BASE_URL = f'http://127.0.0.1:{PORT_NO_SCRIPTS}{API_BASE_PATH}'
    MIN_EXPECTED_APPS = 1
    REQUIRED_APPS = {'temp_sensor'}

    # @verifies REQ_INTEROP_041
    def test_01_list_scripts_returns_501(self):
        """GET /apps/{app}/scripts returns 501 without backend."""
        r = requests.get(
            f'{self.BASE_URL}/apps/temp_sensor/scripts', timeout=5
        )
        self.assertEqual(r.status_code, 501)
        self.assertEqual(r.json()['error_code'], 'not-implemented')

    # @verifies REQ_INTEROP_040
    def test_02_upload_script_returns_501(self):
        """POST /apps/{app}/scripts returns 501 without backend."""
        files = {'file': ('test.py', PYTHON_SCRIPT, 'application/octet-stream')}
        r = requests.post(
            f'{self.BASE_URL}/apps/temp_sensor/scripts',
            files=files,
            timeout=5,
        )
        self.assertEqual(r.status_code, 501)

    # @verifies REQ_INTEROP_042
    def test_03_get_script_returns_501(self):
        """GET /apps/{app}/scripts/{id} returns 501 without backend."""
        r = requests.get(
            f'{self.BASE_URL}/apps/temp_sensor/scripts/some_id', timeout=5
        )
        self.assertEqual(r.status_code, 501)

    # @verifies REQ_INTEROP_043
    def test_04_delete_script_returns_501(self):
        """DELETE /apps/{app}/scripts/{id} returns 501 without backend."""
        r = requests.delete(
            f'{self.BASE_URL}/apps/temp_sensor/scripts/some_id', timeout=5
        )
        self.assertEqual(r.status_code, 501)

    # @verifies REQ_INTEROP_044
    def test_05_start_execution_returns_501(self):
        """POST /apps/{app}/scripts/{id}/executions returns 501."""
        r = requests.post(
            f'{self.BASE_URL}/apps/temp_sensor/scripts/some_id/executions',
            json={'execution_type': 'now'},
            timeout=5,
        )
        self.assertEqual(r.status_code, 501)

    # @verifies REQ_INTEROP_046
    def test_06_get_execution_returns_501(self):
        """GET /apps/{app}/scripts/{id}/executions/{eid} returns 501."""
        r = requests.get(
            f'{self.BASE_URL}/apps/temp_sensor/scripts/s/executions/e',
            timeout=5,
        )
        self.assertEqual(r.status_code, 501)

    # @verifies REQ_INTEROP_047
    def test_07_control_execution_returns_501(self):
        """PUT /apps/{app}/scripts/{id}/executions/{eid} returns 501."""
        r = requests.put(
            f'{self.BASE_URL}/apps/temp_sensor/scripts/s/executions/e',
            json={'action': 'stop'},
            timeout=5,
        )
        self.assertEqual(r.status_code, 501)

    def test_08_delete_execution_returns_501(self):
        """DELETE /apps/{app}/scripts/{id}/executions/{eid} returns 501."""
        r = requests.delete(
            f'{self.BASE_URL}/apps/temp_sensor/scripts/s/executions/e',
            timeout=5,
        )
        self.assertEqual(r.status_code, 501)


# ---------------------------------------------------------------------------
# Scenario 2: CRUD lifecycle
# ---------------------------------------------------------------------------

class TestScriptsCRUD(GatewayTestCase):
    """Upload, list, get, delete lifecycle with scripts backend enabled."""

    BASE_URL = f'http://127.0.0.1:{PORT_WITH_SCRIPTS}{API_BASE_PATH}'
    MIN_EXPECTED_APPS = 1
    REQUIRED_APPS = {'temp_sensor'}

    def _upload_script(self, filename='test_script.py', content=PYTHON_SCRIPT):
        """Upload a script and return (response, script_id)."""
        files = {'file': (filename, content, 'application/octet-stream')}
        r = requests.post(
            f'{self.BASE_URL}/apps/temp_sensor/scripts',
            files=files,
            timeout=5,
        )
        self.assertEqual(
            r.status_code, 201,
            f'Upload failed: {r.status_code} {r.text}',
        )
        data = r.json()
        self.assertIn('id', data)
        return r, data['id']

    # @verifies REQ_INTEROP_041
    def test_01_list_scripts_empty(self):
        """GET /apps/{app}/scripts returns empty items initially."""
        data = self.get_json('/apps/temp_sensor/scripts')
        self.assertIn('items', data)
        self.assertIsInstance(data['items'], list)

    # @verifies REQ_INTEROP_041
    def test_01b_urls_name_the_collection_the_caller_addressed(self):
        """`href`, `_links` and `Location` all use the caller's own collection.

        These used to come from "components if the path says so, else apps".
        The segment is now read out of the request path instead of chosen, so
        this pins both reachable collections - a component caller must not
        start getting `/apps/...` back from the rewritten derivation.

        Areas and functions cannot be checked here: the scripts routes are
        registered for apps and components only (`rest_server.cpp`), so those
        paths never route and no URL is built for them. That is why the old
        guess never actually mislabelled a live response.
        """
        r, script_id = self._upload_script(filename='collection_pin.py')
        self.assertEqual(
            r.headers['Location'],
            f'{API_BASE_PATH}/apps/temp_sensor/scripts/{script_id}')

        listed = self.get_json('/apps/temp_sensor/scripts')
        self.assertEqual(listed['_links']['self'], f'{API_BASE_PATH}/apps/temp_sensor/scripts')
        self.assertEqual(listed['_links']['parent'], f'{API_BASE_PATH}/apps/temp_sensor')
        hrefs = [i['href'] for i in listed['items'] if i['id'] == script_id]
        self.assertEqual(hrefs, [f'{API_BASE_PATH}/apps/temp_sensor/scripts/{script_id}'])

        comp_id = self.get_json('/components')['items'][0]['id']
        comp_listed = self.get_json(f'/components/{comp_id}/scripts')
        self.assertEqual(
            comp_listed['_links']['self'],
            f'{API_BASE_PATH}/components/{comp_id}/scripts')
        self.assertEqual(
            comp_listed['_links']['parent'], f'{API_BASE_PATH}/components/{comp_id}')

    # @verifies REQ_INTEROP_040
    def test_01c_trailing_slash_location_still_resolves(self):
        """A stray trailing slash must not produce a `Location` that 404s.

        Route regexes are anchored with `/?$`, so `POST .../scripts/` routes
        just as `POST .../scripts` does, and nothing normalises the path
        afterwards. Appending the new id to the raw request path then yields
        `.../scripts//new_id`, which no route matches - `([^/]+)` cannot match
        an empty segment - so the header hands the client a dead URI.
        """
        files = {'file': ('slash.py', PYTHON_SCRIPT, 'application/octet-stream')}
        r = requests.post(
            f'{self.BASE_URL}/apps/temp_sensor/scripts/', files=files, timeout=5)
        self.assertEqual(r.status_code, 201, r.text)

        location = r.headers['Location']
        self.assertNotIn('//', location.split('://')[-1])
        self.assertEqual(
            location, f'{API_BASE_PATH}/apps/temp_sensor/scripts/{r.json()["id"]}')

        follow = requests.get(
            self.BASE_URL + location[len(API_BASE_PATH):], timeout=5)
        self.assertEqual(follow.status_code, 200, follow.text)

    # @verifies REQ_INTEROP_040
    def test_02_upload_and_list(self):
        """Upload a script, verify it appears in the list."""
        r, script_id = self._upload_script()
        self.assertIn('Location', r.headers)

        data = self.get_json('/apps/temp_sensor/scripts')
        ids = [item['id'] for item in data['items']]
        self.assertIn(script_id, ids)

    # @verifies REQ_INTEROP_042
    def test_03_get_script_metadata(self):
        """GET /apps/{app}/scripts/{id} returns full metadata."""
        _, script_id = self._upload_script(filename='diag.py')

        data = self.get_json(f'/apps/temp_sensor/scripts/{script_id}')
        self.assertEqual(data['id'], script_id)
        self.assertIn('name', data)
        self.assertIn('href', data)
        self.assertFalse(data['managed'])

    # @verifies REQ_INTEROP_042
    def test_04_get_nonexistent_returns_404(self):
        """GET /apps/{app}/scripts/nonexistent returns 404."""
        r = requests.get(
            f'{self.BASE_URL}/apps/temp_sensor/scripts/nonexistent',
            timeout=5,
        )
        self.assertEqual(r.status_code, 404)

    # @verifies REQ_INTEROP_043
    def test_05_delete_script(self):
        """DELETE /apps/{app}/scripts/{id} removes it."""
        _, script_id = self._upload_script(filename='to_delete.py')

        r = requests.delete(
            f'{self.BASE_URL}/apps/temp_sensor/scripts/{script_id}',
            timeout=5,
        )
        self.assertEqual(r.status_code, 204)

        # Verify deleted
        r = requests.get(
            f'{self.BASE_URL}/apps/temp_sensor/scripts/{script_id}',
            timeout=5,
        )
        self.assertEqual(r.status_code, 404)

    # @verifies REQ_INTEROP_043
    def test_06_delete_nonexistent_returns_404(self):
        """DELETE /apps/{app}/scripts/nonexistent returns 404."""
        r = requests.delete(
            f'{self.BASE_URL}/apps/temp_sensor/scripts/nonexistent',
            timeout=5,
        )
        self.assertEqual(r.status_code, 404)

    # @verifies REQ_INTEROP_040
    def test_07_upload_missing_file_field(self):
        """POST without multipart file field returns 400."""
        r = requests.post(
            f'{self.BASE_URL}/apps/temp_sensor/scripts',
            data='not-multipart',
            headers={'Content-Type': 'application/octet-stream'},
            timeout=5,
        )
        self.assertEqual(r.status_code, 400)

    def test_08_entity_not_found(self):
        """Script endpoints return 404 for nonexistent entity."""
        r = requests.get(
            f'{self.BASE_URL}/apps/nonexistent_app/scripts',
            timeout=5,
        )
        self.assertEqual(r.status_code, 404)

    # @verifies REQ_INTEROP_040
    def test_09_upload_with_metadata(self):
        """Upload with metadata field sets custom name and description."""
        import json as json_mod
        metadata = json_mod.dumps({
            'name': 'Custom Name',
            'description': 'A test script',
        })
        files = {
            'file': ('meta_script.py', PYTHON_SCRIPT, 'application/octet-stream'),
            'metadata': ('metadata', metadata, 'application/json'),
        }
        r = requests.post(
            f'{self.BASE_URL}/apps/temp_sensor/scripts',
            files=files,
            timeout=5,
        )
        self.assertEqual(r.status_code, 201)
        data = r.json()
        script_id = data['id']

        # Verify metadata was applied
        info = self.get_json(f'/apps/temp_sensor/scripts/{script_id}')
        self.assertEqual(info['name'], 'Custom Name')
        self.assertEqual(info['description'], 'A test script')


# ---------------------------------------------------------------------------
# Scenario 3: Execution lifecycle
# ---------------------------------------------------------------------------

class TestScriptsExecution(GatewayTestCase):
    """Script execution: start, poll status, terminate, delete."""

    BASE_URL = f'http://127.0.0.1:{PORT_WITH_SCRIPTS}{API_BASE_PATH}'
    MIN_EXPECTED_APPS = 1
    REQUIRED_APPS = {'temp_sensor'}

    def _upload_script(self, filename='exec_test.py', content=PYTHON_SCRIPT):
        """Upload a script and return the script_id."""
        files = {'file': (filename, content, 'application/octet-stream')}
        r = requests.post(
            f'{self.BASE_URL}/apps/temp_sensor/scripts',
            files=files,
            timeout=5,
        )
        self.assertEqual(r.status_code, 201)
        return r.json()['id']

    def _start_execution(self, script_id, exec_type='now'):
        """Start an execution and return (response, execution_id)."""
        r = requests.post(
            f'{self.BASE_URL}/apps/temp_sensor/scripts/{script_id}/executions',
            json={'execution_type': exec_type},
            timeout=5,
        )
        self.assertEqual(
            r.status_code, 202,
            f'Start execution failed: {r.status_code} {r.text}',
        )
        data = r.json()
        self.assertIn('id', data)
        return r, data['id']

    # @verifies REQ_INTEROP_044
    def test_01_execute_python_script(self):
        """Upload Python script, execute, poll until completed."""
        script_id = self._upload_script(
            filename='fast.py',
            content=PYTHON_SCRIPT,
        )

        r, exec_id = self._start_execution(script_id)
        self.assertIn('Location', r.headers)

        # Poll until completed
        exec_endpoint = f'/apps/temp_sensor/scripts/{script_id}/executions/{exec_id}'
        data = self.poll_endpoint_until(
            exec_endpoint,
            lambda d: d if d.get('status') in ('completed', 'failed') else None,
            timeout=15.0,
            interval=0.5,
        )
        self.assertEqual(data['status'], 'completed')
        # Fast script should produce JSON output
        self.assertIsNotNone(data.get('parameters'))

    # @verifies REQ_INTEROP_044
    def test_02_execute_shell_script(self):
        """Upload shell script, execute, poll until completed."""
        script_id = self._upload_script(
            filename='hello.sh',
            content=SHELL_SCRIPT,
        )

        _, exec_id = self._start_execution(script_id)

        exec_endpoint = f'/apps/temp_sensor/scripts/{script_id}/executions/{exec_id}'
        data = self.poll_endpoint_until(
            exec_endpoint,
            lambda d: d if d.get('status') in ('completed', 'failed') else None,
            timeout=15.0,
            interval=0.5,
        )
        self.assertEqual(data['status'], 'completed')

    # @verifies REQ_INTEROP_047
    def test_03_terminate_running_execution(self):
        """Start long-running script, terminate it via control endpoint."""
        script_id = self._upload_script(
            filename='slow.py',
            content=PYTHON_SLEEP_SCRIPT,
        )

        _, exec_id = self._start_execution(script_id)

        # Poll until execution is running
        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline:
            r = requests.get(
                f'{self.BASE_URL}/apps/temp_sensor/scripts/{script_id}'
                f'/executions/{exec_id}',
                timeout=5,
            )
            if r.status_code == 200 and r.json().get('status') == 'running':
                break
            time.sleep(0.1)

        # Send stop action
        r = requests.put(
            f'{self.BASE_URL}/apps/temp_sensor/scripts/{script_id}'
            f'/executions/{exec_id}',
            json={'action': 'stop'},
            timeout=5,
        )
        self.assertEqual(r.status_code, 200)
        data = r.json()
        self.assertEqual(data['status'], 'terminated')

    def test_04_delete_running_execution_rejected(self):
        """DELETE on a running execution returns 409."""
        script_id = self._upload_script(
            filename='slow2.py',
            content=PYTHON_SLEEP_SCRIPT,
        )

        _, exec_id = self._start_execution(script_id)

        # Poll until execution is running
        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline:
            r = requests.get(
                f'{self.BASE_URL}/apps/temp_sensor/scripts/{script_id}'
                f'/executions/{exec_id}',
                timeout=5,
            )
            if r.status_code == 200 and r.json().get('status') == 'running':
                break
            time.sleep(0.1)

        # Attempt to delete while running
        r = requests.delete(
            f'{self.BASE_URL}/apps/temp_sensor/scripts/{script_id}'
            f'/executions/{exec_id}',
            timeout=5,
        )
        self.assertEqual(r.status_code, 409)
        data = r.json()
        self.assertEqual(data['error_code'], 'vendor-error')
        self.assertEqual(data['vendor_code'], 'x-medkit-script-running')

        # Cleanup: stop the execution
        requests.put(
            f'{self.BASE_URL}/apps/temp_sensor/scripts/{script_id}'
            f'/executions/{exec_id}',
            json={'action': 'stop'},
            timeout=5,
        )

    def test_05_delete_completed_execution(self):
        """DELETE on a completed execution succeeds with 204."""
        script_id = self._upload_script(
            filename='quick.py',
            content=PYTHON_SCRIPT,
        )

        _, exec_id = self._start_execution(script_id)

        # Wait for completion
        exec_endpoint = f'/apps/temp_sensor/scripts/{script_id}/executions/{exec_id}'
        self.poll_endpoint_until(
            exec_endpoint,
            lambda d: d if d.get('status') in ('completed', 'failed') else None,
            timeout=15.0,
            interval=0.5,
        )

        r = requests.delete(
            f'{self.BASE_URL}{exec_endpoint}',
            timeout=5,
        )
        self.assertEqual(r.status_code, 204)

        # Verify deleted
        r = requests.get(
            f'{self.BASE_URL}{exec_endpoint}',
            timeout=5,
        )
        self.assertEqual(r.status_code, 404)

    # @verifies REQ_INTEROP_046
    def test_06_get_nonexistent_execution(self):
        """GET nonexistent execution returns 404."""
        script_id = self._upload_script(filename='for_404.py')
        r = requests.get(
            f'{self.BASE_URL}/apps/temp_sensor/scripts/{script_id}'
            '/executions/nonexistent',
            timeout=5,
        )
        self.assertEqual(r.status_code, 404)

    # @verifies REQ_INTEROP_044
    def test_07_start_execution_missing_type(self):
        """POST without execution_type returns 400."""
        script_id = self._upload_script(filename='no_type.py')
        r = requests.post(
            f'{self.BASE_URL}/apps/temp_sensor/scripts/{script_id}/executions',
            json={},
            timeout=5,
        )
        self.assertEqual(r.status_code, 400)

    # @verifies REQ_INTEROP_044
    def test_08_start_execution_invalid_json(self):
        """POST with invalid JSON body returns 400."""
        script_id = self._upload_script(filename='bad_json.py')
        r = requests.post(
            f'{self.BASE_URL}/apps/temp_sensor/scripts/{script_id}/executions',
            data='not{valid json',
            headers={'Content-Type': 'application/json'},
            timeout=5,
        )
        self.assertEqual(r.status_code, 400)

    # @verifies REQ_INTEROP_046
    def test_09_execution_has_timestamps(self):
        """Completed execution has started_at and completed_at timestamps."""
        script_id = self._upload_script(
            filename='ts_check.py',
            content=PYTHON_SCRIPT,
        )

        _, exec_id = self._start_execution(script_id)

        exec_endpoint = f'/apps/temp_sensor/scripts/{script_id}/executions/{exec_id}'
        data = self.poll_endpoint_until(
            exec_endpoint,
            lambda d: d if d.get('status') in ('completed', 'failed') else None,
            timeout=15.0,
            interval=0.5,
        )
        self.assertEqual(data['status'], 'completed')
        self.assertIsNotNone(data.get('started_at'))
        self.assertIsNotNone(data.get('completed_at'))

    # @verifies REQ_INTEROP_044
    def test_10_bash_script_execution(self):
        """Upload and execute a .bash script."""
        bash_script = '#!/bin/bash\necho "{\\"format\\": \\"bash\\"}"\n'
        script_id = self._upload_script(
            filename='test.bash', content=bash_script
        )
        _, exec_id = self._start_execution(script_id)
        data = self.poll_endpoint_until(
            f'/apps/temp_sensor/scripts/{script_id}/executions/{exec_id}',
            lambda d: d if d.get('status') in ('completed', 'failed') else None,
            timeout=15.0,
        )
        self.assertEqual(data['status'], 'completed')
        # Cleanup
        requests.delete(
            f'{self.BASE_URL}/apps/temp_sensor/scripts/{script_id}'
            f'/executions/{exec_id}',
            timeout=5,
        )
        requests.delete(
            f'{self.BASE_URL}/apps/temp_sensor/scripts/{script_id}',
            timeout=5,
        )

    # @verifies REQ_INTEROP_046
    def test_11_output_content_verified(self):
        """Verify executed script output is correctly returned in execution response."""
        py_script = (
            '#!/usr/bin/env python3\n'
            'import json\n'
            'print(json.dumps({"result": "success", "value": 42}))\n'
        )
        script_id = self._upload_script(
            filename='output_test.py', content=py_script
        )
        _, exec_id = self._start_execution(script_id)
        data = self.poll_endpoint_until(
            f'/apps/temp_sensor/scripts/{script_id}/executions/{exec_id}',
            lambda d: d if d.get('status') == 'completed' else None,
            timeout=15.0,
        )
        self.assertEqual(data['status'], 'completed')
        self.assertIsNotNone(data.get('parameters'))
        self.assertEqual(data['parameters']['result'], 'success')
        self.assertEqual(data['parameters']['value'], 42)
        # Cleanup
        requests.delete(
            f'{self.BASE_URL}/apps/temp_sensor/scripts/{script_id}'
            f'/executions/{exec_id}',
            timeout=5,
        )
        requests.delete(
            f'{self.BASE_URL}/apps/temp_sensor/scripts/{script_id}',
            timeout=5,
        )

    # @verifies REQ_INTEROP_044
    def test_12_parameters_via_stdin(self):
        """Parameters sent in execution request are passed to script via stdin."""
        py_script = (
            '#!/usr/bin/env python3\n'
            'import sys, json\n'
            'data = json.load(sys.stdin)\n'
            'print(json.dumps({"received": data}))\n'
        )
        script_id = self._upload_script(
            filename='stdin_test.py', content=py_script
        )
        # Start execution WITH parameters
        r = requests.post(
            f'{self.BASE_URL}/apps/temp_sensor/scripts/{script_id}/executions',
            json={
                'execution_type': 'now',
                'parameters': {'threshold': 0.5, 'sensor': 'lidar'},
            },
            timeout=5,
        )
        self.assertEqual(r.status_code, 202)
        exec_id = r.json()['id']
        data = self.poll_endpoint_until(
            f'/apps/temp_sensor/scripts/{script_id}/executions/{exec_id}',
            lambda d: d if d.get('status') == 'completed' else None,
            timeout=15.0,
        )
        self.assertEqual(data['status'], 'completed')
        self.assertIsNotNone(data.get('parameters'))
        self.assertEqual(data['parameters']['received']['threshold'], 0.5)
        self.assertEqual(data['parameters']['received']['sensor'], 'lidar')
        # Cleanup
        requests.delete(
            f'{self.BASE_URL}/apps/temp_sensor/scripts/{script_id}'
            f'/executions/{exec_id}',
            timeout=5,
        )
        requests.delete(
            f'{self.BASE_URL}/apps/temp_sensor/scripts/{script_id}',
            timeout=5,
        )

    # @verifies REQ_INTEROP_046
    def test_13_failed_script_execution(self):
        """Script that exits non-zero produces 'failed' status with error info."""
        py_script = (
            '#!/usr/bin/env python3\n'
            'import sys\n'
            'print("error details", file=sys.stderr)\n'
            'sys.exit(1)\n'
        )
        script_id = self._upload_script(
            filename='fail_test.py', content=py_script
        )
        _, exec_id = self._start_execution(script_id)
        data = self.poll_endpoint_until(
            f'/apps/temp_sensor/scripts/{script_id}/executions/{exec_id}',
            lambda d: d if d.get('status') in ('completed', 'failed') else None,
            timeout=15.0,
        )
        self.assertEqual(data['status'], 'failed')
        self.assertIn('error', data)
        self.assertIn('message', data['error'])
        # Cleanup
        requests.delete(
            f'{self.BASE_URL}/apps/temp_sensor/scripts/{script_id}'
            f'/executions/{exec_id}',
            timeout=5,
        )
        requests.delete(
            f'{self.BASE_URL}/apps/temp_sensor/scripts/{script_id}',
            timeout=5,
        )

    # @verifies REQ_INTEROP_040
    # @verifies REQ_INTEROP_041
    # @verifies REQ_INTEROP_042
    # @verifies REQ_INTEROP_043
    # @verifies REQ_INTEROP_044
    # @verifies REQ_INTEROP_046
    def test_14_full_lifecycle(self):
        """Complete lifecycle: upload -> list -> get -> execute -> poll -> verify -> delete."""
        py_script = (
            '#!/usr/bin/env python3\n'
            'import json\n'
            'print(json.dumps({"lifecycle": "complete"}))\n'
        )

        # Upload
        script_id = self._upload_script(
            filename='lifecycle.py', content=py_script
        )

        # List - should contain our script
        r = requests.get(
            f'{self.BASE_URL}/apps/temp_sensor/scripts', timeout=5
        )
        self.assertEqual(r.status_code, 200)
        items = r.json().get('items', [])
        script_ids = [s['id'] for s in items]
        self.assertIn(script_id, script_ids)

        # Get - metadata
        r = requests.get(
            f'{self.BASE_URL}/apps/temp_sensor/scripts/{script_id}',
            timeout=5,
        )
        self.assertEqual(r.status_code, 200)
        self.assertEqual(r.json()['id'], script_id)

        # Execute
        _, exec_id = self._start_execution(script_id)

        # Poll to completion
        data = self.poll_endpoint_until(
            f'/apps/temp_sensor/scripts/{script_id}/executions/{exec_id}',
            lambda d: d if d.get('status') == 'completed' else None,
            timeout=15.0,
        )
        self.assertEqual(data['parameters']['lifecycle'], 'complete')

        # Delete execution
        r = requests.delete(
            f'{self.BASE_URL}/apps/temp_sensor/scripts/{script_id}'
            f'/executions/{exec_id}',
            timeout=5,
        )
        self.assertEqual(r.status_code, 204)

        # Verify execution gone
        r = requests.get(
            f'{self.BASE_URL}/apps/temp_sensor/scripts/{script_id}'
            f'/executions/{exec_id}',
            timeout=5,
        )
        self.assertEqual(r.status_code, 404)

        # Delete script
        r = requests.delete(
            f'{self.BASE_URL}/apps/temp_sensor/scripts/{script_id}',
            timeout=5,
        )
        self.assertEqual(r.status_code, 204)

        # Verify script gone
        r = requests.get(
            f'{self.BASE_URL}/apps/temp_sensor/scripts/{script_id}',
            timeout=5,
        )
        self.assertEqual(r.status_code, 404)


# ---------------------------------------------------------------------------
# Shutdown verification
# ---------------------------------------------------------------------------

@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):
    """Verify all processes exit cleanly."""

    def test_exit_codes(self, proc_info):
        for info in proc_info:
            self.assertIn(
                info.returncode,
                ALLOWED_EXIT_CODES,
                f'Process {info.process_name} exited with {info.returncode}',
            )
