# Copyright 2026 mfaferek93
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

"""End-to-end regression tests for issue #584.

A data trigger on a plugin-provided entity used to pass validation, return
201 ACTIVE, and never fire: topic resolution consulted the ROS-graph topic
list, which attributes plugin-published topics to the gateway node. With the
plugin declaring its value-bridge topics on the owning entity (App::topics),
the trigger must resolve, subscribe, and deliver SSE events. A wrong data
point name must be rejected at create time with a suggestion, and GET /data
must serve the plugin's enumeration route when there is no DataProvider.
"""

import json
import os
import threading
import time
import unittest

from ament_index_python.packages import get_package_prefix
import launch_testing
import launch_testing.actions
import requests

from ros2_medkit_test_utils.constants import ALLOWED_EXIT_CODES, API_BASE_PATH
from ros2_medkit_test_utils.gateway_test_case import GatewayTestCase
from ros2_medkit_test_utils.launch_helpers import create_test_launch


def _get_plugin_path():
    pkg_prefix = get_package_prefix('ros2_medkit_integration_tests')
    return os.path.join(
        pkg_prefix, 'lib', 'ros2_medkit_integration_tests',
        'libtopics_test_plugin.so',
    )


def generate_test_description():
    return create_test_launch(
        demo_nodes=[],
        fault_manager=False,
        gateway_params={
            'plugins': ['topics_test'],
            'plugins.topics_test.path': _get_plugin_path(),
        },
    )


class TestTriggersPluginEntity(GatewayTestCase):
    """Data triggers on a plugin entity with declared topics (issue #584)."""

    MIN_EXPECTED_APPS = 0

    app_id = 'plugin_dev'

    def _wait_for_plugin_entity(self, timeout=30.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            resp = requests.get(f'{self.BASE_URL}/apps', timeout=5)
            if resp.status_code == 200:
                ids = [item.get('id') for item in resp.json().get('items', [])]
                if self.app_id in ids:
                    return
            time.sleep(1.0)
        self.fail(f'plugin entity {self.app_id!r} never appeared in /apps')

    def test_01_trigger_on_plugin_entity_resolves_and_fires(self):
        """The headline regression: create resolves via declared topics and SSE events flow."""
        self._wait_for_plugin_entity()

        resp = requests.post(
            f'{self.BASE_URL}/apps/{self.app_id}/triggers',
            json={
                'resource': f'{API_BASE_PATH}/apps/{self.app_id}/data/value',
                'trigger_condition': {'condition_type': 'OnChange'},
                'multishot': True,
            },
            timeout=5,
        )
        self.assertEqual(resp.status_code, 201, resp.text)
        trigger = resp.json()
        self.assertEqual(trigger['status'], 'active')
        trigger_id = trigger['id']

        events = []
        done = threading.Event()

        def consume():
            try:
                with requests.get(
                    f'{self.BASE_URL}/apps/{self.app_id}/triggers/{trigger_id}/events',
                    stream=True, timeout=30,
                ) as stream:
                    for line in stream.iter_lines():
                        if line and line.startswith(b'data:'):
                            events.append(json.loads(line[5:].decode()))
                            if len(events) >= 2:
                                done.set()
                                return
            except requests.RequestException:
                pass

        worker = threading.Thread(target=consume, daemon=True)
        worker.start()
        # The plugin publishes an incrementing value at 5 Hz, so two OnChange
        # events must arrive well inside this window.
        self.assertTrue(done.wait(timeout=25.0),
                        f'expected >=2 SSE events, got {len(events)}')

        requests.delete(
            f'{self.BASE_URL}/apps/{self.app_id}/triggers/{trigger_id}',
            timeout=5,
        )

    def test_02_wrong_data_point_rejected_with_suggestion(self):
        """A typo is a 400 with did_you_mean at create time, not a dead ACTIVE trigger."""
        self._wait_for_plugin_entity()

        resp = requests.post(
            f'{self.BASE_URL}/apps/{self.app_id}/triggers',
            json={
                'resource': f'{API_BASE_PATH}/apps/{self.app_id}/data/valu',
                'trigger_condition': {'condition_type': 'OnChange'},
                'multishot': True,
            },
            timeout=5,
        )
        self.assertEqual(resp.status_code, 400, resp.text)
        body = resp.json()
        params = body.get('parameters', body.get('params', {}))
        self.assertEqual(params.get('did_you_mean'), 'value', body)

    def test_03_data_route_fallback_serves_and_normalizes(self):
        """GET /data works without a DataProvider and items carry SOVD ids."""
        self._wait_for_plugin_entity()

        resp = requests.get(
            f'{self.BASE_URL}/apps/{self.app_id}/data', timeout=5,
        )
        self.assertEqual(resp.status_code, 200, resp.text)
        items = resp.json().get('items', [])
        self.assertTrue(items, 'expected the plugin route content')
        ids = [item.get('id') for item in items]
        self.assertIn('value', ids, f'normalized id missing: {items}')
        self.assertEqual(items[0].get('category'), 'currentData', items[0])


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        """Check all processes exited cleanly (SIGTERM allowed)."""
        for info in proc_info:
            self.assertIn(
                info.returncode, ALLOWED_EXIT_CODES,
                f'{info.process_name} exited with code {info.returncode}'
            )
