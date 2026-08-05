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

"""Feature tests for the fault-trigger (threshold rule) CRUD API.

Exercises /apps/{app_id}/fault-triggers end to end against a gateway with the
demo route-data plugin loaded (the engine only starts when plugins are
present, and both the create-time data-point validation and the poll fetcher
read the plugin's registered x-plc-data route - the commercial PLC bridge
shape). test_route_plc_app serves level=87.5, so a `>= 80` rule must confirm
a fault and expose it on /faults.

"""

import os
import time
import unittest

from ament_index_python.packages import get_package_prefix
import launch_testing
import launch_testing.actions
import requests

from ros2_medkit_test_utils.constants import (
    ALLOWED_EXIT_CODES,
    API_BASE_PATH,
    get_test_port,
)
from ros2_medkit_test_utils.gateway_test_case import GatewayTestCase
from ros2_medkit_test_utils.launch_helpers import create_gateway_node, create_test_launch

PLUGIN_APP = 'test_route_plc_app'

PORT_ENGINE_OFF = get_test_port(1)
URL_ENGINE_OFF = f'http://localhost:{PORT_ENGINE_OFF}{API_BASE_PATH}'


def _get_plugin_path(so_name):
    pkg_prefix = get_package_prefix('ros2_medkit_gateway')
    return os.path.join(pkg_prefix, 'lib', 'ros2_medkit_gateway', so_name)


def generate_test_description():
    launch_description, context = create_test_launch(
        demo_nodes=['temp_sensor'],
        fault_manager=True,
        gateway_params={
            'plugins': ['test_route_data'],
            'plugins.test_route_data.path':
                _get_plugin_path('libtest_route_data_plugin.so'),
            'fault_triggers.poll_interval_ms': 200,
        },
    )
    # Second gateway with the engine switched off, so the "backend absent"
    # answer can be asserted on the same route set as the live one.
    gateway_engine_off = create_gateway_node(
        name='gateway_fault_triggers_off',
        port=PORT_ENGINE_OFF,
        extra_params={
            'server.host': '127.0.0.1',
            'fault_triggers.enabled': False,
        },
    )
    # Prepend so it comes up alongside the primary gateway, before ReadyToTest.
    launch_description.entities.insert(0, gateway_engine_off)
    context['gateway_fault_triggers_off'] = gateway_engine_off
    return launch_description, context


class TestFaultTriggersApi(GatewayTestCase):
    """CRUD + fire path for threshold rules."""

    MIN_EXPECTED_APPS = 1
    REQUIRED_APPS = {'temp_sensor'}

    def _url(self, suffix=''):
        return f'{self.BASE_URL}/apps/{PLUGIN_APP}/fault-triggers{suffix}'

    def _create(self, body):
        return requests.post(self._url(), json=body, timeout=10)

    def test_01_list_starts_empty(self):
        """GET returns an empty items list before any rule exists."""
        resp = requests.get(self._url(), timeout=10)
        self.assertEqual(resp.status_code, 200)
        self.assertEqual(resp.json().get('items'), [])

    def test_02_create_validates_body(self):
        """Missing fields, bad operator/severity and non-JSON all reject."""
        cases = [
            ({}, "'data_name' is required"),
            ({'data_name': 'level', 'operator': '~', 'threshold': 1,
              'fault_code': 'X', 'severity': 'ERROR'}, 'operator'),
            ({'data_name': 'level', 'operator': '>', 'threshold': 'hot',
              'fault_code': 'X', 'severity': 'ERROR'}, 'threshold'),
            ({'data_name': 'level', 'operator': '>', 'threshold': 1,
              'fault_code': 'X', 'severity': 'LOUD'}, 'severity'),
        ]
        for body, needle in cases:
            resp = self._create(body)
            self.assertEqual(resp.status_code, 400, resp.text)
            self.assertIn(needle, resp.json()['message'])

        # Malformed JSON is a malformed request, not a semantic validation error.
        resp = requests.post(
            self._url(), data='{not json', timeout=10,
            headers={'Content-Type': 'application/json'})
        self.assertEqual(resp.status_code, 400)
        self.assertEqual(resp.json()['error_code'], 'invalid-request')

    def test_03_create_rejects_unknown_data_point(self):
        """A rule on a data point the plugin app does not expose is refused."""
        resp = self._create({'data_name': 'no_such_point', 'operator': '>',
                             'threshold': 1.0, 'fault_code': 'DEAD_RULE',
                             'severity': 'ERROR'})
        self.assertEqual(resp.status_code, 400, resp.text)
        self.assertIn('does not exist', resp.json()['message'])

    def test_04_create_list_fire_delete(self):
        """Create on a real data point, watch it fire, then delete clears it."""
        resp = self._create({'data_name': 'level', 'operator': '>=',
                             'threshold': 80.0, 'fault_code': 'TEST_LEVEL_HIGH',
                             'severity': 'ERROR'})
        self.assertEqual(resp.status_code, 201, resp.text)
        rule = resp.json()
        self.assertTrue(rule['id'])
        # This route is a raw registration, so the Location the document
        # declares for its 201 is set by hand - the typed registry's automatic
        # declaration cannot reach it. Assert the two agree.
        self.assertEqual(
            resp.headers.get('Location'),
            f'/api/v1/apps/{PLUGIN_APP}/fault-triggers/{rule["id"]}')

        listed = requests.get(self._url(), timeout=10).json()['items']
        self.assertIn(rule['id'], [r['id'] for r in listed])

        # Duplicate fault_code (even from another real app) is a conflict: the
        # code is the fault store's primary key. temp_sensor is a genuinely
        # discovered app (so it passes the app-existence check) with no PLC data
        # route, so the rule reaches the global fault_code uniqueness check.
        dup = requests.post(
            f'{self.BASE_URL}/apps/temp_sensor/fault-triggers',
            json={'data_name': 'level', 'operator': '>',
                  'threshold': 1.0, 'fault_code': 'TEST_LEVEL_HIGH',
                  'severity': 'ERROR'}, timeout=10)
        self.assertEqual(dup.status_code, 409, dup.text)

        # level is a canned constant 87.5 >= 80 - the level-triggered engine
        # must confirm the fault within a few 200 ms polls.
        fault = self.poll_for_fault('TEST_LEVEL_HIGH', timeout=15.0)
        self.assertIsNotNone(fault, 'TEST_LEVEL_HIGH never appeared on /faults')

        # Delete: 204, gone from the list, and the asserted fault is cleared.
        resp = requests.delete(self._url(f"/{rule['id']}"), timeout=10)
        self.assertEqual(resp.status_code, 204)
        self.assertNotIn(rule['id'],
                         [r['id'] for r in requests.get(self._url(), timeout=10).json()['items']])
        self.assertTrue(self.poll_until_fault_cleared('TEST_LEVEL_HIGH', timeout=15.0),
                        'TEST_LEVEL_HIGH still active after rule deletion')

    def test_05_delete_unknown_is_404(self):
        resp = requests.delete(self._url('/ftr_none'), timeout=10)
        self.assertEqual(resp.status_code, 404)

    def test_06_create_on_unknown_app_is_404(self):
        """A rule on an app the gateway never discovered is refused (404)."""
        resp = requests.post(
            f'{self.BASE_URL}/apps/ghost_app/fault-triggers',
            json={'data_name': 'level', 'operator': '>',
                  'threshold': 1.0, 'fault_code': 'GHOST_RULE',
                  'severity': 'ERROR'}, timeout=10)
        self.assertEqual(resp.status_code, 404, resp.text)

    def test_07_disconnected_app_holds_rule_state(self):
        """A rule on the down app never fires on its frozen last-known values.

        test_route_plc_down_app reports connected=false while still serving
        level=42.0. The trigger fetcher must treat that as unreadable
        (nullopt), so the level-triggered engine holds state instead of firing
        on a stale number for the whole outage - even though the same payload
        does get freeze-framed (test_entity_freeze_frame.test.py).
        """
        resp = requests.post(
            f'{self.BASE_URL}/apps/test_route_plc_down_app/fault-triggers',
            json={'data_name': 'level', 'operator': '>=',
                  'threshold': 10.0, 'fault_code': 'TEST_DOWN_LEVEL_HIGH',
                  'severity': 'ERROR'}, timeout=10)
        # Creation succeeds: data points are enumerable from last-known content.
        self.assertEqual(resp.status_code, 201, resp.text)
        rule = resp.json()

        # 42.0 >= 10 would fire within a few 200 ms polls if the stale value
        # were fetched; give it ample polls and require silence.
        time.sleep(3.0)
        self.assertIsNone(
            self.poll_for_fault('TEST_DOWN_LEVEL_HIGH', timeout=1.0),
            'rule fired on a disconnected app serving last-known values')

        resp = requests.delete(
            f'{self.BASE_URL}/apps/test_route_plc_down_app/fault-triggers'
            f"/{rule['id']}", timeout=10)
        self.assertEqual(resp.status_code, 204)

    def test_08_engine_absent_answers_501_on_every_verb(self):
        """No engine is a missing backend, not a missing rule.

        The gateway used to answer 404/``resource-not-found`` here, which is the
        same answer an unknown rule id gets - it told a client to go hunting for
        an id when what it had to do was turn the feature on. It now answers the
        gate shape ``/updates`` and ``/triggers`` use.
        """
        deadline = time.monotonic() + 30.0
        while time.monotonic() < deadline:
            try:
                if requests.get(f'{URL_ENGINE_OFF}/health', timeout=2).status_code == 200:
                    break
            except requests.exceptions.RequestException:
                pass
            time.sleep(0.5)
        else:
            self.fail('gateway_fault_triggers_off never became healthy')

        base = f'{URL_ENGINE_OFF}/apps/{PLUGIN_APP}/fault-triggers'
        responses = [
            requests.get(base, timeout=10),
            requests.post(base, json={'data_name': 'level', 'operator': '>',
                                      'threshold': 1.0, 'fault_code': 'OFF_RULE',
                                      'severity': 'ERROR'}, timeout=10),
            requests.delete(f'{base}/ftr_1', timeout=10),
        ]
        for resp in responses:
            self.assertEqual(resp.status_code, 501, resp.text)
            self.assertEqual(resp.json()['error_code'], 'not-implemented')

    def test_09_create_error_codes_name_the_failure(self):
        """400, 404 and 409 each carry their own error code.

        All three used to fall through one status-shaped default, so an unknown
        app read as a missing sub-resource and a duplicate ``fault_code`` read as
        a malformed field the caller could fix by editing the body.
        """
        bad_field = self._create({'data_name': 'level', 'operator': '~',
                                  'threshold': 1.0, 'fault_code': 'CODE_SHAPE_A',
                                  'severity': 'ERROR'})
        self.assertEqual(bad_field.status_code, 400, bad_field.text)
        self.assertEqual(bad_field.json()['error_code'], 'invalid-parameter')

        ghost = requests.post(
            f'{self.BASE_URL}/apps/ghost_app_for_codes/fault-triggers',
            json={'data_name': 'level', 'operator': '>', 'threshold': 1.0,
                  'fault_code': 'CODE_SHAPE_B', 'severity': 'ERROR'}, timeout=10)
        self.assertEqual(ghost.status_code, 404, ghost.text)
        self.assertEqual(ghost.json()['error_code'], 'entity-not-found')

        first = self._create({'data_name': 'level', 'operator': '>=',
                              'threshold': 80.0, 'fault_code': 'CODE_SHAPE_C',
                              'severity': 'ERROR'})
        self.assertEqual(first.status_code, 201, first.text)
        self.addCleanup(requests.delete, self._url(f"/{first.json()['id']}"), timeout=10)

        dup = self._create({'data_name': 'level', 'operator': '>=',
                            'threshold': 90.0, 'fault_code': 'CODE_SHAPE_C',
                            'severity': 'ERROR'})
        self.assertEqual(dup.status_code, 409, dup.text)
        self.assertEqual(dup.json()['error_code'], 'precondition-not-fulfilled')

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def poll_for_fault(self, fault_code, timeout):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            resp = requests.get(f'{self.BASE_URL}/faults', timeout=10)
            if resp.status_code == 200:
                for item in resp.json().get('items', []):
                    if item.get('fault_code') == fault_code:
                        return item
            time.sleep(0.5)
        return None

    def poll_until_fault_cleared(self, fault_code, timeout):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            resp = requests.get(f'{self.BASE_URL}/faults', timeout=10)
            if resp.status_code == 200:
                active = [i for i in resp.json().get('items', [])
                          if i.get('fault_code') == fault_code
                          and i.get('status') not in ('CLEARED',)]
                if not active:
                    return True
            time.sleep(0.5)
        return False


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(
            proc_info, allowable_exit_codes=ALLOWED_EXIT_CODES)
