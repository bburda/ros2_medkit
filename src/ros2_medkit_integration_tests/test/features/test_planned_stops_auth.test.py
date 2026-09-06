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

"""Who may declare a planned stop, and whose name ends up on it.

Declaring and ending a window are writes and need OPERATOR; reading needs
VIEWER. `declared_by` defaults to the authenticated client id, which is the only
reason the field is worth anything on a machine several people can reach.
"""

import unittest

import launch
import launch_testing
import launch_testing.actions
import pytest
import requests

from ros2_medkit_test_utils.constants import (
    ALLOWED_EXIT_CODES,
    API_BASE_PATH,
    get_test_port,
)
from ros2_medkit_test_utils.gateway_test_case import GatewayTestCase
from ros2_medkit_test_utils.launch_helpers import (
    create_fault_manager_node,
    create_gateway_node,
)

AUTH_PORT = get_test_port()
AUTH_BASE_URL = f'http://127.0.0.1:{AUTH_PORT}{API_BASE_PATH}'
PLANNED_STOPS = '/x-medkit-planned-stops'


@pytest.mark.launch_test
def generate_test_description():
    gateway_node = create_gateway_node(
        port=AUTH_PORT,
        extra_params={
            'server.host': '127.0.0.1',
            'auth.enabled': True,
            'auth.jwt_secret': 'planned_stop_auth_secret_key_for_integration_testing_1',
            'auth.jwt_algorithm': 'HS256',
            'auth.token_expiry_seconds': 3600,
            'auth.require_auth_for': 'write',
            'auth.issuer': 'test_gateway',
            'auth.clients': [
                'admin:admin_secret:admin',
                'operator:operator_secret:operator',
                'viewer:viewer_secret:viewer',
            ],
        },
    )
    return launch.LaunchDescription([
        gateway_node,
        create_fault_manager_node(storage_type='memory', rosbag_enabled=False),
        launch_testing.actions.ReadyToTest(),
    ]), {'gateway_node': gateway_node}


class TestPlannedStopsAuth(GatewayTestCase):
    """Roles on the planned-stop routes, and the identity they record."""

    BASE_URL = AUTH_BASE_URL

    def _token(self, client_id, secret):
        response = requests.post(
            f'{self.BASE_URL}/auth/authorize',
            json={
                'grant_type': 'client_credentials',
                'client_id': client_id,
                'client_secret': secret,
            },
            timeout=10,
        )
        self.assertEqual(response.status_code, 200, response.text)
        return response.json()['access_token']

    @staticmethod
    def _bearer(token):
        return {'Authorization': f'Bearer {token}'}

    def test_declaring_needs_operator(self):
        body = {'to': '2037-01-01T00:00:00Z', 'reason': 'role check'}

        viewer = requests.post(
            f'{self.BASE_URL}{PLANNED_STOPS}', json=body,
            headers=self._bearer(self._token('viewer', 'viewer_secret')), timeout=10,
        )
        self.assertEqual(viewer.status_code, 403, viewer.text)

        operator = requests.post(
            f'{self.BASE_URL}{PLANNED_STOPS}', json=body,
            headers=self._bearer(self._token('operator', 'operator_secret')), timeout=10,
        )
        self.assertEqual(operator.status_code, 201, operator.text)
        self.assertEqual(
            operator.json()['declared_by'], 'operator',
            'the window must record who actually declared it'
        )

    def test_a_caller_may_declare_on_somebody_elses_behalf(self):
        response = requests.post(
            f'{self.BASE_URL}{PLANNED_STOPS}',
            json={
                'to': '2037-01-01T00:00:00Z',
                'reason': 'declared by the MES',
                'declared_by': 'mes_line_3',
            },
            headers=self._bearer(self._token('operator', 'operator_secret')),
            timeout=10,
        )
        self.assertEqual(response.status_code, 201, response.text)
        self.assertEqual(response.json()['declared_by'], 'mes_line_3')

    def test_ending_needs_operator_and_reading_needs_viewer(self):
        created = requests.post(
            f'{self.BASE_URL}{PLANNED_STOPS}',
            json={'to': '2037-01-01T00:00:00Z', 'reason': 'end role check'},
            headers=self._bearer(self._token('admin', 'admin_secret')), timeout=10,
        )
        self.assertEqual(created.status_code, 201, created.text)
        window_id = created.json()['id']
        self.assertEqual(created.json()['declared_by'], 'admin')

        viewer_headers = self._bearer(self._token('viewer', 'viewer_secret'))

        # A viewer can read the window it may not touch.
        listing = requests.get(
            f'{self.BASE_URL}{PLANNED_STOPS}', headers=viewer_headers, timeout=10
        )
        self.assertEqual(listing.status_code, 200, listing.text)
        self.assertIn(window_id, [w['id'] for w in listing.json()['items']])

        one = requests.get(
            f'{self.BASE_URL}{PLANNED_STOPS}/{window_id}',
            headers=viewer_headers, timeout=10,
        )
        self.assertEqual(one.status_code, 200, one.text)

        refused = requests.delete(
            f'{self.BASE_URL}{PLANNED_STOPS}/{window_id}',
            headers=viewer_headers, timeout=10,
        )
        self.assertEqual(refused.status_code, 403, refused.text)

        allowed = requests.delete(
            f'{self.BASE_URL}{PLANNED_STOPS}/{window_id}',
            headers=self._bearer(self._token('operator', 'operator_secret')), timeout=10,
        )
        self.assertEqual(allowed.status_code, 200, allowed.text)
        self.assertTrue(allowed.json()['ended_early'])

    def test_an_unauthenticated_write_is_refused(self):
        # require_auth_for=write, so the read is open and the write is not.
        # A window nobody may declare is also a window nobody can be blamed for.
        anonymous = requests.post(
            f'{self.BASE_URL}{PLANNED_STOPS}',
            json={'to': '2037-01-01T00:00:00Z', 'reason': 'no token'}, timeout=10,
        )
        self.assertEqual(anonymous.status_code, 401, anonymous.text)

        listing = requests.get(f'{self.BASE_URL}{PLANNED_STOPS}', timeout=10)
        self.assertEqual(listing.status_code, 200, listing.text)


@launch_testing.post_shutdown_test()
class TestPlannedStopsAuthShutdown(unittest.TestCase):

    def test_gateway_exits_cleanly(self, proc_info, gateway_node):
        launch_testing.asserts.assertExitCodes(
            proc_info, process=gateway_node, allowable_exit_codes=ALLOWED_EXIT_CODES
        )
