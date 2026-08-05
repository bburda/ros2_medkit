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

"""Feature tests for faults API.

Validates fault listing, fault response structure, status filters,
error handling for nonexistent entities and invalid parameters.

"""

import unittest

import launch_testing
import launch_testing.actions
import requests

from ros2_medkit_test_utils.constants import ALLOWED_EXIT_CODES
from ros2_medkit_test_utils.gateway_test_case import GatewayTestCase
from ros2_medkit_test_utils.launch_helpers import create_test_launch


def generate_test_description():
    return create_test_launch(
        demo_nodes=['lidar_sensor'],
        fault_manager=True,
        lidar_faulty=True,
    )


class TestFaultsApi(GatewayTestCase):
    """Faults API tests."""

    MIN_EXPECTED_APPS = 1
    REQUIRED_APPS = {'lidar_sensor'}

    def test_root_endpoint_includes_faults(self):
        """Root endpoint lists faults endpoints and capability.

        @verifies REQ_INTEROP_012
        """
        data = self.get_json('/')

        # Verify faults endpoints are listed
        self.assertIn('endpoints', data)
        self.assertIn(
            'GET /api/v1/faults',
            data['endpoints']
        )
        self.assertIn(
            'GET /api/v1/components/{component_id}/faults',
            data['endpoints']
        )
        self.assertIn(
            'GET /api/v1/components/{component_id}/faults/{fault_code}',
            data['endpoints']
        )
        self.assertIn(
            'DELETE /api/v1/components/{component_id}/faults/{fault_code}',
            data['endpoints']
        )

        # Verify faults capability is listed
        self.assertIn('capabilities', data)
        self.assertIn('faults', data['capabilities'])
        self.assertTrue(data['capabilities']['faults'])

    def test_list_faults_response_structure(self):
        """GET /apps/{app_id}/faults returns valid response structure.

        In the heuristic discovery model, ROS nodes are Apps.

        @verifies REQ_INTEROP_012
        """
        # Use lidar_sensor which is more likely to have faults
        response = requests.get(
            f'{self.BASE_URL}/apps/lidar_sensor/faults',
            timeout=10
        )
        self.assertEqual(response.status_code, 200)

        data = response.json()
        # Items array format with x-medkit extension
        self.assertIn('items', data)
        self.assertIsInstance(data['items'], list)
        self.assertIn('x-medkit', data)
        x_medkit = data['x-medkit']
        self.assertEqual(x_medkit['entity_id'], 'lidar_sensor')
        self.assertIn('source_id', x_medkit)
        self.assertIn('count', x_medkit)

    def test_faults_nonexistent_component(self):
        """GET /components/{component_id}/faults returns 404 for unknown entity.

        @verifies REQ_INTEROP_012
        """
        response = requests.get(
            f'{self.BASE_URL}/components/nonexistent_component/faults',
            timeout=10
        )
        self.assertEqual(response.status_code, 404)

        data = response.json()
        self.assertIn('error_code', data)
        self.assertEqual(data['message'], 'Entity not found')
        # SOVD error format: parameters in parameters field
        self.assertIn('parameters', data)
        self.assertEqual(data['parameters'].get('entity_id'), 'nonexistent_component')

    def test_get_nonexistent_fault(self):
        """GET /apps/{app_id}/faults/{fault_code} returns 404.

        @verifies REQ_INTEROP_013
        """
        response = requests.get(
            f'{self.BASE_URL}/apps/lidar_sensor/faults/NONEXISTENT_FAULT',
            timeout=10
        )
        self.assertEqual(response.status_code, 404)

        data = response.json()
        self.assertIn('error_code', data)
        # SOVD error format: parameters in parameters field
        self.assertIn('parameters', data)
        self.assertEqual(data['parameters'].get('fault_code'), 'NONEXISTENT_FAULT')

    def test_both_verbs_reject_an_oversized_fault_code(self):
        """GET and DELETE both enforce the 256-character `fault_code` bound.

        The OpenAPI document publishes `maxLength: 256` on every route carrying
        `{fault_code}`, from one table keyed by the parameter name - so it
        cannot distinguish a route whose handler checks from one whose handler
        does not, and a bound belongs there only when every handler behind the
        template rejects. Both verbs are driven so that precondition is tested
        rather than assumed; the `config_id` half of the same table is covered
        by `test_configuration_api.test.py`.

        No lock is taken here on purpose. `clear_fault` measures the code
        *after* `validate_lock_access`, so a competing lock would answer 409
        before the length check is reached - a real ordering difference from
        the configuration handlers, and not what this test is pinning.

        @verifies REQ_INTEROP_013
        @verifies REQ_INTEROP_015
        """
        oversized = 'F' * 257
        url = f'{self.BASE_URL}/apps/lidar_sensor/faults/{oversized}'
        for verb, call in (
            ('GET', lambda: requests.get(url, timeout=10)),
            ('DELETE', lambda: requests.delete(url, timeout=10)),
        ):
            with self.subTest(verb=verb):
                response = call()
                self.assertEqual(response.status_code, 400, f'{verb}: {response.text}')
                self.assertIn('error_code', response.json())

        # Control: a code the route serves normally still reaches the store, so
        # the rejections above are the length gate rather than the route
        # refusing every long-ish code.
        inside = f'{self.BASE_URL}/apps/lidar_sensor/faults/{"F" * 64}'
        self.assertEqual(requests.get(inside, timeout=10).status_code, 404)

    def test_both_verbs_serve_a_fault_code_up_to_the_published_bound(self):
        """Every length the document admits reaches the store and answers 404.

        `maxLength: 256` is a promise that a code of 256 characters is a
        well-formed request, so the only honest answer for one that names no
        fault is 404. The bound is enforced in two nodes, and this pins that
        they agree: the gateway gates at 256 (`fault_handlers.cpp`), and the
        fault manager applies `kMaxFaultCodeLength` to the same value on the
        GetFault and ClearFault services. When the two disagreed, the lengths
        between the lower bound and 256 were refused by the fault manager and
        surfaced as 503 - a value the published document calls valid answering
        with a server error.

        128 is driven alongside 129 deliberately: it was the fault manager's
        old bound, so it passed while 129 did not, and keeping both makes a
        regression to any lower bound visible as a split result rather than a
        uniform failure.

        @verifies REQ_INTEROP_013
        @verifies REQ_INTEROP_015
        """
        for length in (128, 129, 255, 256):
            url = f'{self.BASE_URL}/apps/lidar_sensor/faults/{"F" * length}'
            for verb, call in (
                ('GET', lambda u=url: requests.get(u, timeout=10)),
                ('DELETE', lambda u=url: requests.delete(u, timeout=10)),
            ):
                with self.subTest(length=length, verb=verb):
                    response = call()
                    self.assertEqual(
                        response.status_code, 404,
                        f'{verb} at length {length}: {response.status_code} {response.text}'
                    )

    def test_a_refusal_from_the_fault_manager_is_not_a_server_error(self):
        """A code the store declines answers 4xx, not 503.

        The gateway published no `pattern` for `{fault_code}`, only a
        `maxLength`, but the fault manager restricts the character set to
        alphanumerics, underscore, hyphen and dot. So a short code containing
        anything else is admitted by the gateway, reaches the fault manager,
        and comes back refused - a refusal the gateway has to classify.

        This is the same defect as the length disagreement but reached by a
        different route, and it does not depend on any bound: the two nodes can
        agree on `maxLength` exactly and this still fails. The classification
        must therefore rest on which layer answered - the fault manager
        declining a request is a condition of the request, whereas only a
        fault manager that never answered at all is a server-side failure -
        and not on reading the words in the message it returned.

        @verifies REQ_INTEROP_013
        @verifies REQ_INTEROP_015
        """
        # `~` is unreserved in a URL path, so it arrives at the handler intact,
        # and it is outside the character set the fault manager accepts.
        url = f'{self.BASE_URL}/apps/lidar_sensor/faults/F~F'
        for verb, call in (
            ('GET', lambda: requests.get(url, timeout=10)),
            ('DELETE', lambda: requests.delete(url, timeout=10)),
        ):
            with self.subTest(verb=verb):
                response = call()
                self.assertLess(
                    response.status_code, 500,
                    f'{verb}: a refusal from the fault manager reported as '
                    f'{response.status_code}: {response.text}'
                )
                self.assertEqual(response.status_code, 404, f'{verb}: {response.text}')
                self.assertIn('error_code', response.json())

    def test_list_all_faults_globally(self):
        """GET /faults returns all faults across the system.

        This is a convenience API for dashboards and monitoring tools.
        """
        response = requests.get(
            f'{self.BASE_URL}/faults',
            timeout=10
        )
        self.assertEqual(response.status_code, 200)

        data = response.json()
        # Items array format with x-medkit extension
        self.assertIn('items', data)
        self.assertIsInstance(data['items'], list)
        self.assertIn('x-medkit', data)
        x_medkit = data['x-medkit']
        self.assertIn('count', x_medkit)
        self.assertIsInstance(x_medkit['count'], int)
        self.assertEqual(x_medkit['count'], len(data['items']))

    def test_list_all_faults_with_status_filter(self):
        """GET /faults?status={status} filters faults by status."""
        # Test with status=all
        response = requests.get(
            f'{self.BASE_URL}/faults?status=all',
            timeout=10
        )
        self.assertEqual(response.status_code, 200)

        data = response.json()
        self.assertIn('items', data)
        self.assertIn('x-medkit', data)
        self.assertIn('count', data['x-medkit'])

        # Test other valid status values (including healed)
        for status in ['pending', 'confirmed', 'cleared', 'healed']:
            response = requests.get(
                f'{self.BASE_URL}/faults?status={status}',
                timeout=10
            )
            self.assertEqual(response.status_code, 200)

    def test_list_faults_invalid_status_returns_400(self):
        """GET /faults?status=invalid returns 400 Bad Request."""
        response = requests.get(
            f'{self.BASE_URL}/faults?status=invalid_status',
            timeout=10
        )
        self.assertEqual(response.status_code, 400)

        data = response.json()
        self.assertIn('error_code', data)
        self.assertEqual(data['message'], 'Invalid status parameter value')
        # Check parameters in parameters field
        self.assertIn('parameters', data)
        params = data['parameters']
        self.assertIn('allowed_values', params)
        self.assertIn('pending', params['allowed_values'])
        self.assertIn('healed', params['allowed_values'])
        self.assertIn('parameter', params)
        self.assertEqual(params.get('parameter'), 'status')
        self.assertIn('value', params)
        self.assertEqual(params['value'], 'invalid_status')

    def test_component_faults_invalid_status_returns_400(self):
        """GET /apps/{id}/faults?status=invalid returns 400."""
        response = requests.get(
            f'{self.BASE_URL}/apps/lidar_sensor/faults?status=bogus',
            timeout=10
        )
        self.assertEqual(response.status_code, 400)

        data = response.json()
        self.assertIn('error_code', data)
        self.assertEqual(data['message'], 'Invalid status parameter value')
        self.assertIn('parameters', data)
        self.assertEqual(data['parameters'].get('app_id'), 'lidar_sensor')


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        """Check all processes exited cleanly (SIGTERM allowed)."""
        for info in proc_info:
            self.assertIn(
                info.returncode, ALLOWED_EXIT_CODES,
                f'{info.process_name} exited with code {info.returncode}'
            )
