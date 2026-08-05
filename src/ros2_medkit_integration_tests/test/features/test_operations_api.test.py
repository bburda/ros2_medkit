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

"""Feature tests for operations API (services, actions, executions).

Validates operation listing, service calls, action details, operation schema,
error handling, and execution listing.

Uses host-derived default component (SOVD-aligned entity model).

"""

import unittest

import launch_testing
import launch_testing.actions
import requests

from ros2_medkit_test_utils.constants import ALLOWED_EXIT_CODES, API_BASE_PATH
from ros2_medkit_test_utils.gateway_test_case import GatewayTestCase
from ros2_medkit_test_utils.launch_helpers import create_test_launch


def generate_test_description():
    return create_test_launch(
        demo_nodes=['calibration', 'long_calibration'],
        fault_manager=False,
    )


class TestOperationsApi(GatewayTestCase):
    """Operations API tests for services, actions, and executions."""

    MIN_EXPECTED_APPS = 2
    REQUIRED_APPS = {'calibration', 'long_calibration'}
    REQUIRED_OPERATIONS = {'/apps/calibration': 'calibrate'}

    # Cache for the dynamically-discovered host component ID
    _host_component_id = None

    def _get_host_component_id(self):
        """Get the host component ID (cached after first lookup)."""
        if TestOperationsApi._host_component_id is not None:
            return TestOperationsApi._host_component_id

        data = self.get_json('/components')
        components = data.get('items', [])
        self.assertEqual(
            len(components), 1,
            f'Expected exactly 1 host component, got {len(components)}'
        )
        TestOperationsApi._host_component_id = components[0]['id']
        return TestOperationsApi._host_component_id

    # ------------------------------------------------------------------
    # Service calls (test_31-36)
    # ------------------------------------------------------------------

    def test_operation_call_calibrate_service(self):
        """POST /apps/{app_id}/operations/{op}/executions calls a service.

        Operations are exposed on Apps (ROS 2 nodes), not Components.

        @verifies REQ_INTEROP_035
        """
        self.poll_endpoint('/apps/calibration')

        response = requests.post(
            f'{self.BASE_URL}/apps/calibration/operations/calibrate/executions',
            json={},
            timeout=15
        )
        self.assertEqual(response.status_code, 200)

        data = response.json()
        # SOVD service response: {"parameters": {...}}
        self.assertIn('parameters', data)

        # Verify service response structure (std_srvs/srv/Trigger response)
        params = data['parameters']
        self.assertIn('success', params)
        self.assertIn('message', params)
        self.assertIsInstance(params['success'], bool)
        self.assertIsInstance(params['message'], str)

    def test_operation_call_nonexistent_operation(self):
        """Operation call returns 404 for unknown operation.

        @verifies REQ_INTEROP_035
        """
        self.poll_endpoint('/apps/calibration')

        response = requests.post(
            f'{self.BASE_URL}/apps/calibration/operations/nonexistent_op/executions',
            json={},
            timeout=10
        )
        self.assertEqual(response.status_code, 404)

        data = response.json()
        self.assertIn('error_code', data)
        self.assertIn('not found', data['message'].lower())

    def test_operation_call_nonexistent_entity(self):
        """Operation call returns 404 for unknown entity.

        @verifies REQ_INTEROP_035
        """
        response = requests.post(
            f'{self.BASE_URL}/apps/nonexistent_app/operations/calibrate/executions',
            json={},
            timeout=5
        )
        self.assertEqual(response.status_code, 404)

        data = response.json()
        self.assertIn('error_code', data)
        self.assertIn('not found', data['message'].lower())

    def test_operation_call_invalid_entity_id(self):
        """Operation call rejects invalid entity ID.

        @verifies REQ_INTEROP_035
        """
        invalid_ids = [
            'app;drop',
            'app<script>',
            'app name',
        ]

        for invalid_id in invalid_ids:
            response = requests.post(
                f'{self.BASE_URL}/apps/{invalid_id}/operations/calibrate/executions',
                json={},
                timeout=5
            )
            self.assertEqual(
                response.status_code,
                400,
                f'Expected 400 for entity_id: {invalid_id}'
            )

            data = response.json()
            self.assertIn('error_code', data)
            self.assertIn('invalid', data['message'].lower())

    def test_operation_call_invalid_operation_name(self):
        """Operation call rejects invalid operation name.

        @verifies REQ_INTEROP_035
        """
        invalid_names = [
            'op;drop',
            'op<script>',
            'op name',
        ]

        for invalid_name in invalid_names:
            response = requests.post(
                f'{self.BASE_URL}/apps/calibration/operations/{invalid_name}/executions',
                json={},
                timeout=5
            )
            # Accept 400 (invalid) or 404 (not found) - both are valid rejections
            self.assertIn(
                response.status_code,
                [400, 404],
                f'Expected 400 or 404 for operation_name: {invalid_name}'
            )

            data = response.json()
            self.assertIn('error_code', data)

    def test_operation_call_with_invalid_json(self):
        """Operation call returns 400 for invalid JSON body.

        @verifies REQ_INTEROP_035
        """
        response = requests.post(
            f'{self.BASE_URL}/apps/calibration/operations/calibrate/executions',
            data='not valid json',
            headers={'Content-Type': 'application/json'},
            timeout=5
        )
        self.assertEqual(response.status_code, 400)

        data = response.json()
        self.assertIn('error_code', data)
        self.assertIn('json', data['message'].lower())

    # ------------------------------------------------------------------
    # Operation discovery (test_37-38)
    # ------------------------------------------------------------------

    def test_operations_listed_in_app_discovery(self):
        """Operations (services) are available via app detail endpoint.

        @verifies REQ_INTEROP_033
        """
        self.poll_endpoint('/apps/calibration')

        data = self.get_json('/apps/calibration')

        # App detail should have capabilities including operations
        self.assertIn('capabilities', data, 'App should have capabilities')
        cap_names = [c.get('name') for c in data['capabilities']]
        self.assertIn('operations', cap_names, 'App should have operations capability')

        # Check operations endpoint directly
        ops_data = self.get_json('/apps/calibration/operations')
        self.assertIn('items', ops_data, 'Operations endpoint should return items')
        ops = ops_data['items']

        # Find the calibrate operation
        calibrate_op = None
        for op in ops:
            if op['name'] == 'calibrate':
                calibrate_op = op
                break

        self.assertIsNotNone(calibrate_op, 'Calibrate operation should be listed')
        self.assertIn('x-medkit', calibrate_op)
        x_medkit = calibrate_op['x-medkit']
        self.assertIn('ros2', x_medkit)
        self.assertEqual(x_medkit['ros2']['kind'], 'service')
        self.assertEqual(x_medkit['ros2']['type'], 'std_srvs/srv/Trigger')
        self.assertEqual(x_medkit['ros2']['service'], '/powertrain/engine/calibrate')

    def test_root_endpoint_includes_operations(self):
        """Root endpoint lists operations endpoint and capability.

        @verifies REQ_INTEROP_033
        """
        data = self.get_json('/')

        self.assertIn('endpoints', data)
        operations_endpoints = [e for e in data['endpoints'] if 'operations' in e.lower()]
        self.assertGreater(len(operations_endpoints), 0, 'Should have operations endpoints')

        self.assertIn('capabilities', data)
        self.assertIn('operations', data['capabilities'])
        self.assertTrue(data['capabilities']['operations'])

    # ------------------------------------------------------------------
    # Operation schema (test_53)
    # ------------------------------------------------------------------

    def test_service_operation_has_type_info_schema(self):
        """Service operations include type_info with request/response schemas."""
        # The operation is listed by name before its service type is resolved,
        # so wait until type_info carries the request/response schema keys
        # rather than reading the list once and racing type resolution.
        calibrate_op = self.wait_for_operation_type_info(
            '/apps/calibration', 'calibrate', ('request', 'response'),
        )

        # Confirm it is the service operation.
        ros2 = calibrate_op.get('x-medkit', {}).get('ros2', {})
        self.assertEqual(ros2.get('kind'), 'service',
                         'Calibrate service should be listed')

        # Verify type_info is present in x-medkit with request/response schemas
        x_medkit = calibrate_op['x-medkit']
        self.assertIn('type_info', x_medkit, 'Service should have type_info in x-medkit')
        type_info = x_medkit['type_info']

        self.assertIn('request', type_info, 'Service type_info should have request')
        self.assertIn('response', type_info, 'Service type_info should have response')
        self.assertIsInstance(type_info['request'], dict)
        self.assertIsInstance(type_info['response'], dict)

        # std_srvs/srv/Trigger has empty request and response with success+message
        response_schema = type_info['response']
        self.assertIn('properties', response_schema, 'Response schema should have properties')
        self.assertIn('success', response_schema['properties'])
        self.assertIn('message', response_schema['properties'])

    # ------------------------------------------------------------------
    # Operation details (test_84, test_93)
    # ------------------------------------------------------------------

    def test_get_operation_details_for_service(self):
        """GET /{entity}/operations/{op-id} returns operation details for service.

        Uses host-derived default component which aggregates all apps' operations.

        @verifies REQ_INTEROP_034
        """
        comp_id = self._get_host_component_id()
        data = self.get_json(f'/components/{comp_id}/operations')
        self.assertIn('items', data)
        operations = data['items']
        self.assertGreater(len(operations), 0, 'Component should have operations')

        # Find a service (asynchronous_execution: false)
        service_op = None
        for op in operations:
            if not op.get('asynchronous_execution', True):
                service_op = op
                break

        if service_op is None:
            self.fail('No service operations found')
            return

        operation_id = service_op['id']

        # Get the operation details
        response = requests.get(
            f'{self.BASE_URL}/components/{comp_id}/operations/{operation_id}',
            timeout=10
        )
        self.assertEqual(response.status_code, 200)

        data = response.json()
        self.assertIn('item', data)
        item = data['item']

        # Required fields
        self.assertIn('id', item)
        self.assertEqual(item['id'], operation_id)
        self.assertIn('name', item)
        self.assertIn('proximity_proof_required', item)
        self.assertFalse(item['proximity_proof_required'])
        self.assertIn('asynchronous_execution', item)
        self.assertFalse(item['asynchronous_execution'])

        # x-medkit extension
        self.assertIn('x-medkit', item)
        x_medkit = item['x-medkit']
        self.assertIn('ros2', x_medkit)
        self.assertIn('kind', x_medkit['ros2'])
        self.assertEqual(x_medkit['ros2']['kind'], 'service')
        self.assertIn('type', x_medkit['ros2'])
        self.assertIn('service', x_medkit['ros2'])

    def test_get_operation_details_for_apps(self):
        """GET /apps/{id}/operations/{op-id} works for apps.

        @verifies REQ_INTEROP_034
        """
        data = self.get_json('/apps')
        apps = data['items']

        operation_found = False
        for app in apps:
            ops_data = self.get_json(f'/apps/{app["id"]}/operations')
            if ops_data.get('items'):
                operation_id = ops_data['items'][0]['id']

                # Get operation details
                response = requests.get(
                    f'{self.BASE_URL}/apps/{app["id"]}/operations/{operation_id}',
                    timeout=10
                )
                self.assertEqual(response.status_code, 200)

                data = response.json()
                self.assertIn('item', data)
                self.assertIn('id', data['item'])
                self.assertIn('x-medkit', data['item'])

                operation_found = True
                break

        if not operation_found:
            self.fail('No app operations found')

    # ------------------------------------------------------------------
    # Operation errors (test_86)
    # ------------------------------------------------------------------

    def test_get_operation_not_found(self):
        """GET /{entity}/operations/{op-id} returns 404 for nonexistent operation.

        @verifies REQ_INTEROP_034
        """
        comp_id = self._get_host_component_id()
        response = requests.get(
            f'{self.BASE_URL}/components/{comp_id}/operations/nonexistent_op',
            timeout=10
        )
        self.assertEqual(response.status_code, 404)

        data = response.json()
        self.assertIn('error_code', data)
        self.assertIn('message', data)
        self.assertEqual(data['message'], 'Operation not found')

    # ------------------------------------------------------------------
    # Executions (test_87-89)
    # ------------------------------------------------------------------

    def test_list_executions_returns_items_array(self):
        """GET /{entity}/operations/{op-id}/executions returns items array.

        @verifies REQ_INTEROP_036
        """
        # Wait for the operation to be usable, not merely listed: the name
        # appears before the gateway has resolved the ROS interface type.
        self.wait_for_operation_type_info(
            '/apps/long_calibration', 'long_calibration',
            ('goal', 'result', 'feedback'))

        # List executions - should return items array (may be empty)
        response = requests.get(
            f'{self.BASE_URL}/apps/long_calibration/operations/long_calibration/executions',
            timeout=10
        )
        self.assertEqual(response.status_code, 200)

        data = response.json()
        self.assertIn('items', data)
        self.assertIsInstance(data['items'], list)

    def test_async_execution_location_resolves(self):
        """The 202 `Location` names the execution under the addressed collection.

        The gateway used to build this header from a two-way "app or else
        component" choice, so an execution started through `/functions/...` or
        `/areas/...` was handed a `/components/...` URI that resolves to
        nothing. The fixture discovers the `powertrain` function from the demo
        nodes' namespace, and `long_calibration` is an action on it, so this
        drives the branch that was wrong.

        @verifies REQ_INTEROP_035
        """
        self.wait_for_operation('/functions/powertrain', 'long_calibration')

        ops = self.get_json('/functions/powertrain/operations')['items']
        actions = [o for o in ops if o.get('asynchronous_execution')]
        self.assertTrue(actions, 'no action discovered on the function')

        resp = requests.post(
            f'{self.BASE_URL}/functions/powertrain/operations'
            f'/{actions[0]["id"]}/executions',
            json={}, timeout=15)
        self.assertEqual(resp.status_code, 202, resp.text)

        location = resp.headers.get('Location')
        execution_id = resp.json()['id']
        self.assertEqual(
            location,
            f'{API_BASE_PATH}/functions/powertrain/operations'
            f'/{actions[0]["id"]}/executions/{execution_id}')

        # The header is only worth anything if it resolves.
        follow_url = self.BASE_URL + location[len(API_BASE_PATH):]
        self.addCleanup(requests.delete, follow_url, timeout=10)
        follow = requests.get(follow_url, timeout=10)
        self.assertEqual(follow.status_code, 200, follow.text)

    def test_create_execution_for_service(self):
        """POST /{entity}/operations/{op-id}/executions calls service and returns.

        @verifies REQ_INTEROP_035
        """
        comp_id = self._get_host_component_id()
        data = self.get_json(f'/components/{comp_id}/operations')
        operations = data['items']

        service_op = None
        for op in operations:
            if not op.get('asynchronous_execution', True):
                service_op = op
                break

        if service_op is None:
            self.fail('No service operations found')
            return

        operation_id = service_op['id']

        # Call the service
        response = requests.post(
            f'{self.BASE_URL}/components/{comp_id}/operations/{operation_id}/executions',
            json={'parameters': {}},
            timeout=30
        )

        self.assertEqual(
            response.status_code, 200,
            f'Expected 200 for sync service execution, '
            f'got {response.status_code}: {response.text}'
        )

        data = response.json()
        self.assertIn('parameters', data)

    def test_cancel_nonexistent_execution(self):
        """DELETE /{entity}/operations/{op-id}/executions/{exec-id} returns 404.

        @verifies REQ_INTEROP_039
        """
        comp_id = self._get_host_component_id()
        url = (f'{self.BASE_URL}/components/{comp_id}/operations/'
               'nonexistent_op/executions/fake-exec-id')
        response = requests.delete(url, timeout=10)
        self.assertEqual(response.status_code, 404)

        data = response.json()
        self.assertIn('error_code', data)
        self.assertIn('message', data)
        self.assertEqual(data['message'], 'Execution not found')

    # ------------------------------------------------------------------
    # List operations (test_99)
    # ------------------------------------------------------------------

    def test_list_operations(self):
        """GET /apps/{id}/operations returns operations list.

        @verifies REQ_INTEROP_033
        """
        data = self.get_json('/apps')
        self.assertGreater(len(data['items']), 0)
        app_id = data['items'][0]['id']

        response = requests.get(
            f'{self.BASE_URL}/apps/{app_id}/operations',
            timeout=10
        )

        self.assertEqual(response.status_code, 200)
        data = response.json()
        self.assertIn('items', data)


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        """Check all processes exited cleanly (SIGTERM allowed)."""
        for info in proc_info:
            self.assertIn(
                info.returncode, ALLOWED_EXIT_CODES,
                f'{info.process_name} exited with code {info.returncode}'
            )
