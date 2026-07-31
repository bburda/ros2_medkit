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

"""Scenario: Action execution lifecycle — create, poll, cancel, list.

Fixes #222 by ensuring each test creates its OWN execution, eliminating the
concurrent-goal race condition that caused test_101 to fail in the original
monolithic test file.
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
        demo_nodes=['long_calibration', 'calibration'],
        fault_manager=False,
    )


class TestScenarioActionLifecycle(GatewayTestCase):
    """Scenario: Full action execution lifecycle through the REST API.

    Validates that action-backed operations can be created, polled,
    cancelled, listed, and inspected. Each test is self-contained and
    creates its own execution so there are no concurrent-goal races.

    Steps:
    1. Create an action execution and poll it to completion
    2. Create an action execution and cancel it mid-flight
    3. Create a service execution and verify immediate result
    4. Create an execution and verify it appears in the execution list
    5. Verify action operation has type_info with goal/result/feedback schemas
    6. Get operation details for an action and verify structure
    7. Create an execution and get its status via the status endpoint
    8. Create an execution and attempt an unsupported update (PUT), expect error
    """

    MIN_EXPECTED_APPS = 2
    REQUIRED_APPS = {'long_calibration', 'calibration'}

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    APP_ID = 'long_calibration'
    OPERATION_ID = 'long_calibration'
    ENTITY_ENDPOINT = '/apps/long_calibration'

    def _ensure_operation_ready(self):
        """Wait for the long_calibration operation to be USABLE, not just listed.

        An operation appears under /operations as soon as it is discovered by
        name, but the gateway cannot send a goal until it has also resolved the
        underlying ROS interface type. Waiting on the name alone races that
        second step, and creating an execution in the gap returns 500
        x-medkit-ros2-action-unavailable, because the empty type string fails
        typesupport lookup. Wait for the resolved schema instead.
        """
        self.wait_for_operation_type_info(
            self.ENTITY_ENDPOINT, self.OPERATION_ID,
            ('goal', 'result', 'feedback'), max_wait=15.0
        )

    def _create_action_execution(self, order=5):
        """Create an action execution and return (response, data)."""
        response, data = self.create_execution(
            self.ENTITY_ENDPOINT, self.OPERATION_ID,
            input_data={'order': order},
        )
        self.assertEqual(
            response.status_code, 202,
            f'Expected 202 for action creation, got {response.status_code}',
        )
        self.assertIn('id', data)
        return response, data

    def _exec_endpoint(self, execution_id):
        """Return the execution status endpoint path."""
        return (
            f'{self.ENTITY_ENDPOINT}/operations/'
            f'{self.OPERATION_ID}/executions/{execution_id}'
        )

    # ------------------------------------------------------------------
    # Tests
    # ------------------------------------------------------------------

    def test_01_create_action_execution_and_poll_to_completion(self):
        """Create Fibonacci(order=3), poll until COMPLETED.

        @verifies REQ_INTEROP_022
        """
        self._ensure_operation_ready()

        # Send a short goal that will complete quickly
        _, data = self._create_action_execution(order=3)
        execution_id = data['id']

        # Poll for completion
        status_data = self.wait_for_execution_status(
            self._exec_endpoint(execution_id),
            ['completed', 'failed'],
        )
        self.assertEqual(status_data['status'], 'completed')

    def test_02_cancel_action_execution(self):
        """Create Fibonacci(order=20), cancel mid-flight.

        @verifies REQ_INTEROP_022
        """
        self._ensure_operation_ready()

        # Send a long goal that we can cancel
        _, data = self._create_action_execution(order=20)
        execution_id = data['id']

        # Verify execution is at least registered before cancelling
        status_data = self.poll_endpoint(
            self._exec_endpoint(execution_id), timeout=10.0, interval=0.3
        )
        self.assertIn(
            status_data['status'], ['running', 'completed'],
            f'Execution should be running or completed, got {status_data["status"]}',
        )

        # Cancel the execution. The client budget must exceed the gateway's: cancelling is
        # a service round-trip to the action server (up to 2s to find the service plus the
        # configured service_call_timeout_sec, default 10s), so a 10s client timeout would
        # surface a slow-but-successful cancel as an opaque ReadTimeout instead of the
        # server's own diagnosable error.
        response = self.delete_request(
            self._exec_endpoint(execution_id),
            timeout=25,
            expected_status=204,
        )
        self.assertEqual(len(response.content), 0)

    def test_03_service_execution_returns_immediately(self):
        """Create a Trigger service execution, get immediate result.

        @verifies REQ_INTEROP_035
        """
        # Find a service operation on the host-derived default component
        comp_data = self.get_json('/components')
        components = comp_data.get('items', [])
        self.assertGreater(len(components), 0, 'Expected at least one component')
        comp_id = components[0]['id']

        data = self.get_json(f'/components/{comp_id}/operations')
        operations = data['items']

        service_op = None
        for op in operations:
            if not op.get('asynchronous_execution', True):
                service_op = op
                break

        if service_op is None:
            self.fail('No service operations found on component')

        operation_id = service_op['id']

        # Call the service
        response = requests.post(
            f'{self.BASE_URL}/components/{comp_id}/operations/'
            f'{operation_id}/executions',
            json={'parameters': {}},
            timeout=30,
        )

        self.assertEqual(
            response.status_code, 200,
            f'Expected 200 for sync service execution, '
            f'got {response.status_code}: {response.text}',
        )

        data = response.json()
        self.assertIn('parameters', data)

    def test_04_execution_appears_in_list(self):
        """Create execution, then GET list and verify it contains our ID.

        @verifies REQ_INTEROP_022
        """
        self._ensure_operation_ready()

        # Create an execution
        _, data = self._create_action_execution(order=3)
        expected_execution_id = data['id']

        # Wait for it to complete
        self.wait_for_execution_status(
            self._exec_endpoint(expected_execution_id),
            ['completed', 'failed'],
        )

        # List all executions for this operation
        list_data = self.get_json(
            f'{self.ENTITY_ENDPOINT}/operations/'
            f'{self.OPERATION_ID}/executions',
        )
        self.assertIn('items', list_data)
        self.assertIsInstance(list_data['items'], list)

        execution_ids = [item['id'] for item in list_data['items']]
        self.assertIn(expected_execution_id, execution_ids)

    def test_05_action_has_type_info_schema(self):
        """GET operation details, verify type_info with goal/result/feedback schemas."""
        self._ensure_operation_ready()

        # The operation appears by name before its interface type is resolved,
        # so wait until type_info actually carries the action schema keys rather
        # than reading the list once and racing type resolution.
        action_op = self.wait_for_operation_type_info(
            self.ENTITY_ENDPOINT, self.OPERATION_ID,
            ('goal', 'result', 'feedback'),
        )

        # Confirm it is the action operation (not a same-named service).
        ros2 = action_op.get('x-medkit', {}).get('ros2', {})
        self.assertEqual(ros2.get('kind'), 'action',
                         'Long calibration action should be listed')

        # Verify type_info in x-medkit
        x_medkit = action_op['x-medkit']
        self.assertIn('type_info', x_medkit, 'Action should have type_info in x-medkit')
        type_info = x_medkit['type_info']

        self.assertIn('goal', type_info)
        self.assertIn('result', type_info)
        self.assertIn('feedback', type_info)
        self.assertIsInstance(type_info['goal'], dict)
        self.assertIsInstance(type_info['result'], dict)
        self.assertIsInstance(type_info['feedback'], dict)

        # Fibonacci: goal has order, result/feedback have sequence
        goal_schema = type_info['goal']
        result_schema = type_info['result']
        feedback_schema = type_info['feedback']
        self.assertIn('properties', goal_schema)
        self.assertIn('order', goal_schema['properties'])
        self.assertIn('properties', result_schema)
        self.assertIn('sequence', result_schema['properties'])
        self.assertIn('properties', feedback_schema)
        self.assertIn('sequence', feedback_schema['properties'])

    def test_06_get_operation_details(self):
        """GET operation details for an action, verify structure.

        @verifies REQ_INTEROP_034
        """
        self._ensure_operation_ready()

        # Search for an action operation on any component
        data = self.get_json('/components')
        components = data['items']

        action_op = None
        component_id = None

        for comp in components:
            ops_data = self.get_json(f'/components/{comp["id"]}/operations')
            for op in ops_data.get('items', []):
                if op.get('asynchronous_execution', False):
                    action_op = op
                    component_id = comp['id']
                    break
            if action_op:
                break

        if action_op is None:
            self.fail('No action operations found')

        operation_id = action_op['id']

        # Get the operation details
        detail = self.get_json(
            f'/components/{component_id}/operations/{operation_id}',
        )
        self.assertIn('item', detail)
        item = detail['item']

        self.assertIn('asynchronous_execution', item)
        self.assertTrue(item['asynchronous_execution'])

        self.assertIn('x-medkit', item)
        x_medkit = item['x-medkit']
        self.assertIn('ros2', x_medkit)
        self.assertIn('kind', x_medkit['ros2'])
        self.assertEqual(x_medkit['ros2']['kind'], 'action')

    def test_07_get_execution_status(self):
        """Create execution, GET status endpoint — self-contained.

        @verifies REQ_INTEROP_037
        """
        self._ensure_operation_ready()

        # Create a fresh execution
        _, data = self._create_action_execution(order=5)
        execution_id = data['id']

        # Poll for execution to be registered
        status_data = self.poll_endpoint(
            self._exec_endpoint(execution_id), timeout=2.0, interval=0.2
        )
        self.assertIn('status', status_data)
        self.assertIn(status_data['status'], ['running', 'completed', 'failed'])
        self.assertIn('x-medkit', status_data)
        self.assertEqual(status_data['x-medkit']['goal_id'], execution_id)

    def test_08_update_execution_not_implemented(self):
        """Create execution, try PUT to update, expect 400 or 501 — no race condition.

        Each test creates its OWN execution, so there is no concurrent-goal
        issue that plagued the original test_101.

        @verifies REQ_INTEROP_038
        """
        self._ensure_operation_ready()

        # Create a fresh execution (order=10 so it's still running when we PUT)
        _, data = self._create_action_execution(order=10)
        execution_id = data['id']

        # Try to update (pause) the execution — not supported
        response = requests.put(
            f'{self.BASE_URL}{self._exec_endpoint(execution_id)}',
            json={'action': 'pause'},
            timeout=10,
        )

        # PUT for unsupported pause capability returns 400
        self.assertEqual(
            response.status_code, 400,
            f'Expected 400 for unsupported pause, '
            f'got {response.status_code}',
        )

        resp_data = response.json()
        self.assertIn('error_code', resp_data)


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        """Check all processes exited cleanly.

        Allow exit code -15 (SIGTERM) which is expected during shutdown
        after long-running action execution tests.
        """
        for info in proc_info:
            self.assertIn(
                info.returncode, ALLOWED_EXIT_CODES,
                f'{info.process_name} exited with code {info.returncode}'
            )
