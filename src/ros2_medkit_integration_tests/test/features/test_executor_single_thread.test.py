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

"""Single-executor-thread guard for the RPC callback-group split (issue #575).

The RPC response callbacks live in a shared Reentrant callback group so a
second executor thread can dispatch them while the default group is busy.
A Reentrant group must NOT *require* a second thread: with
``server.executor_threads: 1`` and nothing hogging the executor, service
calls, action goals, and action cancels must all still complete - the single
thread services both groups. This guards against a fix that accidentally
made RPC dispatch depend on a dedicated thread (which would deadlock every
single-threaded deployment).
"""

import unittest

import launch_testing
import requests

from ros2_medkit_test_utils.constants import ALLOWED_EXIT_CODES
from ros2_medkit_test_utils.gateway_test_case import GatewayTestCase
from ros2_medkit_test_utils.launch_helpers import create_test_launch


def generate_test_description():
    return create_test_launch(
        demo_nodes=['calibration', 'long_calibration'],
        fault_manager=False,
        gateway_params={
            'server.executor_threads': 1,
        },
    )


class TestExecutorSingleThread(GatewayTestCase):
    """All operation flows complete on a one-thread executor (no deadlock)."""

    MIN_EXPECTED_APPS = 2
    REQUIRED_APPS = {'calibration', 'long_calibration'}

    ACTION_ENDPOINT = '/apps/long_calibration'
    ACTION_OPERATION = 'long_calibration'

    def test_service_operation_completes(self):
        """A synchronous service-backed operation completes on one thread."""
        self.wait_for_operation('/apps/calibration', 'calibrate')

        response = requests.post(
            f'{self.BASE_URL}/apps/calibration/operations/calibrate/executions',
            json={'parameters': {}},
            timeout=30,
        )
        self.assertEqual(
            response.status_code, 200,
            f'Service operation failed with executor_threads=1 '
            f'(HTTP {response.status_code}): {response.text}',
        )

    def test_action_execute_and_cancel_completes(self):
        """Action goal send + mid-flight cancel complete on one thread."""
        self.wait_for_operation_type_info(
            self.ACTION_ENDPOINT, self.ACTION_OPERATION,
            ('goal', 'result', 'feedback'),
        )

        response, data = self.create_execution(
            self.ACTION_ENDPOINT, self.ACTION_OPERATION,
            input_data={'order': 20},
        )
        self.assertEqual(response.status_code, 202)
        execution_id = data['id']

        exec_endpoint = (
            f'{self.ACTION_ENDPOINT}/operations/'
            f'{self.ACTION_OPERATION}/executions/{execution_id}'
        )
        status_data = self.poll_endpoint(exec_endpoint, timeout=10.0, interval=0.3)
        self.assertIn(status_data['status'], ['running', 'completed'])

        response = self.delete_request(
            exec_endpoint,
            timeout=25,
            expected_status=204,
        )
        self.assertEqual(len(response.content), 0)


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        """All processes exit cleanly."""
        for info in proc_info:
            self.assertIn(
                info.returncode, ALLOWED_EXIT_CODES,
                f'{info.process_name} exited with code {info.returncode}',
            )
