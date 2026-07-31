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

"""Cancel against a dead action server maps to 503, not "rejected" (issue #576).

When the action server behind a tracked execution dies, its
``_action/cancel_goal`` service leaves the ROS graph, so a DELETE on the
execution cannot even deliver the cancel request. That is an availability
failure, not a rejection by the server - the gateway must answer
``503`` + ``x-medkit-ros2-action-unavailable``, not the definitive
``400`` + ``x-medkit-ros2-action-rejected`` it used to conflate every
non-success into.

The action server is spawned as a raw subprocess (not a launch action) so
the test can terminate it mid-flight, mirroring the spawn pattern of
``test_graph_event_discovery``.
"""

import os
import signal
import subprocess
import unittest

from ament_index_python.packages import get_package_prefix
from launch import LaunchDescription
import launch_testing
import launch_testing.actions
import requests

from ros2_medkit_test_utils.constants import (
    ALLOWED_EXIT_CODES,
    DEFAULT_DOMAIN_ID,
)
from ros2_medkit_test_utils.coverage import get_coverage_env
from ros2_medkit_test_utils.gateway_test_case import GatewayTestCase
from ros2_medkit_test_utils.launch_helpers import (
    create_gateway_node,
    DEMO_NODE_REGISTRY,
)

APP_ID = 'long_calibration'
OPERATION_ID = 'long_calibration'
ENTITY_ENDPOINT = '/apps/long_calibration'


def generate_test_description():
    # Gateway only - the action server is spawned per-test as a subprocess so
    # it can be killed mid-flight.
    gateway_node = create_gateway_node()
    return (
        LaunchDescription([
            gateway_node,
            launch_testing.actions.ReadyToTest(),
        ]),
        {'gateway_node': gateway_node},
    )


def _resolve_demo_executable(name):
    """Resolve a demo executable to its installed absolute path."""
    pkg = 'ros2_medkit_integration_tests'
    prefix = get_package_prefix(pkg)
    candidate = os.path.join(prefix, 'lib', pkg, name)
    if not os.path.isfile(candidate):
        raise FileNotFoundError(f'demo executable not found: {candidate}')
    return candidate


def _terminate_process(proc):
    if proc is None or proc.poll() is not None:
        return
    proc.send_signal(signal.SIGTERM)
    try:
        proc.wait(timeout=5)
    except subprocess.TimeoutExpired:
        proc.kill()
        proc.wait(timeout=5)


class TestActionCancelUnavailable(GatewayTestCase):
    """DELETE on an execution whose action server died answers 503."""

    MIN_EXPECTED_APPS = 0  # discovery is polled per-test after the spawn

    @classmethod
    def setUpClass(cls):
        super().setUpClass()
        cls._action_proc = None

    @classmethod
    def tearDownClass(cls):
        _terminate_process(cls._action_proc)
        cls._action_proc = None
        super().tearDownClass()

    @classmethod
    def _spawn_action_server(cls):
        executable, ros_name, namespace = DEMO_NODE_REGISTRY['long_calibration']
        env = os.environ.copy()
        env['ROS_DOMAIN_ID'] = str(DEFAULT_DOMAIN_ID)
        env.update(get_coverage_env('ros2_medkit_integration_tests'))
        binary = _resolve_demo_executable(executable)
        return subprocess.Popen(
            [
                binary,
                '--ros-args',
                '-r', f'__ns:={namespace}',
                '-r', f'__node:={ros_name}',
            ],
            env=env,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )

    def test_cancel_after_action_server_death_returns_503(self):
        """Kill the action server post-accept; DELETE maps to 503 unavailable."""
        cls = type(self)
        cls._action_proc = self._spawn_action_server()

        # Wait for the operation to be USABLE (resolved interface type), then
        # start a long-running goal the server will never finish.
        self.wait_for_operation_type_info(
            ENTITY_ENDPOINT, OPERATION_ID,
            ('goal', 'result', 'feedback'), max_wait=30.0,
        )
        _, data = self.create_execution(
            ENTITY_ENDPOINT, OPERATION_ID, input_data={'order': 45},
        )
        execution_id = data['id']
        exec_endpoint = (
            f'{ENTITY_ENDPOINT}/operations/{OPERATION_ID}'
            f'/executions/{execution_id}'
        )

        # The execution must be registered before the server goes away.
        status_data = self.poll_endpoint(exec_endpoint, timeout=10.0, interval=0.3)
        self.assertIn(status_data['status'], ['running', 'completed'])

        # Terminate the action server (SIGTERM: the participant leaves the
        # graph immediately, so the cancel service disappears deterministically
        # instead of lingering for a DDS liveliness lease).
        _terminate_process(cls._action_proc)
        cls._action_proc = None

        # Wait until the gateway's view of the graph has dropped the app -
        # after this, the cancel_goal service is guaranteed gone as well.
        self.poll_endpoint_until(
            '/apps',
            lambda d: d if not any(
                app.get('id') == APP_ID for app in d.get('items', [])
            ) else None,
            timeout=30.0,
            interval=0.5,
        )

        # Cancel now cannot reach any server: availability failure, not a
        # rejection. The gateway waits up to 2s for the service before
        # answering, so give the client comfortable headroom.
        response = requests.delete(
            f'{self.BASE_URL}{exec_endpoint}', timeout=30,
        )
        self.assertEqual(
            response.status_code, 503,
            f'Expected 503 for cancel against a dead action server, got '
            f'{response.status_code}: {response.text}',
        )
        body = response.json()
        self.assertEqual(body.get('error_code'), 'vendor-error')
        self.assertEqual(
            body.get('vendor_code'), 'x-medkit-ros2-action-unavailable',
            f'Expected the availability vendor code, got: {body}',
        )


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        """All processes exit cleanly."""
        for info in proc_info:
            self.assertIn(
                info.returncode, ALLOWED_EXIT_CODES,
                f'{info.process_name} exited with code {info.returncode}',
            )
