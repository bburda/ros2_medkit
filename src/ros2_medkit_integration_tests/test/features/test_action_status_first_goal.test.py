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

"""The `/_action/status` stream must carry the FIRST goal on a path (issue #576).

Since the cancel-timeout work, the action status stream is the gateway's only
source of truth for a goal's terminal state: `update_goal_status` refuses to
leave a terminal status, `map_cancel_result` decides 204-vs-504 by reading it,
and `get_action_result` has no production caller, so nothing else can ever
correct a goal's status.

The stream is subscribed lazily - `send_action_goal` tracks the goal and only
then calls `subscribe_to_action_status` - so on the first goal for a path the
DDS subscription is still matching while the action is already running. An
action that finishes inside that window publishes its terminal status to a
reader that does not exist yet. Whether the gateway ever learns the outcome
then depends entirely on the subscription's durability: a VOLATILE reader is
delivered nothing on match, a TRANSIENT_LOCAL one receives the writer's last
sample, which is exactly the terminal frame it missed.

`Fibonacci(order=1)` runs zero loop iterations in the demo action, so it
terminates essentially at accept time - the window this test needs. The
execution is the first goal of a freshly launched gateway, so no earlier
subscription can mask the effect.
"""

import unittest

import launch_testing
import requests

from ros2_medkit_test_utils.constants import (
    ALLOWED_EXIT_CODES,
    API_BASE_PATH,
    get_test_port,
    get_time_scale,
)
from ros2_medkit_test_utils.gateway_test_case import GatewayTestCase
from ros2_medkit_test_utils.launch_helpers import create_test_launch

# Zero loop iterations in demo_long_calibration_action: the goal succeeds
# immediately after it is accepted.
IMMEDIATE_ORDER = 1

# The action itself needs milliseconds. This budget covers DDS subscription
# matching plus the gateway's own tracking, and is deliberately far larger
# than either so a failure means "never arrived", not "arrived late".
TERMINAL_STATUS_BUDGET_SEC = 20.0 * get_time_scale()

TERMINAL_ROS2_STATUSES = {'succeeded', 'canceled', 'aborted'}


def generate_test_description():
    return create_test_launch(
        demo_nodes=['long_calibration'],
        fault_manager=False,
    )


class TestActionStatusFirstGoal(GatewayTestCase):
    """A goal that terminates during subscription matching still reports terminal."""

    MIN_EXPECTED_APPS = 1
    REQUIRED_APPS = {'long_calibration'}

    def test_first_goal_that_completes_immediately_reports_terminal_status(self):
        """The gateway must learn the outcome of the first goal on a path.

        Without it the execution is pinned at `executing` forever: no further
        status frame is published, no RPC re-reads the goal, a timed-out cancel
        can never reconcile to 204, and at 2x max_age the goal is force-evicted
        with a "server crashed" warning naming a server that did its job.
        """
        self.wait_for_operation('/apps/long_calibration', 'long_calibration', max_wait=45.0)

        created_response = requests.post(
            f'{self.BASE_URL}/apps/long_calibration/operations/long_calibration/executions',
            json={'parameters': {'order': IMMEDIATE_ORDER}},
            timeout=10,
        )
        self.assertEqual(created_response.status_code, 202, created_response.text)
        execution_id = created_response.json()['id']

        endpoint = (
            f'/apps/long_calibration/operations/long_calibration/executions/{execution_id}'
        )
        self.assertEqual(
            created_response.headers.get('Location'), f'{API_BASE_PATH}{endpoint}',
            'the 202 Location must be the absolute path of the execution resource',
        )
        # Follow the Location the server handed out, exactly as a client would:
        # a prefix regression makes this 404 while a test that compares the
        # handler's output to its own input stays green.
        followed = requests.get(
            f"http://localhost:{get_test_port()}{created_response.headers['Location']}", timeout=10
        )
        self.assertEqual(followed.status_code, 200, followed.text)
        self.assertEqual(followed.json().get('capability'), 'execute')

        final = self.poll_endpoint_until(
            endpoint,
            lambda d: d if (d.get('x-medkit') or {}).get('ros2_status') in TERMINAL_ROS2_STATUSES else None,
            timeout=TERMINAL_STATUS_BUDGET_SEC,
            interval=0.3,
        )
        self.assertEqual(
            (final.get('x-medkit') or {}).get('ros2_status'), 'succeeded',
            f'the immediate goal should be reported as succeeded: {final}',
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
