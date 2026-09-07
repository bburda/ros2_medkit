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

"""What the fault list says once the windows stop being readable.

Three claims, all about the same moment: a fault manager that still serves
faults and no longer serves planned stops.

S1  The flag goes QUIET, not false. `expected: false` there would tell an
    operator that nothing was planned during a stop the gateway simply cannot
    read - the same ruling the event stream already follows, applied where a
    reader actually looks.
S2  Window knowledge EXPIRES. A gateway that read a set once used to serve it
    for the life of the process: replace the fault manager and `GET /faults`
    kept naming a window that existed nowhere, while
    `GET /x-medkit-planned-stops` answered 503 at the same instant. Two
    endpoints of one API contradicting each other, indefinitely.
S3  The list stays FAST. The planned-stop client's readiness is part of the
    decision to call it, so a manager without the service costs no timeout.

The stub serves one window, then destroys its planned-stop services. Asserting
only the end state would not show S2: a gateway that never read the window at
all would pass it.
"""

import os
import sys
import time
import unittest

from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
import launch_testing
import launch_testing.actions
import rclpy
from rclpy.node import Node
import requests
from ros2_medkit_msgs.msg import Fault, FaultEvent

from ros2_medkit_test_utils.constants import ALLOWED_EXIT_CODES
from ros2_medkit_test_utils.gateway_test_case import GatewayTestCase
from ros2_medkit_test_utils.launch_helpers import create_gateway_node

PLANNED_STOPS = '/x-medkit-planned-stops'

# The stub answers planned-stop reads for this long, then destroys the services.
DROP_AFTER_SEC = 12.0

# FaultManager::kPlannedStopKnowledgeMaxAge. Past this, a set nothing has
# refreshed is no longer reported.
KNOWLEDGE_MAX_AGE_SEC = 6.0

# A list that has to wait out a service timeout takes seconds; one that knows
# not to ask takes milliseconds. The budget is far below the gateway's own
# timeout so it cannot pass by accident.
LIST_BUDGET_SEC = 1.5
SERVICE_TIMEOUT_SEC = 5.0


def generate_test_description():
    manager = ExecuteProcess(
        cmd=[
            sys.executable,
            os.path.join(os.path.dirname(os.path.abspath(__file__)),
                         'slow_planned_stop_manager.py'),
            '--delay', '0',
            '--window',
            '--drop-after', str(DROP_AFTER_SEC),
        ],
        name='vanishing_planned_stop_manager',
        output='screen',
    )

    gateway = create_gateway_node(
        extra_params={
            'fault_manager.namespace': 'slow',
            'fault_manager.service_timeout_sec': SERVICE_TIMEOUT_SEC,
            'sse.keepalive_interval_sec': 2,
        },
    )

    return (
        LaunchDescription([
            manager,
            TimerAction(period=2.0, actions=[gateway]),
            launch_testing.actions.ReadyToTest(),
        ]),
        {'gateway_node': gateway},
    )


class TestPlannedStopsStaleWindows(GatewayTestCase):
    """A window nobody can read must stop being reported."""

    MIN_EXPECTED_APPS = 0
    REQUIRED_APPS = set()

    @classmethod
    def setUpClass(cls):
        super().setUpClass()
        rclpy.init()
        cls._publisher_node = Node('stale_window_event_publisher')
        cls._event_pub = cls._publisher_node.create_publisher(
            FaultEvent, '/slow/fault_manager/events', 10
        )

    @classmethod
    def tearDownClass(cls):
        cls._publisher_node.destroy_node()
        rclpy.shutdown()

    def _publish_event(self, fault_code):
        event = FaultEvent()
        event.event_type = FaultEvent.EVENT_CONFIRMED
        event.fault.fault_code = fault_code
        event.fault.severity = Fault.SEVERITY_ERROR
        event.fault.status = Fault.STATUS_CONFIRMED
        event.fault.first_occurred.sec = int(time.time())
        event.fault.last_occurred.sec = event.fault.first_occurred.sec
        event.fault.occurrence_count = 1
        event.fault.reporting_sources = ['/powertrain/engine/temp_sensor']
        event.timestamp.sec = event.fault.first_occurred.sec
        self._event_pub.publish(event)
        rclpy.spin_once(self._publisher_node, timeout_sec=0.05)

    def _timed_get(self, path):
        started = time.monotonic()
        response = requests.get(f'{self.BASE_URL}{path}', timeout=SERVICE_TIMEOUT_SEC * 4)
        return response, time.monotonic() - started

    def test_the_flag_goes_quiet_when_the_windows_stop_being_readable(self):
        # S2's first half: the gateway must genuinely READ the window first, or
        # the end state proves nothing.
        windows, _ = self._timed_get(PLANNED_STOPS)
        self.assertEqual(windows.status_code, 200, windows.text)
        self.assertEqual(
            [w['id'] for w in windows.json()['items']], ['stub-window'],
            'the stub must be serving its window while it still has the service'
        )

        listing, _ = self._timed_get('/faults?status=all')
        self.assertEqual(listing.status_code, 200, listing.text)
        self.assertIn(
            'expected_count', listing.json()['x-medkit'],
            'while the windows are readable the tally must be stated'
        )

        # Now let the services go, and the knowledge age out behind them.
        deadline = time.monotonic() + DROP_AFTER_SEC + KNOWLEDGE_MAX_AGE_SEC + 30.0
        while time.monotonic() < deadline:
            listing, elapsed = self._timed_get('/faults?status=all')
            self.assertEqual(listing.status_code, 200, listing.text)
            if 'expected_count' not in listing.json()['x-medkit']:
                break
            time.sleep(0.5)
        else:
            self.fail(
                'the fault list still reported a planned-stop tally long after the '
                'windows became unreadable'
            )

        # S1: quiet, not false - on the items as well as the collection.
        for item in listing.json()['items']:
            self.assertNotIn(
                'expected', item.get('x-medkit', {}),
                'a fault must not be reported as unexpected on a gateway that '
                'cannot read the windows'
            )

        # S2's second half: the two endpoints agree. One saying 503 while the
        # other names a window is the contradiction this rules out.
        windows, _ = self._timed_get(PLANNED_STOPS)
        self.assertEqual(
            windows.status_code, 503,
            'the windows endpoint must admit it cannot read them'
        )

        # S3: and none of that costs a service timeout.
        for _ in range(3):
            _, elapsed = self._timed_get('/faults?status=all')
            self.assertLess(
                elapsed, LIST_BUDGET_SEC,
                'the fault list waited out the planned-stop service timeout; the '
                'client readiness check is not part of the decision to call'
            )

    def test_a_filtered_list_keeps_nothing_it_cannot_judge(self):
        # Asking for one half of a set the gateway cannot read has no honest
        # answer, so neither half is served.
        for value in ('true', 'false'):
            response, elapsed = self._timed_get(f'/faults?status=all&expected={value}')
            self.assertEqual(response.status_code, 200, response.text)
            self.assertEqual(
                response.json()['items'], [],
                f'?expected={value} served items whose flag the gateway does not know'
            )
            self.assertLess(elapsed, LIST_BUDGET_SEC)


@launch_testing.post_shutdown_test()
class TestPlannedStopsStaleWindowsShutdown(unittest.TestCase):

    def test_gateway_exits_cleanly(self, proc_info, gateway_node):
        launch_testing.asserts.assertExitCodes(
            proc_info, process=gateway_node, allowable_exit_codes=ALLOWED_EXIT_CODES
        )
