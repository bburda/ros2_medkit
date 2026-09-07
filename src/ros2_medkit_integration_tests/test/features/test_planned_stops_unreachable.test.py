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

"""Planned-stop routes with no fault manager on the graph.

The windows live in the fault manager, so with none running there is nowhere to
put a declaration and nothing to list. What matters is that the gateway SAYS so
- 503, inside the transport's timeout - rather than hanging, and that the fault
surfaces keep working without the flag they cannot derive.
"""

import json
import threading
import time
import unittest

import launch_testing
import launch_testing.actions
import rclpy
from rclpy.node import Node
import requests
from ros2_medkit_msgs.msg import Fault, FaultEvent

from ros2_medkit_test_utils.constants import ALLOWED_EXIT_CODES
from ros2_medkit_test_utils.gateway_test_case import GatewayTestCase
from ros2_medkit_test_utils.launch_helpers import create_test_launch

PLANNED_STOPS = '/x-medkit-planned-stops'

# The gateway's fault_manager.service_timeout_sec default. A route that answers
# inside this budget answered rather than hung; the margin is what the assertion
# is really about.
SERVICE_TIMEOUT_SEC = 5.0


def generate_test_description():
    return create_test_launch(
        demo_nodes=['temp_sensor'],
        fault_manager=False,
        lidar_faulty=False,
    )


class TestPlannedStopsWithoutFaultManager(GatewayTestCase):
    """No fault manager: every planned-stop route must fail loudly and quickly."""

    MIN_EXPECTED_APPS = 1
    REQUIRED_APPS = {'temp_sensor'}

    @classmethod
    def setUpClass(cls):
        super().setUpClass()
        rclpy.init()
        cls._publisher_node = Node('planned_stop_unreachable_publisher')
        # The gateway subscribes to this topic whether or not a fault manager
        # exists, so a frame can be driven onto the stream with none running -
        # which is the only way to see what the stream says about a flag it
        # cannot derive.
        cls._event_pub = cls._publisher_node.create_publisher(
            FaultEvent, '/fault_manager/events', 10
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
        event.fault.description = 'published with no fault manager running'
        event.fault.first_occurred.sec = int(time.time())
        event.fault.last_occurred.sec = event.fault.first_occurred.sec
        event.fault.occurrence_count = 1
        event.fault.reporting_sources = ['/powertrain/engine/temp_sensor']
        event.timestamp.sec = event.fault.first_occurred.sec
        self._event_pub.publish(event)
        rclpy.spin_once(self._publisher_node, timeout_sec=0.05)

    def _timed(self, method, path, **kwargs):
        started = time.monotonic()
        response = requests.request(
            method, f'{self.BASE_URL}{path}', timeout=SERVICE_TIMEOUT_SEC * 4, **kwargs
        )
        return response, time.monotonic() - started

    def test_declaring_answers_503_rather_than_hanging(self):
        response, elapsed = self._timed(
            'POST', PLANNED_STOPS,
            json={'to': '2037-01-01T00:00:00Z', 'reason': 'nobody home'},
        )
        self.assertEqual(response.status_code, 503, response.text)
        self.assertEqual(response.json()['error_code'], 'service-unavailable')
        self.assertLess(
            elapsed, SERVICE_TIMEOUT_SEC * 3,
            'the route must give up on the transport timeout, not sit on the request'
        )

    def test_listing_answers_503(self):
        response, elapsed = self._timed('GET', PLANNED_STOPS)
        self.assertEqual(response.status_code, 503, response.text)
        self.assertLess(elapsed, SERVICE_TIMEOUT_SEC * 3)

    def test_reading_one_answers_503_not_404(self):
        # 404 would be a claim about the store, and there is no store to make a
        # claim about.
        response, _ = self._timed('GET', f'{PLANNED_STOPS}/whatever')
        self.assertEqual(response.status_code, 503, response.text)

    def test_ending_answers_503_not_404(self):
        response, _ = self._timed('DELETE', f'{PLANNED_STOPS}/whatever')
        self.assertEqual(response.status_code, 503, response.text)

    def test_a_bad_request_is_still_a_bad_request(self):
        # Validation runs before the transport, so the caller is told what is
        # wrong with the request instead of being told the server is down.
        response, _ = self._timed(
            'POST', PLANNED_STOPS, json={'to': 'not a timestamp', 'reason': 'x'}
        )
        self.assertEqual(response.status_code, 400, response.text)
        self.assertEqual(response.json()['parameters']['parameter'], 'to')

    def test_the_stream_says_nothing_rather_than_expected_false(self):
        """R13: a consumer must not read an outage as "nothing is expected"."""
        frames = []
        stop_event = threading.Event()
        response = requests.get(
            f'{self.BASE_URL}/faults/stream', stream=True, timeout=(5, 30)
        )
        self.assertEqual(response.status_code, 200)

        def pump():
            current = {}
            try:
                for line in response.iter_lines(decode_unicode=True):
                    if stop_event.is_set():
                        break
                    if line is None:
                        continue
                    if line == '':
                        if current:
                            frames.append(current)
                            current = {}
                        continue
                    if line.startswith(':'):
                        continue
                    key, _, value = line.partition(':')
                    current[key.strip()] = value.strip()
            except Exception:  # noqa: BLE001 - closed socket on teardown
                pass

        pump_thread = threading.Thread(target=pump, daemon=True)
        pump_thread.start()
        try:
            payload = None
            deadline = time.monotonic() + 25.0
            while payload is None and time.monotonic() < deadline:
                self._publish_event('PS_NO_MANAGER')
                settle = min(time.monotonic() + 1.0, deadline)
                while payload is None and time.monotonic() < settle:
                    for frame in list(frames):
                        data = frame.get('data')
                        if not data:
                            continue
                        candidate = json.loads(data)
                        if candidate.get('fault', {}).get('fault_code') == 'PS_NO_MANAGER':
                            payload = candidate
                            break
                    time.sleep(0.1)
            self.assertIsNotNone(payload, 'no frame for the published event reached the stream')

            extension = payload.get('x-medkit', {})
            self.assertNotIn(
                'expected', extension,
                'with no window set ever read, the frame must not claim the fault was unexpected'
            )
            self.assertNotIn('planned_stop_id', extension)
        finally:
            stop_event.set()
            response.close()
            pump_thread.join(timeout=5)

    def test_the_fault_stream_still_serves_without_the_flag_it_cannot_derive(self):
        # The flag is derived from windows the gateway cannot read. Losing it
        # must not cost the stream: a consumer with no fault manager still needs
        # its connection, and it must not wait a transport timeout per frame.
        started = time.monotonic()
        response = requests.get(
            f'{self.BASE_URL}/faults/stream', stream=True, timeout=(5, 15)
        )
        try:
            self.assertEqual(response.status_code, 200)
            self.assertLess(time.monotonic() - started, SERVICE_TIMEOUT_SEC * 2)
        finally:
            response.close()


@launch_testing.post_shutdown_test()
class TestPlannedStopsUnreachableShutdown(unittest.TestCase):

    def test_gateway_exits_cleanly(self, proc_info, gateway_node):
        launch_testing.asserts.assertExitCodes(
            proc_info, process=gateway_node, allowable_exit_codes=ALLOWED_EXIT_CODES
        )
