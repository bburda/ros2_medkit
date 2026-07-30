#!/usr/bin/env python3
# Copyright 2026 mfaferek93
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

"""End-to-end test for the SSE fault stream with a real fault manager.

Drives a fault through ReportFault (FAILED -> confirm, PASSED x2 -> heal)
and asserts both transitions arrive at an HTTP client on /faults/stream:
fault_confirmed with status CONFIRMED, then fault_cleared with status
HEALED. This is the full pipeline: service -> fault manager -> events
topic -> gateway SSE handler -> HTTP.
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
from ros2_medkit_msgs.msg import Fault
from ros2_medkit_msgs.srv import ReportFault

from ros2_medkit_test_utils.constants import ALLOWED_EXIT_CODES, FAULT_TIMEOUT
from ros2_medkit_test_utils.gateway_test_case import GatewayTestCase
from ros2_medkit_test_utils.launch_helpers import create_test_launch


FAULT_CODE = 'SSE_E2E_HEAL_ME'
PRIME_CODE = 'SSE_E2E_PRIME'
SOURCE_ID = '/powertrain/engine/temp_sensor'


def generate_test_description():
    return create_test_launch(
        demo_nodes=['temp_sensor'],
        fault_manager=True,
        fault_manager_params={
            'storage_type': 'memory',
            # One FAILED confirms (counter -1 <= -1); two PASSED heal
            # (counter -1 -> 0 -> 1 >= 1).
            'confirmation_threshold': -1,
            'healing_enabled': True,
            'healing_threshold': 1,
            'snapshots.rosbag.enabled': False,
        },
    )


class TestSseFaultStreamE2E(GatewayTestCase):
    """Fault confirm + heal must reach an HTTP SSE client."""

    MIN_EXPECTED_APPS = 1
    REQUIRED_APPS = {'temp_sensor'}

    @classmethod
    def setUpClass(cls):
        super().setUpClass()
        rclpy.init()
        cls._reporter = Node('sse_fault_stream_reporter')
        cls._report_client = cls._reporter.create_client(
            ReportFault, '/fault_manager/report_fault'
        )
        assert cls._report_client.wait_for_service(timeout_sec=15.0), \
            'report_fault service not available'

    @classmethod
    def tearDownClass(cls):
        cls._reporter.destroy_node()
        rclpy.shutdown()

    def _report(self, event_type, fault_code=FAULT_CODE):
        req = ReportFault.Request()
        req.fault_code = fault_code
        req.event_type = event_type
        req.severity = Fault.SEVERITY_ERROR
        req.description = 'SSE E2E test fault'
        req.source_id = SOURCE_ID
        future = self._report_client.call_async(req)
        deadline = time.monotonic() + 5.0
        while not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(self._reporter, timeout_sec=0.1)
        self.assertTrue(future.done(), 'report_fault call timed out')
        self.assertTrue(future.result().accepted)

    @staticmethod
    def _pump_stream(response, frames, stop_event):
        """Collect SSE frames as dicts of field -> value."""
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
                    continue  # keepalive comment
                key, _, value = line.partition(':')
                current[key.strip()] = value.strip()
        except Exception:  # noqa: BLE001 - closed socket on test teardown
            pass

    def _prime_stream(self, frames):
        """Block until the fault event pipeline demonstrably reaches the stream.

        /fault_manager/events is reliable but volatile: an event published
        before the gateway's subscription has matched the fault manager's
        publisher is lost outright, and under sanitizer load that matching can
        finish after the first report (ASan CI lost the one-shot confirm to
        exactly this race). Every accepted report publishes an event, so
        repeating a sacrificial fault until one of its frames arrives proves
        service -> fault manager -> events topic -> SSE end to end.
        """
        deadline = time.monotonic() + FAULT_TIMEOUT
        while time.monotonic() < deadline:
            self._report(ReportFault.Request.EVENT_FAILED, fault_code=PRIME_CODE)
            settle = min(time.monotonic() + 1.0, deadline)
            while time.monotonic() < settle:
                for frame in list(frames):
                    data = frame.get('data')
                    if data is None:
                        continue
                    payload = json.loads(data)
                    if payload.get('fault', {}).get('fault_code') == PRIME_CODE:
                        return
                time.sleep(0.1)
        raise AssertionError(
            f'no event for priming fault {PRIME_CODE} on /faults/stream '
            f'within {FAULT_TIMEOUT}s; events pipeline never went live'
        )

    def _wait_for_event(self, frames, event_type, deadline):
        while time.monotonic() < deadline:
            for frame in list(frames):
                if frame.get('event') != event_type or 'data' not in frame:
                    continue
                payload = json.loads(frame['data'])
                if payload['fault']['fault_code'] == FAULT_CODE:
                    return payload
            time.sleep(0.2)
        raise AssertionError(
            f'no {event_type} for {FAULT_CODE} on /faults/stream within '
            f'{FAULT_TIMEOUT}s; frames seen: {list(frames)}'
        )

    def test_confirm_and_heal_reach_http_client(self):
        frames = []
        stop_event = threading.Event()
        response = requests.get(
            f'{self.BASE_URL}/faults/stream', stream=True, timeout=(5, 60)
        )
        self.assertEqual(response.status_code, 200)
        pump = threading.Thread(
            target=self._pump_stream, args=(response, frames, stop_event),
            daemon=True,
        )
        pump.start()
        try:
            self._prime_stream(frames)
            self._report(ReportFault.Request.EVENT_FAILED)
            confirmed = self._wait_for_event(
                frames, 'fault_confirmed', time.monotonic() + FAULT_TIMEOUT
            )
            self.assertEqual(confirmed['fault']['status'], 'CONFIRMED')

            self._report(ReportFault.Request.EVENT_PASSED)
            self._report(ReportFault.Request.EVENT_PASSED)
            cleared = self._wait_for_event(
                frames, 'fault_cleared', time.monotonic() + FAULT_TIMEOUT
            )
            # The heal is the fault's end; status distinguishes it from a
            # manual clear.
            self.assertEqual(cleared['fault']['status'], 'HEALED')
            self.assertIn('last_passed', cleared['fault'])
        finally:
            stop_event.set()
            response.close()
            pump.join(timeout=5)


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        """All processes exited cleanly (SIGTERM allowed for SSE teardown)."""
        for info in proc_info:
            self.assertIn(
                info.returncode, ALLOWED_EXIT_CODES,
                f'{info.process_name} exited with code {info.returncode}'
            )
