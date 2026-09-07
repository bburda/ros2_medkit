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

"""A fault manager that answers slowly must not stall the gateway.

Deriving the planned-stop flag costs a service call. That call used to happen
while the SSE handler held its queue mutex - the same mutex the ROS subscription
callback takes to enqueue an event - so a fault manager that was advertised but
slow stopped the gateway's executor thread, and every SSE client with it, for as
long as the call took.

The instrument is `x-medkit-sse.events_received` on `/health`: fault events the
gateway has ACCEPTED, counted on the executor thread. Measuring frame arrival
instead would not separate the two paths - the stream's own loop legitimately
waits for the refresh on its own thread - and measuring with no fault manager at
all proves nothing, because then the call is skipped and returns in microseconds.
"""

import json
import os
import sys
import threading
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

# The stub's own delay. Anything the enqueue path pays would be a multiple of
# this, so the budget below sits well under it.
MANAGER_DELAY_SEC = 3.0

# What "the enqueue path never blocks" is worth asserting at. Comfortably above
# DDS delivery on a loaded machine, comfortably below one stalled refresh.
ENQUEUE_BUDGET_SEC = 1.5

# Enough events, spaced past the cache's time to live, that the window certainly
# spans one of the stream loop's own refreshes. The FIRST refresh happens in the
# initial-replay block, which is outside the queue lock either way, so measuring
# only around it would prove nothing.
EVENT_COUNT = 10


def generate_test_description():
    manager = ExecuteProcess(
        cmd=[
            sys.executable,
            os.path.join(os.path.dirname(os.path.abspath(__file__)),
                         'slow_planned_stop_manager.py'),
        ],
        name='slow_planned_stop_manager',
        output='screen',
    )

    gateway = create_gateway_node(
        extra_params={
            # Points the gateway at the stub, which is `fault_manager` inside
            # this namespace.
            'fault_manager.namespace': 'slow',
            # The stub answers inside this, so the call SUCCEEDS slowly rather
            # than failing and tripping the failure back-off - a refresh that
            # fails would stop happening and the slowness with it.
            'fault_manager.service_timeout_sec': 10.0,
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


class TestPlannedStopsSlowManager(GatewayTestCase):
    """The blocking half of the derivation must stay off the executor."""

    # Nothing is discovered here: the fixture is about the fault-manager path, so
    # MIN_EXPECTED_APPS of 0 skips the discovery wait entirely.
    MIN_EXPECTED_APPS = 0
    REQUIRED_APPS = set()

    @classmethod
    def setUpClass(cls):
        super().setUpClass()
        rclpy.init()
        cls._publisher_node = Node('slow_manager_event_publisher')
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

    def _events_received(self):
        sse = self.get_json('/health').get('x-medkit-sse', {})
        self.assertIn(
            'events_received', sse,
            'the counter this case measures with is missing from /health'
        )
        return sse['events_received']

    def _wait_for_events(self, target, budget):
        deadline = time.monotonic() + budget
        last = None
        while time.monotonic() < deadline:
            last = self._events_received()
            if last >= target:
                return time.monotonic()
            time.sleep(0.05)
        raise AssertionError(
            f'the gateway accepted {last} of {target} events within {budget}s; '
            'the thread that accepts them is blocked'
        )

    @staticmethod
    def _pump(response, frames, stop_event):
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

    def test_a_slow_manager_does_not_block_the_event_intake(self):
        frames = []
        stop_event = threading.Event()
        response = requests.get(
            f'{self.BASE_URL}/faults/stream', stream=True, timeout=(5, 90)
        )
        self.assertEqual(response.status_code, 200)
        pump = threading.Thread(target=self._pump, args=(response, frames, stop_event),
                                daemon=True)
        pump.start()
        try:
            # Prime. Two things have to happen before anything can be measured:
            # the gateway's subscription has to match this publisher - an event
            # published before it does is lost outright, the volatile-topic race
            # every fault test in this suite primes around - and the stream loop
            # has to do its first refresh, which is the slow one.
            baseline = self._events_received()
            deadline = time.monotonic() + 60.0
            while time.monotonic() < deadline:
                self._publish_event('SLOW_PRIME')
                settle = min(time.monotonic() + 1.0, deadline)
                while time.monotonic() < settle:
                    if self._events_received() > baseline:
                        break
                    time.sleep(0.1)
                if self._events_received() > baseline:
                    break
            self.assertGreater(
                self._events_received(), baseline,
                'no event ever reached the gateway; the fixture never went live'
            )

            # Let the initial-replay refresh finish and the next one fall due.
            # That first one runs before the loop is entered and is outside the
            # queue lock in every variant, so it is not the one under test.
            time.sleep(MANAGER_DELAY_SEC + 2.0)

            # Now the measurement, one event at a time. The worst SINGLE wait is
            # what discriminates: a burst measured end to end hides one stalled
            # refresh inside a generous total, because the events queue in DDS
            # and all arrive at once when the lock is finally released.
            #
            # Spaced past the cache's time to live so at least one refresh
            # certainly falls inside the measured window.
            worst = 0.0
            slowest_event = None
            for index in range(EVENT_COUNT):
                before = self._events_received()
                started = time.monotonic()
                self._publish_event(f'SLOW_EVENT_{index}')
                self._wait_for_events(before + 1, budget=MANAGER_DELAY_SEC * 4)
                elapsed = time.monotonic() - started
                if elapsed > worst:
                    worst = elapsed
                    slowest_event = index
                time.sleep(0.7)
            print(f'worst single enqueue wait: {worst:.3f}s (event {slowest_event})')
            self.assertLess(
                worst, ENQUEUE_BUDGET_SEC,
                f'event {slowest_event} waited {worst:.2f}s to be accepted, which is the '
                f'shape of a {MANAGER_DELAY_SEC}s service call holding the queue mutex'
            )

            # And the stream is still a stream: the frames arrive, late or not.
            deadline = time.monotonic() + 60.0
            seen = set()
            while time.monotonic() < deadline and len(seen) < EVENT_COUNT:
                for frame in list(frames):
                    data = frame.get('data')
                    if not data:
                        continue
                    code = json.loads(data).get('fault', {}).get('fault_code', '')
                    if code.startswith('SLOW_EVENT_'):
                        seen.add(code)
                time.sleep(0.1)
            self.assertEqual(
                len(seen), EVENT_COUNT,
                f'the stream stopped delivering: {sorted(seen)}'
            )
        finally:
            stop_event.set()
            response.close()
            pump.join(timeout=5)

    def test_the_gateway_stays_answerable_while_the_manager_is_slow(self):
        # /health is served by an HTTP worker, but a blocked executor takes the
        # gateway's discovery and subscription work with it, and the counter
        # above is read through this route.
        for _ in range(5):
            started = time.monotonic()
            self.get_json('/health')
            self.assertLess(
                time.monotonic() - started, ENQUEUE_BUDGET_SEC,
                '/health must not wait on the fault manager'
            )


@launch_testing.post_shutdown_test()
class TestPlannedStopsSlowManagerShutdown(unittest.TestCase):

    def test_gateway_exits_cleanly(self, proc_info, gateway_node):
        launch_testing.asserts.assertExitCodes(
            proc_info, process=gateway_node, allowable_exit_codes=ALLOWED_EXIT_CODES
        )
