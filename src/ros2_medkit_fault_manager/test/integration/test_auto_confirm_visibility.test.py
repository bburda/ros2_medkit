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

"""
A time-based confirmation must be as visible as a reported one.

auto_confirm_after_sec promotes a fault that has sat in PREFAILED past its
window. Everything downstream of a confirmation keys off the event stream: the
SSE fault feed the gateway serves, black-box capture, per-entity freeze frames.
A promotion that only reaches the database is an alarm nobody is told about,
so this pins the event rather than the stored status.
"""

import os
import shutil
import tempfile
import threading
import time
import unittest

from launch import LaunchDescription
import launch_ros.actions
import launch_testing.actions
import launch_testing.markers
import rclpy
from rclpy.node import Node
from ros2_medkit_msgs.msg import Fault, FaultEvent
from ros2_medkit_msgs.srv import ReportFault

# Below -1 so the first FAILED lands in PREFAILED and only the timer can promote it.
CONFIRMATION_THRESHOLD = -2
AUTO_CONFIRM_SEC = 3.0

_temp_dirs = []


@launch_testing.markers.keep_alive
def generate_test_description():
    """Launch fault_manager with time-based confirmation enabled."""
    temp_dir = tempfile.mkdtemp(prefix='auto_confirm_visibility_')
    _temp_dirs.append(temp_dir)

    fault_manager_node = launch_ros.actions.Node(
        package='ros2_medkit_fault_manager',
        executable='fault_manager_node',
        name='fault_manager',
        output='screen',
        parameters=[{
            'storage_type': 'sqlite',
            'database_path': os.path.join(temp_dir, 'faults.db'),
            'confirmation_threshold': CONFIRMATION_THRESHOLD,
            'auto_confirm_after_sec': AUTO_CONFIRM_SEC,
        }],
        sigterm_timeout='30',
        sigkill_timeout='15',
    )

    return (
        LaunchDescription([
            fault_manager_node,
            launch_testing.actions.ReadyToTest(),
        ]),
        {
            'fault_manager_node': fault_manager_node,
        },
    )


class TestAutoConfirmVisibility(unittest.TestCase):
    """A timer-driven confirmation must publish the same event a report does."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = Node('test_auto_confirm_client')
        cls.report_client = cls.node.create_client(ReportFault, '/fault_manager/report_fault')

        cls.events = []
        cls.events_lock = threading.Lock()
        cls.node.create_subscription(
            FaultEvent, '/fault_manager/events', cls._on_event, 100
        )

        assert cls.report_client.wait_for_service(timeout_sec=20.0), \
            'report_fault service not available'

    @classmethod
    def _on_event(cls, msg):
        with cls.events_lock:
            cls.events.append((msg.event_type, msg.fault.fault_code))

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def _events_for(self, fault_code):
        with self.events_lock:
            return [e for e in self.events if e[1] == fault_code]

    def _spin_for(self, seconds):
        deadline = time.time() + seconds
        while time.time() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.1)

    def _report_failed(self, fault_code):
        request = ReportFault.Request()
        request.fault_code = fault_code
        request.event_type = ReportFault.Request.EVENT_FAILED
        request.severity = Fault.SEVERITY_ERROR
        request.description = 'auto-confirm visibility test'
        request.source_id = '/test_plc'
        future = self.report_client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=20.0)
        self.assertIsNotNone(future.result(), 'ReportFault timed out')
        self.assertTrue(future.result().accepted)

    def test_01_timer_confirmation_publishes_a_confirmed_event(self):
        """
        The one event a subscriber has to see.

        A single FAILED leaves the fault in PREFAILED, and the reporter never
        sends a second one, so the timer is what confirms it. If that promotion
        does not publish, the SSE feed and every capture path downstream stay
        silent for a fault that is now confirmed in the database.
        """
        code = 'PLC_TIMER_CONFIRMED'
        self._report_failed(code)

        # Nothing confirmed yet: the raise alone must not produce a confirmation.
        self._spin_for(0.5)
        self.assertNotIn(
            FaultEvent.EVENT_CONFIRMED, [e[0] for e in self._events_for(code)],
            'the fault confirmed on its first failed report, so the timer is not '
            'what is under test'
        )

        self._spin_for(AUTO_CONFIRM_SEC + 4.0)

        self.assertIn(
            FaultEvent.EVENT_CONFIRMED, [e[0] for e in self._events_for(code)],
            'a time-based confirmation published no event, so nothing downstream of it is told'
        )

    def test_02_the_event_carries_the_confirmed_fault(self):
        """The published event must name the fault, not just fire."""
        code = 'PLC_TIMER_PAYLOAD'
        self._report_failed(code)
        self._spin_for(AUTO_CONFIRM_SEC + 4.0)

        confirmed = [e for e in self._events_for(code) if e[0] == FaultEvent.EVENT_CONFIRMED]
        self.assertEqual(
            len(confirmed), 1, f'expected exactly one confirmation event, got {confirmed}'
        )
        self.assertEqual(confirmed[0][1], code)


@launch_testing.post_shutdown_test()
class TestAutoConfirmVisibilityShutdown(unittest.TestCase):
    """Check the node exited cleanly and clean up the database."""

    def test_exit_code(self, proc_info, fault_manager_node):
        launch_testing.asserts.assertExitCodes(proc_info, process=fault_manager_node)

    def test_temp_dirs_removed(self):
        for path in _temp_dirs:
            shutil.rmtree(path, ignore_errors=True)
