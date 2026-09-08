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
A time-based confirmation obeys the planned stop like any other.

``auto_confirm_after_sec`` confirms a lingering PREFAILED fault from a timer
rather than from a report, on a code path of its own. A stop that only silenced
the report path would let that timer announce, over the stream, a fault the list
is hiding - so this drives a confirmation THROUGH the timer while a stop stands.
"""

import os
import tempfile
import threading
import time
import unittest

from launch import LaunchDescription
import launch.actions
import launch_ros.actions
import launch_testing.actions
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from ros2_medkit_msgs.msg import Fault, FaultEvent
from ros2_medkit_msgs.srv import GetFault, ListFaults, ReportFault, SetPlannedStop

STORAGE_DIR = tempfile.mkdtemp(prefix='planned_stop_auto_confirm_')
DATABASE_PATH = os.path.join(STORAGE_DIR, 'faults.db')

# One FAILED report leaves the fault PREFAILED; the timer is then the only thing
# that can confirm it.
CONFIRMATION_THRESHOLD = -2
AUTO_CONFIRM_AFTER_SEC = 2.0

MUTED_CODE = 'PS_TIMER_MUTED'
CONTROL_CODE = 'PS_TIMER_CONTROL'


def get_coverage_env():
    """Get environment variables for gcov coverage data collection."""
    try:
        from ament_index_python.packages import get_package_prefix
        pkg_prefix = get_package_prefix('ros2_medkit_fault_manager')
        workspace = os.path.dirname(os.path.dirname(pkg_prefix))
        build_dir = os.path.join(workspace, 'build', 'ros2_medkit_fault_manager')

        if os.path.exists(build_dir):
            return {
                'GCOV_PREFIX': build_dir,
                'GCOV_PREFIX_STRIP': str(build_dir.count(os.sep)),
            }
    except Exception:
        # Coverage environment is optional; on any error, fall back to no extra config
        pass
    return {}


def generate_test_description():
    """Launch a fault manager that auto-confirms lingering PREFAILED faults."""
    fault_manager_env = get_coverage_env()
    fault_manager_env['ROS_LOCALHOST_ONLY'] = '1'

    fault_manager_node = launch_ros.actions.Node(
        package='ros2_medkit_fault_manager',
        executable='fault_manager_node',
        name='fault_manager',
        output='screen',
        additional_env=fault_manager_env,
        parameters=[{
            'storage_type': 'sqlite',
            'database_path': DATABASE_PATH,
            'confirmation_threshold': CONFIRMATION_THRESHOLD,
            'auto_confirm_after_sec': AUTO_CONFIRM_AFTER_SEC,
            'snapshots.enabled': False,
            'snapshots.rosbag.enabled': False,
        }],
        sigterm_timeout='30',
        sigkill_timeout='15',
    )

    return (
        LaunchDescription([
            launch.actions.TimerAction(period=2.0, actions=[fault_manager_node]),
            launch_testing.actions.ReadyToTest(),
        ]),
        {'fault_manager_node': fault_manager_node},
    )


class TestPlannedStopAutoConfirm(unittest.TestCase):
    """The timer path honours the stop, and announces once it is withdrawn."""

    @classmethod
    def setUpClass(cls):
        os.environ['ROS_LOCALHOST_ONLY'] = '1'
        rclpy.init()
        cls.node = Node('test_planned_stop_auto_confirm_client')

        cls.report_client = cls.node.create_client(ReportFault, '/fault_manager/report_fault')
        cls.list_client = cls.node.create_client(ListFaults, '/fault_manager/list_faults')
        cls.get_client = cls.node.create_client(GetFault, '/fault_manager/get_fault')
        cls.set_stop_client = cls.node.create_client(
            SetPlannedStop, '/fault_manager/set_planned_stop')
        for client, name in (
            (cls.report_client, 'report_fault'),
            (cls.list_client, 'list_faults'),
            (cls.get_client, 'get_fault'),
            (cls.set_stop_client, 'set_planned_stop'),
        ):
            assert client.wait_for_service(timeout_sec=20.0), f'{name} service not available'

        cls._events = []
        cls._events_lock = threading.Lock()
        cls._event_sub = cls.node.create_subscription(
            FaultEvent, '/fault_manager/events', cls._on_event,
            QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                       history=HistoryPolicy.KEEP_LAST, depth=200))

        cls._spinning = True
        cls._spin_thread = threading.Thread(target=cls._spin, daemon=True)
        cls._spin_thread.start()
        cls._prime_event_stream()

    @classmethod
    def tearDownClass(cls):
        cls._spinning = False
        cls._spin_thread.join(timeout=5)
        cls.node.destroy_node()
        rclpy.shutdown()

    @classmethod
    def _on_event(cls, msg):
        with cls._events_lock:
            cls._events.append((msg.event_type, msg.fault.fault_code))

    @classmethod
    def _spin(cls):
        while cls._spinning:
            rclpy.spin_once(cls.node, timeout_sec=0.05)

    @classmethod
    def _call(cls, client, request, timeout_sec=15.0):
        future = client.call_async(request)
        deadline = time.monotonic() + timeout_sec
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.005)
        assert future.done(), 'service call timed out'
        return future.result()

    @classmethod
    def _report(cls, fault_code):
        request = ReportFault.Request()
        request.fault_code = fault_code
        request.event_type = ReportFault.Request.EVENT_FAILED
        request.severity = Fault.SEVERITY_ERROR
        request.description = 'auto confirm test fault'
        request.source_id = '/test_node'
        return cls._call(cls.report_client, request)

    @classmethod
    def _count_events(cls, fault_code, event_type):
        with cls._events_lock:
            return sum(1 for kind, code in cls._events
                       if code == fault_code and kind == event_type)

    @classmethod
    def _prime_event_stream(cls):
        """Block until an event demonstrably reaches this subscriber."""
        deadline = time.monotonic() + 40.0
        attempt = 0
        while time.monotonic() < deadline:
            code = f'PS_TIMER_PRIME_{attempt}'
            cls._report(code)
            cls._report(code)
            settle = time.monotonic() + 1.0
            while time.monotonic() < settle:
                if cls._count_events(code, FaultEvent.EVENT_CONFIRMED) > 0:
                    return
                time.sleep(0.05)
            attempt += 1
        raise AssertionError('no event reached the subscriber; the events topic never went live')

    def _set_stop(self, active, *, reason='', declared_by=''):
        request = SetPlannedStop.Request()
        request.active = active
        request.reason = reason
        request.declared_by = declared_by
        return self._call(self.set_stop_client, request)

    def _status(self, fault_code):
        request = GetFault.Request()
        request.fault_code = fault_code
        response = self._call(self.get_client, request)
        return response.fault.status if response.success else None

    def _muted_codes(self):
        request = ListFaults.Request()
        request.filter_by_severity = False
        request.severity = 0
        request.statuses = []
        request.include_muted = True
        request.include_clusters = False
        return {info.fault_code for info in self._call(self.list_client, request).muted_faults}

    def _wait_until(self, predicate, message, timeout=30.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            value = predicate()
            if value:
                return value
            time.sleep(0.1)
        raise AssertionError(message)

    def test_a_timer_confirmation_inside_a_stop_is_held_back_until_it_ends(self):
        # The control fault proves the timer really fires in this configuration:
        # without it, silence for the muted code would prove nothing.
        self._report(CONTROL_CODE)
        self._wait_until(lambda: self._status(CONTROL_CODE) == 'CONFIRMED',
                         'the auto-confirm timer never confirmed the control fault, so the '
                         'muted case below would prove nothing')
        self.assertEqual(1, self._count_events(CONTROL_CODE, FaultEvent.EVENT_CONFIRMED))

        self.assertTrue(self._set_stop(True, reason='timer case', declared_by='tech').success)

        self._report(MUTED_CODE)
        self.assertIn(MUTED_CODE, self._muted_codes())

        # The timer confirms it in the store while the stop stands.
        self._wait_until(lambda: self._status(MUTED_CODE) == 'CONFIRMED',
                         'the auto-confirm timer never confirmed the muted fault')
        self.assertEqual(
            0, self._count_events(MUTED_CODE, FaultEvent.EVENT_CONFIRMED),
            'the timer announced a confirmation the planned stop was holding back')

        withdrawn = self._set_stop(False, reason='timer case over', declared_by='tech')
        self.assertTrue(withdrawn.success)

        self._wait_until(
            lambda: self._count_events(MUTED_CODE, FaultEvent.EVENT_CONFIRMED) == 1,
            'the timer-driven confirmation was never announced after the stop ended')
        self.assertNotIn(MUTED_CODE, self._muted_codes())


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        for info in proc_info:
            self.assertIn(info.returncode, (0, -2, -15),
                          f'{info.process_name} exited with code {info.returncode}')
