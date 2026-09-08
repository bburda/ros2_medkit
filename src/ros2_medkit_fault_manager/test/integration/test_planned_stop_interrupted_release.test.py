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
A switch-off killed halfway is finished by the next startup.

Withdrawing a planned stop writes the declaration, announces the confirmations it
was holding back, and only then drops the ownership flags. A process that dies
between the write and the flags leaves faults owned by a declaration that is
already over - and their confirmations, suppressed while the stop stood, have
been announced by nobody.

The kill is real (SIGTERM, launch respawns the node); the crash *point* is
simulated by editing the store while the manager is down, which is exactly the
state such a crash leaves behind.
"""

import os
import signal
import sqlite3
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
from ros2_medkit_msgs.srv import GetPlannedStop, ListFaults, ReportFault, SetPlannedStop

STORAGE_DIR = tempfile.mkdtemp(prefix='planned_stop_interrupted_')
DATABASE_PATH = os.path.join(STORAGE_DIR, 'faults.db')

OWNED_CODES = ('PS_INTERRUPTED_ONE', 'PS_INTERRUPTED_TWO')

# Long enough that the test can edit the store between the kill and the
# replacement opening it.
RESPAWN_DELAY_SEC = 6.0


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
    """Launch a fault manager launch brings back after it is killed."""
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
            'confirmation_threshold': -1,
            'snapshots.enabled': False,
            'snapshots.rosbag.enabled': False,
        }],
        respawn=True,
        respawn_delay=RESPAWN_DELAY_SEC,
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


class TestInterruptedRelease(unittest.TestCase):
    """A release the previous process did not finish is finished at startup."""

    @classmethod
    def setUpClass(cls):
        os.environ['ROS_LOCALHOST_ONLY'] = '1'
        rclpy.init()
        cls.node = Node('test_planned_stop_interrupted_client')

        cls._events = []
        cls._events_lock = threading.Lock()
        cls._event_sub = cls.node.create_subscription(
            FaultEvent, '/fault_manager/events', cls._on_event,
            QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                       history=HistoryPolicy.KEEP_LAST, depth=200))
        cls._connect()

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    @classmethod
    def _on_event(cls, msg):
        with cls._events_lock:
            cls._events.append((msg.event_type, msg.fault.fault_code))

    @classmethod
    def _connect(cls, timeout_sec=40.0):
        cls.report_client = cls.node.create_client(ReportFault, '/fault_manager/report_fault')
        cls.list_client = cls.node.create_client(ListFaults, '/fault_manager/list_faults')
        cls.set_stop_client = cls.node.create_client(
            SetPlannedStop, '/fault_manager/set_planned_stop')
        cls.get_stop_client = cls.node.create_client(
            GetPlannedStop, '/fault_manager/get_planned_stop')
        for client, name in (
            (cls.report_client, 'report_fault'),
            (cls.list_client, 'list_faults'),
            (cls.set_stop_client, 'set_planned_stop'),
            (cls.get_stop_client, 'get_planned_stop'),
        ):
            assert client.wait_for_service(timeout_sec=timeout_sec), \
                f'{name} service not available'

    def _call(self, client, request, timeout_sec=15.0):
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout_sec)
        self.assertIsNotNone(future.result(), 'service call timed out')
        return future.result()

    def _report(self, fault_code):
        request = ReportFault.Request()
        request.fault_code = fault_code
        request.event_type = ReportFault.Request.EVENT_FAILED
        request.severity = Fault.SEVERITY_ERROR
        request.description = 'interrupted release test fault'
        request.source_id = '/test_node'
        return self._call(self.report_client, request)

    def _set_stop(self, active, *, reason='', declared_by=''):
        request = SetPlannedStop.Request()
        request.active = active
        request.reason = reason
        request.declared_by = declared_by
        return self._call(self.set_stop_client, request)

    def _muted_codes(self):
        request = ListFaults.Request()
        request.filter_by_severity = False
        request.severity = 0
        request.statuses = []
        request.include_muted = True
        request.include_clusters = False
        return {info.fault_code for info in self._call(self.list_client, request).muted_faults}

    def _confirmed_codes(self):
        request = ListFaults.Request()
        request.filter_by_severity = False
        request.severity = 0
        request.statuses = []
        request.include_muted = False
        request.include_clusters = False
        return {fault.fault_code for fault in self._call(self.list_client, request).faults}

    def _count_events(self, fault_code, event_type):
        with self._events_lock:
            return sum(1 for kind, code in self._events
                       if code == fault_code and kind == event_type)

    def _wait_until(self, predicate, message, timeout=60.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if predicate():
                return
            rclpy.spin_once(self.node, timeout_sec=0.05)
        raise AssertionError(message)

    def _spin_for(self, seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    @staticmethod
    def _process_is_running(pid):
        """
        Whether `pid` is still a LIVE process - a zombie counts as gone.

        `launch`'s own `process_details` is no help here: it keeps naming the dead
        process until the respawn replaces it, so waiting on it waits for the
        REPLACEMENT rather than for the death, and anything done "while the node is
        down" would land after the new one had already read the store.
        """
        try:
            with open(f'/proc/{pid}/stat', 'rb') as handle:
                raw = handle.read()
        except (FileNotFoundError, ProcessLookupError):
            return False
        except OSError:
            return True
        state = raw[raw.rfind(b')') + 2:raw.rfind(b')') + 3]
        return state not in (b'Z', b'X')

    @staticmethod
    def _withdraw_declaration_in_the_store():
        """
        Leave the store exactly as a crash mid-release would.

        The declaration is withdrawn - that write happens first - while the
        ownership flags, which are cleared last, are still set.
        """
        connection = sqlite3.connect(DATABASE_PATH)
        try:
            connection.execute(
                'UPDATE planned_stop SET active = 0, ended_at_ns = ? WHERE id = 1',
                (int(time.time() * 1e9),))
            connection.commit()
            owned = connection.execute(
                'SELECT COUNT(*) FROM faults WHERE planned_stop_owned != 0').fetchone()[0]
        finally:
            connection.close()
        return owned

    def test_startup_finishes_a_release_the_previous_process_did_not(
            self, fault_manager_node):
        self.assertTrue(self._set_stop(True, reason='cell 4 rebuild',
                                       declared_by='plant_manager').success)
        for code in OWNED_CODES:
            self._report(code)
        self._wait_until(lambda: set(OWNED_CODES) <= self._muted_codes(),
                         'the faults were never marked by the stop')
        for code in OWNED_CODES:
            self.assertEqual(0, self._count_events(code, FaultEvent.EVENT_CONFIRMED))

        original_pid = fault_manager_node.process_details['pid']
        os.kill(original_pid, signal.SIGTERM)

        deadline = time.monotonic() + 30.0
        while time.monotonic() < deadline and self._process_is_running(original_pid):
            time.sleep(0.1)
        self.assertFalse(self._process_is_running(original_pid),
                         'the fault manager was still running, so the store edit below would '
                         'not be the state a crash leaves')

        still_owned = self._withdraw_declaration_in_the_store()
        self.assertEqual(len(OWNED_CODES), still_owned,
                         'the store did not hold the ownership flags a crash would leave')

        self._wait_until(
            lambda: fault_manager_node.process_details is not None
            and fault_manager_node.process_details['pid'] != original_pid,
            'the fault manager was never restarted')
        self._connect()

        # Startup sees flags behind a withdrawn declaration and finishes the job.
        for code in OWNED_CODES:
            self._wait_until(
                lambda code=code: self._count_events(code, FaultEvent.EVENT_CONFIRMED) == 1,
                f'{code} was never announced after the interrupted release')

        self._spin_for(3.0)
        for code in OWNED_CODES:
            self.assertEqual(1, self._count_events(code, FaultEvent.EVENT_CONFIRMED),
                             f'{code} was announced more than once')
            self.assertIn(code, self._confirmed_codes())

        self.assertEqual(set(), set(OWNED_CODES) & self._muted_codes(),
                         'the faults are still marked after the release was finished')
        self.assertFalse(self._call(self.get_stop_client, GetPlannedStop.Request()).active)

        # And the flags are gone, so a second restart announces nothing again.
        connection = sqlite3.connect(f'file:{DATABASE_PATH}?mode=ro', uri=True)
        try:
            remaining = connection.execute(
                'SELECT COUNT(*) FROM faults WHERE planned_stop_owned != 0').fetchone()[0]
        finally:
            connection.close()
        self.assertEqual(0, remaining)


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        for info in proc_info:
            self.assertIn(info.returncode, (0, -2, -15),
                          f'{info.process_name} exited with code {info.returncode}')
