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
A planned stop declared before a restart is still in force after it.

The subject IS the restart, so the fault manager runs with ``respawn`` and the
test kills it by PID: a weekend stop must not end because the box rebooted.
Runs on SQLite, the shipped default and the only backend with a file behind it.
"""

import os
import signal
import tempfile
import time
import unittest

from launch import LaunchDescription
import launch.actions
import launch_ros.actions
import launch_testing.actions
import rclpy
from rclpy.node import Node
from ros2_medkit_msgs.msg import Fault
from ros2_medkit_msgs.srv import GetFault, GetPlannedStop, ListFaults, ReportFault, SetPlannedStop

STORAGE_DIR = tempfile.mkdtemp(prefix='planned_stop_restart_')
DATABASE_PATH = os.path.join(STORAGE_DIR, 'faults.db')

REASON = 'weekend shutdown, cell 4'
DECLARED_BY = 'plant_manager'
BEFORE_CODE = 'PS_RESTART_BEFORE'
AFTER_CODE = 'PS_RESTART_AFTER'


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
    """Launch a fault manager that launch brings back after it is killed."""
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
        # The restart is the subject. The delay gives the DDS participant time to
        # go before its replacement appears, so the test sees a real gap rather
        # than two managers on one domain.
        respawn=True,
        respawn_delay=1.0,
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


class TestPlannedStopSurvivesRestart(unittest.TestCase):
    """The declaration outlives the process that made it."""

    @classmethod
    def setUpClass(cls):
        os.environ['ROS_LOCALHOST_ONLY'] = '1'
        rclpy.init()
        cls.node = Node('test_planned_stop_restart_client')
        cls._connect()

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    @classmethod
    def _connect(cls, timeout_sec=30.0):
        """Create clients and wait for the manager's services to answer."""
        cls.report_client = cls.node.create_client(ReportFault, '/fault_manager/report_fault')
        cls.list_client = cls.node.create_client(ListFaults, '/fault_manager/list_faults')
        cls.get_client = cls.node.create_client(GetFault, '/fault_manager/get_fault')
        cls.set_stop_client = cls.node.create_client(
            SetPlannedStop, '/fault_manager/set_planned_stop')
        cls.get_stop_client = cls.node.create_client(
            GetPlannedStop, '/fault_manager/get_planned_stop')
        for client, name in (
            (cls.report_client, 'report_fault'),
            (cls.list_client, 'list_faults'),
            (cls.get_client, 'get_fault'),
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
        request.description = 'restart test fault'
        request.source_id = '/test_node'
        return self._call(self.report_client, request)

    def _set_stop(self, active, *, reason='', declared_by=''):
        request = SetPlannedStop.Request()
        request.active = active
        request.reason = reason
        request.declared_by = declared_by
        return self._call(self.set_stop_client, request)

    def _get_stop(self):
        return self._call(self.get_stop_client, GetPlannedStop.Request())

    def _muted_codes(self):
        request = ListFaults.Request()
        request.filter_by_severity = False
        request.severity = 0
        request.statuses = []
        request.include_muted = True
        request.include_clusters = False
        response = self._call(self.list_client, request)
        return {info.fault_code: info for info in response.muted_faults}

    def _confirmed_codes(self):
        request = ListFaults.Request()
        request.filter_by_severity = False
        request.severity = 0
        request.statuses = []
        request.include_muted = False
        request.include_clusters = False
        return {fault.fault_code for fault in self._call(self.list_client, request).faults}

    def test_a_stop_declared_before_a_restart_still_mutes_after_it(self, fault_manager_node):
        declared = self._set_stop(True, reason=REASON, declared_by=DECLARED_BY)
        self.assertTrue(declared.success)
        self.assertFalse(declared.was_active)

        before = self._get_stop()
        self.assertTrue(before.active)
        self.assertEqual(REASON, before.reason)

        self._report(BEFORE_CODE)
        self.assertIn(BEFORE_CODE, self._muted_codes())

        original_pid = fault_manager_node.process_details['pid']
        os.kill(original_pid, signal.SIGTERM)

        deadline = time.monotonic() + 60.0
        while time.monotonic() < deadline:
            current = fault_manager_node.process_details
            if current is not None and current['pid'] != original_pid:
                break
            time.sleep(0.2)
        self.assertNotEqual(
            original_pid, fault_manager_node.process_details['pid'],
            'the fault manager was never restarted, so nothing below tests a restart')

        self._connect(timeout_sec=60.0)

        after = self._get_stop()
        self.assertTrue(after.active, 'the planned stop did not survive the restart')
        self.assertEqual(REASON, after.reason)
        self.assertEqual(DECLARED_BY, after.declared_by)
        self.assertEqual(before.since.sec, after.since.sec,
                         'the restart moved the time the stop started')
        self.assertEqual(before.since.nanosec, after.since.nanosec)

        # The declaration is not a museum piece: a fault reported after the
        # restart is muted by it.
        self._report(AFTER_CODE)
        muted = self._muted_codes()
        self.assertIn(AFTER_CODE, muted,
                      'a fault reported after the restart was not muted by the standing stop')
        self.assertEqual('planned_stop', muted[AFTER_CODE].rule_id)
        self.assertNotIn(AFTER_CODE, self._confirmed_codes())

        withdrawn = self._set_stop(False, reason='plant back up', declared_by=DECLARED_BY)
        self.assertTrue(withdrawn.success)
        self.assertTrue(withdrawn.was_active)
        self.assertFalse(self._get_stop().active)
        self.assertIn(AFTER_CODE, self._confirmed_codes())


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        for info in proc_info:
            self.assertIn(info.returncode, (0, -2, -15),
                          f'{info.process_name} exited with code {info.returncode}')
