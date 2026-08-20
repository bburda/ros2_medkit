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

"""Integration tests for per-entity debounce thresholds."""

import os
import unittest

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
import launch_ros.actions
import launch_testing.actions
import launch_testing.markers
import rclpy
from rclpy.node import Node
from ros2_medkit_msgs.msg import Fault
from ros2_medkit_msgs.srv import GetFault, ListFaults, ReportFault


def generate_test_description():
    """Launch fault_manager with per-entity thresholds config."""
    pkg_share = get_package_share_directory('ros2_medkit_fault_manager')
    thresholds_config = os.path.join(
        pkg_share, 'test', 'test_entity_thresholds.yaml'
    )

    fault_manager_node = launch_ros.actions.Node(
        package='ros2_medkit_fault_manager',
        executable='fault_manager_node',
        name='fault_manager',
        output='screen',
        parameters=[{
            'storage_type': 'memory',
            'confirmation_threshold': -5,  # Global: need 5 events
            'healing_enabled': False,       # Global: no healing
            'healing_threshold': 10,
            'entity_thresholds.config_file': thresholds_config,
        }],
        # Give the node room to flush coverage data at shutdown before SIGKILL.
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


class TestPerEntityThresholds(unittest.TestCase):
    """Integration tests for per-entity debounce thresholds."""

    @classmethod
    def setUpClass(cls):
        """Initialize ROS 2 context and service clients."""
        rclpy.init()
        cls.node = Node('test_entity_thresholds_client')

        cls.report_client = cls.node.create_client(
            ReportFault, '/fault_manager/report_fault'
        )
        cls.list_client = cls.node.create_client(
            ListFaults, '/fault_manager/list_faults'
        )
        cls.get_client = cls.node.create_client(
            GetFault, '/fault_manager/get_fault'
        )

        assert cls.report_client.wait_for_service(timeout_sec=10.0), \
            'report_fault service not available'
        assert cls.list_client.wait_for_service(timeout_sec=10.0), \
            'list_faults service not available'
        assert cls.get_client.wait_for_service(timeout_sec=10.0), \
            'get_fault service not available'

    @classmethod
    def tearDownClass(cls):
        """Shutdown ROS 2."""
        cls.node.destroy_node()
        rclpy.shutdown()

    def _call(self, client, request):
        """Call service synchronously."""
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)
        self.assertIsNotNone(future.result(), 'Service call timed out')
        return future.result()

    def _report(self, fault_code, source_id, severity=Fault.SEVERITY_ERROR):
        """Report a FAILED event."""
        req = ReportFault.Request()
        req.fault_code = fault_code
        req.event_type = ReportFault.Request.EVENT_FAILED
        req.severity = severity
        req.description = f'Test fault from {source_id}'
        req.source_id = source_id
        resp = self._call(self.report_client, req)
        self.assertTrue(resp.accepted)

    def _report_passed(self, fault_code, source_id):
        """Report a PASSED event."""
        req = ReportFault.Request()
        req.fault_code = fault_code
        req.event_type = ReportFault.Request.EVENT_PASSED
        req.severity = 0
        req.description = ''
        req.source_id = source_id
        resp = self._call(self.report_client, req)
        self.assertTrue(resp.accepted)

    def _get_fault(self, fault_code):
        """Get a single fault."""
        req = GetFault.Request()
        req.fault_code = fault_code
        resp = self._call(self.get_client, req)
        self.assertTrue(resp.success, resp.error_message)
        return resp.fault

    def test_01_lidar_confirms_immediately(self):
        """Lidar has confirmation_threshold=-1, single event should confirm."""
        self._report('LIDAR.BLOCKED', '/sensors/lidar/front')

        fault = self._get_fault('LIDAR.BLOCKED')
        self.assertEqual(fault.status, Fault.STATUS_CONFIRMED)
        print(f'Lidar fault confirmed immediately: {fault.status}')

    def test_02_motor_stays_prefailed(self):
        """Motor has confirmation_threshold=-3, single event should be PREFAILED."""
        self._report('MOTOR.OVERHEAT', '/powertrain/motor/left')

        fault = self._get_fault('MOTOR.OVERHEAT')
        self.assertEqual(fault.status, Fault.STATUS_PREFAILED)
        print(f'Motor fault prefailed after 1 event: {fault.status}')

    def test_03_motor_confirms_at_threshold(self):
        """Motor confirms after 3 FAILED events (threshold=-3)."""
        # Event 1 already reported in test_02, counter=-1
        # Events 2 and 3
        self._report('MOTOR.OVERHEAT', '/powertrain/motor/left')
        self._report('MOTOR.OVERHEAT', '/powertrain/motor/left')

        fault = self._get_fault('MOTOR.OVERHEAT')
        self.assertEqual(fault.status, Fault.STATUS_CONFIRMED)
        print(f'Motor fault confirmed after 3 events: {fault.status}')

    def test_04_unknown_entity_uses_global(self):
        """Unknown entity uses global threshold=-5, stays PREFAILED after 1 event."""
        self._report('UNKNOWN.FAULT', '/some/unknown/subsystem')

        fault = self._get_fault('UNKNOWN.FAULT')
        self.assertEqual(fault.status, Fault.STATUS_PREFAILED)
        print(f'Unknown entity fault prefailed: {fault.status}')

    def test_05_unknown_entity_still_prefailed_at_3(self):
        """Global threshold=-5, still PREFAILED after 3 events (not at -5 yet)."""
        # Event 1 already in test_04
        self._report('UNKNOWN.FAULT', '/some/unknown/subsystem')
        self._report('UNKNOWN.FAULT', '/some/unknown/subsystem')

        fault = self._get_fault('UNKNOWN.FAULT')
        self.assertEqual(fault.status, Fault.STATUS_PREFAILED)
        print(f'Unknown entity still prefailed at 3 events: {fault.status}')

    def test_06_unknown_entity_confirms_at_5(self):
        """Global threshold=-5, confirms at 5 events."""
        # Events 1-3 already reported, need 2 more
        self._report('UNKNOWN.FAULT', '/some/unknown/subsystem')
        self._report('UNKNOWN.FAULT', '/some/unknown/subsystem')

        fault = self._get_fault('UNKNOWN.FAULT')
        self.assertEqual(fault.status, Fault.STATUS_CONFIRMED)
        print(f'Unknown entity confirmed at 5 events: {fault.status}')


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):
    """Verify fault_manager exits cleanly."""

    def test_exit_code(self, proc_info):
        """Check process exit code."""
        launch_testing.asserts.assertExitCodes(proc_info)
