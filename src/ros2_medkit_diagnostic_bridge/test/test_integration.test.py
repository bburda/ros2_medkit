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

"""Integration tests for ros2_medkit_diagnostic_bridge."""

import time
import unittest

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from launch import LaunchDescription
import launch_ros.actions
import launch_testing.actions
import launch_testing.markers
import rclpy
from rclpy.node import Node
from ros2_medkit_msgs.msg import Fault
from ros2_medkit_msgs.srv import ListFaults


def generate_test_description():
    """Generate launch description with fault_manager and diagnostic_bridge nodes."""
    fault_manager_node = launch_ros.actions.Node(
        package='ros2_medkit_fault_manager',
        executable='fault_manager_node',
        name='fault_manager',
        output='screen',
        parameters=[{
            'storage_type': 'memory',
            'confirmation_threshold': -1,  # Immediate confirmation
            'healing_enabled': True,
            'healing_threshold': 1,
        }],
        # Give the node room to flush coverage data at shutdown before SIGKILL.
        sigterm_timeout='30',
        sigkill_timeout='15',
    )

    diagnostic_bridge_node = launch_ros.actions.Node(
        package='ros2_medkit_diagnostic_bridge',
        executable='diagnostic_bridge_node',
        name='diagnostic_bridge',
        output='screen',
        parameters=[{
            'diagnostics_topic': '/diagnostics',
            'auto_generate_codes': True,
            'use_hardware_id_as_source_id': True,
        }],
        # Give the node room to flush coverage data at shutdown before SIGKILL.
        sigterm_timeout='30',
        sigkill_timeout='15',
    )

    return (
        LaunchDescription([
            fault_manager_node,
            diagnostic_bridge_node,
            launch_testing.actions.ReadyToTest(),
        ]),
        {
            'fault_manager_node': fault_manager_node,
            'diagnostic_bridge_node': diagnostic_bridge_node,
        },
    )


class TestDiagnosticBridgeIntegration(unittest.TestCase):
    """Integration tests for DiagnosticBridge."""

    @classmethod
    def setUpClass(cls):
        """Initialize ROS 2 context."""
        rclpy.init()
        cls.node = Node('test_diagnostic_bridge_client')

        # Create publisher for diagnostics
        cls.diag_pub = cls.node.create_publisher(DiagnosticArray, '/diagnostics', 10)

        # Create client for ListFaults service
        cls.list_faults_client = cls.node.create_client(
            ListFaults, '/fault_manager/list_faults'
        )

        # Wait for services
        assert cls.list_faults_client.wait_for_service(timeout_sec=10.0), \
            'ListFaults service not available'

        # Wait for the bridge's /diagnostics subscription to match this
        # publisher before any test publishes. The bridge subscribes with
        # volatile QoS, so a sample published before the match is silently
        # dropped and never becomes a fault - a fixed sleep here raced that
        # match under load.
        assert cls._wait_for_bridge_subscription(timeout_sec=15.0), \
            'diagnostic_bridge did not subscribe to /diagnostics'

    @classmethod
    def _wait_for_bridge_subscription(cls, *, timeout_sec=15.0):
        """
        Wait until the bridge's /diagnostics subscription matches our publisher.

        Polls the publisher's matched-subscription count (spinning so DDS
        discovery can progress) instead of sleeping a fixed interval, so
        published diagnostics are guaranteed to be delivered rather than
        dropped by the volatile QoS during the discovery window.
        """
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            if cls.diag_pub.get_subscription_count() >= 1:
                return True
            rclpy.spin_once(cls.node, timeout_sec=0.1)
        return False

    @classmethod
    def tearDownClass(cls):
        """Shutdown ROS 2 context."""
        cls.node.destroy_node()
        rclpy.shutdown()

    def list_faults(self, statuses=None):
        """Get faults from FaultManager (empty list if the call times out)."""
        request = ListFaults.Request()
        request.filter_by_severity = False
        request.statuses = statuses or []

        future = self.list_faults_client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)
        result = future.result()
        return result.faults if result is not None else []

    def publish_diagnostic(self, name, level, message='Test message', hardware_id='test_hw'):
        """Publish a single diagnostic message."""
        msg = DiagnosticArray()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.status = [DiagnosticStatus(
            level=level,
            name=name,
            message=message,
            hardware_id=hardware_id,
        )]
        self.diag_pub.publish(msg)

        # Give time for message to be processed
        time.sleep(0.3)

    def publish_until(self, name, level, expected_code, *, predicate=None,
                      message='Test message', hardware_id='test_hw', statuses=None, timeout=25.0):
        """
        Republish a diagnostic until a matching fault satisfies *predicate*.

        Two discovery hops gate the first fault: the bridge's /diagnostics
        subscription must match this publisher (handled in setUpClass) and the
        bridge's ReportFault client must have discovered fault_manager, else
        ``send_report`` drops the fire-and-forget request. That client readiness
        is not observable from here and the report is one-shot, so re-emit the
        stimulus and poll rather than publishing once and racing the hop.
        Re-emission also drives WARN/PASSED past their occurrence threshold
        within the filter window deterministically.

        Returns the matching fault once *predicate* holds; fails at the
        deadline.
        """
        if predicate is None:
            def predicate(_fault):
                return True
        deadline = time.monotonic() + timeout
        last = None
        while time.monotonic() < deadline:
            self.publish_diagnostic(name, level, message, hardware_id)
            fault = next(
                (f for f in self.list_faults(statuses=statuses)
                 if f.fault_code == expected_code),
                None,
            )
            if fault is not None:
                last = fault
                if predicate(fault):
                    return fault
        self.fail(
            f'Fault {expected_code} not satisfied within {timeout}s '
            f'(last seen: {last})'
        )

    def test_01_error_diagnostic_creates_fault(self):
        """Test that ERROR diagnostic creates a fault in FaultManager."""
        fault = self.publish_until(
            'test_sensor', DiagnosticStatus.ERROR, 'TEST_SENSOR',
            message='Sensor failure',
        )
        self.assertEqual(fault.severity, Fault.SEVERITY_ERROR)

    def test_02_warn_diagnostic_creates_fault(self):
        """
        Test that WARN diagnostic creates a fault.

        WARN does not bypass the reporter's local filter (threshold=3), so
        publish_until re-emits until the occurrence threshold is met and the
        report reaches fault_manager.
        """
        fault = self.publish_until(
            'warning_component', DiagnosticStatus.WARN, 'WARNING_COMPONENT',
            message='Low battery',
        )
        self.assertEqual(fault.severity, Fault.SEVERITY_WARN)

    def test_03_stale_diagnostic_creates_critical_fault(self):
        """Test that STALE diagnostic creates a CRITICAL fault."""
        fault = self.publish_until(
            'stale_sensor', DiagnosticStatus.STALE, 'STALE_SENSOR',
            message='No data',
        )
        self.assertEqual(fault.severity, Fault.SEVERITY_CRITICAL)

    def test_04_ok_diagnostic_heals_fault(self):
        """Test that OK diagnostic sends PASSED and heals fault."""
        # First create a fault.
        self.publish_until(
            'healing_test', DiagnosticStatus.ERROR, 'HEALING_TEST',
            message='Error',
        )

        # Send OK until the fault heals. PASSED events share the reporter's
        # local threshold (3) and the healing service call is async, so
        # re-emit and poll until the fault reaches HEALED instead of publishing
        # a fixed number of times and racing propagation.
        fault = self.publish_until(
            'healing_test', DiagnosticStatus.OK, 'HEALING_TEST',
            predicate=lambda f: f.status == Fault.STATUS_HEALED,
            statuses=[Fault.STATUS_CONFIRMED, Fault.STATUS_HEALED],
        )
        self.assertEqual(fault.status, Fault.STATUS_HEALED)

    def test_05_fault_code_generation(self):
        """Test auto-generation of fault codes from diagnostic names."""
        test_cases = [
            ('motor temp', 'MOTOR_TEMP'),
            ('sensor: status', 'SENSOR_STATUS'),
            ('/robot/arm', 'ROBOT_ARM'),
        ]

        for diag_name, expected_code in test_cases:
            self.publish_until(
                diag_name, DiagnosticStatus.ERROR, expected_code,
            )

    def test_06_hardware_id_is_forwarded_as_source(self):
        """Test that hardware_id is forwarded into fault reporting_sources."""
        self.publish_until(
            'shared_sensor', DiagnosticStatus.STALE, 'SHARED_SENSOR',
            message='No data from source A',
            hardware_id='/my_lidar_driver',
            predicate=lambda f: '/my_lidar_driver' in f.reporting_sources,
        )

        fault = self.publish_until(
            'shared_sensor', DiagnosticStatus.STALE, 'SHARED_SENSOR',
            message='No data from source B',
            hardware_id='/my_camera_driver',
            predicate=lambda f: '/my_lidar_driver' in f.reporting_sources
            and '/my_camera_driver' in f.reporting_sources,
        )

        self.assertEqual(fault.severity, Fault.SEVERITY_CRITICAL)
        self.assertIn('/my_lidar_driver', fault.reporting_sources)
        self.assertIn('/my_camera_driver', fault.reporting_sources)

    def test_07_empty_hardware_id_falls_back_to_bridge_source(self):
        """Test that empty hardware_id falls back to the bridge node FQN."""
        fault = self.publish_until(
            'fallback_sensor', DiagnosticStatus.STALE, 'FALLBACK_SENSOR',
            message='No data with empty hardware id',
            hardware_id='',
            predicate=lambda f: '/diagnostic_bridge' in f.reporting_sources,
        )

        self.assertIn('/diagnostic_bridge', fault.reporting_sources)


@launch_testing.post_shutdown_test()
class TestDiagnosticBridgeShutdown(unittest.TestCase):
    """Test clean shutdown."""

    def test_exit_code(self, proc_info):
        """Verify nodes exit cleanly."""
        launch_testing.asserts.assertExitCodes(proc_info)
