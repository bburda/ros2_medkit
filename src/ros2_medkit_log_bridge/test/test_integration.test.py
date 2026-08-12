#!/usr/bin/env python3
# Copyright 2026 mfaferek93, bburda
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

"""End-to-end integration tests for ros2_medkit_log_bridge (/rosout -> fault)."""

import time
import unittest

from launch import LaunchDescription
import launch_ros.actions
import launch_testing.actions
import launch_testing.markers
from rcl_interfaces.msg import Log
import rclpy
from rclpy.node import Node
from ros2_medkit_msgs.msg import Fault
from ros2_medkit_msgs.srv import ListFaults

# rcl_interfaces/msg/Log levels are 'byte'-typed constants (Python bytes in
# rclpy), so use plain ints for the uint8 'level' field.
LOG_INFO = 20
LOG_WARN = 30
LOG_ERROR = 40


def generate_test_description():
    """Launch description with fault_manager and log_bridge nodes."""
    fault_manager_node = launch_ros.actions.Node(
        package='ros2_medkit_fault_manager',
        executable='fault_manager_node',
        name='fault_manager',
        output='screen',
        parameters=[{
            'storage_type': 'memory',
            'confirmation_threshold': -1,  # immediate confirmation
            'healing_enabled': True,
            'healing_threshold': 1,
        }],
        # Give the node room to flush coverage data at shutdown before SIGKILL.
        sigterm_timeout='30',
        sigkill_timeout='15',
    )

    log_bridge_node = launch_ros.actions.Node(
        package='ros2_medkit_log_bridge',
        executable='log_bridge_node',
        name='log_bridge',
        output='screen',
        parameters=[{
            'rosout_topic': '/rosout',
            'report_cooldown_sec': 0.0,  # no bridge cooldown in the test
        }],
        # Give the node room to flush coverage data at shutdown before SIGKILL.
        sigterm_timeout='30',
        sigkill_timeout='15',
    )

    # A second bridge carrying two values the node used to swallow: a
    # max_tracked_nodes below its own floor, and a NaN cooldown. NaN is the
    # dangerous one - the old check was "negative -> 0.0", and every comparison
    # against NaN is false, so it passed through into the duration the cooldown
    # window is built from. Only this node's startup log is read.
    bad_params_bridge_node = launch_ros.actions.Node(
        package='ros2_medkit_log_bridge',
        executable='log_bridge_node',
        name='log_bridge_bad_params',
        output='screen',
        parameters=[{
            'max_tracked_nodes': 0,
            'report_cooldown_sec': float('nan'),
            # Keep it out of the fault path; this node exists for its log only.
            'include_only_nodes': ['__none__'],
        }],
        sigterm_timeout='30',
        sigkill_timeout='15',
    )

    return (
        LaunchDescription([
            fault_manager_node,
            log_bridge_node,
            bad_params_bridge_node,
            launch_testing.actions.ReadyToTest(),
        ]),
        {
            'fault_manager_node': fault_manager_node,
            'log_bridge_node': log_bridge_node,
            'bad_params_bridge_node': bad_params_bridge_node,
        },
    )


class TestLogBridgeParameterRefusal(unittest.TestCase):
    """A coerced parameter is reported, and NaN never reaches the window."""

    def test_out_of_range_parameters_are_reported(self, proc_output, bad_params_bridge_node):
        """
        Both parameters name the value that was actually supplied.

        max_tracked_nodes was clamped in silence, so a config asking for 0 ran
        at 1 with nothing to show it. report_cooldown_sec was worse: the check
        read "negative -> 0.0", which is false for NaN, so NaN passed both that
        test and the "<= 0 disables the cooldown" test and reached
        Duration::from_seconds, where converting it is undefined.
        """
        for fragment in (
            'max_tracked_nodes 0 clamped to 1',
            'report_cooldown_sec nan must be finite and >= 0',
        ):
            with self.subTest(fragment=fragment):
                proc_output.assertWaitFor(
                    fragment, process=bad_params_bridge_node, timeout=30,
                )


class TestLogBridgeIntegration(unittest.TestCase):
    """End-to-end /rosout -> fault tests."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = Node('test_log_bridge_client')
        cls.rosout_pub = cls.node.create_publisher(Log, '/rosout', 10)
        cls.list_faults_client = cls.node.create_client(
            ListFaults, '/fault_manager/list_faults'
        )
        assert cls.list_faults_client.wait_for_service(timeout_sec=10.0), \
            'ListFaults service not available'
        # Wait for log_bridge_node to match our /rosout publisher. A fixed sleep
        # is not enough on a slow or loaded machine: if the bridge has not
        # discovered us yet, the first log we publish is dropped before the
        # subscription exists and its fault never appears (the later tests still
        # pass because discovery has completed by then). Poll the matched
        # subscription count instead.
        deadline = time.time() + 15.0
        while cls.rosout_pub.get_subscription_count() < 1 and time.time() < deadline:
            rclpy.spin_once(cls.node, timeout_sec=0.1)
        assert cls.rosout_pub.get_subscription_count() >= 1, \
            'log_bridge did not subscribe to /rosout within the timeout'

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def list_faults(self, statuses=None):
        request = ListFaults.Request()
        request.filter_by_severity = False
        request.statuses = statuses or []
        future = self.list_faults_client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)
        return future.result().faults

    def publish_log(self, node_name, level, message):
        msg = Log()
        msg.stamp = self.node.get_clock().now().to_msg()
        msg.level = level
        msg.name = node_name
        msg.msg = message
        self.rosout_pub.publish(msg)
        time.sleep(0.4)

    def find_fault(self, faults, code_prefix):
        return next((f for f in faults if f.fault_code.startswith(code_prefix)), None)

    def wait_for_fault(self, code_prefix, timeout=12.0):
        """Poll list_faults until a fault with the given code prefix appears."""
        deadline = time.time() + timeout
        fault = None
        while time.time() < deadline:
            fault = self.find_fault(self.list_faults(), code_prefix)
            if fault is not None:
                return fault
            time.sleep(0.5)
        return fault

    def publish_until(self, node_name, level, code_prefix, *, message,
                      timeout=25.0):
        """
        Republish a log until a fault with the given code prefix appears.

        An ERROR log promotes immediately, but the report reaching
        fault_manager needs a second discovery hop: the bridge's ReportFault
        client must have discovered fault_manager, else the fire-and-forget
        report is dropped. That client readiness is not observable from here
        and the report is one-shot, so re-emit the log and poll rather than
        publishing once and racing the hop (which flaked on a loaded machine).
        """
        deadline = time.time() + timeout
        fault = None
        while time.time() < deadline:
            self.publish_log(node_name, level, message)
            fault = self.find_fault(self.list_faults(), code_prefix)
            if fault is not None:
                return fault
        return fault

    def test_01_error_log_creates_attributed_fault(self):
        """An ERROR log becomes a CONFIRMED fault attributed to the node FQN."""
        fault = self.publish_until(
            'planner_server', LOG_ERROR, 'LOG_PLANNER_SERVER_',
            message='failed to compute path')
        self.assertIsNotNone(fault, 'expected a LOG_PLANNER_SERVER_* fault')
        self.assertEqual(fault.severity, Fault.SEVERITY_ERROR)
        # The fault must associate with the node FQN, not the raw logger name.
        self.assertIn('/planner_server', list(fault.reporting_sources))

    def test_02_warn_log_needs_threshold(self):
        """A single WARN is held by the LocalFilter; repeats promote it."""
        self.publish_log('controller', LOG_WARN, 'goal tolerance exceeded')
        time.sleep(1.0)
        self.assertIsNone(
            self.find_fault(self.list_faults(), 'LOG_CONTROLLER_'),
            'a single WARN should be held by the LocalFilter')

        # default_threshold is 3 within default_window_sec; send enough repeats.
        for _ in range(3):
            self.publish_log('controller', LOG_WARN, 'goal tolerance exceeded')

        fault = self.wait_for_fault('LOG_CONTROLLER_')
        self.assertIsNotNone(fault, 'repeated WARN should promote to a fault')
        self.assertEqual(fault.severity, Fault.SEVERITY_WARN)

    def test_03_info_log_dropped(self):
        """INFO is below the floor and never becomes a fault."""
        self.publish_log('chatter', LOG_INFO, 'all good')
        time.sleep(1.5)  # settle; a promoted fault would have appeared by now
        self.assertIsNone(self.find_fault(self.list_faults(), 'LOG_CHATTER_'))


@launch_testing.post_shutdown_test()
class TestLogBridgeShutdown(unittest.TestCase):
    """Test clean shutdown."""

    def test_exit_code(self, proc_info):
        # OK, SIGINT, SIGTERM: the launch harness stops the long-running node by
        # signal, so SIGTERM is a clean shutdown (matches ALLOWED_EXIT_CODES used
        # across the other integration tests). The bare default ([0]) made this
        # flaky whenever the harness escalated SIGINT to SIGTERM.
        launch_testing.asserts.assertExitCodes(proc_info, allowable_exit_codes=[0, -2, -15])
