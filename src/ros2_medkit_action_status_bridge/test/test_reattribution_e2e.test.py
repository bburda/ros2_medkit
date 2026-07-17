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
End-to-end regression for provisional-source re-attribution (issue #467).

This drives the REAL action_status_bridge (not a hand-rolled reporter) against a
real FaultManager. The only substitution is a test-double executable that forces
the inherently racy DDS-FQN resolution to return "" for the first few rescans,
reproducing the "action aborts before discovery resolves the server FQN" race
deterministically. Everything else - reporter_for's provisional bookkeeping,
reattribute_provisional(), report() with supersedes_source_id, the rescan timer -
is the unmodified production bridge.

Asserted end to end:
  1. the fault first surfaces under the provisional action-name source
     (/test_action), because the FQN was unresolved at first report; then
  2. once the FQN resolves, the bridge re-reports under it with the provisional
     source superseded, so reporting_sources ends as {FQN} with the provisional
     source dropped.

Step 2 is the exact behavior that was impossible before #467 (append-only
reporting_sources + strict-AND per-entity scope filter left the fault stuck
under the provisional source forever).
"""

import time
import unittest

from action_msgs.msg import GoalStatus, GoalStatusArray
from launch import LaunchDescription
import launch_ros.actions
import launch_testing.actions
import launch_testing.markers
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from ros2_medkit_msgs.srv import ListFaults
from unique_identifier_msgs.msg import UUID

STATUS_TOPIC = '/test_action/_action/status'
ABORTED_CODE = 'ACTION_TEST_ACTION_ABORTED'
# The action-interface name the bridge falls back to before discovery resolves.
PROVISIONAL_SOURCE = '/test_action'
# This test node publishes the status topic, so it IS the action server: its node
# FQN is what the fault must be re-attributed to.
SERVER_FQN = '/test_action_status_client'


def generate_test_description():
    """fault_manager + the slow-discovery test-double bridge."""
    fault_manager_node = launch_ros.actions.Node(
        package='ros2_medkit_fault_manager',
        executable='fault_manager_node',
        name='fault_manager',
        output='screen',
        parameters=[{
            'storage_type': 'memory',
            'confirmation_threshold': -1,  # confirm on the first FAILED
            'healing_enabled': True,
            'healing_threshold': 0,
        }],
        # Give the node room to flush coverage data at shutdown before SIGKILL.
        sigterm_timeout='30',
        sigkill_timeout='15',
    )
    # Test-double: real bridge, deterministic slow-discovery FQN resolution.
    slow_discovery_bridge = launch_ros.actions.Node(
        package='ros2_medkit_action_status_bridge',
        executable='slow_discovery_bridge',
        name='action_status_bridge',
        output='screen',
        parameters=[{
            'rescan_period_sec': 1.0,  # fast rescans so re-attribution is prompt
        }],
        # Give the node room to flush coverage data at shutdown before SIGKILL.
        sigterm_timeout='30',
        sigkill_timeout='15',
    )
    return (
        LaunchDescription([
            fault_manager_node,
            slow_discovery_bridge,
            launch_testing.actions.ReadyToTest(),
        ]),
        {
            'fault_manager_node': fault_manager_node,
            'slow_discovery_bridge': slow_discovery_bridge,
        },
    )


class TestReattributionEndToEnd(unittest.TestCase):
    """A fault reported under a provisional source is re-attributed to the FQN."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = Node('test_action_status_client')
        # Match the bridge's subscription QoS so the latched status is delivered.
        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )
        cls.status_pub = cls.node.create_publisher(GoalStatusArray, STATUS_TOPIC, qos)
        cls.list_faults_client = cls.node.create_client(
            ListFaults, '/fault_manager/list_faults'
        )
        assert cls.list_faults_client.wait_for_service(timeout_sec=15.0), \
            'ListFaults service not available'

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def _reporting_sources(self, code):
        """reporting_sources for a fault_code, or None if the fault is absent."""
        request = ListFaults.Request()
        request.filter_by_severity = False
        request.statuses = []
        future = self.list_faults_client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)
        result = future.result()
        if result is None:
            return None
        for fault in result.faults:
            if fault.fault_code == code:
                return list(fault.reporting_sources)
        return None

    def _wait_for_bridge_watching(self, timeout=15.0):
        deadline = time.time() + timeout
        while time.time() < deadline:
            infos = self.node.get_subscriptions_info_by_topic(STATUS_TOPIC)
            if any(info.node_name == 'action_status_bridge' for info in infos):
                return True
            rclpy.spin_once(self.node, timeout_sec=0.1)
        return False

    def _spin(self, seconds):
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def test_provisional_fault_is_reattributed_to_server_fqn(self):
        """Provisional (action-name) source -> superseded -> server FQN."""
        self.assertTrue(
            self._wait_for_bridge_watching(),
            'bridge did not subscribe to the status topic in time',
        )
        # Abort a goal. The test-double resolves the FQN as "" for the first few
        # rescans, so this first report lands under the provisional source.
        self.status_pub.publish(_status_array(GoalStatus.STATUS_ABORTED, 1))

        # 1. The fault must appear under the provisional action-name source. Poll
        #    for a bounded window and record that we observed it there - this is
        #    the state that used to be permanent before #467.
        saw_provisional = False
        deadline = time.time() + 20.0
        while time.time() < deadline:
            sources = self._reporting_sources(ABORTED_CODE)
            if sources is not None and PROVISIONAL_SOURCE in sources and SERVER_FQN not in sources:
                saw_provisional = True
                break
            self._spin(0.2)
        self.assertTrue(
            saw_provisional,
            'fault never surfaced under the provisional action-name source '
            f'{PROVISIONAL_SOURCE!r}',
        )

        # 2. Once discovery resolves, reattribute_provisional() re-reports under
        #    the FQN with the provisional source superseded. The fault must end
        #    with the FQN present and the provisional source dropped.
        reattributed = False
        deadline = time.time() + 30.0
        while time.time() < deadline:
            sources = self._reporting_sources(ABORTED_CODE)
            if sources is not None and SERVER_FQN in sources and PROVISIONAL_SOURCE not in sources:
                reattributed = True
                break
            self._spin(0.2)
        self.assertTrue(
            reattributed,
            'fault was not re-attributed to the server FQN '
            f'{SERVER_FQN!r} (provisional source not superseded)',
        )


def _status_array(status, goal_byte):
    gsa = GoalStatusArray()
    gs = GoalStatus()
    gs.goal_info.goal_id = UUID(uuid=[goal_byte] * 16)
    gs.status = int(status)
    gsa.status_list = [gs]
    return gsa


@launch_testing.post_shutdown_test()
class TestReattributionShutdown(unittest.TestCase):
    """Confirm the fault_manager and bridge exited cleanly."""

    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
