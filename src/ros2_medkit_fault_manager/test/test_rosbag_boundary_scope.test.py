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
Entity scoping of a boundary recording, against a real fault_manager_node.

``test_rosbag_boundary.test.py`` runs with an explicit topic list, so it proves
a boundary recording exists but says nothing about what goes into it. ``entity``
is the DEFAULT topic mode, and there the recording carries a filter: the
faulting node's topics and nothing else. Two things about that filter are only
worth believing against a real node:

* a recording opened at the boundary resolves its own scope, rather than
  inheriting or losing the previous recording's;
* a fault confirming inside that window attaches and widens the scope, so its
  own topics are in the bag from the attach onwards.

A third node publishes throughout and never faults. Without it both assertions
would also hold for a recording that gave up on scoping and wrote every topic,
which is the failure mode the filter exists to prevent.

Not covered here, deliberately: the finalise-versus-boundary race on the filter
needs the finalisation slowed down to be reproducible, which is a storage double
in the C++ tests and has no equivalent against a launched node.
"""

import os
import tempfile
import time
import unittest

from launch import LaunchDescription
import launch_ros.actions
import launch_testing.actions
import launch_testing.markers
import rclpy
from rclpy.node import Node
from ros2_medkit_msgs.msg import Fault
from ros2_medkit_msgs.srv import GetRosbag, ReportFault
from std_msgs.msg import Float32


DURATION_SEC = 3.0
DURATION_AFTER_SEC = 4.0
STORAGE_FORMAT = 'mcap'

SOURCE_A = 'scope_source_a'
SOURCE_B = 'scope_source_b'
SOURCE_BYSTANDER = 'scope_bystander'

TOPIC_A = '/scope/a_telemetry'
TOPIC_B = '/scope/b_telemetry'
TOPIC_BYSTANDER = '/scope/bystander_telemetry'


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
        # Coverage environment is optional; on any error, fall back to none
        pass
    return {}


@launch_testing.markers.keep_alive
def generate_test_description():
    """Launch the fault_manager in the default entity topic mode."""
    storage_path = tempfile.mkdtemp(prefix='rosbag_boundary_scope_')

    fault_manager_env = get_coverage_env()
    fault_manager_env['ROS_LOCALHOST_ONLY'] = '1'

    fault_manager_node = launch_ros.actions.Node(
        package='ros2_medkit_fault_manager',
        executable='fault_manager_node',
        name='fault_manager',
        output='screen',
        additional_env=fault_manager_env,
        parameters=[{
            'storage_type': 'memory',
            'confirmation_threshold': -1,
            'snapshots.rosbag.enabled': True,
            'snapshots.rosbag.duration_sec': DURATION_SEC,
            'snapshots.rosbag.duration_after_sec': DURATION_AFTER_SEC,
            # The default, and the whole point of this file.
            'snapshots.rosbag.topics': 'entity',
            'snapshots.rosbag.format': STORAGE_FORMAT,
            'snapshots.rosbag.storage_path': storage_path,
            'snapshots.rosbag.max_bag_size_mb': 10,
            'snapshots.rosbag.max_total_storage_mb': 50,
            'snapshots.rosbag.auto_cleanup': True,
            'snapshots.rosbag.lazy_start': False,
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
            'rosbag_storage_path': storage_path,
        },
    )


class TestRosbagBoundaryScope(unittest.TestCase):
    """Entity scoping of a recording opened at the window boundary."""

    @classmethod
    def setUpClass(cls):
        os.environ['ROS_LOCALHOST_ONLY'] = '1'
        rclpy.init()
        cls.node = Node('boundary_scope_driver')

        # One node per source, because entity scope is resolved from the ROS
        # graph: the faulting node's own publications are what defines it.
        cls.node_a = Node(SOURCE_A)
        cls.node_b = Node(SOURCE_B)
        cls.node_bystander = Node(SOURCE_BYSTANDER)
        cls.pub_a = cls.node_a.create_publisher(Float32, TOPIC_A, 10)
        cls.pub_b = cls.node_b.create_publisher(Float32, TOPIC_B, 10)
        cls.pub_bystander = cls.node_bystander.create_publisher(
            Float32, TOPIC_BYSTANDER, 10
        )

        cls.report_fault_client = cls.node.create_client(
            ReportFault, '/fault_manager/report_fault'
        )
        cls.get_rosbag_client = cls.node.create_client(
            GetRosbag, '/fault_manager/get_rosbag'
        )
        assert cls.report_fault_client.wait_for_service(timeout_sec=15.0), \
            'report_fault service not available'
        assert cls.get_rosbag_client.wait_for_service(timeout_sec=15.0), \
            'get_rosbag service not available'

        # Entity mode discovers topics as they appear, so wait until the capture
        # has subscribed to all three. Before that a message is never buffered
        # and an absent topic in the bag would mean nothing.
        deadline = time.time() + 20.0
        while time.time() < deadline:
            subscribed = all(
                cls.node.get_subscriptions_info_by_topic(topic)
                for topic in (TOPIC_A, TOPIC_B, TOPIC_BYSTANDER)
            )
            if subscribed:
                break
            cls._publish_once()
            time.sleep(0.2)
        for topic in (TOPIC_A, TOPIC_B, TOPIC_BYSTANDER):
            assert cls.node.get_subscriptions_info_by_topic(topic), \
                f'fault_manager never subscribed to {topic}'

    @classmethod
    def tearDownClass(cls):
        for node in (cls.node_a, cls.node_b, cls.node_bystander, cls.node):
            node.destroy_node()
        rclpy.shutdown()

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    @classmethod
    def _publish_once(cls):
        msg = Float32()
        msg.data = 1.0
        cls.pub_a.publish(msg)
        cls.pub_b.publish(msg)
        cls.pub_bystander.publish(msg)

    def _publish_for(self, duration_sec, rate_hz=20.0):
        """Publish on all three topics for *duration_sec*."""
        deadline = time.time() + duration_sec
        period = 1.0 / rate_hz
        while time.time() < deadline:
            self._publish_once()
            time.sleep(period)

    def _call_service(self, client, request, timeout_sec=10.0):
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout_sec)
        self.assertIsNotNone(future.result(), 'Service call timed out')
        return future.result()

    def _report_fault(self, fault_code, source_node):
        request = ReportFault.Request()
        request.fault_code = fault_code
        request.event_type = ReportFault.Request.EVENT_FAILED
        request.severity = Fault.SEVERITY_ERROR
        request.description = 'boundary scope test fault'
        request.source_id = f'/{source_node}'
        return self._call_service(self.report_fault_client, request)

    def _get_rosbag(self, fault_code):
        request = GetRosbag.Request()
        request.fault_code = fault_code
        return self._call_service(self.get_rosbag_client, request)

    def _wait_for_rosbag(self, fault_code, timeout=30.0):
        deadline = time.time() + timeout
        response = self._get_rosbag(fault_code)
        while time.time() < deadline:
            if response is not None and response.success:
                return response
            self._publish_once()
            time.sleep(0.3)
            response = self._get_rosbag(fault_code)
        return response

    def _read_bag(self, bag_path):
        """Return ``(topics, message_count)`` read from the finalised bag."""
        import rosbag2_py
        reader = rosbag2_py.SequentialReader()
        reader.open(
            rosbag2_py.StorageOptions(uri=bag_path, storage_id=STORAGE_FORMAT),
            rosbag2_py.ConverterOptions('', ''),
        )
        topics = {t.name for t in reader.get_all_topics_and_types()}
        count = 0
        while reader.has_next():
            reader.read_next()
            count += 1
        return topics, count

    # ------------------------------------------------------------------
    # Tests
    # ------------------------------------------------------------------

    def test_01_boundary_recording_is_scoped_and_widens_on_attach(self):
        """
        A boundary recording is scoped, and widens when a fault attaches.

        @verifies REQ_INTEROP_088
        """
        # Fill the buffer, then flush it with a fault from node A. Everything
        # published during A's window goes into A's bag rather than the buffer.
        self._publish_for(DURATION_SEC + 1.0)
        self._report_fault('SCOPE_PRIMARY', SOURCE_A)

        primary = self._wait_for_rosbag('SCOPE_PRIMARY')
        self.assertIsNotNone(primary, 'GetRosbag never answered for the first recording')
        self.assertTrue(primary.success, 'the first recording never finalised')

        # The buffer is empty by construction now: this is the boundary, and B
        # is the fault that has to resolve a scope of its own.
        self._report_fault('SCOPE_BOUNDARY', SOURCE_B)
        time.sleep(1.0)
        self._publish_for(1.0)

        # Inside B's window, so this one attaches and widens the scope to A too.
        self._report_fault('SCOPE_ATTACHED', SOURCE_A)
        self._publish_for(DURATION_AFTER_SEC)

        boundary = self._wait_for_rosbag('SCOPE_BOUNDARY')
        self.assertIsNotNone(boundary, 'GetRosbag never answered for the boundary recording')
        self.assertTrue(boundary.success, 'the boundary recording never finalised')
        attached = self._wait_for_rosbag('SCOPE_ATTACHED')
        self.assertIsNotNone(attached, 'GetRosbag never answered for the attached fault')
        self.assertTrue(attached.success, 'the attached fault got no row')
        self.assertEqual(
            attached.file_path, boundary.file_path,
            'the attached fault got its own recording instead of sharing the open one',
        )

        topics, count = self._read_bag(boundary.file_path)
        self.assertGreater(count, 0, 'the boundary recording finalised empty')
        self.assertIn(
            TOPIC_B, topics,
            "the boundary recording is missing its own faulting node's topic",
        )
        self.assertIn(
            TOPIC_A, topics,
            'the attached fault got a row for a recording holding none of its topics',
        )
        self.assertNotIn(
            TOPIC_BYSTANDER, topics,
            'the recording wrote a topic belonging to no fault, so it is not scoped '
            'at all and the two assertions above prove nothing',
        )


@launch_testing.post_shutdown_test()
class TestRosbagBoundaryScopeShutdown(unittest.TestCase):
    """Shutdown checks."""

    def test_exit_code(self, proc_info):
        """Verify fault_manager exits cleanly."""
        launch_testing.asserts.assertExitCodes(
            proc_info,
            process='fault_manager_node'
        )

    def test_cleanup_storage_directory(self, rosbag_storage_path):
        """Remove this run's bag storage directory."""
        import shutil
        if os.path.exists(rosbag_storage_path):
            shutil.rmtree(rosbag_storage_path, ignore_errors=True)
