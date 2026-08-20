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

"""Integration test for entity-scoped rosbag capture (Issue #426)."""

# The default "entity" topic mode subscribes broadly for pre-roll but, when a fault
# is confirmed, writes only the topics of the faulting source node. This test runs
# two publisher nodes and verifies a fault sourced at one node yields a bag holding
# that node's topics and NOT the other node's topic.

import os
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
from ros2_medkit_msgs.srv import GetRosbag, ReportFault

ROSBAG_STORAGE_PATH = tempfile.mkdtemp(prefix='rosbag_entity_test_')
PUBLISHER_SCRIPT_PATH = None

# Two publisher nodes with distinct names so the gateway can resolve each entity's
# topics via the ROS graph. "robot_planner" owns /plan + /cmd_vel; "robot_camera"
# owns /telemetry. Default QoS (reliable) also exercises the qos_match path.
PUBLISHER_SCRIPT = """
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String


class Planner(Node):
    def __init__(self):
        super().__init__('robot_planner')
        self.plan_pub = self.create_publisher(String, '/plan', 10)
        self.cmd_pub = self.create_publisher(String, '/cmd_vel', 10)
        self.timer = self.create_timer(0.05, self.publish)
        self.counter = 0

    def publish(self):
        msg = String()
        msg.data = f'plan_{self.counter}'
        self.plan_pub.publish(msg)
        self.cmd_pub.publish(msg)
        self.counter += 1


class Camera(Node):
    def __init__(self):
        super().__init__('robot_camera')
        self.tele_pub = self.create_publisher(String, '/telemetry', 10)
        self.timer = self.create_timer(0.05, self.publish)
        self.counter = 0

    def publish(self):
        msg = String()
        msg.data = f'telemetry_{self.counter}'
        self.tele_pub.publish(msg)
        self.counter += 1


def main():
    rclpy.init()
    executor = SingleThreadedExecutor()
    planner = Planner()
    camera = Camera()
    executor.add_node(planner)
    executor.add_node(camera)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        planner.destroy_node()
        camera.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
"""


def generate_test_description():
    global PUBLISHER_SCRIPT_PATH
    script_file = tempfile.NamedTemporaryFile(mode='w', suffix='.py', delete=False)
    script_file.write(PUBLISHER_SCRIPT)
    script_file.close()
    PUBLISHER_SCRIPT_PATH = script_file.name

    # Inherit the CMake-injected per-test ROS_DOMAIN_ID; ROS_LOCALHOST_ONLY
    # keeps discovery on loopback.
    env = os.environ.copy()
    env['ROS_LOCALHOST_ONLY'] = '1'

    publishers = launch.actions.ExecuteProcess(
        cmd=['python3', script_file.name],
        output='screen',
        name='entity_publishers',
        env=env,
    )

    fault_manager_node = launch_ros.actions.Node(
        package='ros2_medkit_fault_manager',
        executable='fault_manager_node',
        name='fault_manager',
        output='screen',
        additional_env={'ROS_LOCALHOST_ONLY': '1'},
        parameters=[{
            'storage_type': 'memory',
            'confirmation_threshold': -1,
            'snapshots.rosbag.enabled': True,
            'snapshots.rosbag.duration_sec': 2.0,
            'snapshots.rosbag.duration_after_sec': 0.5,
            # "entity" is the default; set explicitly to document the intent.
            'snapshots.rosbag.topics': 'entity',
            'snapshots.rosbag.format': 'sqlite3',
            'snapshots.rosbag.storage_path': ROSBAG_STORAGE_PATH,
            'snapshots.rosbag.lazy_start': False,
        }],
        # Give the node room to flush coverage data at shutdown before SIGKILL.
        sigterm_timeout='30',
        sigkill_timeout='15',
    )

    delayed_fault_manager = launch.actions.TimerAction(
        period=8.0,
        actions=[fault_manager_node],
    )

    return (
        LaunchDescription([
            publishers,
            delayed_fault_manager,
            launch_testing.actions.ReadyToTest(),
        ]),
        {'fault_manager_node': fault_manager_node, 'publishers': publishers},
    )


class TestRosbagEntityScope(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        # Inherit the CMake-injected per-test ROS_DOMAIN_ID; keep discovery
        # on loopback.
        os.environ['ROS_LOCALHOST_ONLY'] = '1'
        rclpy.init()
        cls.node = Node('test_entity_scope_client')
        cls.report_fault_client = cls.node.create_client(
            ReportFault, '/fault_manager/report_fault')
        cls.get_rosbag_client = cls.node.create_client(GetRosbag, '/fault_manager/get_rosbag')
        assert cls.report_fault_client.wait_for_service(timeout_sec=20.0), \
            'report_fault unavailable'
        assert cls.get_rosbag_client.wait_for_service(timeout_sec=20.0), 'get_rosbag unavailable'
        # Let dynamic discovery subscribe to the publisher topics and the ring buffer
        # fill (needs > duration_sec of buffered messages before the fault is reported).
        deadline = time.time() + 15.0
        while time.time() < deadline:
            rclpy.spin_once(cls.node, timeout_sec=0.2)
            if cls.node.get_publishers_info_by_topic('/plan'):
                break
        time.sleep(5.0)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def _call(self, client, request, timeout_sec=10.0):
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout_sec)
        self.assertIsNotNone(future.result(), 'Service call timed out')
        return future.result()

    def _bag_topics(self, bag_path):
        import rosbag2_py
        reader = rosbag2_py.SequentialReader()
        reader.open(
            rosbag2_py.StorageOptions(uri=bag_path, storage_id='sqlite3'),
            rosbag2_py.ConverterOptions('', ''),
        )
        return {t.name for t in reader.get_all_topics_and_types()}

    def test_bag_scoped_to_faulting_entity(self):
        """A fault sourced at /robot_planner yields a bag with the planner's topics only."""
        request = ReportFault.Request()
        request.fault_code = 'ENTITY_SCOPE_TEST'
        request.event_type = ReportFault.Request.EVENT_FAILED
        request.severity = Fault.SEVERITY_ERROR
        request.description = 'entity scope test'
        request.source_id = '/robot_planner'
        response = self._call(self.report_fault_client, request)
        self.assertTrue(response.accepted)

        # Poll for the finished bag (post-fault recording + flush is asynchronous).
        rosbag_request = GetRosbag.Request()
        rosbag_request.fault_code = 'ENTITY_SCOPE_TEST'
        rosbag_response = None
        deadline = time.time() + 12.0
        while time.time() < deadline:
            rosbag_response = self._call(self.get_rosbag_client, rosbag_request)
            if rosbag_response.success:
                break
            time.sleep(0.5)
        self.assertTrue(rosbag_response.success,
                        f'GetRosbag failed: {rosbag_response.error_message}')
        self.assertTrue(os.path.exists(rosbag_response.file_path))

        topics = self._bag_topics(rosbag_response.file_path)
        print(f'Bag topics: {sorted(topics)}')

        # Planner's own topics are captured.
        self.assertIn('/plan', topics)
        self.assertIn('/cmd_vel', topics)
        # The other entity's topic is excluded even though it was buffered.
        self.assertNotIn('/telemetry', topics)


@launch_testing.post_shutdown_test()
class TestRosbagEntityScopeShutdown(unittest.TestCase):

    def test_exit_code(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info, process='fault_manager_node')

    def test_cleanup(self):
        import shutil
        if os.path.exists(ROSBAG_STORAGE_PATH):
            shutil.rmtree(ROSBAG_STORAGE_PATH, ignore_errors=True)
        if PUBLISHER_SCRIPT_PATH and os.path.exists(PUBLISHER_SCRIPT_PATH):
            os.unlink(PUBLISHER_SCRIPT_PATH)
