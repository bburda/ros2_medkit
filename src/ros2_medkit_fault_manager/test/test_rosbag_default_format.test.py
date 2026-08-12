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
What an operator who configured nothing actually gets on disk.

test_rosbag_integration.test.py sets `snapshots.rosbag.format` explicitly and
pins it. This launches the fault manager WITHOUT that parameter, so the
struct/parameter default is what is under test, not an operator's choice.
"""

import os
import tempfile
import time
import unittest

from launch import LaunchDescription
import launch.actions
import launch_ros.actions
import launch_testing.actions
import launch_testing.markers
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from ros2_medkit_msgs.msg import Fault
from ros2_medkit_msgs.srv import GetRosbag, ReportFault
from sensor_msgs.msg import Temperature


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
        # Coverage environment is optional; on any error, fall back to no extra coverage config
        pass
    return {}


# Create a temp directory for rosbag storage that persists for test duration
ROSBAG_STORAGE_PATH = tempfile.mkdtemp(prefix='rosbag_default_format_test_')

# Path to temp publisher script (set in generate_test_description, cleaned up in shutdown test)
PUBLISHER_SCRIPT_PATH = None


def generate_test_description():
    """Generate launch description with fault_manager node with rosbag enabled."""
    # Use Python script for publishers to ensure proper topic type registration
    # ros2 topic pub has issues with topic type discovery in some configurations
    publisher_script = """
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Temperature
from std_msgs.msg import String
import time

class TestPublisher(Node):
    def __init__(self):
        super().__init__('test_publisher')
        import os
        domain_id = os.environ.get('ROS_DOMAIN_ID', 'not set')
        self.get_logger().info(f'Using ROS_DOMAIN_ID: {domain_id}')

        # Use sensor data QoS to match rosbag capture subscriptions
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.temp_pub = self.create_publisher(Temperature, '/test/temperature', qos)
        self.string_pub = self.create_publisher(String, '/test/status', qos)
        self.timer = self.create_timer(0.1, self.publish)
        self.counter = 0
        self.get_logger().info('TestPublisher started')

    def publish(self):
        temp_msg = Temperature()
        temp_msg.temperature = 25.0 + self.counter * 0.1
        temp_msg.variance = 0.1
        self.temp_pub.publish(temp_msg)

        string_msg = String()
        string_msg.data = f'status_{self.counter}'
        self.string_pub.publish(string_msg)
        self.counter += 1
        if self.counter % 50 == 0:
            self.get_logger().info(f'Published {self.counter} messages')
            # Log available topics and types for debugging
            topics = self.get_topic_names_and_types()
            topic_info = [(t, types) for t, types in topics if 'test' in t]
            if topic_info:
                self.get_logger().info(f'Test topics visible: {topic_info}')

def main():
    rclpy.init()
    node = TestPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
"""

    # Write publisher script to temp file and run it
    # Store path in global for cleanup in post-shutdown test
    global PUBLISHER_SCRIPT_PATH
    script_file = tempfile.NamedTemporaryFile(mode='w', suffix='.py', delete=False)
    script_file.write(publisher_script)
    script_file.close()
    PUBLISHER_SCRIPT_PATH = script_file.name

    # Inherit the ROS_DOMAIN_ID the domain wrapper allocated for this test, so
    # it is isolated from everything else running in parallel.
    # ROS_LOCALHOST_ONLY keeps discovery on loopback.
    env = os.environ.copy()
    env['ROS_LOCALHOST_ONLY'] = '1'

    test_publisher = launch.actions.ExecuteProcess(
        cmd=['python3', script_file.name],
        output='screen',
        name='test_publisher',
        env=env,
    )

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
            'confirmation_threshold': -1,  # Single report confirms immediately
            # Rosbag configuration. Deliberately no 'snapshots.rosbag.format' here:
            # the default is what this test verifies.
            'snapshots.rosbag.enabled': True,
            'snapshots.rosbag.duration_sec': 2.0,  # 2 second buffer
            'snapshots.rosbag.duration_after_sec': 0.5,  # 0.5 second after confirm
            # Use explicit topics - faster than discovery, more reliable for tests
            'snapshots.rosbag.topics': '/test/temperature,/test/status',
            'snapshots.rosbag.storage_path': ROSBAG_STORAGE_PATH,
            'snapshots.rosbag.max_bag_size_mb': 10,
            'snapshots.rosbag.max_total_storage_mb': 50,
            'snapshots.rosbag.auto_cleanup': True,
            # lazy_start=false: Start recording immediately
            'snapshots.rosbag.lazy_start': False,
        }],
        # Give the node room to flush coverage data at shutdown before SIGKILL.
        sigterm_timeout='30',
        sigkill_timeout='15',
    )

    # Delay fault_manager start so test_publisher has time to register topics
    # DDS discovery can take several seconds to propagate topic types
    delayed_fault_manager = launch.actions.TimerAction(
        period=8.0,
        actions=[fault_manager_node],
    )

    return (
        LaunchDescription([
            # Start publisher node first
            test_publisher,
            # Start fault_manager after delay
            delayed_fault_manager,
            launch_testing.actions.ReadyToTest(),
        ]),
        {
            'fault_manager_node': fault_manager_node,
            'test_publisher': test_publisher,
        },
    )


class TestRosbagDefaultFormat(unittest.TestCase):
    """A fault manager launched with no format parameter (@verifies REQ_INTEROP_088)."""

    @classmethod
    def setUpClass(cls):
        """Initialize ROS 2 context and create service clients."""
        # Match the launch processes: inherit the CMake-injected
        # ROS_DOMAIN_ID and keep discovery on loopback.
        os.environ['ROS_LOCALHOST_ONLY'] = '1'
        rclpy.init()
        cls.node = Node('test_rosbag_default_format_client')

        # Create service clients
        cls.report_fault_client = cls.node.create_client(
            ReportFault, '/fault_manager/report_fault'
        )
        cls.get_rosbag_client = cls.node.create_client(
            GetRosbag, '/fault_manager/get_rosbag'
        )

        # Wait for services to be available
        assert cls.report_fault_client.wait_for_service(timeout_sec=15.0), \
            'report_fault service not available'
        assert cls.get_rosbag_client.wait_for_service(timeout_sec=15.0), \
            'get_rosbag service not available'

        # Wait for the background publisher to come up before letting the rosbag
        # ring buffer fill (duration_sec=2.0 configured), so a slow publisher
        # start does not leave the buffer empty.
        deadline = time.time() + 15.0
        while (not cls.node.get_publishers_info_by_topic('/test/temperature')
               and time.time() < deadline):
            time.sleep(0.2)
        time.sleep(3.0)

    @classmethod
    def tearDownClass(cls):
        """Shutdown ROS 2 context."""
        cls.node.destroy_node()
        rclpy.shutdown()

    def _call_service(self, client, request, timeout_sec=10.0):
        """Call a service and wait for response."""
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout_sec)
        self.assertIsNotNone(future.result(), 'Service call timed out')
        return future.result()

    def _report_fault(self, fault_code, description='Test fault'):
        """Report a fault and return the response."""
        request = ReportFault.Request()
        request.fault_code = fault_code
        request.event_type = ReportFault.Request.EVENT_FAILED
        request.severity = Fault.SEVERITY_ERROR
        request.description = description
        request.source_id = '/test_node'
        return self._call_service(self.report_fault_client, request)

    def _wait_for_rosbag(self, fault_code, timeout=12.0):
        """
        Poll GetRosbag until the async post-fault recording is written.

        The recording plus its post-fault flush (duration_after_sec) can take
        longer than a fixed sleep under sanitizer/coverage load, so a one-shot
        call flakes. Mirror the polling loop in test_rosbag_entity_scope.
        """
        request = GetRosbag.Request()
        request.fault_code = fault_code
        deadline = time.time() + timeout
        response = self._call_service(self.get_rosbag_client, request)
        while time.time() < deadline:
            if response is not None and response.success:
                return response
            time.sleep(0.5)
            response = self._call_service(self.get_rosbag_client, request)
        return response

    def _wait_for_buffered_data(self, count=5, timeout=8.0):
        """
        Wait until the rosbag ring buffer holds fresh data before a confirmation.

        Mirror of the same helper in test_rosbag_integration.test.py: subscribe to
        the same source the fault manager records and wait for messages to arrive,
        rather than sleeping a fixed amount.
        """
        received = 0

        def _cb(_msg):
            nonlocal received
            received += 1

        # Match the publisher's BEST_EFFORT sensor QoS, else no messages arrive.
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        sub = self.node.create_subscription(Temperature, '/test/temperature', _cb, qos)
        try:
            deadline = time.time() + timeout
            while received < count and time.time() < deadline:
                rclpy.spin_once(self.node, timeout_sec=0.1)
        finally:
            self.node.destroy_subscription(sub)
        return received >= count

    def test_01_default_format_is_mcap(self):
        """
        A fault manager launched with no format parameter writes an mcap bag.

        The unit case pins the struct default; this pins what an operator who
        configured nothing actually gets on disk, which is the thing the docs
        and the README promise.
        """
        fault_code = 'DEFAULT_FORMAT_TEST'

        # Buffer should already have messages from background publishers.
        self.assertTrue(self._wait_for_buffered_data(),
                        'ring buffer never refilled after startup')
        response = self._report_fault(fault_code, 'Default rosbag format test fault')
        self.assertTrue(response.accepted)

        # Poll until the async post-fault recording is written.
        rosbag_response = self._wait_for_rosbag(fault_code)
        self.assertTrue(rosbag_response.success,
                        f'GetRosbag failed: {rosbag_response.error_message}')
        self.assertEqual(rosbag_response.format, 'mcap')

        contents = os.listdir(rosbag_response.file_path)
        self.assertTrue(
            any(name.endswith('.mcap') for name in contents),
            f'no .mcap file in the bag written with the default format: {contents}',
        )


@launch_testing.post_shutdown_test()
class TestRosbagDefaultFormatShutdown(unittest.TestCase):
    """Post-shutdown tests."""

    def test_exit_code(self, proc_info):
        """Verify fault_manager exits cleanly."""
        launch_testing.asserts.assertExitCodes(
            proc_info,
            process='fault_manager_node'
        )

    def test_cleanup_temp_directory(self):
        """Clean up temporary rosbag storage directory and publisher script."""
        import shutil
        if os.path.exists(ROSBAG_STORAGE_PATH):
            shutil.rmtree(ROSBAG_STORAGE_PATH, ignore_errors=True)
            print(f'Cleaned up temp directory: {ROSBAG_STORAGE_PATH}')

        # Clean up the temporary publisher script
        if PUBLISHER_SCRIPT_PATH and os.path.exists(PUBLISHER_SCRIPT_PATH):
            os.unlink(PUBLISHER_SCRIPT_PATH)
            print(f'Cleaned up temp script: {PUBLISHER_SCRIPT_PATH}')
