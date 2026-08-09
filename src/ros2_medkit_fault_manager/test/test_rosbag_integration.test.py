#!/usr/bin/env python3
# Copyright 2025 mfaferek93
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

"""Integration tests for rosbag capture feature (Issue #120)."""

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
from ros2_medkit_msgs.srv import ClearFault, GetRosbag, ReportFault
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
ROSBAG_STORAGE_PATH = tempfile.mkdtemp(prefix='rosbag_test_')

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
            # Rosbag configuration
            'snapshots.rosbag.enabled': True,
            'snapshots.rosbag.duration_sec': 2.0,  # 2 second buffer
            'snapshots.rosbag.duration_after_sec': 0.5,  # 0.5 second after confirm
            # Use explicit topics - faster than discovery, more reliable for tests
            'snapshots.rosbag.topics': '/test/temperature,/test/status',
            'snapshots.rosbag.format': 'mcap',
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


class TestRosbagCaptureIntegration(unittest.TestCase):
    """Integration tests for rosbag capture feature (@verifies REQ_INTEROP_088)."""

    @classmethod
    def setUpClass(cls):
        """Initialize ROS 2 context and create service clients."""
        # Match the launch processes: inherit the CMake-injected
        # ROS_DOMAIN_ID and keep discovery on loopback.
        os.environ['ROS_LOCALHOST_ONLY'] = '1'
        rclpy.init()
        cls.node = Node('test_rosbag_client')

        # Create service clients
        cls.report_fault_client = cls.node.create_client(
            ReportFault, '/fault_manager/report_fault'
        )
        cls.clear_fault_client = cls.node.create_client(
            ClearFault, '/fault_manager/clear_fault'
        )
        cls.get_rosbag_client = cls.node.create_client(
            GetRosbag, '/fault_manager/get_rosbag'
        )

        # Wait for services to be available
        assert cls.report_fault_client.wait_for_service(timeout_sec=15.0), \
            'report_fault service not available'
        assert cls.clear_fault_client.wait_for_service(timeout_sec=15.0), \
            'clear_fault service not available'
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

        A previous fault's post-fault recording window diverts incoming messages
        away from the ring buffer, so right after it closes the buffer can be
        empty and flush_to_bag creates no bag. There is no service to read the
        buffer, so subscribe to the same source the fault manager records and
        wait until ``count`` messages arrive - by then the co-subscribed fault
        manager has buffered a comparable amount. Adaptive, unlike a fixed sleep.
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

    def test_01_rosbag_created_on_fault_confirmation(self):
        """Test that rosbag file is created when fault is confirmed."""
        fault_code = 'ROSBAG_TEST_001'

        # Report fault - buffer should already have messages from background publishers
        # The previous test's post-fault recording window diverts incoming messages away
        # from the ring buffer, so right after it closes the buffer can be empty and
        # flush_to_bag creates no bag. Wait for it to refill before reporting.
        self.assertTrue(self._wait_for_buffered_data(),
                        'ring buffer never refilled after the previous post-fault window')
        response = self._report_fault(fault_code, 'Rosbag integration test fault')
        self.assertTrue(response.accepted)
        print(f'Fault {fault_code} reported and confirmed')

        # Poll until the async post-fault recording is written.
        rosbag_response = self._wait_for_rosbag(fault_code)

        self.assertTrue(rosbag_response.success,
                        f'GetRosbag failed: {rosbag_response.error_message}')
        self.assertGreater(len(rosbag_response.file_path), 0)
        self.assertEqual(rosbag_response.format, 'mcap')
        self.assertGreater(rosbag_response.duration_sec, 0)
        self.assertGreater(rosbag_response.size_bytes, 0)

        # Verify file exists
        self.assertTrue(os.path.exists(rosbag_response.file_path),
                        f'Rosbag file not found: {rosbag_response.file_path}')

        print(f'Rosbag created: {rosbag_response.file_path}')
        print(f'  Duration: {rosbag_response.duration_sec:.2f}s')
        print(f'  Size: {rosbag_response.size_bytes} bytes')
        print(f'  Format: {rosbag_response.format}')

    def test_02_rosbag_auto_cleanup_on_clear(self):
        """Test that rosbag file is deleted when fault is cleared (auto_cleanup=true)."""
        fault_code = 'ROSBAG_CLEANUP_TEST'

        # Report fault - buffer has messages from background publishers
        # The previous test's post-fault recording window diverts incoming messages away
        # from the ring buffer, so right after it closes the buffer can be empty and
        # flush_to_bag creates no bag. Wait for it to refill before reporting.
        self.assertTrue(self._wait_for_buffered_data(),
                        'ring buffer never refilled after the previous post-fault window')
        response = self._report_fault(fault_code, 'Cleanup test fault')
        self.assertTrue(response.accepted)

        # Poll until the async post-fault recording is written.
        rosbag_response = self._wait_for_rosbag(fault_code)

        self.assertTrue(rosbag_response.success)
        bag_path = rosbag_response.file_path
        self.assertTrue(os.path.exists(bag_path))
        print(f'Rosbag exists before clear: {bag_path}')

        # Clear the fault
        clear_request = ClearFault.Request()
        clear_request.fault_code = fault_code

        clear_response = self._call_service(self.clear_fault_client, clear_request)
        self.assertTrue(clear_response.success)
        print('Fault cleared')

        # Poll until the async cleanup removes the bag file.
        deadline = time.time() + 10.0
        while os.path.exists(bag_path) and time.time() < deadline:
            time.sleep(0.5)

        # Verify rosbag is deleted
        self.assertFalse(os.path.exists(bag_path),
                         f'Rosbag should be deleted but still exists: {bag_path}')
        print('Rosbag auto-deleted on clear')

        # GetRosbag should now fail
        rosbag_request = GetRosbag.Request()
        rosbag_request.fault_code = fault_code
        rosbag_response2 = self._call_service(self.get_rosbag_client, rosbag_request)
        self.assertFalse(rosbag_response2.success)
        print(f'GetRosbag after clear: {rosbag_response2.error_message}')

    def test_03_get_rosbag_nonexistent_fault(self):
        """Test GetRosbag returns error for nonexistent fault."""
        request = GetRosbag.Request()
        request.fault_code = 'NONEXISTENT_ROSBAG_FAULT'

        response = self._call_service(self.get_rosbag_client, request)

        self.assertFalse(response.success)
        self.assertIn('not found', response.error_message.lower())
        print(f'Nonexistent fault error: {response.error_message}')

    def test_04_get_rosbag_empty_fault_code(self):
        """Test GetRosbag returns error for empty fault code."""
        request = GetRosbag.Request()
        request.fault_code = ''

        response = self._call_service(self.get_rosbag_client, request)

        self.assertFalse(response.success)
        self.assertIn('empty', response.error_message.lower())
        print(f'Empty fault code error: {response.error_message}')

    def test_05_multiple_faults_separate_bags(self):
        """Test that multiple faults create separate rosbag files."""
        fault_codes = ['MULTI_BAG_A', 'MULTI_BAG_B']
        bag_paths = []

        # test_02's post-fault recording window keeps new messages out of the
        # ring buffer, so it can still be empty when this test starts. Wait for
        # fresh buffered data before the first confirmation, otherwise
        # flush_to_bag finds an empty buffer and creates no bag for MULTI_BAG_A.
        self.assertTrue(self._wait_for_buffered_data(),
                        'ring buffer did not refill before reporting faults')

        # Report multiple faults - buffer has messages from background publishers
        for fault_code in fault_codes:
            # A fault confirming while the previous post-roll still runs attaches
            # to that recording (shared bag), so wait until the previous fault's
            # bag is finalized (its metadata row exists) instead of sleeping: a
            # fixed sleep only exceeds duration_after_sec on an unloaded machine.
            if bag_paths:
                self.assertTrue(self._wait_for_buffered_data(),
                                'ring buffer did not refill between faults')
            response = self._report_fault(fault_code, f'Multi-bag test: {fault_code}')
            self.assertTrue(response.accepted)

            rosbag_response = self._wait_for_rosbag(fault_code)
            self.assertTrue(rosbag_response.success,
                            f'GetRosbag failed for {fault_code}: {rosbag_response.error_message}')
            self.assertTrue(os.path.exists(rosbag_response.file_path))
            bag_paths.append(rosbag_response.file_path)
            print(f'Rosbag for {fault_code}: {rosbag_response.file_path}')

        # Verify bags are different files
        self.assertNotEqual(bag_paths[0], bag_paths[1])
        print('Multiple faults have separate rosbag files')

    def test_06_rosbag_contains_recorded_duration(self):
        """Test that rosbag duration approximately matches configuration."""
        fault_code = 'DURATION_TEST'

        # Report fault - buffer should have ~duration_sec worth of messages
        # Same refill race as test_01/test_02: the previous test's post-fault window
        # diverts messages away from the ring buffer, so reporting straight after it
        # closes finds the buffer empty and no bag is created.
        self.assertTrue(self._wait_for_buffered_data(),
                        'ring buffer never refilled after the previous post-fault window')
        response = self._report_fault(fault_code, 'Duration test fault')
        self.assertTrue(response.accepted)

        # Poll until the async post-fault recording is written.
        rosbag_response = self._wait_for_rosbag(fault_code)

        self.assertTrue(rosbag_response.success)

        # Duration should be approximately duration_sec + duration_after_sec
        # Allow some tolerance for timing
        expected_min = 1.5  # At least some data
        expected_max = 4.0  # Upper bound with tolerance
        self.assertGreaterEqual(rosbag_response.duration_sec, expected_min,
                                f'Duration too short: {rosbag_response.duration_sec}')
        self.assertLessEqual(rosbag_response.duration_sec, expected_max,
                             f'Duration too long: {rosbag_response.duration_sec}')

        print(f'Rosbag duration: {rosbag_response.duration_sec:.2f}s '
              f'(expected: ~2.5s with tolerance)')

    def test_07_burst_faults_share_one_recording(self):
        """Faults confirmed back-to-back attach to one shared recording."""
        fault_codes = ['BURST_SHARED_A', 'BURST_SHARED_B']

        # A previous test's post-roll may have drained the ring buffer.
        self.assertTrue(self._wait_for_buffered_data(),
                        'ring buffer did not refill before reporting faults')

        # No sleep between the reports: the second confirmation lands inside the
        # first fault's post-roll window and must attach to its recording.
        for fault_code in fault_codes:
            response = self._report_fault(fault_code, f'Burst test: {fault_code}')
            self.assertTrue(response.accepted)

        responses = {}
        for fault_code in fault_codes:
            rosbag_response = self._wait_for_rosbag(fault_code)
            self.assertTrue(rosbag_response.success,
                            f'GetRosbag failed for {fault_code}: '
                            f'{rosbag_response.error_message}')
            self.assertTrue(os.path.exists(rosbag_response.file_path))
            responses[fault_code] = rosbag_response

        self.assertEqual(responses['BURST_SHARED_A'].file_path,
                         responses['BURST_SHARED_B'].file_path,
                         'faults of one burst must share a single recording')
        print(f'Shared recording: {responses["BURST_SHARED_A"].file_path}')


@launch_testing.post_shutdown_test()
class TestRosbagCaptureShutdown(unittest.TestCase):
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
