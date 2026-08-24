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

"""
End-to-end tests for keeping several recordings per fault code (Issue #620).

The other rosbag suites all run at the shipped default of one recording per
fault, which is what every release before this one could store. This one raises
the cap and drives the case the contributor reported: a fault that confirms,
clears and confirms again. Under the old schema the second recording silently
replaced the first, so the black box only ever held the most recent event.

Runs against SQLite rather than the in-memory backend on purpose. SQLite is the
shipped default, it enforces the cap in SQL, and the unique index this change
introduces only exists there - the in-memory parity is covered by
test_rosbag_storage_parity at unit level.
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
from ros2_medkit_msgs.msg import Fault, FaultEvent
from ros2_medkit_msgs.srv import ClearFault, GetRosbag, ListRosbags, ReportFault
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


ROSBAG_STORAGE_PATH = tempfile.mkdtemp(prefix='rosbag_history_')
DATABASE_PATH = os.path.join(ROSBAG_STORAGE_PATH, 'faults.db')
MAX_BAGS_PER_FAULT = 3

PUBLISHER_SCRIPT_PATH = None


def generate_test_description():
    """Launch a fault_manager keeping several recordings per fault code."""
    publisher_script = """
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Temperature
from std_msgs.msg import String

class TestPublisher(Node):
    def __init__(self):
        super().__init__('history_publisher')
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.temp_pub = self.create_publisher(Temperature, '/test/temperature', qos)
        self.string_pub = self.create_publisher(String, '/test/status', qos)
        self.timer = self.create_timer(0.1, self.publish)
        self.counter = 0

    def publish(self):
        temp_msg = Temperature()
        temp_msg.temperature = 25.0 + self.counter * 0.1
        temp_msg.variance = 0.1
        self.temp_pub.publish(temp_msg)

        string_msg = String()
        string_msg.data = f'status_{self.counter}'
        self.string_pub.publish(string_msg)
        self.counter += 1

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

    global PUBLISHER_SCRIPT_PATH
    script_file = tempfile.NamedTemporaryFile(mode='w', suffix='.py', delete=False)
    script_file.write(publisher_script)
    script_file.close()
    PUBLISHER_SCRIPT_PATH = script_file.name

    env = os.environ.copy()
    env['ROS_LOCALHOST_ONLY'] = '1'

    test_publisher = launch.actions.ExecuteProcess(
        cmd=['python3', script_file.name],
        output='screen',
        name='history_publisher',
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
            # SQLite, not memory: the cap is enforced in SQL there, and the
            # unique index that replaced the old UNIQUE(fault_code) only exists
            # in this backend.
            'storage_type': 'sqlite',
            'database_path': DATABASE_PATH,
            'confirmation_threshold': -1,
            'snapshots.rosbag.enabled': True,
            'snapshots.rosbag.duration_sec': 2.0,
            'snapshots.rosbag.duration_after_sec': 0.5,
            'snapshots.rosbag.topics': '/test/temperature,/test/status',
            'snapshots.rosbag.format': 'mcap',
            'snapshots.rosbag.storage_path': ROSBAG_STORAGE_PATH,
            'snapshots.rosbag.max_bag_size_mb': 10,
            'snapshots.rosbag.max_total_storage_mb': 50,
            # The whole point: keep a history instead of replacing.
            'snapshots.rosbag.max_bags_per_fault': MAX_BAGS_PER_FAULT,
            # Clearing a fault must not take its recordings with it - the
            # scenario under test is confirm / clear / confirm.
            'snapshots.rosbag.auto_cleanup': False,
            'snapshots.rosbag.lazy_start': False,
        }],
        sigterm_timeout='30',
        sigkill_timeout='15',
    )

    delayed_fault_manager = launch.actions.TimerAction(
        period=8.0,
        actions=[fault_manager_node],
    )

    return (
        LaunchDescription([
            test_publisher,
            delayed_fault_manager,
            launch_testing.actions.ReadyToTest(),
        ]),
        {
            'fault_manager_node': fault_manager_node,
            'test_publisher': test_publisher,
        },
    )


class TestRosbagHistory(unittest.TestCase):
    """Several recordings per fault code, addressed by recording id."""

    @classmethod
    def setUpClass(cls):
        os.environ['ROS_LOCALHOST_ONLY'] = '1'
        rclpy.init()
        cls.node = Node('test_rosbag_history_client')

        cls.report_fault_client = cls.node.create_client(
            ReportFault, '/fault_manager/report_fault'
        )
        cls.clear_fault_client = cls.node.create_client(
            ClearFault, '/fault_manager/clear_fault'
        )
        cls.get_rosbag_client = cls.node.create_client(
            GetRosbag, '/fault_manager/get_rosbag'
        )
        cls.list_rosbags_client = cls.node.create_client(
            ListRosbags, '/fault_manager/list_rosbags'
        )

        assert cls.report_fault_client.wait_for_service(timeout_sec=15.0), \
            'report_fault service not available'
        assert cls.clear_fault_client.wait_for_service(timeout_sec=15.0), \
            'clear_fault service not available'
        assert cls.get_rosbag_client.wait_for_service(timeout_sec=15.0), \
            'get_rosbag service not available'
        assert cls.list_rosbags_client.wait_for_service(timeout_sec=15.0), \
            'list_rosbags service not available'

        deadline = time.time() + 15.0
        while (not cls.node.get_publishers_info_by_topic('/test/temperature')
               and time.time() < deadline):
            time.sleep(0.2)
        time.sleep(3.0)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def _call_service(self, client, request, timeout_sec=10.0):
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout_sec)
        self.assertIsNotNone(future.result(), 'Service call timed out')
        return future.result()

    def _report_fault(self, fault_code, description='History test fault'):
        request = ReportFault.Request()
        request.fault_code = fault_code
        request.event_type = ReportFault.Request.EVENT_FAILED
        request.severity = Fault.SEVERITY_ERROR
        request.description = description
        request.source_id = '/test_node'
        return self._call_service(self.report_fault_client, request)

    def _clear_fault(self, fault_code):
        request = ClearFault.Request()
        request.fault_code = fault_code
        return self._call_service(self.clear_fault_client, request)

    def _get_by_fault(self, fault_code, timeout=12.0):
        """Poll GetRosbag by fault code until the post-roll recording lands."""
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

    def _wait_for_new_recording(self, fault_code, known_ids, timeout=15.0):
        """Poll until the fault's newest recording is one we have not seen."""
        deadline = time.time() + timeout
        response = None
        while time.time() < deadline:
            response = self._get_by_fault(fault_code, timeout=2.0)
            if (response is not None and response.success
                    and response.recording_id not in known_ids):
                return response
            time.sleep(0.5)
        return response

    def _get_by_recording(self, recording_id):
        request = GetRosbag.Request()
        request.recording_id = recording_id
        return self._call_service(self.get_rosbag_client, request)

    def _wait_for_buffered_data(self, count=5, timeout=8.0):
        """
        Wait until the ring buffer holds fresh data before a confirmation.

        A previous fault's post-roll diverts incoming messages straight into the
        open bag, so right after that window closes the buffer can be empty and
        the next confirmation produces no bag at all. Reset the streak on every
        CONFIRMED so the messages counted here provably arrived with no post-roll
        newly opened partway through - see the same helper in
        test_rosbag_integration.
        """
        received = 0

        def _cb(_msg):
            nonlocal received
            received += 1

        def _on_event(msg):
            nonlocal received
            if msg.event_type == FaultEvent.EVENT_CONFIRMED:
                received = 0

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        events_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=100,
        )
        sub = self.node.create_subscription(Temperature, '/test/temperature', _cb, qos)
        events_sub = self.node.create_subscription(
            FaultEvent, '/fault_manager/events', _on_event, events_qos
        )
        try:
            deadline = time.time() + timeout
            while received < count and time.time() < deadline:
                rclpy.spin_once(self.node, timeout_sec=0.1)
        finally:
            self.node.destroy_subscription(sub)
            self.node.destroy_subscription(events_sub)
        return received >= count

    def _record_occurrence(self, fault_code, known_ids):
        """Confirm the fault once and return its new recording."""
        self.assertTrue(self._wait_for_buffered_data(),
                        'ring buffer never refilled after the previous post-roll')
        response = self._report_fault(fault_code)
        self.assertTrue(response.accepted)

        recording = self._wait_for_new_recording(fault_code, known_ids)
        self.assertIsNotNone(recording)
        self.assertTrue(recording.success,
                        f'no recording for {fault_code}: {recording.error_message}')
        self.assertNotIn(recording.recording_id, known_ids,
                         'the re-confirmation reused the previous recording')
        return recording

    def test_01_reconfirming_a_fault_keeps_the_earlier_recording(self):
        """The reported bug: the second confirmation used to overwrite the first."""
        fault_code = 'FLAPPING_SENSOR'

        first = self._record_occurrence(fault_code, set())
        self.assertTrue(os.path.exists(first.file_path))
        print(f'First occurrence: {first.recording_id}')

        # Clearing is what a technician does after acknowledging. With
        # auto_cleanup off the evidence has to survive it.
        self.assertTrue(self._clear_fault(fault_code).success)

        second = self._record_occurrence(fault_code, {first.recording_id})
        print(f'Second occurrence: {second.recording_id}')

        self.assertNotEqual(first.file_path, second.file_path,
                            'the two occurrences must be separate bags')
        self.assertTrue(os.path.exists(first.file_path),
                        'the first recording was overwritten - this is issue #620')
        self.assertTrue(os.path.exists(second.file_path))

        # Both remain individually addressable by their own id.
        for recording in (first, second):
            fetched = self._get_by_recording(recording.recording_id)
            self.assertTrue(fetched.success,
                            f'recording {recording.recording_id} not retrievable: '
                            f'{fetched.error_message}')
            self.assertEqual(fetched.file_path, recording.file_path)
            self.assertIn(fault_code, fetched.fault_codes)

    def test_02_fault_code_lookup_still_serves_the_newest_recording(self):
        """The compatibility window: a fault-code URL keeps working."""
        fault_code = 'FLAPPING_SENSOR'

        # Established by test_01; the newest of them is what a bare fault code
        # has always meant, and must still mean.
        newest = self._get_by_fault(fault_code)
        self.assertTrue(newest.success)
        self.assertGreater(len(newest.recording_id), 0,
                           'a fault-code lookup must still name the recording it served')

        by_id = self._get_by_recording(newest.recording_id)
        self.assertTrue(by_id.success)
        self.assertEqual(by_id.file_path, newest.file_path,
                         'both addressing modes must resolve to the same bytes')
        self.assertEqual(by_id.recording_id, newest.recording_id)

    def test_03_the_cap_evicts_the_oldest_recording(self):
        """Past max_bags_per_fault the oldest goes, newest are kept."""
        fault_code = 'CAPPED_FAULT'

        recordings = []
        seen = set()
        for occurrence in range(MAX_BAGS_PER_FAULT + 1):
            cleared = self._clear_fault(fault_code)
            # Nothing to acknowledge before the first occurrence; from then on a
            # failed clear means the fault never left CONFIRMED, and the next
            # occurrence would reuse the recording instead of making a new one -
            # which surfaces later as a timeout that names the wrong cause.
            if occurrence > 0:
                self.assertTrue(cleared.success,
                                f'clear before occurrence {occurrence} failed: {cleared.message}')
            recording = self._record_occurrence(fault_code, seen)
            seen.add(recording.recording_id)
            recordings.append(recording)

        oldest = recordings[0]
        survivors = recordings[1:]

        # The eviction happens inside the store, so it is done by the time the
        # newest recording is readable.
        deadline = time.time() + 10.0
        while os.path.exists(oldest.file_path) and time.time() < deadline:
            time.sleep(0.5)

        self.assertFalse(os.path.exists(oldest.file_path),
                         f'the cap of {MAX_BAGS_PER_FAULT} did not evict the oldest bag')
        evicted = self._get_by_recording(oldest.recording_id)
        self.assertFalse(evicted.success, 'the evicted recording is still addressable')

        for recording in survivors:
            self.assertTrue(os.path.exists(recording.file_path),
                            f'{recording.recording_id} was evicted but is within the cap')
            self.assertTrue(self._get_by_recording(recording.recording_id).success)

    def test_04_list_rosbags_names_every_recording(self):
        """The gateway builds one descriptor per recording out of this list."""
        request = ListRosbags.Request()
        request.entity_fqn = '/test_node'
        response = self._call_service(self.list_rosbags_client, request)

        self.assertTrue(response.success, response.error_message)
        self.assertEqual(len(response.recording_ids), len(response.file_paths),
                         'recording_ids must be parallel to the other arrays')
        self.assertEqual(len(response.recording_ids), len(response.fault_codes))

        # A fault that confirmed several times contributes several rows, each
        # naming its own recording. Before this change it could only ever
        # contribute one.
        capped_ids = {rid for rid, code in zip(response.recording_ids, response.fault_codes)
                      if code == 'CAPPED_FAULT'}
        self.assertEqual(len(capped_ids), MAX_BAGS_PER_FAULT,
                         f'expected {MAX_BAGS_PER_FAULT} recordings for CAPPED_FAULT, '
                         f'got {sorted(capped_ids)}')

        for recording_id, file_path in zip(response.recording_ids, response.file_paths):
            self.assertEqual(recording_id, os.path.basename(file_path.rstrip('/')),
                             'recording id must be the bag directory basename')

    def test_05_unknown_recording_id_is_reported_not_guessed(self):
        """An id that is neither a recording nor a fault code fails cleanly."""
        response = self._get_by_recording('fault_NO_SUCH_BAG_1700000000000')
        self.assertFalse(response.success)
        self.assertGreater(len(response.error_message), 0)

    def test_06_a_traversal_shaped_recording_id_is_rejected(self):
        """The id becomes a URL segment and is validated like a fault code."""
        for bad_id in ('../../etc/passwd', 'fault/../../X', 'has spaces'):
            response = self._get_by_recording(bad_id)
            self.assertFalse(response.success, f'{bad_id!r} was accepted')


@launch_testing.post_shutdown_test()
class TestRosbagHistoryShutdown(unittest.TestCase):
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

        if PUBLISHER_SCRIPT_PATH and os.path.exists(PUBLISHER_SCRIPT_PATH):
            os.unlink(PUBLISHER_SCRIPT_PATH)
