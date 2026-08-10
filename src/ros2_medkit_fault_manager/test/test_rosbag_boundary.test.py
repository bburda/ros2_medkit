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
Integration tests for the post-fault window boundary (Issue #574).

A fault confirming right AFTER the previous fault's post-roll finalised finds
the ring buffer empty by construction: the flush drained it and the post-roll
diverted every incoming message straight to the bag. The fault must still get
a black box - a post-fault-only bag over its own ``duration_after_sec`` window.

Unlike ``test_rosbag_integration.test.py`` there is NO free-running publisher
process here: the test class owns the only publisher on the captured topic, so
going silent before a report pins the empty-buffer boundary state
deterministically (the ``_wait_for_buffered_data`` guards used elsewhere exist
to dodge exactly the state these tests need to hit).

Parametrized over both storage formats so zero-message finalization is proven
on sqlite3 AND mcap.
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
from ros2_medkit_msgs.srv import ClearFault, GetRosbag, ReportFault
from std_msgs.msg import Float32


# The issue's configuration: 2s pre-fault buffer, 0.5s post-fault window.
DURATION_SEC = 2.0
DURATION_AFTER_SEC = 0.5
PROBE_TOPIC = '/boundary/telemetry'


def post_only_log_line(fault_code):
    """
    Return the log line the fault manager emits on the boundary path.

    It carries the fault code because ``assertWaitFor`` scans everything
    received so far: a code-less pattern would also match the line some earlier
    fault logged, and the wait would return on a stale match.
    """
    return f"fault '{fault_code}' - recording post-fault window only"


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


@launch_testing.parametrize('storage_format', ['sqlite3', 'mcap'])
def generate_test_description(storage_format):
    """Launch the fault_manager with rosbag capture on an explicit test topic."""
    storage_path = tempfile.mkdtemp(prefix=f'rosbag_boundary_{storage_format}_')

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
            'snapshots.rosbag.enabled': True,
            'snapshots.rosbag.duration_sec': DURATION_SEC,
            'snapshots.rosbag.duration_after_sec': DURATION_AFTER_SEC,
            # Explicit topic owned by the test class - no other publisher can
            # refill the buffer behind the test's back.
            'snapshots.rosbag.topics': PROBE_TOPIC,
            'snapshots.rosbag.format': storage_format,
            'snapshots.rosbag.storage_path': storage_path,
            'snapshots.rosbag.max_bag_size_mb': 10,
            'snapshots.rosbag.max_total_storage_mb': 50,
            'snapshots.rosbag.auto_cleanup': True,
            'snapshots.rosbag.lazy_start': False,
        }],
        # Room to flush coverage data at shutdown before SIGKILL.
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
            'storage_format': storage_format,
            'rosbag_storage_path': storage_path,
        },
    )


class TestRosbagBoundary(unittest.TestCase):
    """Post-fault window boundary tests."""

    @classmethod
    def setUpClass(cls):
        """Create the probe publisher and service clients."""
        os.environ['ROS_LOCALHOST_ONLY'] = '1'
        rclpy.init()
        cls.node = Node('boundary_test_driver')

        # Publisher must exist early: the fault_manager's explicit-topic rosbag
        # capture retries type discovery for only ~10s after startup.
        cls.probe_pub = cls.node.create_publisher(Float32, PROBE_TOPIC, 10)

        cls.report_fault_client = cls.node.create_client(
            ReportFault, '/fault_manager/report_fault'
        )
        cls.get_rosbag_client = cls.node.create_client(
            GetRosbag, '/fault_manager/get_rosbag'
        )
        cls.clear_fault_client = cls.node.create_client(
            ClearFault, '/fault_manager/clear_fault'
        )
        assert cls.report_fault_client.wait_for_service(timeout_sec=15.0), \
            'report_fault service not available'
        assert cls.get_rosbag_client.wait_for_service(timeout_sec=15.0), \
            'get_rosbag service not available'
        assert cls.clear_fault_client.wait_for_service(timeout_sec=15.0), \
            'clear_fault service not available'

        # Wait until the fault_manager's capture subscription on the probe
        # topic exists; before that, published messages are never buffered.
        deadline = time.time() + 15.0
        while (not cls.node.get_subscriptions_info_by_topic(PROBE_TOPIC)
               and time.time() < deadline):
            time.sleep(0.2)
        assert cls.node.get_subscriptions_info_by_topic(PROBE_TOPIC), \
            'fault_manager never subscribed to the probe topic'

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _call_service(self, client, request, timeout_sec=10.0):
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout_sec)
        self.assertIsNotNone(future.result(), 'Service call timed out')
        return future.result()

    def _report_fault(self, fault_code):
        request = ReportFault.Request()
        request.fault_code = fault_code
        request.event_type = ReportFault.Request.EVENT_FAILED
        request.severity = Fault.SEVERITY_ERROR
        request.description = 'boundary test fault'
        request.source_id = '/boundary_test_driver'
        return self._call_service(self.report_fault_client, request)

    def _clear_fault(self, fault_code):
        request = ClearFault.Request()
        request.fault_code = fault_code
        return self._call_service(self.clear_fault_client, request)

    def _get_rosbag(self, fault_code):
        request = GetRosbag.Request()
        request.fault_code = fault_code
        return self._call_service(self.get_rosbag_client, request)

    def _wait_for_rosbag(self, fault_code, timeout=15.0):
        """Poll GetRosbag until the recording finalises and its row appears."""
        deadline = time.time() + timeout
        response = self._get_rosbag(fault_code)
        while time.time() < deadline:
            if response is not None and response.success:
                return response
            time.sleep(0.3)
            response = self._get_rosbag(fault_code)
        return response

    def _read_bag(self, bag_path, storage_format):
        """
        Return ``(topics, message_count)`` read from the finalised bag itself.

        A row, a non-zero ``size_bytes`` and a downloadable payload are all
        equally true of a bag containing no messages, so reading the bag is the
        only assertion that can tell the two apart.
        """
        import rosbag2_py
        reader = rosbag2_py.SequentialReader()
        reader.open(
            rosbag2_py.StorageOptions(uri=bag_path, storage_id=storage_format),
            rosbag2_py.ConverterOptions('', ''),
        )
        topics = {t.name for t in reader.get_all_topics_and_types()}
        count = 0
        while reader.has_next():
            reader.read_next()
            count += 1
        return topics, count

    def _publish_for(self, duration_sec, rate_hz=20.0):
        """Publish probe messages for *duration_sec*, then go silent."""
        msg = Float32()
        msg.data = 1.0
        deadline = time.time() + duration_sec
        period = 1.0 / rate_hz
        while time.time() < deadline:
            self.probe_pub.publish(msg)
            time.sleep(period)

    # ------------------------------------------------------------------
    # Tests
    # ------------------------------------------------------------------

    def test_01_boundary_fault_gets_post_fault_only_bag(self, proc_output, storage_format):
        """
        A fault confirmed right after the previous window closes gets a bag.

        Also pins duration_sec honesty: the post-only row reports ~the
        post-fault window; the full row reports what it really holds.

        @verifies REQ_INTEROP_088
        """
        # Fill the ring buffer, then go silent BEFORE reporting so no message
        # can slip into the buffer between A's finalize and B's flush.
        self._publish_for(DURATION_SEC + 0.5)

        response = self._report_fault('BOUNDARY_A')
        self.assertTrue(response.accepted)
        bag_a = self._wait_for_rosbag('BOUNDARY_A')
        self.assertTrue(bag_a.success,
                        f'fault A must get a full bag: {bag_a.error_message}')

        # A's row appearing means its post-roll finalised. The buffer is empty
        # by construction (drained by the flush, post-roll diverted the rest,
        # nothing published since). Report B immediately - the boundary case.
        response = self._report_fault('BOUNDARY_B')
        self.assertTrue(response.accepted)

        # Wait until the post-fault-only recording opens, then resume
        # publishing so B's bag has content during its window.
        proc_output.assertWaitFor(
            expected_output=post_only_log_line('BOUNDARY_B'), timeout=10.0,
        )
        self._publish_for(1.0)

        bag_b = self._wait_for_rosbag('BOUNDARY_B')
        self.assertTrue(
            bag_b.success,
            'a fault confirmed right after the post-fault window closed must '
            f'get its own post-fault-only bag: {bag_b.error_message}')
        self.assertNotEqual(bag_b.file_path, bag_a.file_path,
                            'the boundary fault opens its own recording')
        self.assertTrue(os.path.exists(bag_b.file_path))
        self.assertGreater(bag_b.size_bytes, 0)

        # The claim the whole slice exists for: B's bag holds B's post-fault
        # window. size_bytes and a served download are true of an empty bag too
        # (test_02 produces exactly such a bag), so read the bag itself.
        topics, count = self._read_bag(bag_b.file_path, storage_format)
        self.assertIn(
            PROBE_TOPIC, topics,
            'the post-fault-only bag recorded no messages on the captured '
            'topic - an empty black box is the failure this fixes')
        self.assertGreater(
            count, 0,
            'the post-fault-only bag finalised empty: its window was not recorded')

        # duration_sec honesty (tolerance, not equality): the post-only bag
        # spans ~duration_after_sec plus executor lag - far below the old
        # hardcoded pre+post (2.5s).
        self.assertGreaterEqual(bag_b.duration_sec, 0.3)
        self.assertLessEqual(
            bag_b.duration_sec, 1.5,
            'post-fault-only duration must reflect actual content, '
            'not configured pre+post')

        # The full bag reports the real span of what it holds, which is NOT
        # bounded by duration_sec + duration_after_sec: the ring buffer is
        # pruned only when a message arrives, so once this test goes silent
        # its last ~2.5s of history stays buffered and is flushed in full,
        # and the recording then stays open across the confirm-to-flush delay
        # and the post-fault window. Deliberate - a black box must keep the
        # final messages of a topic that stopped publishing. What matters is
        # that a bag with pre-fault history spans materially more than a
        # post-fault-only one.
        self.assertGreaterEqual(
            bag_a.duration_sec, DURATION_SEC,
            'the full bag must span at least the pre-fault history it holds')
        self.assertGreater(
            bag_a.duration_sec, 2 * bag_b.duration_sec,
            'a bag with pre-fault history must span materially more than a '
            'post-fault-only bag')

    def test_02_zero_message_boundary_bag_finalizes(self, storage_format):
        """
        Publishers silent for the whole window: zero-message bag, readable.

        The recording closes with no messages at all; it must still finalise
        cleanly (metadata row + bag on disk) and GetRosbag must keep serving
        it - on both storage formats.

        @verifies REQ_INTEROP_088
        """
        self._publish_for(DURATION_SEC + 0.5)

        response = self._report_fault('ZERO_A')
        self.assertTrue(response.accepted)
        bag_a = self._wait_for_rosbag('ZERO_A')
        self.assertTrue(bag_a.success,
                        f'fault A must get a full bag: {bag_a.error_message}')

        # Boundary fault with publishers silent for the WHOLE post-roll.
        response = self._report_fault('ZERO_B')
        self.assertTrue(response.accepted)

        bag_b = self._wait_for_rosbag('ZERO_B')
        self.assertTrue(
            bag_b.success,
            'a zero-message post-fault-only bag must still finalise and be '
            f'served: {bag_b.error_message}')
        self.assertEqual(bag_b.format, storage_format)
        self.assertTrue(os.path.isdir(bag_b.file_path))

        # The finalized bag directory must hold metadata.yaml plus the inner
        # data file the gateway's bulk-data download resolves.
        contents = os.listdir(bag_b.file_path)
        self.assertIn('metadata.yaml', contents)
        inner = [f for f in contents if f.endswith('.db3') or f.endswith('.mcap')]
        self.assertTrue(inner,
                        f'no finalized storage file in zero-message bag: {contents}')

        # This bag is supposed to be the empty one - pin that, so the content
        # assertion in test_01 is known to discriminate rather than to be
        # trivially true of every bag this suite produces.
        _, count = self._read_bag(bag_b.file_path, storage_format)
        self.assertEqual(count, 0, 'the silent window should have recorded nothing')

        # It still reports the span the RECORDING was open, not a content span:
        # a window during which nothing was published is still a window covered.
        self.assertGreaterEqual(bag_b.duration_sec, 0.3)
        self.assertLessEqual(bag_b.duration_sec, 1.5)

        # GetRosbag re-checks the file on disk and deletes stale rows;
        # succeeding again proves the reader tolerates the empty bag.
        again = self._get_rosbag('ZERO_B')
        self.assertTrue(again.success)


@launch_testing.post_shutdown_test()
class TestRosbagBoundaryShutdown(unittest.TestCase):
    """Post-shutdown checks."""

    def test_exit_code(self, proc_info):
        """Verify fault_manager exits cleanly."""
        launch_testing.asserts.assertExitCodes(
            proc_info,
            process='fault_manager_node'
        )

    def test_cleanup_storage_directory(self, rosbag_storage_path):
        """Remove this parametrization's bag storage directory."""
        import shutil
        if os.path.exists(rosbag_storage_path):
            shutil.rmtree(rosbag_storage_path, ignore_errors=True)
