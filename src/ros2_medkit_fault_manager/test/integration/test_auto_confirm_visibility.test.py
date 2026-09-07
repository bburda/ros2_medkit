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
A time-based confirmation must be as visible as a reported one.

auto_confirm_after_sec promotes a fault that has sat in PREFAILED past its
window. The SSE fault feed the gateway serves, the trigger subscribers and the
per-entity freeze frames all key off the published event, and black-box capture
is enqueued alongside it. A promotion that only reaches the database is an alarm
nobody is told about and no recording is made for, so this pins both halves.
"""

import json
import os
import shutil
import tempfile
import threading
import time
import unittest

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
import launch_ros.actions
import launch_testing.actions
import launch_testing.markers
import rclpy
from rclpy.node import Node
from ros2_medkit_msgs.msg import Fault, FaultEvent
from ros2_medkit_msgs.srv import GetSnapshots, ListFaults, ReportFault
from sensor_msgs.msg import Temperature

# Below -1 so the first FAILED lands in PREFAILED and only the timer can promote it.
CONFIRMATION_THRESHOLD = -2
AUTO_CONFIRM_SEC = 3.0

# The node runs the promotion timer once a second, so a confirmation lands
# within AUTO_CONFIRM_SEC + 1 of the last FAILED under no load. The waits below
# are generous on top of that because CI runs this under ASan and TSan; they
# bound the wait, they do not define the contract.
CONFIRM_WAIT_SEC = AUTO_CONFIRM_SEC + 12.0

# Mapped to /test/temperature by test_snapshots.yaml, so a confirmation on this
# code is expected to capture that topic.
SNAPSHOT_FAULT_CODE = 'TEST_SNAPSHOT_FAULT'

EVENT_TOPIC = '/fault_manager/events'

_temp_dirs = []


@launch_testing.markers.keep_alive
def generate_test_description():
    """Launch fault_manager with time-based confirmation and snapshots on."""
    temp_dir = tempfile.mkdtemp(prefix='auto_confirm_visibility_')
    _temp_dirs.append(temp_dir)

    pkg_share = get_package_share_directory('ros2_medkit_fault_manager')
    snapshot_config = os.path.join(pkg_share, 'test', 'test_snapshots.yaml')
    correlation_config = os.path.join(pkg_share, 'test', 'test_correlation.yaml')

    fault_manager_node = launch_ros.actions.Node(
        package='ros2_medkit_fault_manager',
        executable='fault_manager_node',
        name='fault_manager',
        output='screen',
        parameters=[{
            'storage_type': 'sqlite',
            'database_path': os.path.join(temp_dir, 'faults.db'),
            'confirmation_threshold': CONFIRMATION_THRESHOLD,
            'auto_confirm_after_sec': AUTO_CONFIRM_SEC,
            'snapshots.enabled': True,
            'snapshots.config_file': snapshot_config,
            'snapshots.timeout_sec': 3.0,
            'snapshots.background_capture': False,
            # Carries a hierarchical rule with mute_symptoms, so a symptom of
            # ESTOP_001 is muted while the timer still confirms it.
            'correlation.config_file': correlation_config,
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
        },
    )


class TestAutoConfirmVisibility(unittest.TestCase):
    """A timer-driven confirmation must publish an event and capture evidence."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        try:
            cls.node = Node('test_auto_confirm_client')
            cls.report_client = cls.node.create_client(ReportFault, '/fault_manager/report_fault')
            cls.snapshot_client = cls.node.create_client(
                GetSnapshots, '/fault_manager/get_snapshots'
            )
            cls.list_client = cls.node.create_client(ListFaults, '/fault_manager/list_faults')
            cls.temp_publisher = cls.node.create_publisher(Temperature, '/test/temperature', 10)

            cls.events = []
            cls.events_lock = threading.Lock()
            cls.event_sub = cls.node.create_subscription(
                FaultEvent, EVENT_TOPIC, cls._on_event, 100
            )

            assert cls.report_client.wait_for_service(timeout_sec=30.0), \
                'report_fault service not available'
            assert cls.snapshot_client.wait_for_service(timeout_sec=30.0), \
                'get_snapshots service not available'
            assert cls.list_client.wait_for_service(timeout_sec=30.0), \
                'list_faults service not available'

            # The publisher uses volatile durability, so an event sent before the
            # subscription matches is lost. Without this wait an assertion that
            # no event arrived could pass because nothing was listening yet.
            # Node.count_publishers rather than Subscription.get_publisher_count:
            # the latter does not exist in rclpy on every supported distro.
            deadline = time.monotonic() + 30.0
            while time.monotonic() < deadline and cls.node.count_publishers(EVENT_TOPIC) == 0:
                rclpy.spin_once(cls.node, timeout_sec=0.1)
            assert cls.node.count_publishers(EVENT_TOPIC) > 0, \
                'fault event publisher never matched; absence assertions would be meaningless'
        except Exception:
            rclpy.shutdown()
            raise

    @classmethod
    def _on_event(cls, msg):
        with cls.events_lock:
            cls.events.append(msg)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def _events_for(self, fault_code):
        with self.events_lock:
            return [e for e in self.events if e.fault.fault_code == fault_code]

    def _spin_for(self, seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.1)

    def _wait_for_confirmed_event(self, fault_code, timeout_sec):
        """Spin until a confirmation for this code arrives, returning it or None."""
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            for event in self._events_for(fault_code):
                if event.event_type == FaultEvent.EVENT_CONFIRMED:
                    return event
            rclpy.spin_once(self.node, timeout_sec=0.1)
        return None

    def _report_failed(self, fault_code, severity=Fault.SEVERITY_ERROR):
        request = ReportFault.Request()
        request.fault_code = fault_code
        request.event_type = ReportFault.Request.EVENT_FAILED
        request.severity = severity
        request.description = 'auto-confirm visibility test'
        request.source_id = '/test_plc'
        future = self.report_client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=30.0)
        self.assertIsNotNone(future.result(), 'ReportFault timed out')
        self.assertTrue(future.result().accepted)

    def test_01_timer_confirmation_publishes_a_confirmed_event(self):
        """
        The one event a subscriber has to see.

        A single FAILED leaves the fault in PREFAILED, and the reporter never
        sends a second one, so the timer is what confirms it. If that promotion
        does not publish, the SSE feed and the trigger subscribers stay silent
        for a fault that is now confirmed in the database.
        """
        code = 'PLC_TIMER_CONFIRMED'
        raised_at = time.monotonic()
        self._report_failed(code)

        # The raise alone must not confirm, or the timer is not what is measured.
        self._spin_for(0.5)
        self.assertNotIn(
            FaultEvent.EVENT_CONFIRMED, [e.event_type for e in self._events_for(code)],
            'the fault confirmed on its first failed report'
        )

        event = self._wait_for_confirmed_event(code, CONFIRM_WAIT_SEC)
        self.assertIsNotNone(
            event, 'a time-based confirmation published no event, so nothing downstream is told'
        )
        self.assertGreaterEqual(
            time.monotonic() - raised_at, AUTO_CONFIRM_SEC,
            'the confirmation arrived before the configured window had elapsed'
        )

    def test_02_the_event_carries_the_confirmed_fault(self):
        """The event must carry the fault, not just its code."""
        code = 'PLC_TIMER_PAYLOAD'
        self._report_failed(code)

        event = self._wait_for_confirmed_event(code, CONFIRM_WAIT_SEC)
        self.assertIsNotNone(event, 'no confirmation event arrived')
        self.assertEqual(event.fault.fault_code, code)
        self.assertEqual(
            event.fault.status, Fault.STATUS_CONFIRMED,
            'the event carries a fault that is not confirmed'
        )
        self.assertEqual(event.fault.severity, Fault.SEVERITY_ERROR)
        self.assertIn('/test_plc', event.fault.reporting_sources)

    def test_03_timer_confirmation_captures_a_snapshot(self):
        """
        The other half of the fix, and the one an event assertion cannot reach.

        Confirmation is what triggers black-box capture. A promotion that
        publishes but never enqueues capture leaves an alarm with no evidence
        behind it, which is what the operator opens the fault to look for.

        @verifies REQ_INTEROP_088
        """
        temp_msg = Temperature()
        temp_msg.temperature = 85.5
        temp_msg.variance = 0.1

        stop_publishing = threading.Event()

        def keep_publishing():
            while not stop_publishing.is_set():
                self.temp_publisher.publish(temp_msg)
                time.sleep(0.05)

        pub_thread = threading.Thread(target=keep_publishing)
        pub_thread.start()
        try:
            self._report_failed(SNAPSHOT_FAULT_CODE)
            event = self._wait_for_confirmed_event(SNAPSHOT_FAULT_CODE, CONFIRM_WAIT_SEC)
            self.assertIsNotNone(event, 'no confirmation event arrived')

            snapshot = self._wait_for_snapshot(SNAPSHOT_FAULT_CODE, timeout_sec=20.0)
        finally:
            stop_publishing.set()
            pub_thread.join()

        self.assertIsNotNone(
            snapshot,
            'a time-based confirmation captured no snapshot, so the alarm has no evidence'
        )
        parsed = json.loads(snapshot)
        self.assertEqual(parsed['fault_code'], SNAPSHOT_FAULT_CODE)
        self.assertIn(
            '/test/temperature', parsed['topics'],
            f'the configured topic was not captured, only {list(parsed["topics"])}'
        )

    def test_04_a_muted_symptom_is_confirmed_but_not_announced(self):
        """
        The gate the report path has and the timer used to be missing.

        Correlation mutes a symptom so that only its root cause is announced.
        The report path wraps every confirmation publish in that check; the
        timer did not, so a symptom promoted by the timer reached subscribers
        while the default fault list still hid it. The symptom must still
        confirm in the store: it is the announcement that is suppressed, not
        the confirmation.
        """
        root = 'ESTOP_001'
        symptom = 'MOTOR_COMM_1'

        # Root first, then the symptom inside the rule's window, so correlation
        # sees the second as caused by the first.
        self._report_failed(root)
        self._report_failed(symptom)

        root_event = self._wait_for_confirmed_event(root, CONFIRM_WAIT_SEC)
        self.assertIsNotNone(root_event, 'the root cause was never announced')

        # Give the timer a full further window: if the symptom were going to be
        # announced, it would have been by now.
        self._spin_for(AUTO_CONFIRM_SEC + 2.0)
        announced = [
            e for e in self._events_for(symptom) if e.event_type == FaultEvent.EVENT_CONFIRMED
        ]
        self.assertEqual(
            [], announced,
            'a muted symptom was announced as confirmed, which is what muting exists to prevent'
        )

        request = ListFaults.Request()
        request.statuses = ['CONFIRMED']
        request.include_muted = True
        future = self.list_client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=30.0)
        self.assertIsNotNone(future.result(), 'ListFaults timed out')
        confirmed = [f.fault_code for f in future.result().faults]
        self.assertIn(
            symptom, confirmed,
            'the symptom never confirmed, so this run does not show that muting is what '
            'suppressed the event'
        )

    def _wait_for_snapshot(self, fault_code, timeout_sec):
        """Poll GetSnapshots until one exists for this fault, or give up."""
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            request = GetSnapshots.Request()
            request.fault_code = fault_code
            request.topic = ''
            future = self.snapshot_client.call_async(request)
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=10.0)
            result = future.result()
            if result is not None and result.success and result.data:
                # success with an empty topics map is what the service answers
                # when the fault exists but nothing was ever captured for it, so
                # the payload has to be inspected rather than merely present.
                parsed = json.loads(result.data)
                if parsed.get('topics'):
                    return result.data
            rclpy.spin_once(self.node, timeout_sec=0.2)
        return None


@launch_testing.post_shutdown_test()
class TestAutoConfirmVisibilityShutdown(unittest.TestCase):
    """Check the node exited cleanly and the databases are gone."""

    def test_exit_code(self, proc_info, fault_manager_node):
        launch_testing.asserts.assertExitCodes(proc_info, process=fault_manager_node)

    def test_temp_dirs_removed(self):
        # Idempotent: the parametrized suite runs this once per launch and the
        # list is module-level, so a directory may already be gone. The contract
        # is the end state, not that this call did the removing.
        for path in _temp_dirs:
            shutil.rmtree(path, ignore_errors=True)
            self.assertFalse(os.path.exists(path), f'{path} survived cleanup')
