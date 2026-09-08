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
A planned stop declared before a restart is still in force after it.

The subject IS the restart, so the fault manager runs with ``respawn`` and the
test kills it by PID: a weekend stop must not end because the box rebooted.
Runs on SQLite, the shipped default and the only backend with a file behind it.
"""

import os
import signal
import tempfile
import threading
import time
import unittest

from launch import LaunchDescription
import launch.actions
import launch_ros.actions
import launch_testing.actions
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from ros2_medkit_msgs.msg import Fault, FaultEvent
from ros2_medkit_msgs.srv import GetFault, GetPlannedStop, ListFaults, ReportFault, SetPlannedStop

STORAGE_DIR = tempfile.mkdtemp(prefix='planned_stop_restart_')
DATABASE_PATH = os.path.join(STORAGE_DIR, 'faults.db')
CORRELATION_PATH = os.path.join(STORAGE_DIR, 'correlation.yaml')

# A rule whose mute is NOT persisted anywhere, so a restart is where the
# difference between "the stop owns this cycle" and "a rule is muting it" shows.
CORRELATION_RULES = """
correlation:
  enabled: true
  patterns:
    restart_motor_errors:
      codes: ["MOTOR_RESTART_*"]
  rules:
    - id: restart_estop_cascade
      mode: hierarchical
      root_cause:
        codes: ["ESTOP_RESTART_001"]
      symptoms:
        - pattern: restart_motor_errors
      window_ms: 60000
      mute_symptoms: true
      auto_clear_with_root: false
"""

RULE_ROOT_CODE = 'ESTOP_RESTART_001'
RULE_SYMPTOM_CODE = 'MOTOR_RESTART_LEFT'

REASON = 'weekend shutdown, cell 4'
DECLARED_BY = 'plant_manager'
BEFORE_CODE = 'PS_RESTART_BEFORE'
SECOND_BEFORE_CODE = 'PS_RESTART_BEFORE_TWO'
AFTER_CODE = 'PS_RESTART_AFTER'


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
        # Coverage environment is optional; on any error, fall back to no extra config
        pass
    return {}


def generate_test_description():
    """Launch a fault manager that launch brings back after it is killed."""
    with open(CORRELATION_PATH, 'w') as handle:
        handle.write(CORRELATION_RULES)

    fault_manager_env = get_coverage_env()
    fault_manager_env['ROS_LOCALHOST_ONLY'] = '1'

    fault_manager_node = launch_ros.actions.Node(
        package='ros2_medkit_fault_manager',
        executable='fault_manager_node',
        name='fault_manager',
        output='screen',
        additional_env=fault_manager_env,
        parameters=[{
            'storage_type': 'sqlite',
            'database_path': DATABASE_PATH,
            'confirmation_threshold': -1,
            'correlation.config_file': CORRELATION_PATH,
            'snapshots.enabled': False,
            'snapshots.rosbag.enabled': False,
        }],
        # The restart is the subject. The delay gives the DDS participant time to
        # go before its replacement appears, so the test sees a real gap rather
        # than two managers on one domain.
        respawn=True,
        respawn_delay=1.0,
        sigterm_timeout='30',
        sigkill_timeout='15',
    )

    return (
        LaunchDescription([
            launch.actions.TimerAction(period=2.0, actions=[fault_manager_node]),
            launch_testing.actions.ReadyToTest(),
        ]),
        {'fault_manager_node': fault_manager_node},
    )


class TestPlannedStopSurvivesRestart(unittest.TestCase):
    """The declaration outlives the process that made it."""

    @classmethod
    def setUpClass(cls):
        os.environ['ROS_LOCALHOST_ONLY'] = '1'
        rclpy.init()
        cls.node = Node('test_planned_stop_restart_client')

        # The subscription outlives the manager it listens to and rematches the
        # replacement's publisher, which is what lets one test span the restart.
        cls._events = []
        cls._events_lock = threading.Lock()
        cls._event_sub = cls.node.create_subscription(
            FaultEvent, '/fault_manager/events', cls._on_event,
            QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                       history=HistoryPolicy.KEEP_LAST, depth=200))
        cls._connect()

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    @classmethod
    def _on_event(cls, msg):
        with cls._events_lock:
            cls._events.append((msg.event_type, msg.fault.fault_code))

    def _count_events(self, fault_code, event_type):
        with self._events_lock:
            return sum(1 for kind, code in self._events
                       if code == fault_code and kind == event_type)

    @classmethod
    def _connect(cls, timeout_sec=30.0):
        """Create clients and wait for the manager's services to answer."""
        cls.report_client = cls.node.create_client(ReportFault, '/fault_manager/report_fault')
        cls.list_client = cls.node.create_client(ListFaults, '/fault_manager/list_faults')
        cls.get_client = cls.node.create_client(GetFault, '/fault_manager/get_fault')
        cls.set_stop_client = cls.node.create_client(
            SetPlannedStop, '/fault_manager/set_planned_stop')
        cls.get_stop_client = cls.node.create_client(
            GetPlannedStop, '/fault_manager/get_planned_stop')
        for client, name in (
            (cls.report_client, 'report_fault'),
            (cls.list_client, 'list_faults'),
            (cls.get_client, 'get_fault'),
            (cls.set_stop_client, 'set_planned_stop'),
            (cls.get_stop_client, 'get_planned_stop'),
        ):
            assert client.wait_for_service(timeout_sec=timeout_sec), \
                f'{name} service not available'

    def _call(self, client, request, timeout_sec=15.0):
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout_sec)
        self.assertIsNotNone(future.result(), 'service call timed out')
        return future.result()

    def _spin_for(self, seconds):
        """Pump the executor so published events reach the subscription."""
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def _wait_until(self, predicate, message, timeout=30.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if predicate():
                return
            rclpy.spin_once(self.node, timeout_sec=0.05)
        raise AssertionError(message)

    def _report(self, fault_code):
        request = ReportFault.Request()
        request.fault_code = fault_code
        request.event_type = ReportFault.Request.EVENT_FAILED
        request.severity = Fault.SEVERITY_ERROR
        request.description = 'restart test fault'
        request.source_id = '/test_node'
        return self._call(self.report_client, request)

    def _set_stop(self, active, *, reason='', declared_by=''):
        request = SetPlannedStop.Request()
        request.active = active
        request.reason = reason
        request.declared_by = declared_by
        return self._call(self.set_stop_client, request)

    def _get_stop(self):
        return self._call(self.get_stop_client, GetPlannedStop.Request())

    def _muted_codes(self):
        request = ListFaults.Request()
        request.filter_by_severity = False
        request.severity = 0
        request.statuses = []
        request.include_muted = True
        request.include_clusters = False
        response = self._call(self.list_client, request)
        return {info.fault_code: info for info in response.muted_faults}

    def _confirmed_codes(self):
        request = ListFaults.Request()
        request.filter_by_severity = False
        request.severity = 0
        request.statuses = []
        request.include_muted = False
        request.include_clusters = False
        return {fault.fault_code for fault in self._call(self.list_client, request).faults}

    def test_a_stop_declared_before_a_restart_still_mutes_after_it(self, fault_manager_node):
        # A rule-muted fault whose cycle starts BEFORE the stop. Its mute lives only
        # in the engine, so the restart is where the stop could wrongly adopt it.
        self._report(RULE_ROOT_CODE)
        self._report(RULE_SYMPTOM_CODE)
        self._wait_until(lambda: RULE_SYMPTOM_CODE in self._muted_codes(),
                         f'{RULE_SYMPTOM_CODE} was never muted by the rule')
        self.assertEqual('restart_estop_cascade', self._muted_codes()[RULE_SYMPTOM_CODE].rule_id)

        declared = self._set_stop(True, reason=REASON, declared_by=DECLARED_BY)
        self.assertTrue(declared.success)
        self.assertFalse(declared.was_active)

        before = self._get_stop()
        self.assertTrue(before.active)
        self.assertEqual(REASON, before.reason)

        self._report(BEFORE_CODE)
        self._report(SECOND_BEFORE_CODE)
        muted_before = self._muted_codes()
        self.assertIn(BEFORE_CODE, muted_before)
        self.assertIn(SECOND_BEFORE_CODE, muted_before)
        for code in (BEFORE_CODE, SECOND_BEFORE_CODE):
            self.assertEqual(0, self._count_events(code, FaultEvent.EVENT_CONFIRMED),
                             'a fault the stop marked was announced anyway')

        original_pid = fault_manager_node.process_details['pid']
        os.kill(original_pid, signal.SIGTERM)

        deadline = time.monotonic() + 60.0
        while time.monotonic() < deadline:
            current = fault_manager_node.process_details
            if current is not None and current['pid'] != original_pid:
                break
            time.sleep(0.2)
        self.assertNotEqual(
            original_pid, fault_manager_node.process_details['pid'],
            'the fault manager was never restarted, so nothing below tests a restart')

        self._connect(timeout_sec=60.0)

        after = self._get_stop()
        self.assertTrue(after.active, 'the planned stop did not survive the restart')
        self.assertEqual(REASON, after.reason)
        self.assertEqual(DECLARED_BY, after.declared_by)
        self.assertEqual(before.since.sec, after.since.sec,
                         'the restart moved the time the stop started')
        self.assertEqual(before.since.nanosec, after.since.nanosec)

        # Exactly what the stop owned comes back - nothing else. Asserted before
        # anything new is reported, so a fault raised after the restart cannot
        # stand in for one the restore was supposed to find.
        restored = self._muted_codes()
        self.assertEqual(
            {BEFORE_CODE, SECOND_BEFORE_CODE}, set(restored),
            'the restart restored something other than exactly the faults the stop owned')
        # The rule's mute is not persisted, so it is gone after the restart - and the
        # stop must not have adopted a cycle that started before it was declared.
        self.assertNotIn(RULE_SYMPTOM_CODE, restored)
        self.assertIn(RULE_SYMPTOM_CODE, self._confirmed_codes())

        # The declaration is not a museum piece: a fault reported after the
        # restart is muted by it.
        self._report(AFTER_CODE)
        muted = self._muted_codes()
        self.assertIn(AFTER_CODE, muted,
                      'a fault reported after the restart was not muted by the standing stop')
        self.assertEqual('planned_stop', muted[AFTER_CODE].rule_id)
        self.assertNotIn(AFTER_CODE, self._confirmed_codes())

        # The mute set lived in the process that is gone, so the manager rebuilds it
        # from the store: the faults marked before the restart are still marked.
        for code in (BEFORE_CODE, SECOND_BEFORE_CODE):
            self.assertIn(code, restored,
                          f'{code} was silently released by the restart, so its '
                          'confirmation could never be announced')
            self.assertEqual('planned_stop', restored[code].rule_id)
            self.assertNotIn(code, self._confirmed_codes())

        withdrawn = self._set_stop(False, reason='plant back up', declared_by=DECLARED_BY)
        self.assertTrue(withdrawn.success)
        self.assertTrue(withdrawn.was_active)
        final_state = self._get_stop()
        self.assertFalse(final_state.active)
        self.assertEqual(REASON, final_state.reason)
        self.assertGreater(final_state.ended_at.sec, 0)
        self.assertIn(AFTER_CODE, self._confirmed_codes())

        # Every fault the stop was holding - across the restart - is released and
        # announced exactly once.
        for code in (BEFORE_CODE, SECOND_BEFORE_CODE, AFTER_CODE):
            self._wait_until(
                lambda code=code: self._count_events(code, FaultEvent.EVENT_CONFIRMED) == 1,
                f'{code} was never announced at the switch-off')
            self.assertIn(code, self._confirmed_codes())

        self.assertEqual(
            0, self._count_events(RULE_SYMPTOM_CODE, FaultEvent.EVENT_CONFIRMED),
            'the switch-off announced a fault the stop never owned')

        # A second frame arriving late would still be a double announcement.
        self._spin_for(3.0)
        for code in (BEFORE_CODE, SECOND_BEFORE_CODE, AFTER_CODE):
            self.assertEqual(1, self._count_events(code, FaultEvent.EVENT_CONFIRMED),
                             f'{code} was announced more than once')


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        for info in proc_info:
            self.assertIn(info.returncode, (0, -2, -15),
                          f'{info.process_name} exited with code {info.returncode}')
