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
End-to-end tests for the planned-stop switch on a live fault manager.

Drives the real services over the real ROS graph against the SQLite backend
with the audit log on, and reads the audit database directly, because the
manager exposes no service that reads it back.

The instrument for "the stream stayed quiet" is a subscriber on the manager's
own ``~/events`` topic, counted PER FAULT CODE: a count alone would pass while
the wrong fault was announced.
"""

import json
import os
import sqlite3
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
from ros2_medkit_msgs.srv import (
    ClearFault,
    GetFault,
    GetPlannedStop,
    GetSnapshots,
    ListFaults,
    ReportFault,
    SetPlannedStop,
)
from sensor_msgs.msg import Temperature

STORAGE_DIR = tempfile.mkdtemp(prefix='planned_stop_')
DATABASE_PATH = os.path.join(STORAGE_DIR, 'faults.db')
AUDIT_PATH = os.path.join(STORAGE_DIR, 'fault_audit.db')
CORRELATION_PATH = os.path.join(STORAGE_DIR, 'correlation.yaml')

# Two FAILED reports confirm, one leaves the fault PREFAILED, which is what the
# mid-debounce case needs.
CONFIRMATION_THRESHOLD = -2

SOURCE_ID = '/test_node'
CAPTURE_TOPIC = '/test/temperature'

# Two rules. The hierarchical one mutes MOTOR_PS_* while ESTOP_PS_001 stands; the
# auto-cluster one claims VALVE_PS_* faults without ever writing them into the mute
# map, which is the case a planned stop's own entry has to survive. Both windows are
# long enough that no test can lose the correlation to its own pacing.
CORRELATION_RULES = """
correlation:
  enabled: true
  patterns:
    ps_motor_errors:
      codes: ["MOTOR_PS_*"]
    ps_valve_errors:
      codes: ["VALVE_PS_*"]
  rules:
    - id: ps_estop_cascade
      name: "E-Stop Cascade"
      mode: hierarchical
      root_cause:
        codes: ["ESTOP_PS_001"]
      symptoms:
        - pattern: ps_motor_errors
      window_ms: 60000
      mute_symptoms: true
      auto_clear_with_root: true
    - id: ps_valve_storm
      name: "Valve Storm"
      mode: auto_cluster
      match:
        - pattern: ps_valve_errors
      min_count: 2
      window_ms: 60000
      show_as_single: true
      representative: first
"""


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
    """Launch a fault manager with SQLite storage, the audit log and one rule."""
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
            'confirmation_threshold': CONFIRMATION_THRESHOLD,
            'audit_log.enabled': True,
            'audit_log.database_path': AUDIT_PATH,
            'correlation.config_file': CORRELATION_PATH,
            # Capture must run for a muted fault exactly as it does for any
            # other, so it is on and pointed at a topic this test publishes.
            'snapshots.enabled': True,
            'snapshots.default_topics': [CAPTURE_TOPIC],
            'snapshots.timeout_sec': 2.0,
            'snapshots.recapture_cooldown_sec': 0.0,
            'snapshots.rosbag.enabled': False,
        }],
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


class TestPlannedStop(unittest.TestCase):
    """The switch marks, counts, audits and later releases what it muted."""

    @classmethod
    def setUpClass(cls):
        os.environ['ROS_LOCALHOST_ONLY'] = '1'
        rclpy.init()
        cls.node = Node('test_planned_stop_client')

        cls.report_client = cls.node.create_client(ReportFault, '/fault_manager/report_fault')
        cls.list_client = cls.node.create_client(ListFaults, '/fault_manager/list_faults')
        cls.get_client = cls.node.create_client(GetFault, '/fault_manager/get_fault')
        cls.clear_client = cls.node.create_client(ClearFault, '/fault_manager/clear_fault')
        cls.snapshots_client = cls.node.create_client(GetSnapshots, '/fault_manager/get_snapshots')
        cls.set_stop_client = cls.node.create_client(
            SetPlannedStop, '/fault_manager/set_planned_stop')
        cls.get_stop_client = cls.node.create_client(
            GetPlannedStop, '/fault_manager/get_planned_stop')

        for client, name in (
            (cls.report_client, 'report_fault'),
            (cls.list_client, 'list_faults'),
            (cls.get_client, 'get_fault'),
            (cls.clear_client, 'clear_fault'),
            (cls.snapshots_client, 'get_snapshots'),
            (cls.set_stop_client, 'set_planned_stop'),
            (cls.get_stop_client, 'get_planned_stop'),
        ):
            assert client.wait_for_service(timeout_sec=20.0), f'{name} service not available'

        # A topic for the capture set to sample, so a muted fault's freeze frame
        # is a real reading rather than an empty one.
        cls._temperature_pub = cls.node.create_publisher(
            Temperature, CAPTURE_TOPIC,
            QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                       history=HistoryPolicy.KEEP_LAST, depth=10))
        cls._publish_timer = cls.node.create_timer(0.05, cls._publish_temperature)

        # Deep enough that a switch-off releasing hundreds of faults at once
        # cannot overrun the reader while the spin thread is between wakeups.
        cls._events = []
        cls._events_lock = threading.Lock()
        cls._event_sub = cls.node.create_subscription(
            FaultEvent, '/fault_manager/events', cls._on_event,
            QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                       history=HistoryPolicy.KEEP_LAST, depth=2000))

        cls._spinning = True
        cls._spin_thread = threading.Thread(target=cls._spin, daemon=True)
        cls._spin_thread.start()

        cls._prime_event_stream()

    @classmethod
    def tearDownClass(cls):
        cls._spinning = False
        cls._spin_thread.join(timeout=5)
        cls.node.destroy_node()
        rclpy.shutdown()

    def tearDown(self):
        # The switch is one declaration per manager, so a test that leaves it on
        # would decide the next test's outcome.
        self._set_stop(False, reason='teardown', declared_by='test')

    @classmethod
    def _publish_temperature(cls):
        msg = Temperature()
        msg.temperature = 42.0
        msg.variance = 0.1
        cls._temperature_pub.publish(msg)

    @classmethod
    def _on_event(cls, msg):
        with cls._events_lock:
            cls._events.append((msg.event_type, msg.fault.fault_code, msg.fault.status))

    @classmethod
    def _spin(cls):
        while cls._spinning:
            rclpy.spin_once(cls.node, timeout_sec=0.05)

    @classmethod
    def _call(cls, client, request, timeout_sec=15.0):
        future = client.call_async(request)
        deadline = time.monotonic() + timeout_sec
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.005)
        assert future.done(), 'service call timed out'
        return future.result()

    @classmethod
    def _report(cls, fault_code, event_type=ReportFault.Request.EVENT_FAILED):
        request = ReportFault.Request()
        request.fault_code = fault_code
        request.event_type = event_type
        request.severity = Fault.SEVERITY_ERROR
        request.description = 'planned stop test fault'
        request.source_id = SOURCE_ID
        return cls._call(cls.report_client, request)

    @classmethod
    def _confirm(cls, fault_code):
        """Drive a fault to CONFIRMED with the configured debounce threshold."""
        for _ in range(abs(CONFIRMATION_THRESHOLD)):
            cls._report(fault_code)

    @classmethod
    def _prime_event_stream(cls):
        """
        Block until an event demonstrably reaches this subscriber.

        ``~/events`` is reliable but volatile: an event published before the
        subscription has matched the publisher is lost outright, so a one-shot
        assertion on the first fault of the suite would race the match.
        """
        deadline = time.monotonic() + 30.0
        attempt = 0
        while time.monotonic() < deadline:
            code = f'PS_PRIME_{attempt}'
            cls._confirm(code)
            settle = time.monotonic() + 1.0
            while time.monotonic() < settle:
                if cls._count_events(code, FaultEvent.EVENT_CONFIRMED) > 0:
                    return
                time.sleep(0.05)
            attempt += 1
        raise AssertionError('no event reached the subscriber; the events topic never went live')

    @classmethod
    def _count_events(cls, fault_code, event_type):
        with cls._events_lock:
            return sum(1 for kind, code, _ in cls._events
                       if code == fault_code and kind == event_type)

    @classmethod
    def _event_types(cls, fault_code):
        """
        Return every event type published for this code, as a set.

        Counting one type cannot see a leak of another: a stop that announced an
        UPDATE it should have withheld passes a CONFIRMED-only assertion.
        """
        with cls._events_lock:
            return {kind for kind, code, _ in cls._events if code == fault_code}

    def _set_stop(self, active, *, reason='', declared_by=''):
        request = SetPlannedStop.Request()
        request.active = active
        request.reason = reason
        request.declared_by = declared_by
        return self._call(self.set_stop_client, request)

    def _get_stop(self):
        return self._call(self.get_stop_client, GetPlannedStop.Request())

    def _list(self, *, statuses=None, include_muted=False):
        request = ListFaults.Request()
        request.filter_by_severity = False
        request.severity = 0
        request.statuses = statuses if statuses is not None else []
        request.include_muted = include_muted
        request.include_clusters = False
        return self._call(self.list_client, request)

    def _codes_in_default_list(self):
        return {fault.fault_code for fault in self._list().faults}

    def _muted_entry(self, fault_code):
        response = self._list(include_muted=True)
        for info in response.muted_faults:
            if info.fault_code == fault_code:
                return info
        return None

    def _clear(self, fault_code):
        request = ClearFault.Request()
        request.fault_code = fault_code
        request.skip_correlation_auto_clear = False
        return self._call(self.clear_client, request)

    def _get_fault(self, fault_code):
        request = GetFault.Request()
        request.fault_code = fault_code
        return self._call(self.get_client, request)

    def _captured_topics(self, fault_code):
        """Topic names present in the fault's stored capture, empty when none."""
        request = GetSnapshots.Request()
        request.fault_code = fault_code
        request.topic = ''
        response = self._call(self.snapshots_client, request)
        if not response.success or not response.data:
            return {}
        return json.loads(response.data).get('topics', {})

    @staticmethod
    def _audit_rows(transition=None):
        """Read the audit database directly - no service serves it back."""
        connection = sqlite3.connect(f'file:{AUDIT_PATH}?mode=ro', uri=True)
        try:
            if transition is None:
                cursor = connection.execute(
                    'SELECT fault_code, transition, status, source_id, description FROM audit_log '
                    'ORDER BY seq')
            else:
                cursor = connection.execute(
                    'SELECT fault_code, transition, status, source_id, description FROM audit_log '
                    'WHERE transition = ? ORDER BY seq', (transition,))
            return cursor.fetchall()
        finally:
            connection.close()

    def _wait_until(self, predicate, message, timeout=20.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            value = predicate()
            if value:
                return value
            time.sleep(0.1)
        raise AssertionError(message)

    # ------------------------------------------------------------------
    # Cases
    # ------------------------------------------------------------------

    def test_a_fault_raised_during_a_stop_is_marked_not_dropped(self):
        code = 'PS_MARKED'

        response = self._set_stop(True, reason='line 3 maintenance', declared_by='shift_lead')
        self.assertTrue(response.success)
        self.assertFalse(response.was_active)

        self._confirm(code)

        # In the store, CONFIRMED, and absent from the default list.
        self._wait_until(lambda: self._get_fault(code).success, f'{code} never reached the store')
        self.assertEqual('CONFIRMED', self._get_fault(code).fault.status)
        self.assertNotIn(code, self._codes_in_default_list())

        muted = self._muted_entry(code)
        self.assertIsNotNone(muted, f'{code} is not listed as muted')
        self.assertEqual('planned_stop', muted.rule_id)
        self.assertEqual('PLANNED_STOP', muted.root_cause_code)
        self.assertGreaterEqual(self._list().muted_count, 1)

        # Capture ran anyway: the evidence a muted fault leaves is the same.
        captured = self._wait_until(
            lambda: self._captured_topics(code),
            f'no snapshot was captured for the muted fault {code}')
        self.assertIn(CAPTURE_TOPIC, captured)

        # Re-report the marked fault while the stop still stands. This is the path
        # that produces EVENT_UPDATED, and it is the only way an assertion about
        # "nothing was announced" can see an update leaking out.
        self._report(code)
        self._wait_until(lambda: self._get_fault(code).fault.occurrence_count >= 1,
                         f'{code} left the store')

        # Nothing at all was announced while the stop stood - not the confirmation,
        # and not the update either.
        self.assertEqual(set(), self._event_types(code))

        withdrawal = self._set_stop(False, reason='line 3 back up', declared_by='shift_lead')
        self.assertTrue(withdrawal.success)
        self.assertTrue(withdrawal.was_active)

        self._wait_until(lambda: code in self._codes_in_default_list(),
                         f'{code} did not return to the default list after the stop ended')
        self.assertIsNone(self._muted_entry(code))
        self.assertEqual(
            1, self._count_events(code, FaultEvent.EVENT_CONFIRMED),
            'the confirmation held back by the stop must be announced exactly once')
        self.assertEqual({FaultEvent.EVENT_CONFIRMED}, self._event_types(code),
                         'the switch-off announces the confirmation and nothing else')

    def test_a_fault_cleared_during_a_stop_never_reaches_the_default_list(self):
        code = 'PS_CLEARED_INSIDE'

        self._set_stop(True, reason='changeover', declared_by='maintenance')
        self._confirm(code)
        self._wait_until(lambda: self._get_fault(code).success, f'{code} never reached the store')
        self.assertIsNotNone(self._muted_entry(code))

        cleared = self._clear(code)
        self.assertTrue(cleared.success)

        self.assertNotIn(code, self._codes_in_default_list())
        self.assertIsNone(self._muted_entry(code))

        # It is still in the store, and in the audit trail.
        stored = self._get_fault(code)
        self.assertTrue(stored.success)
        self.assertEqual('CLEARED', stored.fault.status)
        cleared_codes = {fault.fault_code for fault in self._list(statuses=['CLEARED']).faults}
        self.assertIn(code, cleared_codes)
        transitions = {row[1] for row in self._audit_rows() if row[0] == code}
        self.assertIn('occurred', transitions)
        self.assertIn('confirmed', transitions)
        self.assertIn('cleared', transitions)

        # The acknowledgement is published, as it is for any muted fault; the
        # confirmation behind the mute is not. Assert the whole set, so an
        # announcement of any other kind is a failure rather than an unchecked
        # extra frame.
        self.assertEqual({FaultEvent.EVENT_CLEARED}, self._event_types(code))
        self.assertEqual(1, self._count_events(code, FaultEvent.EVENT_CLEARED))

        self._set_stop(False, reason='done', declared_by='maintenance')

        self.assertNotIn(code, self._codes_in_default_list())
        self.assertEqual(
            {FaultEvent.EVENT_CLEARED}, self._event_types(code),
            'a fault that was acknowledged inside the stop has nothing left to announce')

    def test_repeating_a_request_changes_nothing_and_records_nothing(self):
        started_before = len(self._audit_rows('planned_stop_started'))
        ended_before = len(self._audit_rows('planned_stop_ended'))

        first_on = self._set_stop(True, reason='weekend shutdown', declared_by='plant_manager')
        self.assertTrue(first_on.success)
        self.assertFalse(first_on.was_active)

        second_on = self._set_stop(True, reason='ignored', declared_by='someone_else')
        self.assertTrue(second_on.success)
        self.assertTrue(second_on.was_active)

        started = self._audit_rows('planned_stop_started')
        self.assertEqual(started_before + 1, len(started),
                         'declaring a stop that is already in force must write no audit record')
        self.assertEqual('__audit__', started[-1][0])
        self.assertEqual('ACTIVE', started[-1][2])
        self.assertEqual('plant_manager', started[-1][3])
        self.assertEqual('weekend shutdown', started[-1][4])

        # The declaration the manager serves is the first one, not the repeat.
        state = self._get_stop()
        self.assertTrue(state.active)
        self.assertEqual('weekend shutdown', state.reason)
        self.assertEqual('plant_manager', state.declared_by)

        first_off = self._set_stop(False, reason='restart', declared_by='plant_manager')
        self.assertTrue(first_off.success)
        self.assertTrue(first_off.was_active)

        second_off = self._set_stop(False, reason='ignored', declared_by='someone_else')
        self.assertTrue(second_off.success)
        self.assertFalse(second_off.was_active)

        ended = self._audit_rows('planned_stop_ended')
        self.assertEqual(ended_before + 1, len(ended),
                         'withdrawing a stop that is not in force must write no audit record')
        self.assertEqual('__audit__', ended[-1][0])
        self.assertEqual('INACTIVE', ended[-1][2])
        self.assertEqual('plant_manager', ended[-1][3])
        self.assertEqual('restart', ended[-1][4])

        state = self._get_stop()
        self.assertFalse(state.active)
        # The declaration outlives the stop: the repeat did not overwrite it, and
        # the withdrawal did not erase it.
        self.assertEqual('weekend shutdown', state.reason)
        self.assertEqual('plant_manager', state.declared_by)
        self.assertGreater(state.ended_at.sec, 0)

    def test_the_reason_and_declarer_may_be_empty_or_long(self):
        empty = self._set_stop(True, reason='', declared_by='')
        self.assertTrue(empty.success)
        state = self._get_stop()
        self.assertTrue(state.active)
        self.assertEqual('', state.reason)
        self.assertEqual('', state.declared_by)
        self.assertGreater(state.since.sec, 0, 'a declared stop must carry the time it started')

        self._set_stop(False)

        long_reason = 'r' * 1024
        self.assertTrue(self._set_stop(True, reason=long_reason, declared_by='a' * 256).success)
        state = self._get_stop()
        self.assertTrue(state.active)
        self.assertEqual(long_reason, state.reason)
        self.assertEqual('a' * 256, state.declared_by)
        self.assertEqual(long_reason, self._audit_rows('planned_stop_started')[-1][4])

    def test_two_hundred_faults_muted_by_one_stop_are_all_released(self):
        codes = [f'PS_SCALE_{index:03d}' for index in range(200)]

        self._set_stop(True, reason='annual shutdown', declared_by='plant_manager')
        for code in codes:
            self._confirm(code)

        listed = self._list(include_muted=True)
        muted_codes = {info.fault_code for info in listed.muted_faults}
        missing = [code for code in codes if code not in muted_codes]
        self.assertEqual([], missing, f'{len(missing)} of 200 faults were not marked muted')

        visible = self._codes_in_default_list()
        self.assertEqual([], [code for code in codes if code in visible],
                         'a muted fault must not appear in the default list')

        self._set_stop(False, reason='shutdown over', declared_by='plant_manager')

        self._wait_until(
            lambda: not [code for code in codes if code not in self._codes_in_default_list()],
            'not every fault the stop muted came back to the default list')

        def _miscounted():
            return [(code, self._count_events(code, FaultEvent.EVENT_CONFIRMED))
                    for code in codes
                    if self._count_events(code, FaultEvent.EVENT_CONFIRMED) != 1]

        deadline = time.monotonic() + 30.0
        wrong = _miscounted()
        while wrong and time.monotonic() < deadline:
            time.sleep(0.2)
            wrong = _miscounted()
        self.assertEqual(
            [], wrong,
            f'{len(wrong)} of 200 faults were not announced exactly once at the switch-off; '
            f'first offenders: {wrong[:10]}')

    def test_a_fault_mid_debounce_when_the_stop_ends_confirms_afterwards(self):
        code = 'PS_MID_DEBOUNCE'

        self._set_stop(True, reason='calibration', declared_by='tech')
        # One report short of confirmation: the fault is PREFAILED and muted.
        self._report(code)
        self._wait_until(lambda: self._get_fault(code).success, f'{code} never reached the store')
        self.assertEqual('PREFAILED', self._get_fault(code).fault.status)
        self.assertIsNotNone(self._muted_entry(code))

        self._set_stop(False, reason='calibration over', declared_by='tech')

        self.assertIsNone(self._muted_entry(code))
        self.assertEqual(0, self._count_events(code, FaultEvent.EVENT_CONFIRMED),
                         'a fault still short of confirmation has nothing to announce')

        # The debounce carries on across the switch: one more report confirms it,
        # and with the stop gone the confirmation is announced.
        self._report(code)
        self._wait_until(lambda: self._count_events(code, FaultEvent.EVENT_CONFIRMED) == 1,
                         'the confirmation after the stop ended was never announced')
        self.assertEqual('CONFIRMED', self._get_fault(code).fault.status)
        self.assertIn(code, self._codes_in_default_list())

    def test_a_fault_already_up_when_the_stop_begins_stays_visible(self):
        """A cycle that started before the stop is not the stop's to hide."""
        code = 'PS_ALREADY_UP'

        # Confirmed and announced with no stop in force.
        self._confirm(code)
        self._wait_until(lambda: self._count_events(code, FaultEvent.EVENT_CONFIRMED) == 1,
                         f'{code} was never announced before the stop')
        self.assertIn(code, self._codes_in_default_list())

        self._set_stop(True, reason='line 3 maintenance', declared_by='shift_lead')

        # A level-triggered reporter keeps sending FAILED while the condition
        # holds. None of those reports starts a cycle.
        for _ in range(3):
            self._report(code)

        self.assertIsNone(self._muted_entry(code),
                          'the stop took over a fault whose cycle started before it')
        self.assertIn(code, self._codes_in_default_list(),
                      'a standing alarm left the fault list when the stop was declared')

        self._set_stop(False, reason='line 3 back up', declared_by='shift_lead')

        self.assertEqual(
            1, self._count_events(code, FaultEvent.EVENT_CONFIRMED),
            'the switch-off announced a confirmation that had already been announced')

    def test_a_fault_raised_again_after_a_clear_during_the_stop_is_marked(self):
        """Re-raising a cleared fault starts a new cycle, and the stop takes it."""
        code = 'PS_RERAISED'

        self._confirm(code)
        self._wait_until(lambda: self._get_fault(code).success, f'{code} never reached the store')
        self.assertTrue(self._clear(code).success)

        self._set_stop(True, reason='changeover', declared_by='maintenance')
        self._confirm(code)

        muted = self._wait_until(lambda: self._muted_entry(code),
                                 f'the re-raised {code} was not marked by the stop')
        self.assertEqual('planned_stop', muted.rule_id)
        self.assertNotIn(code, self._codes_in_default_list())

        before = self._count_events(code, FaultEvent.EVENT_CONFIRMED)
        self._set_stop(False, reason='done', declared_by='maintenance')
        self._wait_until(
            lambda: self._count_events(code, FaultEvent.EVENT_CONFIRMED) == before + 1,
            'the new cycle the stop held back was never announced')

    def test_a_scoped_acknowledgement_still_releases_the_stops_mute(self):
        """A clear that skips the correlation cascade still drops the stop's mute."""
        code = 'PS_SCOPED_ACK'

        self._set_stop(True, reason='cell 7 changeover', declared_by='maintenance')
        self._confirm(code)
        self._wait_until(lambda: self._muted_entry(code), f'{code} was not marked by the stop')
        before_count = self._list().muted_count

        # The shape every per-entity DELETE takes: acknowledge this fault only,
        # without walking the correlation graph into other entities.
        request = ClearFault.Request()
        request.fault_code = code
        request.skip_correlation_auto_clear = True
        self.assertTrue(self._call(self.clear_client, request).success)

        self.assertIsNone(self._muted_entry(code),
                          'an acknowledged fault stayed listed as muted by the stop')
        self.assertLess(self._list().muted_count, before_count,
                        'muted_count still counts a fault that was acknowledged')

        self._set_stop(False, reason='done', declared_by='maintenance')
        self.assertEqual(0, self._count_events(code, FaultEvent.EVENT_CONFIRMED))

    def test_a_cluster_rule_does_not_orphan_the_stops_mute(self):
        """A cluster claims the mute without owning it, so the stop still releases."""
        first = 'VALVE_PS_A'
        second = 'VALVE_PS_B'

        self._set_stop(True, reason='valve bank swap', declared_by='maintenance')
        self._confirm(first)
        self._confirm(second)

        entries = self._wait_until(
            lambda: [self._muted_entry(first), self._muted_entry(second)]
            if self._muted_entry(first) and self._muted_entry(second) else None,
            'the clustered faults were not marked by the stop')
        for entry in entries:
            self.assertEqual('planned_stop', entry.rule_id,
                             'the cluster took an entry it never writes')

        visible = self._codes_in_default_list()
        self.assertNotIn(first, visible)
        self.assertNotIn(second, visible)

        self._set_stop(False, reason='swap done', declared_by='maintenance')

        self._wait_until(
            lambda: all(code in self._codes_in_default_list() for code in (first, second)),
            'a fault a cluster rule claimed was orphaned in the mute map')
        for code in (first, second):
            self.assertIsNone(self._muted_entry(code))
            self.assertEqual(1, self._count_events(code, FaultEvent.EVENT_CONFIRMED))

    def test_the_last_declaration_is_readable_after_the_withdrawal(self):
        """The reason survives the withdrawal, with the time the stop ended."""
        self._set_stop(True, reason='line 3 quarterly maintenance', declared_by='shift_lead')
        during = self._get_stop()
        self.assertTrue(during.active)
        self.assertEqual(0, during.ended_at.sec)

        self._set_stop(False, reason='plant back up', declared_by='night_shift')

        after = self._get_stop()
        self.assertFalse(after.active)
        self.assertEqual('line 3 quarterly maintenance', after.reason)
        self.assertEqual('shift_lead', after.declared_by)
        self.assertEqual(during.since.sec, after.since.sec)
        self.assertGreater(after.ended_at.sec, 0)

        # The withdrawal's own reason and declarer are in the audit row, not in the
        # declaration.
        ended = self._audit_rows('planned_stop_ended')[-1]
        self.assertEqual('night_shift', ended[3])
        self.assertEqual('plant back up', ended[4])

    def test_the_switch_does_not_release_a_rule_muted_symptom(self):
        root = 'ESTOP_PS_001'
        symptom = 'MOTOR_PS_LEFT'

        self._set_stop(True, reason='e-stop drill', declared_by='safety')
        self._confirm(root)
        self._confirm(symptom)

        self._wait_until(lambda: self._get_fault(symptom).success,
                         f'{symptom} never reached the store')
        root_muted = self._muted_entry(root)
        self.assertIsNotNone(root_muted)
        self.assertEqual('planned_stop', root_muted.rule_id)
        symptom_muted = self._muted_entry(symptom)
        self.assertIsNotNone(symptom_muted)
        self.assertEqual('ps_estop_cascade', symptom_muted.rule_id,
                         'the correlation rule, not the switch, must own this mute')

        self._set_stop(False, reason='drill over', declared_by='safety')

        self._wait_until(lambda: root in self._codes_in_default_list(),
                         'the root cause did not return to the default list')
        self.assertEqual(1, self._count_events(root, FaultEvent.EVENT_CONFIRMED))

        still_muted = self._muted_entry(symptom)
        self.assertIsNotNone(still_muted, 'the switch released a symptom a rule was muting')
        self.assertEqual('ps_estop_cascade', still_muted.rule_id)
        self.assertNotIn(symptom, self._codes_in_default_list())
        self.assertEqual(0, self._count_events(symptom, FaultEvent.EVENT_CONFIRMED))


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        for info in proc_info:
            self.assertIn(info.returncode, (0, -2, -15),
                          f'{info.process_name} exited with code {info.returncode}')
