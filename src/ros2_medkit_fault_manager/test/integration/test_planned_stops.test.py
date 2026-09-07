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
Planned-stop windows against a real fault_manager over SQLite.

Every case here needs a node whose lifetime the test controls - a different
retention bound per case, and one case that kills the node and brings it back on
the same database - so the launch description starts nothing and each test spawns
its own fault_manager. SQLite rather than the in-memory backend because that is
the shipped default and the only one a restart can be asked about at all.
"""

import os
import shutil
import subprocess
import tempfile
import time
import unittest

from ament_index_python.packages import get_package_prefix
from builtin_interfaces.msg import Time
import launch
import launch_testing
import launch_testing.actions
import launch_testing.markers
import rclpy
from rclpy.node import Node
from ros2_medkit_msgs.msg import Fault
from ros2_medkit_msgs.srv import (
    ClearFault,
    DeclarePlannedStop,
    EndPlannedStop,
    ListFaults,
    ListPlannedStops,
    ReportFault,
)


@launch_testing.markers.keep_alive
def generate_test_description():
    """Start nothing: every case owns its fault_manager process."""
    return launch.LaunchDescription([launch_testing.actions.ReadyToTest()])


def fault_manager_executable():
    prefix = get_package_prefix('ros2_medkit_fault_manager')
    return os.path.join(prefix, 'lib', 'ros2_medkit_fault_manager', 'fault_manager_node')


def to_time(seconds_float):
    """Convert float seconds since the epoch to builtin_interfaces/Time."""
    msg = Time()
    msg.sec = int(seconds_float)
    msg.nanosec = int(round((seconds_float - msg.sec) * 1e9))
    if msg.nanosec >= 1000000000:
        msg.sec += 1
        msg.nanosec -= 1000000000
    return msg


def to_float(time_msg):
    return time_msg.sec + time_msg.nanosec * 1e-9


class PlannedStopFixture(unittest.TestCase):
    """
    One fault_manager per test, under a node name only that test uses.

    Three lessons are baked into this fixture, each learned by watching this file
    fail. A manager that outlives its test keeps answering /fault_manager/* and
    the assertions then describe somebody else's database, so every START gets a
    node name no other manager has ever used - a restart included, which also
    makes "the second process is a different process" something the test states
    rather than assumes. Waiting for a killed node's services to LEAVE the graph
    is not a usable substitute: the departure rides on the participant lease and
    takes tens of seconds. And a child holding the launch harness's stdout turns
    a five-second suite into a ctest timeout, so children get no inherited pipes.
    """

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = Node('planned_stop_test_driver')
        cls.workdir = tempfile.mkdtemp(prefix='planned_stops_')

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()
        shutil.rmtree(cls.workdir, ignore_errors=True)

    manager_serial = 0

    def setUp(self):
        self.proc = None
        self.manager_name = ''
        self._clients = {}
        self.db_path = os.path.join(self.workdir, f'{self.id().rsplit(".", 1)[-1][:40]}.db')

    def tearDown(self):
        self._stop_manager()
        for client in self._clients.values():
            self.node.destroy_client(client)
        self._clients = {}

    # --- process control ---------------------------------------------------

    def _start_manager(self, extra_params=None):
        self._stop_manager()
        PlannedStopFixture.manager_serial += 1
        self.manager_name = f'fm_{PlannedStopFixture.manager_serial}'
        args = [
            fault_manager_executable(),
            '--ros-args',
            '-r', f'__node:={self.manager_name}',
            '-p', 'storage_type:=sqlite',
            '-p', f'database_path:={self.db_path}',
            '-p', 'confirmation_threshold:=-1',
            '-p', 'healing_enabled:=true',
            '-p', 'healing_threshold:=1',
            '-p', 'snapshots.enabled:=false',
            '-p', 'snapshots.rosbag.enabled:=false',
        ]
        for key, value in (extra_params or {}).items():
            args.extend(['-p', f'{key}:={value}'])
        self.proc = subprocess.Popen(
            args,
            env=os.environ.copy(),
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        # Nothing may be asserted until the node this test started is answering.
        self._client(ListPlannedStops, 'list_planned_stops')

    def _stop_manager(self):
        if self.proc is None:
            return
        self.proc.terminate()
        try:
            self.proc.wait(timeout=15)
        except subprocess.TimeoutExpired:
            self.proc.kill()
            self.proc.wait(timeout=15)
        self.proc = None

    # --- service plumbing --------------------------------------------------

    def _client(self, srv_type, name, timeout=30.0):
        """
        Keep one client per (manager, service) and reuse it for the test.

        Creating a fresh client for every call churns the DDS graph: each one
        discovers the service again, and under the load of the whole integration
        suite one of them eventually costs more than the call budget. The keys
        carry the manager's node name, which is unique per START, so a restart
        gets fresh clients without any cache invalidation.
        """
        key = (self.manager_name, name)
        client = self._clients.get(key)
        if client is None:
            client = self.node.create_client(srv_type, f'/{self.manager_name}/{name}')
            self._clients[key] = client
        self.assertTrue(
            client.wait_for_service(timeout_sec=timeout),
            f'{name} service never appeared on {self.manager_name}',
        )
        return client

    def _call(self, client, request, timeout=15.0):
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout)
        result = future.result()
        self.assertIsNotNone(result, 'service call timed out')
        return result

    def _declare(self, starts_at, ends_at, reason='changeover', declared_by='tester'):
        request = DeclarePlannedStop.Request()
        request.starts_at = to_time(starts_at)
        request.ends_at = to_time(ends_at)
        request.reason = reason
        request.declared_by = declared_by
        return self._call(self._client(DeclarePlannedStop, 'declare_planned_stop'), request)

    def _list_stops(self, active_only=False, now=None):
        request = ListPlannedStops.Request()
        request.active_only = active_only
        request.now = to_time(now) if now is not None else Time()
        return self._call(self._client(ListPlannedStops, 'list_planned_stops'), request)

    def _report(self, fault_code, event_type, source_id='/test/reporter'):
        request = ReportFault.Request()
        request.fault_code = fault_code
        request.event_type = event_type
        request.severity = Fault.SEVERITY_ERROR
        request.description = 'planned stop parity fault'
        request.source_id = source_id
        return self._call(self._client(ReportFault, 'report_fault'), request)

    def _get_fault(self, fault_code, statuses=None):
        request = ListFaults.Request()
        request.statuses = statuses or [
            Fault.STATUS_PREFAILED,
            Fault.STATUS_PREPASSED,
            Fault.STATUS_CONFIRMED,
            Fault.STATUS_HEALED,
            Fault.STATUS_CLEARED,
        ]
        response = self._call(self._client(ListFaults, 'list_faults'), request)
        for fault in response.faults:
            if fault.fault_code == fault_code:
                return fault
        return None

    def _wait_for_status(self, fault_code, status, timeout=20.0):
        deadline = time.monotonic() + timeout
        last = None
        while time.monotonic() < deadline:
            last = self._get_fault(fault_code)
            if last is not None and last.status == status:
                return last
            time.sleep(0.3)
        raise AssertionError(
            f'{fault_code} never reached {status}; last seen: '
            f'{last.status if last else "absent"}'
        )


class TestPlannedStops(PlannedStopFixture):
    """Planned-stop windows end to end against a real fault_manager."""

    RETENTION_CAP = 5

    def test_window_survives_a_restart_and_still_covers_new_faults(self):
        self._start_manager()

        now = time.time()
        declared = self._declare(now - 60.0, now + 900.0, reason='maintenance weekend')
        self.assertTrue(declared.success, declared.message)
        window_id = declared.stop.id
        starts_at = to_float(declared.stop.starts_at)
        ends_at = to_float(declared.stop.ends_at)

        # The fault manager must be gone, not merely quiet, before the second one
        # opens the same database file.
        self._stop_manager()
        self._start_manager()

        listed = self._list_stops()
        ids = [stop.id for stop in listed.stops]
        self.assertIn(window_id, ids, 'the declared window did not survive the restart')

        survivor = next(stop for stop in listed.stops if stop.id == window_id)
        self.assertEqual(survivor.reason, 'maintenance weekend')
        self.assertAlmostEqual(to_float(survivor.starts_at), starts_at, places=6)
        self.assertAlmostEqual(to_float(survivor.ends_at), ends_at, places=6)

        # A fault raised after the restart falls inside the window that predates it.
        self._report('RESTART_INSIDE_WINDOW', ReportFault.Request.EVENT_FAILED)
        fault = self._wait_for_status('RESTART_INSIDE_WINDOW', Fault.STATUS_CONFIRMED)
        first_occurred = to_float(fault.first_occurred)
        self.assertGreaterEqual(first_occurred, starts_at)
        self.assertLessEqual(first_occurred, ends_at)

        active = self._list_stops(active_only=True)
        self.assertIn(window_id, [stop.id for stop in active.stops])

    def test_expected_and_unexpected_faults_take_the_same_path(self):
        """An expected fault must be indistinguishable from a surprise in storage."""
        self._start_manager()

        now = time.time()
        declared = self._declare(now - 5.0, now + 600.0)
        self.assertTrue(declared.success, declared.message)
        inside_start = to_float(declared.stop.starts_at)
        inside_end = to_float(declared.stop.ends_at)

        # Both faults are driven through the identical sequence, at the same
        # moment, inside the same window; only the assertion below distinguishes
        # them, and it must find nothing to distinguish.
        codes = ('PARITY_INSIDE_WINDOW', 'PARITY_OUTSIDE_WINDOW')
        for code in codes:
            self._report(code, ReportFault.Request.EVENT_FAILED)
        records = {code: self._wait_for_status(code, Fault.STATUS_CONFIRMED) for code in codes}

        inside = records['PARITY_INSIDE_WINDOW']
        self.assertGreaterEqual(to_float(inside.first_occurred), inside_start)
        self.assertLessEqual(to_float(inside.first_occurred), inside_end)

        for code in codes:
            self.assertEqual(records[code].status, Fault.STATUS_CONFIRMED)
            self.assertEqual(records[code].occurrence_count, 1)
            self.assertEqual(records[code].severity, Fault.SEVERITY_ERROR)
            self.assertEqual(list(records[code].reporting_sources), ['/test/reporter'])

        # Heal both, then reactivate both: the cycle counter must move identically.
        for code in codes:
            self._report(code, ReportFault.Request.EVENT_PASSED)
            self._report(code, ReportFault.Request.EVENT_PASSED)
        healed = {code: self._wait_for_status(code, Fault.STATUS_HEALED) for code in codes}
        for code in codes:
            self.assertEqual(healed[code].occurrence_count, 1)
            self.assertNotEqual(to_float(healed[code].last_passed), 0.0)

        for code in codes:
            clear_request = ClearFault.Request()
            clear_request.fault_code = code
            cleared = self._call(self._client(ClearFault, 'clear_fault'), clear_request)
            self.assertTrue(cleared.success, f'{code}: {cleared.message}')

        for code in codes:
            self._report(code, ReportFault.Request.EVENT_FAILED)
        reactivated = {code: self._wait_for_status(code, Fault.STATUS_CONFIRMED) for code in codes}
        for code in codes:
            self.assertEqual(
                reactivated[code].occurrence_count, 2,
                f'{code}: a re-raise after a clear is a new cycle, window or not',
            )

        fields = ('status', 'occurrence_count', 'severity')
        for field in fields:
            self.assertEqual(
                getattr(reactivated['PARITY_INSIDE_WINDOW'], field),
                getattr(reactivated['PARITY_OUTSIDE_WINDOW'], field),
                f'the window changed {field}, which it must never do',
            )

    def test_bound_of_one_keeps_the_newest_ended_window(self):
        """CONFIG SWEEP: the smallest legal bound."""
        self._start_manager({'planned_stop.max_windows': 1})

        now = time.time()
        first = self._declare(now - 100.0, now - 90.0, reason='old changeover')
        self.assertTrue(first.success, first.message)
        second = self._declare(now - 50.0, now - 40.0, reason='newer changeover')
        self.assertTrue(second.success, second.message)

        listed = self._list_stops()
        self.assertEqual(len(listed.stops), 1)
        self.assertEqual(listed.stops[0].id, second.stop.id)

    def test_default_bound_keeps_every_window_of_a_small_run(self):
        """CONFIG SWEEP: the shipped default keeps far more than a handful."""
        self._start_manager()

        now = time.time()
        declared_ids = []
        for i in range(12):
            start = now - 1000.0 + i * 10.0
            response = self._declare(start, start + 5.0)
            self.assertTrue(response.success, response.message)
            declared_ids.append(response.stop.id)

        listed = self._list_stops()
        self.assertEqual(len(listed.stops), 12)
        self.assertEqual(sorted(stop.id for stop in listed.stops), sorted(declared_ids))

    def test_declaring_past_the_cap_prunes_ended_windows_and_spares_the_live_one(self):
        """SCALE: past the cap, where the guard actually engages."""
        self._start_manager({'planned_stop.max_windows': self.RETENTION_CAP})

        now = time.time()
        ended_ids = []
        for i in range(self.RETENTION_CAP + 4):
            start = now - 2000.0 + i * 10.0
            response = self._declare(start, start + 5.0, reason=f'ended_{i}')
            self.assertTrue(response.success, response.message)
            ended_ids.append(response.stop.id)

        live = self._declare(now - 5.0, now + 3600.0, reason='live')
        self.assertTrue(live.success, live.message)

        listed = self._list_stops()
        # The bound counts windows that are no longer active; an active one is
        # always kept and is never dropped to make room. So the table holds at
        # most the cap plus however many are still running.
        active = sum(
            1 for stop in listed.stops if to_float(stop.ends_at) > time.time()
        )
        self.assertLessEqual(
            len(listed.stops), self.RETENTION_CAP + active,
            'stored count must be at most the cap plus the active windows',
        )
        self.assertLessEqual(
            len(listed.stops) - active, self.RETENTION_CAP,
            'the ended windows alone must fit the configured bound',
        )

        surviving = [stop.id for stop in listed.stops]
        self.assertIn(live.stop.id, surviving, 'a running window must survive the cap')
        self.assertNotIn(ended_ids[0], surviving, 'the oldest declaration goes first')
        self.assertNotIn(ended_ids[1], surviving)
        self.assertIn(ended_ids[-1], surviving, 'the newest ended declarations stay')

        active = self._list_stops(active_only=True)
        self.assertEqual([stop.id for stop in active.stops], [live.stop.id])

    def test_ending_early_moves_the_end_and_cannot_be_repeated(self):
        """CHANGE: ending a window mid-run moves where the boundary sits."""
        self._start_manager()

        now = time.time()
        declared = self._declare(now - 30.0, now + 3600.0)
        self.assertTrue(declared.success, declared.message)
        window_id = declared.stop.id

        end_request = EndPlannedStop.Request()
        end_request.id = window_id
        end_request.at = Time()  # zero = now
        ended = self._call(self._client(EndPlannedStop, 'end_planned_stop'), end_request)
        self.assertTrue(ended.success, ended.message)
        self.assertTrue(ended.stop.ended_early)
        new_end = to_float(ended.stop.ends_at)
        self.assertLess(new_end, now + 3600.0, 'ending early must pull ends_at back')
        self.assertGreaterEqual(new_end, now - 30.0)

        # The window no longer covers now, so it is not active any more.
        active = self._list_stops(active_only=True)
        self.assertNotIn(window_id, [stop.id for stop in active.stops])

        again = self._call(self._client(EndPlannedStop, 'end_planned_stop'), end_request)
        self.assertFalse(again.success)
        self.assertIn('already ended', again.message)

        unknown = EndPlannedStop.Request()
        unknown.id = 'no_such_window'
        missing = self._call(self._client(EndPlannedStop, 'end_planned_stop'), unknown)
        self.assertFalse(missing.success)
        self.assertIn('no planned stop', missing.message)

    def test_the_service_refuses_an_unset_start(self):
        """R14: zero is what an unset Time reads as, not a request for "now"."""
        self._start_manager()

        request = DeclarePlannedStop.Request()
        request.starts_at = Time()  # unset
        request.ends_at = to_time(time.time() + 600.0)
        request.reason = 'no start given'
        response = self._call(
            self._client(DeclarePlannedStop, 'declare_planned_stop'), request
        )
        self.assertFalse(response.success)
        self.assertEqual(
            response.outcome,
            DeclarePlannedStop.Response.OUTCOME_INVALID_INSTANT,
        )
        self.assertIn('epoch', response.message)
        self.assertEqual(len(self._list_stops().stops), 0)

    def test_a_declaration_reported_as_stored_is_there_afterwards(self):
        """A cap filled by a window that cannot be pruned must not eat the new one."""
        self._start_manager({'planned_stop.max_windows': 1})

        now = time.time()
        live = self._declare(now - 10.0, now + 3600.0, reason='running')
        self.assertTrue(live.success, live.message)

        ended = self._declare(now - 500.0, now - 400.0, reason='already over')
        self.assertTrue(ended.success, ended.message)
        self.assertEqual(ended.outcome, DeclarePlannedStop.Response.OUTCOME_STORED)

        ids = [stop.id for stop in self._list_stops().stops]
        self.assertIn(
            ended.stop.id, ids,
            'the declaration answered success and then was not there'
        )
        self.assertIn(live.stop.id, ids, 'and a running window is still never pruned')

    def test_a_window_that_never_started_is_cancelled(self):
        """R12: it marked nothing, so it is removed rather than inverted."""
        self._start_manager()

        now = time.time()
        future = self._declare(now + 3600.0, now + 7200.0, reason='next weekend')
        self.assertTrue(future.success, future.message)

        request = EndPlannedStop.Request()
        request.id = future.stop.id
        request.at = Time()  # now, which is before the window starts
        response = self._call(self._client(EndPlannedStop, 'end_planned_stop'), request)

        self.assertTrue(response.success, response.message)
        self.assertEqual(response.outcome, EndPlannedStop.Response.OUTCOME_CANCELLED)
        self.assertTrue(response.stop.cancelled)
        self.assertFalse(response.stop.ended_early)
        self.assertNotIn(
            future.stop.id, [stop.id for stop in self._list_stops().stops],
            'a cancelled window is gone, not stored with an inverted interval'
        )

    def test_a_backdated_end_cannot_rewrite_when_a_stop_finished(self):
        self._start_manager()

        now = time.time()
        window = self._declare(now - 600.0, now + 600.0, reason='running')
        self.assertTrue(window.success, window.message)

        first = EndPlannedStop.Request()
        first.id = window.stop.id
        first.at = to_time(now)
        ended = self._call(self._client(EndPlannedStop, 'end_planned_stop'), first)
        self.assertTrue(ended.success, ended.message)
        self.assertEqual(ended.outcome, EndPlannedStop.Response.OUTCOME_ENDED)
        settled_end = to_float(ended.stop.ends_at)

        backdated = EndPlannedStop.Request()
        backdated.id = window.stop.id
        backdated.at = to_time(now - 300.0)
        again = self._call(self._client(EndPlannedStop, 'end_planned_stop'), backdated)
        self.assertFalse(again.success)
        self.assertEqual(again.outcome, EndPlannedStop.Response.OUTCOME_ALREADY_ENDED)

        stored = next(
            stop for stop in self._list_stops().stops if stop.id == window.stop.id
        )
        self.assertAlmostEqual(to_float(stored.ends_at), settled_end, places=6)

    def test_a_finished_window_cannot_be_re_ended_by_a_backdated_at(self):
        """R15: which situation a window is in is judged against the manager's clock."""
        self._start_manager()

        now = time.time()
        # Declared and finished in the past. Its end is a fact now.
        window = self._declare(now - 600.0, now - 300.0, reason='already over')
        self.assertTrue(window.success, window.message)
        original_end = to_float(window.stop.ends_at)

        # An instant INSIDE the original span, which the old rule read as "still
        # running at that instant" and accepted.
        backdated = EndPlannedStop.Request()
        backdated.id = window.stop.id
        backdated.at = to_time(now - 450.0)
        response = self._call(self._client(EndPlannedStop, 'end_planned_stop'), backdated)

        self.assertFalse(response.success, 'a finished window is immutable')
        self.assertEqual(response.outcome, EndPlannedStop.Response.OUTCOME_ALREADY_ENDED)

        stored = next(
            stop for stop in self._list_stops().stops if stop.id == window.stop.id
        )
        self.assertAlmostEqual(to_float(stored.ends_at), original_end, places=6)
        self.assertFalse(stored.ended_early)

    def test_a_finished_window_is_not_cancelled_by_an_at_before_its_start(self):
        """The cancel branch must test the clock, not the instant it was handed."""
        self._start_manager()

        now = time.time()
        window = self._declare(now - 600.0, now - 300.0, reason='already over')
        self.assertTrue(window.success, window.message)

        request = EndPlannedStop.Request()
        request.id = window.stop.id
        request.at = to_time(now - 900.0)  # before the window even started
        response = self._call(self._client(EndPlannedStop, 'end_planned_stop'), request)

        self.assertFalse(response.success)
        self.assertEqual(response.outcome, EndPlannedStop.Response.OUTCOME_ALREADY_ENDED)
        self.assertIn(
            window.stop.id, [stop.id for stop in self._list_stops().stops],
            'a finished window must not be deleted by a request that cannot touch it'
        )

    def test_at_outside_the_running_span_is_refused(self):
        """`at` may only refine where a RUNNING window ends."""
        self._start_manager()

        now = time.time()
        window = self._declare(now - 300.0, now + 300.0, reason='running')
        self.assertTrue(window.success, window.message)

        too_early = EndPlannedStop.Request()
        too_early.id = window.stop.id
        too_early.at = to_time(now - 600.0)
        response = self._call(self._client(EndPlannedStop, 'end_planned_stop'), too_early)
        self.assertFalse(response.success)
        self.assertEqual(response.outcome, EndPlannedStop.Response.OUTCOME_INVALID_AT)

        too_late = EndPlannedStop.Request()
        too_late.id = window.stop.id
        too_late.at = to_time(now + 200.0)  # has not happened yet
        response = self._call(self._client(EndPlannedStop, 'end_planned_stop'), too_late)
        self.assertFalse(response.success)
        self.assertEqual(response.outcome, EndPlannedStop.Response.OUTCOME_INVALID_AT)

        stored = next(
            stop for stop in self._list_stops().stops if stop.id == window.stop.id
        )
        self.assertFalse(stored.ended_early, 'a refused request must not have moved anything')

        # And an instant inside the running span is accepted.
        legal = EndPlannedStop.Request()
        legal.id = window.stop.id
        legal.at = to_time(now - 10.0)
        response = self._call(self._client(EndPlannedStop, 'end_planned_stop'), legal)
        self.assertTrue(response.success, response.message)
        self.assertEqual(response.outcome, EndPlannedStop.Response.OUTCOME_ENDED)
        self.assertAlmostEqual(to_float(response.stop.ends_at), now - 10.0, places=3)

    def test_a_window_that_is_not_an_interval_is_refused(self):
        self._start_manager()

        now = time.time()
        equal = self._declare(now, now)
        self.assertFalse(equal.success, 'a zero-length window is not an interval')
        self.assertEqual(
            equal.outcome, DeclarePlannedStop.Response.OUTCOME_INVALID_INTERVAL
        )
        self.assertIn('strictly after', equal.message)

        reversed_window = self._declare(now + 60.0, now)
        self.assertFalse(reversed_window.success)

        self.assertEqual(len(self._list_stops().stops), 0)
