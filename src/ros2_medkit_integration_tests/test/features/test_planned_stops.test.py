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

"""End-to-end acceptance for planned-stop windows: real gateway, real fault manager.

An operator declares a window over REST, faults are reported inside and outside
it through the real ReportFault service, and the flag has to come back the same
way on three surfaces: the global fault list, the per-entity list, and the live
event stream. The filter has to hide exactly one of the two faults each way, and
the count beside the list has to agree with the flags on the items.

Every assertion below reads the flag off the ITEM or off the FRAME for a named
fault code. Asserting on a count, or on "some event in the stream", would pass
against a gateway that flagged the wrong fault.
"""

import calendar
import json
import threading
import time
import unittest

import launch_testing
import launch_testing.actions
import rclpy
from rclpy.node import Node
import requests
from ros2_medkit_msgs.msg import Fault
from ros2_medkit_msgs.srv import ClearFault, ReportFault

from ros2_medkit_test_utils.constants import ALLOWED_EXIT_CODES, FAULT_TIMEOUT
from ros2_medkit_test_utils.gateway_test_case import GatewayTestCase
from ros2_medkit_test_utils.launch_helpers import create_test_launch

PLANNED_STOPS = '/x-medkit-planned-stops'
SOURCE_ID = '/powertrain/engine/temp_sensor'
APP_ID = 'temp_sensor'


def generate_test_description():
    return create_test_launch(
        demo_nodes=['temp_sensor'],
        fault_manager=True,
        lidar_faulty=False,
        fault_manager_params={
            'storage_type': 'memory',
            # One FAILED confirms; two PASSED heal. Keeps every case a single
            # report away from the transition it is about.
            'confirmation_threshold': -1,
            'healing_enabled': True,
            'healing_threshold': 1,
            'snapshots.rosbag.enabled': False,
        },
    )


class TestPlannedStops(GatewayTestCase):
    """Declaring a window changes what the gateway REPORTS, never what it does."""

    MIN_EXPECTED_APPS = 1
    REQUIRED_APPS = {APP_ID}

    @classmethod
    def setUpClass(cls):
        super().setUpClass()
        rclpy.init()
        cls._reporter = Node('planned_stop_reporter')
        cls._report_client = cls._reporter.create_client(
            ReportFault, '/fault_manager/report_fault'
        )
        cls._clear_client = cls._reporter.create_client(
            ClearFault, '/fault_manager/clear_fault'
        )
        assert cls._report_client.wait_for_service(timeout_sec=30.0), \
            'report_fault service not available'
        assert cls._clear_client.wait_for_service(timeout_sec=30.0), \
            'clear_fault service not available'

    @classmethod
    def tearDownClass(cls):
        cls._reporter.destroy_node()
        rclpy.shutdown()

    def tearDown(self):
        """End every window this case left open.

        The cases share one gateway and one fault manager, so a window left
        running would make the NEXT case's faults expected for a reason that case
        never declared. That is not hypothetical: it is how the first version of
        this file turned seven honest assertions red at once. Ended windows are
        left alone on purpose - they are still the reason the faults they already
        covered read as expected, which several cases below check.
        """
        try:
            active = self.get_json(f'{PLANNED_STOPS}?active=true')
        except AssertionError:
            return
        for window in active.get('items', []):
            requests.delete(
                f'{self.BASE_URL}{PLANNED_STOPS}/{window["id"]}', timeout=10
            )

    # --- driving the real fault pipeline -----------------------------------

    def _report(self, fault_code, event_type=None):
        request = ReportFault.Request()
        request.fault_code = fault_code
        request.event_type = (
            ReportFault.Request.EVENT_FAILED if event_type is None else event_type
        )
        request.severity = Fault.SEVERITY_ERROR
        request.description = 'planned stop e2e fault'
        request.source_id = SOURCE_ID
        future = self._report_client.call_async(request)
        deadline = time.monotonic() + 10.0
        while not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(self._reporter, timeout_sec=0.1)
        self.assertTrue(future.done(), f'report_fault({fault_code}) timed out')
        self.assertTrue(future.result().accepted)

    def _clear(self, fault_code):
        request = ClearFault.Request()
        request.fault_code = fault_code
        future = self._clear_client.call_async(request)
        deadline = time.monotonic() + 10.0
        while not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(self._reporter, timeout_sec=0.1)
        self.assertTrue(future.done(), f'clear_fault({fault_code}) timed out')

    # --- reading the gateway ------------------------------------------------

    def _wait_for_fault_in_list(self, fault_code, timeout=None):
        """Poll GET /faults until the code shows up, and return its item."""
        limit = timeout if timeout is not None else FAULT_TIMEOUT
        deadline = time.monotonic() + limit
        last = None
        while time.monotonic() < deadline:
            last = self._items(self.get_json('/faults?status=all'))
            for item in last:
                if item.get('fault_code') == fault_code:
                    return item
            time.sleep(0.3)
        raise AssertionError(
            f'{fault_code} never reached GET /faults; saw '
            f'{[i.get("fault_code") for i in (last or [])]}'
        )

    @staticmethod
    def _items(payload):
        return payload.get('items', [])

    def _find(self, payload, fault_code):
        for item in self._items(payload):
            if item.get('fault_code') == fault_code:
                return item
        return None

    def _declare(self, body, expected_status=201):
        response = requests.post(
            f'{self.BASE_URL}{PLANNED_STOPS}', json=body, timeout=10
        )
        self.assertEqual(
            response.status_code, expected_status,
            f'POST {PLANNED_STOPS} {body} -> {response.status_code}: {response.text}'
        )
        return response.json()

    @staticmethod
    def _iso(offset_sec):
        return time.strftime(
            '%Y-%m-%dT%H:%M:%SZ', time.gmtime(time.time() + offset_sec)
        )

    # --- the acceptance -----------------------------------------------------

    def test_01_declared_window_marks_only_the_fault_it_covers(self):
        """One fault inside a window, one outside: list, flag, id, count, filter."""
        window = self._declare({
            'from': self._iso(-60),
            'to': self._iso(3600),
            'reason': 'line changeover',
        })
        for key in ('id', 'from', 'to', 'reason', 'declared_by', 'declared_at',
                    'ended_early'):
            self.assertIn(key, window, f'POST response is missing {key}')
        self.assertFalse(window['ended_early'])
        self.assertEqual(window['reason'], 'line changeover')
        window_id = window['id']

        # Both faults are raised while the window is open, but only one is
        # reported under a code the window covers - the other one's cycle is
        # started AFTER the window is ended, further down. Here both are inside.
        self._report('PS_INSIDE_WINDOW')
        inside = self._wait_for_fault_in_list('PS_INSIDE_WINDOW')

        self.assertIn('x-medkit', inside, 'the list item carries no vendor extension')
        self.assertTrue(
            inside['x-medkit']['expected'],
            'a fault raised inside a declared window must be reported as expected'
        )
        self.assertEqual(inside['x-medkit']['planned_stop_id'], window_id)

        listing = self.get_json('/faults?status=all')
        self.assertGreaterEqual(listing['x-medkit']['expected_count'], 1)

        # The item's own flag and the count must agree - the count is derived
        # from the same pass, and a count that drifted from the items would make
        # every "N faults, M of them planned" summary wrong.
        flagged = [i for i in self._items(listing)
                   if i.get('x-medkit', {}).get('expected')]
        self.assertEqual(listing['x-medkit']['expected_count'], len(flagged))

    def test_02_a_fault_outside_every_window_is_not_expected(self):
        """The other half of the pair, and the filter in both directions."""
        # A window that closed before this fault's cycle starts.
        self._declare({
            'from': self._iso(-7200),
            'to': self._iso(-3600),
            'reason': 'last week',
        })
        self._report('PS_OUTSIDE_WINDOW')
        outside = self._wait_for_fault_in_list('PS_OUTSIDE_WINDOW')
        self.assertFalse(
            outside['x-medkit']['expected'],
            'a fault whose cycle started after every window closed is a surprise'
        )
        self.assertNotIn(
            'planned_stop_id', outside['x-medkit'],
            'an unexpected fault must not name a window'
        )

        only_expected = self.get_json('/faults?status=all&expected=true')
        codes = [i['fault_code'] for i in self._items(only_expected)]
        self.assertIn('PS_INSIDE_WINDOW', codes)
        self.assertNotIn('PS_OUTSIDE_WINDOW', codes)
        for item in self._items(only_expected):
            self.assertTrue(item['x-medkit']['expected'])

        only_unexpected = self.get_json('/faults?status=all&expected=false')
        codes = [i['fault_code'] for i in self._items(only_unexpected)]
        self.assertIn('PS_OUTSIDE_WINDOW', codes)
        self.assertNotIn('PS_INSIDE_WINDOW', codes)
        for item in self._items(only_unexpected):
            self.assertFalse(item['x-medkit']['expected'])

        both = self.get_json('/faults?status=all&expected=all')
        codes = [i['fault_code'] for i in self._items(both)]
        self.assertIn('PS_INSIDE_WINDOW', codes)
        self.assertIn('PS_OUTSIDE_WINDOW', codes)

        unfiltered = self.get_json('/faults?status=all')
        self.assertEqual(
            sorted(i['fault_code'] for i in self._items(unfiltered)),
            sorted(codes),
            'omitting the parameter must mean "all", not a default filter'
        )

    def test_03_per_entity_list_agrees_with_the_global_one(self):
        """A scoped list must not answer differently about the same fault."""
        per_entity = self.get_json(f'/apps/{APP_ID}/faults?status=all')
        inside = self._find(per_entity, 'PS_INSIDE_WINDOW')
        outside = self._find(per_entity, 'PS_OUTSIDE_WINDOW')
        self.assertIsNotNone(inside, 'the per-entity list lost the expected fault')
        self.assertIsNotNone(outside)
        self.assertTrue(inside['x-medkit']['expected'])
        self.assertFalse(outside['x-medkit']['expected'])
        self.assertIn('expected_count', per_entity['x-medkit'])

        scoped_expected = self.get_json(f'/apps/{APP_ID}/faults?status=all&expected=true')
        codes = [i['fault_code'] for i in self._items(scoped_expected)]
        self.assertIn('PS_INSIDE_WINDOW', codes)
        self.assertNotIn('PS_OUTSIDE_WINDOW', codes)

    def test_04_fault_detail_carries_the_same_flag(self):
        """A UI that opens a flagged row must not find the flag gone."""
        detail = self.get_json(f'/apps/{APP_ID}/faults/PS_INSIDE_WINDOW')
        self.assertTrue(detail['x-medkit']['expected'])
        self.assertIn('planned_stop_id', detail['x-medkit'])

        plain = self.get_json(f'/apps/{APP_ID}/faults/PS_OUTSIDE_WINDOW')
        self.assertFalse(plain['x-medkit']['expected'])

    def test_05_event_stream_carries_the_flag_per_fault(self):
        """The stream and the list must tell a consumer the same thing."""
        frames = []
        stop_event = threading.Event()
        response = requests.get(
            f'{self.BASE_URL}/faults/stream', stream=True, timeout=(5, 60)
        )
        self.assertEqual(response.status_code, 200)
        pump = threading.Thread(
            target=self._pump_stream, args=(response, frames, stop_event),
            daemon=True,
        )
        pump.start()
        try:
            self._prime_stream(frames)

            # A window covering only what is raised from here on.
            window = self._declare({
                'to': self._iso(3600),
                'reason': 'stream changeover',
            })
            self.assertEqual(
                window['declared_by'], 'anonymous',
                'with auth off there is nobody to attribute the declaration to'
            )

            self._report('PS_STREAM_INSIDE')
            inside_payload = self._wait_for_frame(frames, 'PS_STREAM_INSIDE')
            self.assertTrue(
                inside_payload['x-medkit']['expected'],
                'the frame for the fault raised inside the window must be flagged'
            )
            self.assertEqual(
                inside_payload['x-medkit']['planned_stop_id'], window['id']
            )

            # End the window, then raise the second fault: same stream, same
            # gateway, opposite answer.
            ended = requests.delete(
                f'{self.BASE_URL}{PLANNED_STOPS}/{window["id"]}', timeout=10
            )
            self.assertEqual(ended.status_code, 200, ended.text)
            self.assertTrue(ended.json()['ended_early'])

            self._report('PS_STREAM_OUTSIDE')
            outside_payload = self._wait_for_frame(frames, 'PS_STREAM_OUTSIDE')
            self.assertFalse(
                outside_payload['x-medkit']['expected'],
                'a fault raised after the window was ended must not be flagged'
            )
            self.assertNotIn('planned_stop_id', outside_payload['x-medkit'])

            # Counting frames per code proves the two assertions above were made
            # about two different faults, not twice about one.
            per_code = {}
            for frame in list(frames):
                data = frame.get('data')
                if not data:
                    continue
                code = json.loads(data).get('fault', {}).get('fault_code')
                if code in ('PS_STREAM_INSIDE', 'PS_STREAM_OUTSIDE'):
                    per_code[code] = per_code.get(code, 0) + 1
            self.assertGreaterEqual(per_code.get('PS_STREAM_INSIDE', 0), 1)
            self.assertGreaterEqual(per_code.get('PS_STREAM_OUTSIDE', 0), 1)
        finally:
            stop_event.set()
            response.close()
            pump.join(timeout=5)

    def test_06_ending_a_window_keeps_marking_what_came_before_it(self):
        """CHANGE: the boundary moves, and only forward."""
        window = self._declare({'to': self._iso(3600), 'reason': 'cut short'})
        self._report('PS_BEFORE_EARLY_END')
        before = self._wait_for_fault_in_list('PS_BEFORE_EARLY_END')
        self.assertTrue(before['x-medkit']['expected'])

        ended = requests.delete(
            f'{self.BASE_URL}{PLANNED_STOPS}/{window["id"]}', timeout=10
        )
        self.assertEqual(ended.status_code, 200, ended.text)
        self.assertTrue(ended.json()['ended_early'])
        self.assertLess(ended.json()['to'], window['to'])

        self._report('PS_AFTER_EARLY_END')
        after = self._wait_for_fault_in_list('PS_AFTER_EARLY_END')
        self.assertFalse(
            after['x-medkit']['expected'],
            'ending a window must stop it marking faults raised afterwards'
        )

        still_flagged = self._find(
            self.get_json('/faults?status=all'), 'PS_BEFORE_EARLY_END'
        )
        self.assertTrue(
            still_flagged['x-medkit']['expected'],
            'ending a window must not un-mark the faults it already covered'
        )
        self.assertEqual(
            still_flagged['x-medkit']['planned_stop_id'], window['id']
        )

        second_end = requests.delete(
            f'{self.BASE_URL}{PLANNED_STOPS}/{window["id"]}', timeout=10
        )
        self.assertEqual(second_end.status_code, 400, second_end.text)
        # Vendor codes travel in `vendor_code` under the SOVD `vendor-error`
        # envelope, which is what a generic client keys on.
        self.assertEqual(second_end.json()['error_code'], 'vendor-error')
        self.assertEqual(
            second_end.json()['vendor_code'], 'x-medkit-planned-stop-ended'
        )

    def test_07_a_window_declared_after_the_fault_marks_it(self):
        """CHANGE: a stop is a fact about the plant, not about typing speed."""
        self._report('PS_DECLARED_LATE')
        late = self._wait_for_fault_in_list('PS_DECLARED_LATE')
        self.assertFalse(
            late['x-medkit']['expected'],
            'nothing covers it yet'
        )
        first_occurred = late['first_occurred']

        window = self._declare({
            'from': self._iso(-300),
            'to': self._iso(300),
            'reason': 'declared after the fact',
        })

        # The cache behind the flag re-reads on a short time to live; a write
        # through this gateway invalidates it, so the next read already sees it.
        deadline = time.monotonic() + FAULT_TIMEOUT
        while time.monotonic() < deadline:
            item = self._find(self.get_json('/faults?status=all'), 'PS_DECLARED_LATE')
            if item is not None and item['x-medkit']['expected']:
                self.assertEqual(item['x-medkit']['planned_stop_id'], window['id'])
                self.assertEqual(item['first_occurred'], first_occurred,
                                 'declaring a window must not touch the fault')
                return
            time.sleep(0.3)
        raise AssertionError(
            'a window declared after the fault did not mark it within '
            f'{FAULT_TIMEOUT}s'
        )

    def test_08_a_new_cycle_after_a_clear_is_judged_on_its_own(self):
        """first_occurred moves with the cycle, and so must the flag."""
        # Cycle one, inside a window.
        window = self._declare({'to': self._iso(3600), 'reason': 'cycle one'})
        self._report('PS_CYCLE')
        first = self._wait_for_fault_in_list('PS_CYCLE')
        self.assertTrue(first['x-medkit']['expected'])
        self.assertEqual(first['occurrence_count'], 1)

        self._clear('PS_CYCLE')
        requests.delete(f'{self.BASE_URL}{PLANNED_STOPS}/{window["id"]}', timeout=10)

        # Cycle two, with no window open.
        self._report('PS_CYCLE')
        deadline = time.monotonic() + FAULT_TIMEOUT
        while time.monotonic() < deadline:
            item = self._find(self.get_json('/faults?status=all'), 'PS_CYCLE')
            if item is not None and item.get('occurrence_count') == 2:
                self.assertFalse(
                    item['x-medkit']['expected'],
                    'the second cycle started outside every window'
                )
                return
            time.sleep(0.3)
        raise AssertionError('PS_CYCLE never reached a second occurrence')

    # --- the REST surface itself -------------------------------------------

    def test_09_windows_are_listed_read_back_and_filterable_by_active(self):
        active = self._declare({'to': self._iso(3600), 'reason': 'still running'})
        past = self._declare({
            'from': self._iso(-7200), 'to': self._iso(-3600), 'reason': 'finished',
        })

        listing = self.get_json(PLANNED_STOPS)
        ids = [w['id'] for w in listing['items']]
        self.assertIn(active['id'], ids)
        self.assertIn(past['id'], ids, 'an ended window is why old faults read as expected')

        only_active = self.get_json(f'{PLANNED_STOPS}?active=true')
        active_ids = [w['id'] for w in only_active['items']]
        self.assertIn(active['id'], active_ids)
        self.assertNotIn(past['id'], active_ids)

        one = self.get_json(f'{PLANNED_STOPS}/{active["id"]}')
        self.assertEqual(one['id'], active['id'])
        self.assertEqual(one['reason'], 'still running')

        missing = requests.get(f'{self.BASE_URL}{PLANNED_STOPS}/no_such_window', timeout=10)
        self.assertEqual(missing.status_code, 404)

        gone = requests.delete(
            f'{self.BASE_URL}{PLANNED_STOPS}/no_such_window', timeout=10
        )
        self.assertEqual(gone.status_code, 404)

    def test_10_a_window_that_is_not_an_interval_is_refused(self):
        now = self._iso(0)
        self._declare({'to': now, 'from': now, 'reason': 'zero length'},
                      expected_status=400)
        self._declare({'from': self._iso(600), 'to': self._iso(60), 'reason': 'reversed'},
                      expected_status=400)
        self._declare({'to': 'yesterday afternoon', 'reason': 'unparsable'},
                      expected_status=400)
        self._declare({'to': '2026-09-06T12:00:00+02:00', 'reason': 'not utc'},
                      expected_status=400)
        self._declare({'from': 'nonsense', 'to': self._iso(600), 'reason': 'bad from'},
                      expected_status=400)

        # builtin_interfaces/Time carries its seconds in an int32, so a window
        # past January 2038 has no representation. Refused rather than wrapped:
        # the narrowing would hand back a window with a negative end, covering
        # nothing, while telling the operator it was accepted.
        self._declare({'to': '2099-01-01T00:00:00Z', 'reason': 'past 2038'},
                      expected_status=400)

        # A missing `to` is the request DTO's business, and it must still be 400.
        response = requests.post(
            f'{self.BASE_URL}{PLANNED_STOPS}', json={'reason': 'no end'}, timeout=10
        )
        self.assertEqual(response.status_code, 400, response.text)

    def test_11_an_absent_from_means_now(self):
        before = time.time()
        window = self._declare({'to': self._iso(3600), 'reason': 'starts now'})
        after = time.time()

        started = calendar.timegm(
            time.strptime(window['from'][:19], '%Y-%m-%dT%H:%M:%S')
        )
        # Two seconds of slack: the gateway stamps the window from the fault
        # manager's clock, and the assertion is that it is "now", not that the
        # two machines agree to the millisecond.
        self.assertGreaterEqual(started, before - 2.0)
        self.assertLessEqual(started, after + 2.0)

    def test_12_the_expected_filter_refuses_a_value_it_does_not_define(self):
        response = requests.get(
            f'{self.BASE_URL}/faults?status=all&expected=maybe', timeout=10
        )
        self.assertEqual(response.status_code, 400, response.text)
        self.assertEqual(response.json()['parameters']['parameter'], 'expected')

        scoped = requests.get(
            f'{self.BASE_URL}/apps/{APP_ID}/faults?expected=perhaps', timeout=10
        )
        self.assertEqual(scoped.status_code, 400, scoped.text)

    def test_13_root_endpoint_advertises_the_routes(self):
        data = self.get_json('/')
        self.assertIn(f'POST /api/v1{PLANNED_STOPS}', data['endpoints'])
        self.assertIn(f'GET /api/v1{PLANNED_STOPS}', data['endpoints'])
        self.assertIn(
            f'GET /api/v1{PLANNED_STOPS}/{{planned_stop_id}}', data['endpoints']
        )
        self.assertIn(
            f'DELETE /api/v1{PLANNED_STOPS}/{{planned_stop_id}}', data['endpoints']
        )

    # --- stream plumbing ----------------------------------------------------

    @staticmethod
    def _pump_stream(response, frames, stop_event):
        current = {}
        try:
            for line in response.iter_lines(decode_unicode=True):
                if stop_event.is_set():
                    break
                if line is None:
                    continue
                if line == '':
                    if current:
                        frames.append(current)
                        current = {}
                    continue
                if line.startswith(':'):
                    continue  # keepalive comment
                key, _, value = line.partition(':')
                current[key.strip()] = value.strip()
        except Exception:  # noqa: BLE001 - closed socket on test teardown
            pass

    def _prime_stream(self, frames):
        """Block until the event pipeline demonstrably reaches the stream.

        /fault_manager/events is reliable but volatile: an event published before
        the gateway's subscription has matched is lost outright. Repeating a
        sacrificial fault until one of its frames arrives proves the whole chain
        is live before anything is asserted on it.
        """
        deadline = time.monotonic() + FAULT_TIMEOUT
        while time.monotonic() < deadline:
            self._report('PS_STREAM_PRIME')
            settle = min(time.monotonic() + 1.0, deadline)
            while time.monotonic() < settle:
                for frame in list(frames):
                    data = frame.get('data')
                    if data and json.loads(data).get(
                            'fault', {}).get('fault_code') == 'PS_STREAM_PRIME':
                        return
                time.sleep(0.1)
        raise AssertionError('the fault event pipeline never reached the stream')

    def _wait_for_frame(self, frames, fault_code):
        deadline = time.monotonic() + FAULT_TIMEOUT
        while time.monotonic() < deadline:
            for frame in list(frames):
                data = frame.get('data')
                if not data:
                    continue
                payload = json.loads(data)
                if payload.get('fault', {}).get('fault_code') == fault_code:
                    return payload
            time.sleep(0.2)
        raise AssertionError(
            f'no frame for {fault_code} on /faults/stream within {FAULT_TIMEOUT}s'
        )


@launch_testing.post_shutdown_test()
class TestPlannedStopsShutdown(unittest.TestCase):

    def test_gateway_exits_cleanly(self, proc_info, gateway_node):
        launch_testing.asserts.assertExitCodes(
            proc_info, process=gateway_node, allowable_exit_codes=ALLOWED_EXIT_CODES
        )
