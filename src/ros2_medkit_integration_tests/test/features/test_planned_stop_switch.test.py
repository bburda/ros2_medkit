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

"""The planned-stop switch driven the way an operator reaches it.

No route was added for the switch: the fault manager's services are SOVD
operations on its own App entity, so an operator turns the stop on with
``POST /apps/fault_manager/operations/set_planned_stop/executions``. This test
drives it that way, watches ``GET /faults`` and the ``/faults/stream`` SSE feed,
and asserts what an operator sees: nothing about the faults raised during the
stop until it ends, then every one of them at once.

The stream instrument counts ``fault_confirmed`` frames PER FAULT CODE, before
and after the switch-off. A total alone would pass while the wrong fault was
announced.
"""

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
from ros2_medkit_msgs.srv import ReportFault

from ros2_medkit_test_utils.constants import ALLOWED_EXIT_CODES, FAULT_TIMEOUT
from ros2_medkit_test_utils.gateway_test_case import GatewayTestCase
from ros2_medkit_test_utils.launch_helpers import create_test_launch

SOURCE_ID = '/powertrain/engine/temp_sensor'
MUTED_CODE = 'GW_PS_MUTED'
SECOND_CODE = 'GW_PS_MUTED_TWO'
AFTER_CODE = 'GW_PS_AFTER'
ALREADY_UP_CODE = 'GW_PS_ALREADY_UP'

# How long to keep watching after the counts are right. The publish path is one
# service call plus a topic hop plus the SSE writer, so a second frame that the
# assertion could otherwise race arrives well inside this.
SETTLE_SEC = 4.0

SET_STOP = '/apps/fault_manager/operations/set_planned_stop/executions'
GET_STOP = '/apps/fault_manager/operations/get_planned_stop/executions'


def generate_test_description():
    return create_test_launch(
        demo_nodes=['temp_sensor'],
        fault_manager=True,
        fault_manager_params={
            'storage_type': 'memory',
            'confirmation_threshold': -1,
            'snapshots.rosbag.enabled': False,
        },
    )


class TestPlannedStopSwitch(GatewayTestCase):
    """An operator declares a stop over SOVD and the fault view obeys it."""

    MIN_EXPECTED_APPS = 2
    # The fault manager is an App like any other node, which is exactly why the
    # switch needs no route of its own.
    REQUIRED_APPS = {'temp_sensor', 'fault_manager'}

    @classmethod
    def setUpClass(cls):
        super().setUpClass()
        rclpy.init()
        cls._reporter = Node('planned_stop_switch_reporter')
        cls._report_client = cls._reporter.create_client(
            ReportFault, '/fault_manager/report_fault')
        assert cls._report_client.wait_for_service(timeout_sec=20.0), \
            'report_fault service not available'

        # One stream for the whole class. Frames accumulate across the cases and
        # every assertion counts them per fault code, so there is nothing to
        # reset between tests - and a stream reopened per test costs far more
        # than it proves, because the gateway only notices a closed SSE client
        # on its next write.
        cls._frames = []
        cls._stop_event = threading.Event()
        cls._response = requests.get(
            f'{cls.BASE_URL}/faults/stream', stream=True, timeout=(5, 300))
        assert cls._response.status_code == 200, 'the fault stream did not open'
        cls._pump = threading.Thread(
            target=cls._pump_stream, args=(cls._response, cls._frames, cls._stop_event),
            daemon=True)
        cls._pump.start()
        cls._prime_stream()

    @classmethod
    def tearDownClass(cls):
        cls._stop_event.set()
        cls._response.close()
        cls._pump.join(timeout=10)
        cls._reporter.destroy_node()
        rclpy.shutdown()

    def tearDown(self):
        # One declaration per manager: a test that left it on would decide the
        # next one's outcome.
        self._set_stop(False, reason='teardown', declared_by='test')

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    @classmethod
    def _report(cls, fault_code):
        request = ReportFault.Request()
        request.fault_code = fault_code
        request.event_type = ReportFault.Request.EVENT_FAILED
        request.severity = Fault.SEVERITY_ERROR
        request.description = 'planned stop switch test fault'
        request.source_id = SOURCE_ID
        future = cls._report_client.call_async(request)
        deadline = time.monotonic() + 10.0
        while not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(cls._reporter, timeout_sec=0.05)
        assert future.done(), 'report_fault call timed out'
        assert future.result().accepted, 'report_fault was rejected'

    @staticmethod
    def _pump_stream(response, frames, stop_event):
        """Collect SSE frames as dicts of field -> value."""
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

    @classmethod
    def _prime_stream(cls):
        """Block until the fault event pipeline demonstrably reaches the stream.

        ``/fault_manager/events`` is reliable but volatile: an event published
        before the gateway's subscription has matched the publisher is lost, and
        a one-shot assertion would race that match.
        """
        deadline = time.monotonic() + FAULT_TIMEOUT
        attempt = 0
        while time.monotonic() < deadline:
            # A fresh code per attempt: re-reporting one that is already
            # CONFIRMED publishes fault_updated, never fault_confirmed.
            code = f'GW_PS_PRIME_{attempt}'
            cls._report(code)
            settle = min(time.monotonic() + 1.0, deadline)
            while time.monotonic() < settle:
                if cls._count_confirmed(code) > 0:
                    return
                time.sleep(0.1)
            attempt += 1
        raise AssertionError(
            f'no priming event on /faults/stream within {FAULT_TIMEOUT}s; '
            'the events pipeline never went live')

    @classmethod
    def _count_confirmed(cls, fault_code):
        count = 0
        for frame in list(cls._frames):
            if frame.get('event') != 'fault_confirmed' or 'data' not in frame:
                continue
            payload = json.loads(frame['data'])
            if payload.get('fault', {}).get('fault_code') == fault_code:
                count += 1
        return count

    def _set_stop(self, active, *, reason='', declared_by=''):
        response = requests.post(
            f'{self.BASE_URL}{SET_STOP}',
            json={'parameters': {'active': active, 'reason': reason,
                                 'declared_by': declared_by}},
            timeout=20)
        self.assertEqual(200, response.status_code, response.text)
        return response.json()['parameters']

    def _get_stop(self):
        response = requests.post(f'{self.BASE_URL}{GET_STOP}', json={}, timeout=20)
        self.assertEqual(200, response.status_code, response.text)
        return response.json()['parameters']

    def _faults(self, *, include_muted=False):
        """Return the fault list an operator sees, and the set of codes in it.

        No ``status`` filter: the default view (pending + confirmed) is the one
        the switch is supposed to change.
        """
        params = {'include_muted': 'true'} if include_muted else {}
        response = requests.get(f'{self.BASE_URL}/faults', params=params, timeout=15)
        self.assertEqual(200, response.status_code, response.text)
        data = response.json()
        return data, {item['fault_code'] for item in data.get('items', [])}

    def _wait_until(self, predicate, message, timeout=None):
        deadline = time.monotonic() + (timeout if timeout is not None else FAULT_TIMEOUT)
        while time.monotonic() < deadline:
            value = predicate()
            if value:
                return value
            time.sleep(0.2)
        raise AssertionError(message)

    # ------------------------------------------------------------------
    # Cases
    # ------------------------------------------------------------------

    def test_the_switch_is_an_operation_on_the_fault_manager_entity(self):
        """The switch is reachable as an operation on the fault manager entity.

        @verifies REQ_INTEROP_035
        """
        data = self.get_json('/apps/fault_manager/operations')
        operation_ids = {item['id'] for item in data.get('items', [])}
        self.assertIn('set_planned_stop', operation_ids)
        self.assertIn('get_planned_stop', operation_ids)

        state = self._get_stop()
        self.assertFalse(state['active'])

    def test_faults_raised_during_a_stop_surface_only_when_it_ends(self):
        """Faults raised inside a stop stay hidden until it is withdrawn.

        @verifies REQ_INTEROP_012
        """
        declared = self._set_stop(True, reason='line 3 maintenance', declared_by='shift_lead')
        self.assertTrue(declared['success'])
        self.assertFalse(declared['was_active'])

        state = self._get_stop()
        self.assertTrue(state['active'])
        self.assertEqual('line 3 maintenance', state['reason'])
        self.assertEqual('shift_lead', state['declared_by'])

        before_muted = self._faults()[0]['x-medkit']['muted_count']

        self._report(MUTED_CODE)
        self._report(SECOND_CODE)

        # The default view hides them; asking for muted entries shows them, with
        # the switch named as the reason.
        def _both_muted():
            data, codes = self._faults(include_muted=True)
            muted = {entry['fault_code']: entry
                     for entry in data['x-medkit'].get('muted_faults', [])}
            if MUTED_CODE not in muted or SECOND_CODE not in muted:
                return None
            # Asking for muted entries also stops them being stripped from items.
            self.assertIn(MUTED_CODE, codes)
            self.assertIn(SECOND_CODE, codes)
            return muted

        muted = self._wait_until(_both_muted, 'the faults raised during the stop were not marked')
        for code in (MUTED_CODE, SECOND_CODE):
            self.assertEqual('planned_stop', muted[code]['rule_id'])
            self.assertEqual('PLANNED_STOP', muted[code]['root_cause_code'])

        data, visible = self._faults()
        self.assertNotIn(MUTED_CODE, visible)
        self.assertNotIn(SECOND_CODE, visible)
        self.assertGreaterEqual(data['x-medkit']['muted_count'], before_muted + 2)

        # And the stream said nothing about either of them.
        self.assertEqual(0, self._count_confirmed(MUTED_CODE))
        self.assertEqual(0, self._count_confirmed(SECOND_CODE))

        withdrawn = self._set_stop(False, reason='line 3 back up', declared_by='shift_lead')
        self.assertTrue(withdrawn['success'])
        self.assertTrue(withdrawn['was_active'])

        self._wait_until(
            lambda: all(code in self._faults()[1] for code in (MUTED_CODE, SECOND_CODE)),
            'the faults the stop muted never appeared in the default list after it ended')
        self._wait_until(
            lambda: self._count_confirmed(MUTED_CODE) == 1
            and self._count_confirmed(SECOND_CODE) == 1,
            'the stream did not carry exactly one confirmation per released fault')

        # Reaching one is not the claim: staying at one is. A duplicate frame
        # published a moment later would satisfy the poll above and never be seen.
        time.sleep(SETTLE_SEC)
        self.assertEqual(1, self._count_confirmed(MUTED_CODE),
                         'a second confirmation arrived after the first')
        self.assertEqual(1, self._count_confirmed(SECOND_CODE),
                         'a second confirmation arrived after the first')

        # The declaration stays readable after the plant is back up, with the time
        # it ended, so "why was line 3 quiet?" is answerable over the same route.
        state = self._get_stop()
        self.assertFalse(state['active'])
        self.assertEqual('line 3 maintenance', state['reason'])
        self.assertEqual('shift_lead', state['declared_by'])
        self.assertGreater(state['ended_at']['sec'], 0)

    def test_a_fault_raised_after_the_stop_is_announced_at_once(self):
        """A fault raised once the stop is over is announced immediately.

        @verifies REQ_INTEROP_012
        """
        self._set_stop(True, reason='short pause', declared_by='tech')
        self._set_stop(False, reason='pause over', declared_by='tech')

        self._report(AFTER_CODE)

        self._wait_until(lambda: self._count_confirmed(AFTER_CODE) == 1,
                         'a fault raised after the stop ended was not announced')
        time.sleep(SETTLE_SEC)
        self.assertEqual(1, self._count_confirmed(AFTER_CODE))
        self.assertIn(AFTER_CODE, self._faults()[1])

    def test_a_fault_already_up_when_the_stop_begins_stays_visible(self):
        """A standing alarm is not hidden, and not announced twice.

        @verifies REQ_INTEROP_012
        """
        self._report(ALREADY_UP_CODE)
        self._wait_until(lambda: self._count_confirmed(ALREADY_UP_CODE) == 1,
                         'the fault was never announced before the stop')
        self.assertIn(ALREADY_UP_CODE, self._faults()[1])

        self._set_stop(True, reason='line 3 maintenance', declared_by='shift_lead')

        # The reporter keeps sending FAILED while the condition holds.
        for _ in range(3):
            self._report(ALREADY_UP_CODE)

        data, visible = self._faults(include_muted=True)
        muted_codes = {entry['fault_code']
                       for entry in data['x-medkit'].get('muted_faults', [])}
        self.assertNotIn(ALREADY_UP_CODE, muted_codes,
                         'the stop took over a cycle that started before it')
        self.assertIn(ALREADY_UP_CODE, self._faults()[1],
                      'a standing alarm vanished from GET /faults when the stop began')

        self._set_stop(False, reason='line 3 back up', declared_by='shift_lead')

        time.sleep(SETTLE_SEC)
        self.assertEqual(
            1, self._count_confirmed(ALREADY_UP_CODE),
            'the switch-off announced a confirmation the operator had already seen')


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        """All processes exited cleanly (SIGTERM allowed for SSE teardown)."""
        for info in proc_info:
            self.assertIn(
                info.returncode, ALLOWED_EXIT_CODES,
                f'{info.process_name} exited with code {info.returncode}'
            )
