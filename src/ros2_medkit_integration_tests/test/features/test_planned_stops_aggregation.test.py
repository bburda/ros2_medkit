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

"""The planned-stop flag through an aggregating gateway.

Windows are per fault manager: the peer's gateway derives the flag for the
peer's faults from the peer's own windows, and the aggregator relays what the
peer said. Two things have to hold through that relay, and neither is visible
from a single gateway.

A1  ``?expected=`` reaches the peer and is applied to what comes back. Asserting
    only that the aggregator returns SOMETHING would pass against a gateway that
    forwarded a path with no query on it - which is what it did: the peer
    answered its whole list and every item came through, flag and all.
A2  ``expected_count`` counts the list that was served, peer items included.
    A count taken before the merge describes the local half and nothing else.
A3  The whole query reaches the peer, not just ``expected``. ``?status=`` was
    dropped by the same line, and unlike ``expected`` the aggregator has no
    second chance at it: a peer asked nothing answers its DEFAULT status set, so
    a request for cleared faults came back carrying the peer's confirmed ones and
    missing its cleared one. This is the assertion the forwarding has to satisfy
    on its own - filtering peer items locally, which A1 also allows, cannot.

The aggregator runs a fault manager of its own - ``GET /faults`` answers 503
without one - but nothing ever reports to it, and it sits on a domain of its
own, so every fault in the merged list demonstrably came from the peer.
"""

import os
import time
import unittest

from launch import LaunchDescription
from launch.actions import TimerAction
import launch_testing
import launch_testing.actions
import rclpy
from rclpy.node import Node
import requests
from ros2_medkit_msgs.msg import Fault
from ros2_medkit_msgs.srv import ClearFault, ReportFault

from ros2_medkit_test_utils.constants import (
    ALLOWED_EXIT_CODES,
    API_BASE_PATH,
    FAULT_TIMEOUT,
    GATEWAY_STARTUP_TIMEOUT,
    get_test_domain_id,
    get_test_port,
)
from ros2_medkit_test_utils.launch_helpers import (
    create_fault_manager_node,
    create_gateway_node,
)

AGG_PORT = get_test_port(0)
PEER_PORT = get_test_port(1)
AGG_URL = f'http://localhost:{AGG_PORT}{API_BASE_PATH}'
PEER_URL = f'http://localhost:{PEER_PORT}{API_BASE_PATH}'

# The launch default domain is the PEER's: the test process has to reach the
# peer's fault_manager over ROS to raise anything at all. The aggregator is
# pushed onto a domain of its own, which is also the deployment being described.
PEER_DOMAIN_ID = get_test_domain_id(0)
AGG_DOMAIN_ID = get_test_domain_id(1)

PLANNED_STOPS = '/x-medkit-planned-stops'
PEER_NAME = 'remote_gateway'
INSIDE_CODE = 'PS_PEER_INSIDE_WINDOW'
OUTSIDE_CODE = 'PS_PEER_OUTSIDE_WINDOW'
SOURCE_ID = '/powertrain/engine/temp_sensor'


def generate_test_description():
    aggregator = create_gateway_node(
        name='aggregating_gateway_node',
        port=AGG_PORT,
        extra_params={
            'aggregation.enabled': True,
            'aggregation.timeout_ms': 5000,
            'aggregation.announce': False,
            'aggregation.discover': False,
            'aggregation.peer_urls': [f'http://localhost:{PEER_PORT}'],
            'aggregation.peer_names': [PEER_NAME],
            'server.http_thread_pool_size': 16,
        },
        extra_env={'ROS_DOMAIN_ID': str(AGG_DOMAIN_ID)},
    )

    peer_gateway = create_gateway_node(
        name='remote_gateway_node',
        port=PEER_PORT,
        extra_params={'server.http_thread_pool_size': 16},
    )

    # The aggregator needs a fault manager to answer /faults at all; nothing
    # reports to it, and it is on the aggregator's own domain, so the merged
    # list is exactly what the peer contributed.
    # Default node name on purpose: the gateway addresses its fault manager by
    # the /fault_manager service namespace, which the node name sets. The two
    # managers are on different DDS domains, so the shared name collides with
    # nothing.
    aggregator_fault_manager = create_fault_manager_node(
        storage_type='memory',
        rosbag_enabled=False,
        extra_params={'snapshots.rosbag.enabled': False},
        extra_env={'ROS_DOMAIN_ID': str(AGG_DOMAIN_ID)},
    )

    peer_fault_manager = create_fault_manager_node(
        storage_type='memory',
        rosbag_enabled=False,
        extra_params={
            # One FAILED confirms, so a fault is in the list one call after it
            # is raised.
            'confirmation_threshold': -1,
            'healing_enabled': False,
            'snapshots.rosbag.enabled': False,
        },
    )

    launch_description = LaunchDescription([
        peer_gateway,
        peer_fault_manager,
        TimerAction(period=1.0, actions=[aggregator, aggregator_fault_manager]),
        launch_testing.actions.ReadyToTest(),
    ])

    return (
        launch_description,
        {'gateway_node': aggregator, 'peer_gateway': peer_gateway},
    )


class TestPlannedStopsAggregation(unittest.TestCase):
    """The flag and the filter have to survive the relay."""

    @classmethod
    def setUpClass(cls):
        cls._wait_for(AGG_URL)
        cls._wait_for(PEER_URL)

        # Pinned rather than inherited: this node has to land on the PEER's
        # domain, where the fault_manager it reports to lives.
        previous_domain = os.environ.get('ROS_DOMAIN_ID')
        os.environ['ROS_DOMAIN_ID'] = str(PEER_DOMAIN_ID)
        try:
            rclpy.init()
            cls._reporter = Node('planned_stop_agg_reporter')
        finally:
            if previous_domain is None:
                os.environ.pop('ROS_DOMAIN_ID', None)
            else:
                os.environ['ROS_DOMAIN_ID'] = previous_domain

        cls._report_client = cls._reporter.create_client(
            ReportFault, '/fault_manager/report_fault'
        )
        cls._clear_client = cls._reporter.create_client(
            ClearFault, '/fault_manager/clear_fault'
        )
        assert cls._report_client.wait_for_service(timeout_sec=30.0), \
            'peer report_fault service not available'
        assert cls._clear_client.wait_for_service(timeout_sec=30.0), \
            'peer clear_fault service not available'

    @classmethod
    def tearDownClass(cls):
        cls._reporter.destroy_node()
        rclpy.shutdown()

    @staticmethod
    def _wait_for(base_url):
        deadline = time.monotonic() + GATEWAY_STARTUP_TIMEOUT
        while time.monotonic() < deadline:
            try:
                if requests.get(f'{base_url}/health', timeout=2).status_code == 200:
                    return
            except requests.RequestException:
                pass
            time.sleep(0.5)
        raise AssertionError(f'{base_url} never became healthy')

    def _report(self, fault_code):
        request = ReportFault.Request()
        request.fault_code = fault_code
        request.event_type = ReportFault.Request.EVENT_FAILED
        request.severity = Fault.SEVERITY_ERROR
        request.description = 'peer fault for the aggregated planned-stop flag'
        request.source_id = SOURCE_ID
        future = self._report_client.call_async(request)
        deadline = time.monotonic() + 10.0
        while not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(self._reporter, timeout_sec=0.1)
        self.assertTrue(future.done(), f'report_fault({fault_code}) timed out')
        self.assertTrue(future.result().accepted)

    @staticmethod
    def _items(url):
        response = requests.get(url, timeout=15)
        assert response.status_code == 200, f'{url} -> {response.status_code}: {response.text}'
        return response.json()

    def _wait_for_code(self, url, fault_code):
        deadline = time.monotonic() + FAULT_TIMEOUT
        while time.monotonic() < deadline:
            payload = self._items(url)
            for item in payload.get('items', []):
                if item.get('fault_code') == fault_code:
                    return payload, item
            time.sleep(0.5)
        raise AssertionError(f'{fault_code} never reached {url}')

    def _clear(self, fault_code):
        request = ClearFault.Request()
        request.fault_code = fault_code
        future = self._clear_client.call_async(request)
        deadline = time.monotonic() + 10.0
        while not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(self._reporter, timeout_sec=0.1)
        self.assertTrue(future.done(), f'clear_fault({fault_code}) timed out')
        self.assertTrue(future.result().success, future.result().message)

    def test_the_flag_and_the_filter_survive_the_relay(self):
        # Declared on the PEER, because that is where the fault manager holding
        # the window and the faults it covers lives.
        window = requests.post(
            f'{PEER_URL}{PLANNED_STOPS}',
            json={'to': time.strftime('%Y-%m-%dT%H:%M:%SZ',
                                      time.gmtime(time.time() + 3600)),
                  'reason': 'peer changeover'},
            timeout=10,
        )
        self.assertEqual(window.status_code, 201, window.text)
        window_id = window.json()['id']

        self._report(INSIDE_CODE)
        _, inside = self._wait_for_code(f'{PEER_URL}/faults?status=all', INSIDE_CODE)
        self.assertTrue(
            inside['x-medkit']['expected'],
            'the PEER must flag its own fault before the relay can carry it'
        )

        # A1: the aggregator's merged list carries the peer's flag...
        payload, relayed = self._wait_for_code(f'{AGG_URL}/faults?status=all', INSIDE_CODE)
        self.assertTrue(
            relayed['x-medkit']['expected'],
            'the aggregator dropped or overwrote the flag the peer derived'
        )
        self.assertEqual(relayed['x-medkit']['planned_stop_id'], window_id)

        # A2: ...and the count describes the merged list, not the local half.
        flagged = [i for i in payload['items'] if i.get('x-medkit', {}).get('expected')]
        self.assertGreaterEqual(len(flagged), 1)
        self.assertEqual(
            payload['x-medkit']['expected_count'], len(flagged),
            'expected_count must count the list that was served, peer items included'
        )

        # End the window on the peer, then raise a second fault there: same
        # aggregator, same route, opposite answer.
        ended = requests.delete(f'{PEER_URL}{PLANNED_STOPS}/{window_id}', timeout=10)
        self.assertEqual(ended.status_code, 200, ended.text)

        self._report(OUTSIDE_CODE)
        self._wait_for_code(f'{AGG_URL}/faults?status=all', OUTSIDE_CODE)

        # A1 again, through the filter this time. Without the query forwarded,
        # the peer answered its whole list and both codes came through.
        deadline = time.monotonic() + FAULT_TIMEOUT
        while time.monotonic() < deadline:
            only_expected = self._items(f'{AGG_URL}/faults?status=all&expected=true')
            codes = [i['fault_code'] for i in only_expected['items']]
            if INSIDE_CODE in codes and OUTSIDE_CODE not in codes:
                break
            time.sleep(0.5)
        self.assertIn(INSIDE_CODE, codes)
        self.assertNotIn(
            OUTSIDE_CODE, codes,
            'a peer fault outside every window must not survive ?expected=true'
        )
        for item in only_expected['items']:
            self.assertTrue(item['x-medkit']['expected'])
        self.assertEqual(
            only_expected['x-medkit']['expected_count'], len(only_expected['items'])
        )

        only_unexpected = self._items(f'{AGG_URL}/faults?status=all&expected=false')
        codes = [i['fault_code'] for i in only_unexpected['items']]
        self.assertIn(OUTSIDE_CODE, codes)
        self.assertNotIn(INSIDE_CODE, codes)
        self.assertEqual(only_unexpected['x-medkit']['expected_count'], 0)

        # A3: the WHOLE query has to reach the peer. Clear one of the two peer
        # faults and ask the aggregator for cleared ones only. A peer that was
        # asked nothing answers its default set - pending and confirmed - so a
        # gateway that dropped the query returns the fault it was not asked for
        # and loses the one it was.
        self._clear(INSIDE_CODE)
        deadline = time.monotonic() + FAULT_TIMEOUT
        cleared_codes = []
        while time.monotonic() < deadline:
            cleared_only = self._items(f'{AGG_URL}/faults?status=cleared')
            cleared_codes = [i['fault_code'] for i in cleared_only['items']]
            if INSIDE_CODE in cleared_codes:
                break
            time.sleep(0.5)
        self.assertIn(
            INSIDE_CODE, cleared_codes,
            'the peer was never asked for cleared faults, so it did not send any'
        )
        self.assertNotIn(
            OUTSIDE_CODE, cleared_codes,
            'the peer answered its default status set instead of the one requested'
        )


@launch_testing.post_shutdown_test()
class TestPlannedStopsAggregationShutdown(unittest.TestCase):

    def test_gateways_exit_cleanly(self, proc_info, gateway_node, peer_gateway):
        for process in (gateway_node, peer_gateway):
            launch_testing.asserts.assertExitCodes(
                proc_info, process=process, allowable_exit_codes=ALLOWED_EXIT_CODES
            )
