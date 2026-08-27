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

"""End-to-end specification for /faults/stream on an aggregating gateway.

An aggregator runs on its own ROS domain, with its own fault_manager that no
producer reports to. A stream fed from that graph alone is open, valid, and
sends nothing but keepalives - and a client reading it reports the system as
healthy while its peers are on fire. In a deployment where the aggregator is
the only port an operator can reach, that is the only fault stream there is.

THE RULES

S1  A fault raised on a peer reaches a client attached to the AGGREGATOR's
    stream, and says which peer it came from. Asserting "some event arrived"
    would not show this: the aggregator has a fault_manager of its own, so an
    event with no attribution proves nothing about where it was raised.
S2  The peer's own stream still carries it. The relay is an addition, not a
    redirection.
S3  The relay costs a peer SSE client slot only while somebody is listening.
    Measured on the PEER's own occupancy (``x-medkit-sse`` on its ``/health``),
    because the claim is about a connection the peer is holding and only the
    peer can answer that. A count of calls made from here would say nothing.
    ``sse.max_clients`` defaults to 2, so an aggregator holding one at rest is
    an outage waiting for the second operator.
S4  A stream request carrying X-Medkit-No-Fan-Out is served from the local
    graph only. That header is what an aggregating peer sends when it relays,
    and without it a chain of them would carry one event round the loop for
    ever.
"""

import json
import os
import threading
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
from ros2_medkit_msgs.srv import ReportFault
from ros2_medkit_test_utils.constants import (
    ALLOWED_EXIT_CODES,
    API_BASE_PATH,
    DISCOVERY_TIMEOUT,
    FAULT_TIMEOUT,
    get_test_domain_id,
    get_test_port,
    get_time_scale,
)
from ros2_medkit_test_utils.launch_helpers import (
    create_fault_manager_node,
    create_gateway_node,
)

AGG_PORT = get_test_port(0)
PEER_PORT = get_test_port(1)
AGG_URL = f'http://localhost:{AGG_PORT}{API_BASE_PATH}'
PEER_URL = f'http://localhost:{PEER_PORT}{API_BASE_PATH}'

# The launch default domain is the PEER's, because the test process itself has
# to reach the peer's fault_manager over ROS to raise anything at all. The
# aggregator is the one pushed onto a domain of its own, which is also the
# deployment being described: it shares a graph with nothing.
PEER_DOMAIN_ID = get_test_domain_id(0)
AGG_DOMAIN_ID = get_test_domain_id(1)

PEER_NAME = 'remote_gateway'
FAULT_CODE = 'AGG_STREAM_E2E'
PRIME_CODE = 'AGG_STREAM_PRIME'
LOCAL_CODE = 'AGG_STREAM_LOCAL_ONLY'
SOURCE_ID = '/powertrain/engine/temp_sensor'
SERVICE_NAME = '/fault_manager/report_fault'
FAULT_MANAGER_NODE = 'fault_manager'

TIMEOUT = DISCOVERY_TIMEOUT * get_time_scale()

# Both gateways run with this, so a disconnect is seen in about a second rather
# than thirty. The test asserts on WHETHER a slot is released, not on how fast,
# so a short interval costs it no falsifying power.
KEEPALIVE_SEC = 2


def generate_test_description():
    agg_env = {'ROS_DOMAIN_ID': str(AGG_DOMAIN_ID)}

    aggregator = create_gateway_node(
        name='aggregating_gateway_node',
        port=AGG_PORT,
        extra_params={
            'aggregation.enabled': True,
            'aggregation.peer_urls': [f'http://localhost:{PEER_PORT}'],
            'aggregation.peer_names': [PEER_NAME],
            # Held above the default so a relay opened while a client attaches
            # is not competing with this gateway's own stream for slots. The
            # peer keeps the DEFAULT, because S3 is a claim about what the
            # relay costs a peer that was never configured for it.
            'sse.max_clients': 6,
            'server.http_thread_pool_size': 16,
            # A closed SSE connection is noticed on the next write, so the
            # keepalive interval is also the detection latency this test waits
            # out three times. At the 30 s default that is most of the test's
            # wall clock and it pushed the CI job past its cap.
            'sse.keepalive_interval_sec': KEEPALIVE_SEC,
        },
        extra_env=agg_env,
    )

    peer_gateway = create_gateway_node(
        name='remote_gateway_node',
        port=PEER_PORT,
        # Pinned rather than inherited: S1 holds the relay and a direct
        # consumer at once, and a closed stream is still counted until the
        # peer's next keepalive, so the default of 2 would refuse a later test
        # for reasons that have nothing to do with what it asserts.
        extra_params={
            'sse.max_clients': 6,
            'server.http_thread_pool_size': 16,
            'sse.keepalive_interval_sec': KEEPALIVE_SEC,
        },
    )

    peer_fault_manager = create_fault_manager_node(
        storage_type='memory',
        rosbag_enabled=False,
        extra_params={
            # One FAILED confirms; the stream carries the transition.
            'confirmation_threshold': -1,
            'healing_enabled': False,
            'snapshots.rosbag.enabled': False,
        },
    )

    # The aggregator deliberately runs NO fault manager of its own. An earlier
    # version launched one so that "an event arrived" could not trivially mean
    # "it was relayed" - but that made the assertion ambiguous instead of
    # falsifiable: both managers answer the same service name, so a report
    # could land on either and a LOCAL event carries no peer tag. What makes
    # the attribution falsifiable is the pair of assertions below: the
    # aggregator's copy must name the peer, and the peer's own copy of the SAME
    # event must not. A gateway that tagged everything, or nothing, fails one
    # of them.

    launch_description = LaunchDescription([
        peer_gateway,
        peer_fault_manager,
        TimerAction(period=1.0, actions=[aggregator]),
        launch_testing.actions.ReadyToTest(),
    ])

    return (
        launch_description,
        {
            'gateway_node': aggregator,
            'peer_gateway': peer_gateway,
        },
    )


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
    except Exception:  # noqa: BLE001 - closed socket on teardown
        pass


class AggregatorFaultStreamTest(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls._wait_for_peer_online()
        # Pinned, not inherited. This node has to land on the PEER's domain,
        # because that is where the fault_manager it reports to lives, and the
        # launch process this runs in does not reliably still carry the domain
        # it started with once an action with an ROS_DOMAIN_ID of its own has
        # run. Inheriting put the reporter on the aggregator's domain instead,
        # where nothing answers report_fault.
        previous_domain = os.environ.get('ROS_DOMAIN_ID')
        os.environ['ROS_DOMAIN_ID'] = str(PEER_DOMAIN_ID)
        try:
            rclpy.init()
            cls._reporter = Node('agg_stream_reporter')
        finally:
            if previous_domain is None:
                del os.environ['ROS_DOMAIN_ID']
            else:
                os.environ['ROS_DOMAIN_ID'] = previous_domain
        cls._report_client = cls._reporter.create_client(
            ReportFault, SERVICE_NAME)
        # Waits for the fault_manager NODE, not for the service name and not
        # for `service_is_ready`. Both of the alternatives lie here. The name
        # `/fault_manager/report_fault` appears on a domain that only holds a
        # CLIENT of it - both gateways create one - so a wait keyed on the
        # service passes on a domain where nothing can answer. And
        # `service_is_ready` is `rcl_service_server_is_available`, which reports
        # whether this client's endpoints have been matched, a separate
        # condition that stayed false for a service that was present and
        # callable. A node carrying that name is the one signal that means the
        # server is really here.
        service_wait = 60.0 * get_time_scale()
        deadline = time.monotonic() + service_wait
        while time.monotonic() < deadline:
            names = [name for name, _ in cls._reporter.get_node_names_and_namespaces()]
            if FAULT_MANAGER_NODE in names:
                return
            rclpy.spin_once(cls._reporter, timeout_sec=0.2)
        raise AssertionError(
            f'the {FAULT_MANAGER_NODE} node never appeared on the peer domain '
            f'{PEER_DOMAIN_ID} within {service_wait}s; nodes seen: '
            f'{cls._reporter.get_node_names_and_namespaces()}')

    @classmethod
    def tearDownClass(cls):
        cls._reporter.destroy_node()
        rclpy.shutdown()

    @classmethod
    def _wait_for_peer_online(cls):
        deadline = time.time() + TIMEOUT
        last = None
        while time.time() < deadline:
            try:
                response = requests.get(f'{AGG_URL}/health', timeout=10)
                if response.status_code == 200:
                    body = response.json()
                    peers = body.get('x-medkit', {}).get('peers') or body.get('peers') or []
                    if any(p.get('status') == 'online' for p in peers):
                        return
                    last = peers
            except requests.RequestException as exc:
                last = str(exc)
            time.sleep(0.5)
        raise AssertionError(f'the aggregator never saw its peer come online; last saw {last}')

    def _report(self, fault_code):
        req = ReportFault.Request()
        req.fault_code = fault_code
        req.event_type = ReportFault.Request.EVENT_FAILED
        req.severity = Fault.SEVERITY_ERROR
        req.description = 'aggregator stream e2e'
        req.source_id = SOURCE_ID
        # Retried, because a request sent before the endpoints finish matching
        # is dropped rather than queued, and nothing tells the caller that
        # happened - the future simply never completes. Reissuing is what turns
        # a slow match into a late reply instead of a failed test.
        deadline = time.monotonic() + FAULT_TIMEOUT
        attempts = 0
        while time.monotonic() < deadline:
            future = self._report_client.call_async(req)
            attempts += 1
            attempt_deadline = min(time.monotonic() + 5.0, deadline)
            while not future.done() and time.monotonic() < attempt_deadline:
                rclpy.spin_once(self._reporter, timeout_sec=0.1)
            if future.done():
                self.assertTrue(future.result().accepted)
                return
            self._report_client.remove_pending_request(future)
        self.fail(
            f'report_fault never replied for {fault_code} after {attempts} attempt(s) '
            f'over {FAULT_TIMEOUT}s; nodes seen: '
            f'{self._reporter.get_node_names_and_namespaces()}')

    def _open_stream(self, url, headers=None):
        response = requests.get(url, stream=True, timeout=(10, 120), headers=headers or {})
        self.assertEqual(response.status_code, 200, f'{url} did not open: {response.status_code}')
        frames = []
        stop = threading.Event()
        pump = threading.Thread(target=_pump_stream, args=(response, frames, stop), daemon=True)
        pump.start()

        def close():
            stop.set()
            response.close()
            pump.join(timeout=5)

        self.addCleanup(close)
        return frames

    def _await_fault(self, frames, fault_code, timeout):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            for frame in list(frames):
                data = frame.get('data')
                if not data:
                    continue
                payload = json.loads(data)
                if payload.get('fault', {}).get('fault_code') == fault_code:
                    return payload
            time.sleep(0.2)
        return None

    def _peer_sse_clients(self):
        """How many SSE streams the PEER is holding, from its own /health."""
        response = requests.get(f'{PEER_URL}/health', timeout=10)
        self.assertEqual(response.status_code, 200, response.text)
        body = response.json()
        sse = body.get('x-medkit-sse')
        self.assertIsInstance(
            sse, dict,
            f"the peer's /health does not report SSE occupancy, so this test has no "
            f'instrument: {body}')
        return sse['connected_clients']

    def _agg_sse_clients(self):
        """How many SSE streams the AGGREGATOR is serving, from its own /health."""
        response = requests.get(f'{AGG_URL}/health', timeout=10)
        self.assertEqual(response.status_code, 200, response.text)
        sse = response.json().get('x-medkit-sse')
        self.assertIsInstance(
            sse, dict, f'aggregator /health has no SSE occupancy: {response.text[:300]}')
        return sse['connected_clients']

    def _settle_peer_sse(self, timeout=60 * get_time_scale()):
        """Wait for a quiet system, then return the peer's SSE count.

        Two things have to settle, and only one of them is on the peer. A
        closed SSE connection is noticed at the server's next write, which for
        an idle stream is the keepalive - 30 s away. So an earlier case's
        stream is still counted on the AGGREGATOR for a while, which keeps its
        relay open, which keeps a stream counted on the PEER. Waiting for the
        peer's number to stop moving is not enough: it stops moving at ONE, and
        a measurement taken then attributes a stale relay to the new client.
        """
        deadline = time.monotonic() + timeout
        previous = None
        stable_since = None
        last_agg = None
        while time.monotonic() < deadline:
            last_agg = self._agg_sse_clients()
            current = self._peer_sse_clients()
            if last_agg == 0 and current == previous:
                if stable_since is None:
                    stable_since = time.monotonic()
                elif time.monotonic() - stable_since >= 5:
                    return current
            else:
                previous = current
                stable_since = None
            time.sleep(1)
        raise AssertionError(
            f'the system never went quiet within {timeout}s (aggregator {last_agg} clients, '
            f'peer {previous}); nothing measured against it below would mean anything')

    def _await_peer_sse(self, predicate, timeout, what):
        deadline = time.monotonic() + timeout
        seen = None
        while time.monotonic() < deadline:
            seen = self._peer_sse_clients()
            if predicate(seen):
                return seen
            time.sleep(1)
        raise AssertionError(f'{what}; the peer reports {seen} SSE clients')

    def test_s1_a_peers_fault_reaches_the_aggregators_stream_attributed(self):
        """S1 + S2: the event arrives on both streams, and names its peer."""
        agg_frames = self._open_stream(f'{AGG_URL}/faults/stream')
        peer_frames = self._open_stream(f'{PEER_URL}/faults/stream')

        # /fault_manager/events is reliable but volatile: an event published
        # before the relay's HTTP connection is established is gone. Repeat a
        # sacrificial code until one of its frames lands, which proves the whole
        # chain - report -> peer fault_manager -> peer SSE -> relay -> here.
        primed = None
        deadline = time.monotonic() + FAULT_TIMEOUT
        while primed is None and time.monotonic() < deadline:
            self._report(PRIME_CODE)
            primed = self._await_fault(agg_frames, PRIME_CODE, timeout=2.0)
        self.assertIsNotNone(
            primed,
            f'no relayed event for the priming fault reached the aggregator within '
            f'{FAULT_TIMEOUT}s; frames seen: {list(agg_frames)[:5]}')

        self._report(FAULT_CODE)
        relayed = self._await_fault(agg_frames, FAULT_CODE, timeout=FAULT_TIMEOUT)
        self.assertIsNotNone(
            relayed,
            f'the peer raised {FAULT_CODE} and the aggregator never carried it; '
            f'frames seen: {list(agg_frames)[:5]}')
        self.assertEqual(
            relayed.get('x-medkit', {}).get('peer'), PEER_NAME,
            f'a relayed event must name the gateway that raised it, or a client cannot tell '
            f'it from one this aggregator raised itself: {relayed}')

        # S2: the relay adds a consumer, it does not take the peer's own away.
        direct = self._await_fault(peer_frames, FAULT_CODE, timeout=FAULT_TIMEOUT)
        self.assertIsNotNone(direct, "the peer's own stream lost the event")
        self.assertNotIn(
            'peer', direct.get('x-medkit', {}),
            f'the gateway that raised the fault must not attribute it to a peer: {direct}')

    def test_s3_the_relay_holds_a_peer_slot_only_while_a_client_listens(self):
        """S3: the peer's own occupancy, before, during and after."""
        idle_before = self._settle_peer_sse()

        response = requests.get(f'{AGG_URL}/faults/stream', stream=True, timeout=(10, 60))
        self.assertEqual(response.status_code, 200)
        frames = []
        stop = threading.Event()
        pump = threading.Thread(target=_pump_stream, args=(response, frames, stop), daemon=True)
        pump.start()
        try:
            during = self._await_peer_sse(
                lambda n: n == idle_before + 1, timeout=30 * get_time_scale(),
                what=(f'a client on the aggregator should cost the peer exactly one '
                      f'stream, and the peer held {idle_before} before it attached'))
            self.assertEqual(during, idle_before + 1)
        finally:
            stop.set()
            response.close()
            pump.join(timeout=5)

        # Up to the peer's keepalive interval: it learns the relay is gone at
        # its next write, not at the close.
        self._await_peer_sse(
            lambda n: n <= idle_before, timeout=30 * get_time_scale(),
            what=(f'the aggregator kept a stream open on the peer after its last client '
                  f'left, against {idle_before} at rest. sse.max_clients defaults to 2, so '
                  f'an idle aggregator holding one is an outage waiting for the second '
                  f'operator to connect'))

    def test_s4_a_no_fan_out_request_is_served_from_the_local_graph_only(self):
        """S4: the header an aggregating peer sends must not open a relay."""
        idle_before = self._settle_peer_sse()

        frames = self._open_stream(
            f'{AGG_URL}/faults/stream', headers={'X-Medkit-No-Fan-Out': '1'})

        # The peer must not be asked for anything. Given time to be wrong: the
        # relay connects within a second or so when it is going to.
        time.sleep(10)
        self.assertEqual(
            self._peer_sse_clients(), idle_before,
            'a request carrying X-Medkit-No-Fan-Out cost the peer a stream, so it opened a '
            'relay; a chain of aggregating gateways would carry one event round the loop')

        self._report(LOCAL_CODE)
        leaked = self._await_fault(frames, LOCAL_CODE, timeout=10 * get_time_scale())
        self.assertIsNone(
            leaked,
            f'a suppressed stream carried a peer-owned event anyway: {leaked}')
        if leaked is not None:  # pragma: no cover - guarded by the assert above
            self.assertNotIn('peer', leaked.get('x-medkit', {}))


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info, allowable_exit_codes=ALLOWED_EXIT_CODES)
