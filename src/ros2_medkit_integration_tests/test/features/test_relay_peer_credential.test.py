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

"""End-to-end specification for relaying from a peer that requires auth.

An aggregator holds ONE stream per peer, opened when the first local client
attaches and shared by every client after it, so it has no client whose token
it could carry. Against a peer that authenticates its reads it was therefore
refused, and the aggregator's own stream went on answering 200 while carrying
local events only - the same open, valid, empty stream this whole change set
exists to remove, one layer further out.

``aggregation.peer_auth_header`` is what the gateway presents on connections it
opens on its OWN behalf. This test drives the whole path a value in that key
travels: parameter -> AggregationConfig -> PeerClient health check ->
healthy_peer_endpoints -> RelayTarget -> the peer's /faults/stream.

THE RULES

C1  An aggregator holding the credential relays a fault raised on the
    authenticating peer, and says which peer it came from.
C2  An aggregator with no credential relays NOTHING from that peer. This is the
    control: without it C1 would pass just as well against a peer that never
    checked the Authorization header at all, and would prove nothing about the
    credential.
C3  The peer records the aggregator as online only where the credential is
    configured. The health check is the first thing that would 401, and a peer
    reported as down never reaches the relay at all - so this pins WHERE the
    credential is needed, not just that the relay ends up working.
"""

import base64
import hashlib
import hmac
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

AUTHED_AGG_PORT = get_test_port(0)
PEER_PORT = get_test_port(1)
BARE_AGG_PORT = get_test_port(2)

AUTHED_AGG_URL = f'http://localhost:{AUTHED_AGG_PORT}{API_BASE_PATH}'
PEER_URL = f'http://localhost:{PEER_PORT}{API_BASE_PATH}'
BARE_AGG_URL = f'http://localhost:{BARE_AGG_PORT}{API_BASE_PATH}'

# The launch default domain is the PEER's: the test process reports the fault
# over ROS, and the fault_manager it reports to lives there. Each aggregator is
# pushed onto a domain of its own, which is the deployment being described -
# it shares a graph with nothing and HTTP is all it has.
PEER_DOMAIN_ID = get_test_domain_id(0)
AUTHED_AGG_DOMAIN_ID = get_test_domain_id(1)
BARE_AGG_DOMAIN_ID = get_test_domain_id(2)

JWT_SECRET = 'relay_credential_e2e_secret_key_not_a_real_one_0123456789'
JWT_ISSUER = 'relay_credential_e2e'

PEER_NAME = 'authenticating_peer'
FAULT_CODE = 'RELAY_CREDENTIAL_E2E'
SOURCE_ID = '/powertrain/engine/temp_sensor'
SERVICE_NAME = '/fault_manager/report_fault'
FAULT_MANAGER_NODE = 'fault_manager'

TIMEOUT = DISCOVERY_TIMEOUT * get_time_scale()
KEEPALIVE_SEC = 2


def _b64(raw):
    """base64url without padding, which is what JWT uses."""
    return base64.urlsafe_b64encode(raw).rstrip(b'=')


def _mint_access_token(role='admin', lifetime_sec=3600):
    """Sign an HS256 access token the peer gateway will accept.

    Minted here rather than fetched from the peer's /auth/authorize because the
    aggregator needs it as a LAUNCH parameter, and at that point no gateway is
    running yet to issue one. Built on hmac and hashlib from the standard
    library rather than a JWT package: a dependency missing from one CI image
    would fail this on a distro for a reason that has nothing to do with what
    it asserts.

    The gateway verifies HS256 over the shared secret and the issuer, and reads
    the role claim; jwt-cpp additionally enforces exp.
    """
    header = {'alg': 'HS256', 'typ': 'access'}
    now = int(time.time())
    payload = {
        'iss': JWT_ISSUER,
        'sub': 'aggregator',
        'iat': now,
        'exp': now + lifetime_sec,
        'jti': 'relay-credential-e2e',
        'role': role,
    }
    signing_input = (
        _b64(json.dumps(header, separators=(',', ':')).encode())
        + b'.'
        + _b64(json.dumps(payload, separators=(',', ':')).encode())
    )
    signature = hmac.new(JWT_SECRET.encode(), signing_input, hashlib.sha256).digest()
    return (signing_input + b'.' + _b64(signature)).decode()


PEER_CREDENTIAL = f'Bearer {_mint_access_token()}'


def generate_test_description():
    # require_auth_for=all, so READS need a credential too. Under the "write"
    # default the peer would answer /health and /faults/stream to anyone and
    # this test would pass without the feature it is here to check.
    peer_gateway = create_gateway_node(
        name='authenticating_peer_node',
        port=PEER_PORT,
        extra_params={
            'auth.enabled': True,
            'auth.jwt_secret': JWT_SECRET,
            'auth.jwt_algorithm': 'HS256',
            'auth.issuer': JWT_ISSUER,
            'auth.require_auth_for': 'all',
            'auth.clients': ['aggregator:aggregator_secret:admin'],
            # Two aggregators relay from this peer at once and a closed stream
            # is still counted until the next keepalive, so the default of 2
            # would refuse one for reasons unrelated to the credential.
            'sse.max_clients': 6,
            'server.http_thread_pool_size': 16,
            'sse.keepalive_interval_sec': KEEPALIVE_SEC,
        },
    )

    peer_fault_manager = create_fault_manager_node(
        storage_type='memory',
        rosbag_enabled=False,
        extra_params={
            'confirmation_threshold': -1,
            'healing_enabled': False,
            'snapshots.rosbag.enabled': False,
        },
    )

    def aggregator(name, port, domain, credential):
        params = {
            'aggregation.enabled': True,
            'aggregation.peer_urls': [f'http://localhost:{PEER_PORT}'],
            'aggregation.peer_names': [PEER_NAME],
            'sse.max_clients': 6,
            'server.http_thread_pool_size': 16,
            'sse.keepalive_interval_sec': KEEPALIVE_SEC,
        }
        if credential:
            params['aggregation.peer_auth_header'] = credential
        return create_gateway_node(
            name=name,
            port=port,
            extra_params=params,
            extra_env={'ROS_DOMAIN_ID': str(domain)},
        )

    authed_aggregator = aggregator(
        'authed_aggregator_node', AUTHED_AGG_PORT, AUTHED_AGG_DOMAIN_ID, PEER_CREDENTIAL)
    # Identical but for the credential. Any difference between the two is the
    # credential and nothing else.
    bare_aggregator = aggregator(
        'bare_aggregator_node', BARE_AGG_PORT, BARE_AGG_DOMAIN_ID, None)

    return (
        LaunchDescription([
            peer_gateway,
            peer_fault_manager,
            TimerAction(period=1.0, actions=[authed_aggregator, bare_aggregator]),
            launch_testing.actions.ReadyToTest(),
        ]),
        {
            'gateway_node': authed_aggregator,
            'peer_gateway': peer_gateway,
            'bare_aggregator': bare_aggregator,
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
                continue
            key, _, value = line.partition(':')
            current[key.strip()] = value.strip()
    except Exception:  # noqa: BLE001 - closed socket on teardown
        pass


class RelayPeerCredentialTest(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls._wait_for_gateways()
        # Pinned to the peer's domain: that is where the fault_manager this
        # reports to lives, and the launch process does not reliably still
        # carry the domain it started on once an action with an ROS_DOMAIN_ID
        # of its own has run.
        previous_domain = os.environ.get('ROS_DOMAIN_ID')
        os.environ['ROS_DOMAIN_ID'] = str(PEER_DOMAIN_ID)
        try:
            rclpy.init()
            cls._reporter = Node('relay_credential_reporter')
        finally:
            if previous_domain is None:
                del os.environ['ROS_DOMAIN_ID']
            else:
                os.environ['ROS_DOMAIN_ID'] = previous_domain

        cls._report_client = cls._reporter.create_client(ReportFault, SERVICE_NAME)
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
    def _wait_for_gateways(cls):
        """Both aggregators answer their own /health, which needs no auth."""
        deadline = time.time() + TIMEOUT
        pending = [AUTHED_AGG_URL, BARE_AGG_URL]
        last = None
        while time.time() < deadline and pending:
            for url in list(pending):
                try:
                    if requests.get(f'{url}/health', timeout=10).status_code == 200:
                        pending.remove(url)
                except requests.RequestException as exc:
                    last = str(exc)
            if pending:
                time.sleep(0.5)
        if pending:
            raise AssertionError(f'gateways never came up: {pending}; last error {last}')

    @staticmethod
    def _peer_status(agg_url):
        """How this aggregator currently reports its peer."""
        body = requests.get(f'{agg_url}/health', timeout=10).json()
        peers = body.get('x-medkit', {}).get('peers') or body.get('peers') or []
        for peer in peers:
            if peer.get('name') == PEER_NAME:
                return peer.get('status')
        return None

    def _wait_for_peer_status(self, agg_url, wanted):
        deadline = time.time() + TIMEOUT
        last = None
        while time.time() < deadline:
            last = self._peer_status(agg_url)
            if last == wanted:
                return True
            time.sleep(0.5)
        return last

    def _report(self, fault_code):
        req = ReportFault.Request()
        req.fault_code = fault_code
        req.event_type = ReportFault.Request.EVENT_FAILED
        req.severity = Fault.SEVERITY_ERROR
        req.description = 'relay credential e2e'
        req.source_id = SOURCE_ID
        deadline = time.monotonic() + FAULT_TIMEOUT
        while time.monotonic() < deadline:
            future = self._report_client.call_async(req)
            attempt_deadline = min(time.monotonic() + 5.0, deadline)
            while not future.done() and time.monotonic() < attempt_deadline:
                rclpy.spin_once(self._reporter, timeout_sec=0.1)
            if future.done():
                self.assertTrue(future.result().accepted)
                return
            self._report_client.remove_pending_request(future)
        self.fail(f'report_fault never replied for {fault_code}')

    def _open_stream(self, url):
        response = requests.get(f'{url}/faults/stream', stream=True, timeout=(10, 120))
        self.assertEqual(response.status_code, 200, f'{url} did not open its stream')
        frames = []
        stop = threading.Event()
        pump = threading.Thread(target=_pump_stream, args=(response, frames, stop), daemon=True)
        pump.start()
        return response, frames, stop, pump

    @staticmethod
    def _close_stream(response, stop, pump):
        stop.set()
        response.close()
        pump.join(timeout=5)

    def test_c3_the_peer_is_online_only_where_the_credential_is_configured(self):
        """C3: the health check is where a missing credential first shows."""
        authed = self._wait_for_peer_status(AUTHED_AGG_URL, 'online')
        self.assertIs(
            authed, True,
            'the aggregator holding the credential never saw its peer come online; '
            f'last status {authed}')
        # And the control really is refused, rather than merely slower.
        self.assertNotEqual(
            self._peer_status(BARE_AGG_URL), 'online',
            'the aggregator WITHOUT a credential reported the authenticating peer as '
            'online, so the peer is not actually checking the header and every other '
            'assertion in this file is vacuous')

    def test_c1_c2_only_the_credentialed_aggregator_relays_the_peers_fault(self):
        """C1 and C2: asserted on ONE fault, so the two cannot disagree."""
        self.assertIs(
            self._wait_for_peer_status(AUTHED_AGG_URL, 'online'), True,
            'the credentialed aggregator never saw its peer come online')

        authed_res, authed_frames, authed_stop, authed_pump = self._open_stream(AUTHED_AGG_URL)
        bare_res, bare_frames, bare_stop, bare_pump = self._open_stream(BARE_AGG_URL)
        try:
            # Relays are opened on the first attached client, so give them a
            # moment to reach the peer before the fault exists.
            time.sleep(2.0 * get_time_scale())
            self._report(FAULT_CODE)

            deadline = time.time() + FAULT_TIMEOUT
            relayed = None
            while time.time() < deadline and relayed is None:
                for frame in list(authed_frames):
                    if FAULT_CODE in frame.get('data', ''):
                        relayed = frame
                        break
                time.sleep(0.2)

            self.assertIsNotNone(
                relayed,
                'the credentialed aggregator never relayed the fault; frames seen: '
                f'{authed_frames}')
            payload = json.loads(relayed['data'])
            self.assertEqual(
                payload.get('x-medkit', {}).get('peer'), PEER_NAME,
                'the relayed event did not name the peer it came from, so it cannot be '
                f'told from an event raised locally: {payload}')

            # The control. Checked AFTER the credentialed one has the event, so
            # this is not just a race that the bare aggregator has yet to lose.
            self.assertFalse(
                [f for f in bare_frames if FAULT_CODE in f.get('data', '')],
                'the aggregator with no credential relayed the fault anyway, so the '
                f'credential is not what admits it: {bare_frames}')
        finally:
            self._close_stream(authed_res, authed_stop, authed_pump)
            self._close_stream(bare_res, bare_stop, bare_pump)


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info, allowable_exit_codes=ALLOWED_EXIT_CODES)
