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

"""End-to-end specification for why a peer dropped out of a fanned-out answer.

A fanned-out collection whose peer failed answers 200 with ``x-medkit.partial``
and ``failed_peers``. Those name WHICH peer contributed nothing and nothing
more, so a peer that ran out of time, one that answered an error status and one
that is not there any more are the same entry - and a client cannot tell a busy
subsystem from a dead one, which is the only question a partial answer raises.

THE RULES

R1  A peer slower than the aggregator's metadata budget is reported as
    ``timeout``.
R2  The SAME request, against the SAME peer, from an aggregator whose budget
    outlasts the peer's own answer, is reported as ``error-status``. This is
    the pair that matters: two aggregators, one peer, one request, two reasons.
    A classifier that returned one constant would pass R1 alone.
R3  A peer that is gone is reported as ``unreachable``, never as ``timeout``.
    The two point an operator at different boxes.
R4  ``failed_peers`` keeps its shape. Every deployed client reads a list of
    names there, and the reasons arrive beside it rather than in place of it.

NOT COVERED HERE, and why. ``too-large``, ``invalid-response`` and ``canceled``
are classified in the same table but are not reachable through a healthy
ros2_medkit peer over the contract: the first needs a peer serving a body past
the 10 MB cap on a LIST route, which no list produces; the second needs a peer
answering non-JSON on a JSON route; the third is this gateway's own shutdown.
They are stated as unreachable rather than left looking untested.
"""

import os
import signal
import tempfile
import time
import unittest

from launch import LaunchDescription
from launch.actions import TimerAction
import launch_ros.actions
import launch_testing.actions
import requests
from ros2_medkit_test_utils.constants import (
    ALLOWED_EXIT_CODES,
    API_BASE_PATH,
    DISCOVERY_TIMEOUT,
    get_test_domain_id,
    get_test_port,
    get_time_scale,
)
from ros2_medkit_test_utils.launch_helpers import create_gateway_node

TIGHT_PORT = get_test_port(0)
PATIENT_PORT = get_test_port(1)
PEER_PORT = get_test_port(2)

TIGHT_URL = f'http://localhost:{TIGHT_PORT}{API_BASE_PATH}'
PATIENT_URL = f'http://localhost:{PATIENT_PORT}{API_BASE_PATH}'
PEER_URL = f'http://localhost:{PEER_PORT}{API_BASE_PATH}'

TIGHT_DOMAIN_ID = get_test_domain_id(0)
PATIENT_DOMAIN_ID = get_test_domain_id(1)
PEER_DOMAIN_ID = get_test_domain_id(2)

# The peer's parameter round trip is held at 6 s, so the two aggregators sit on
# either side of it by construction rather than by luck.
PEER_PARAM_TIMEOUT_SEC = 6.0
TIGHT_METADATA_MS = 800
PATIENT_METADATA_MS = 20000

TIMEOUT = DISCOVERY_TIMEOUT * get_time_scale()

PEER_COMPONENT = 'remote-ecu'
UNRESPONSIVE_APP = 'remote_unresponsive_param'
PEER_NAMESPACE = '/chassis/brakes'
# Declared on all three gateways so it MERGES rather than being routed whole to
# the peer. A routed request is a forward, which is a different path; only a
# merged entity fans out, and fan-out is what carries failed_peers.
MERGED_FUNCTION = 'remote_health'

_KILLED_PIDS = set()

AGGREGATOR_MANIFEST = f"""\
manifest_version: "1.0"
metadata:
  name: "Aggregating ECU"
  version: "1.0.0"
config:
  unmanifested_nodes: ignore
functions:
  - id: {MERGED_FUNCTION}
    name: "Remote Health Monitoring"
    category: monitoring
"""

PEER_MANIFEST = f"""\
manifest_version: "1.0"
metadata:
  name: "Remote ECU"
  version: "1.0.0"
config:
  unmanifested_nodes: ignore
components:
  - id: {PEER_COMPONENT}
    name: "Remote ECU"
apps:
  - id: {UNRESPONSIVE_APP}
    name: "Unresponsive Parameter Node"
    is_located_on: {PEER_COMPONENT}
    ros_binding:
      node_name: unresponsive_param_node
      namespace: {PEER_NAMESPACE}
functions:
  - id: {MERGED_FUNCTION}
    name: "Remote Health Monitoring"
    category: monitoring
    hosted_by:
      - {UNRESPONSIVE_APP}
"""


def _write_manifest(content, prefix):
    fd, path = tempfile.mkstemp(suffix='.yaml', prefix=prefix)
    with os.fdopen(fd, 'w') as handle:
        handle.write(content)
    return path


def generate_test_description():
    aggregator_manifest = _write_manifest(AGGREGATOR_MANIFEST, 'test_peer_reasons_agg_')
    peer_manifest = _write_manifest(PEER_MANIFEST, 'test_peer_reasons_peer_')
    peer_domain_env = {'ROS_DOMAIN_ID': str(PEER_DOMAIN_ID)}

    def aggregator(name, port, domain, metadata_ms):
        return create_gateway_node(
            name=name,
            port=port,
            extra_params={
                'discovery.mode': 'hybrid',
                'discovery.manifest_path': aggregator_manifest,
                'discovery.manifest_strict_validation': False,
                'aggregation.enabled': True,
                'aggregation.timeout_ms': metadata_ms,
                # Held above the metadata budget on both so the forward budget
                # cannot be what differs; the fan-out path reads the metadata
                # budget and that is the key under test.
                'aggregation.forward_timeout_ms': max(metadata_ms, 20000),
                'aggregation.peer_urls': [f'http://localhost:{PEER_PORT}'],
                'aggregation.peer_names': ['remote_gateway'],
            },
            extra_env={'ROS_DOMAIN_ID': str(domain)},
        )

    tight_gateway = aggregator(
        'tight_gateway_node', TIGHT_PORT, TIGHT_DOMAIN_ID, TIGHT_METADATA_MS)
    patient_gateway = aggregator(
        'patient_gateway_node', PATIENT_PORT, PATIENT_DOMAIN_ID, PATIENT_METADATA_MS)

    peer_gateway = create_gateway_node(
        name='remote_gateway_node',
        port=PEER_PORT,
        extra_params={
            'discovery.mode': 'hybrid',
            'discovery.manifest_path': peer_manifest,
            'discovery.manifest_strict_validation': False,
            'operations.parameter_service_timeout_sec': PEER_PARAM_TIMEOUT_SEC,
        },
        extra_env=peer_domain_env,
    )

    unresponsive = launch_ros.actions.Node(
        package='ros2_medkit_integration_tests',
        executable='demo_unresponsive_param_node',
        name='unresponsive_param_node',
        namespace=PEER_NAMESPACE,
        output='screen',
        additional_env=peer_domain_env,
    )

    launch_description = LaunchDescription([
        tight_gateway,
        patient_gateway,
        peer_gateway,
        TimerAction(period=2.0, actions=[unresponsive]),
        launch_testing.actions.ReadyToTest(),
    ])

    return (
        launch_description,
        {
            'gateway_node': tight_gateway,
            'patient_gateway': patient_gateway,
            'peer_gateway': peer_gateway,
        },
    )


def _config_url(base_url):
    return f'{base_url}/functions/{MERGED_FUNCTION}/configurations'


class PeerFailureReasonsTest(unittest.TestCase):
    """Runs in name order: the kill in test_3 must come after the two reads."""

    @classmethod
    def setUpClass(cls):
        for url, label in ((PEER_URL, 'peer'), (TIGHT_URL, 'tight'), (PATIENT_URL, 'patient')):
            cls._wait_for_healthy_peer_or_self(url, label)

    @classmethod
    def _wait_for_healthy_peer_or_self(cls, base_url, label):
        deadline = time.time() + TIMEOUT
        last = None
        while time.time() < deadline:
            try:
                response = requests.get(f'{base_url}/health', timeout=10)
                if response.status_code == 200:
                    body = response.json()
                    if label == 'peer':
                        return
                    peers = body.get('x-medkit', {}).get('peers') or body.get('peers') or []
                    if any(p.get('status') == 'online' for p in peers):
                        return
                    last = peers
            except requests.RequestException as exc:
                last = str(exc)
            time.sleep(0.5)
        raise AssertionError(f'{label} gateway never saw a healthy peer; last saw {last}')

    def _fan_out_failure(self, base_url, timeout):
        response = requests.get(_config_url(base_url), timeout=timeout)
        self.assertEqual(
            response.status_code, 200,
            f'a fanned-out listing whose peer failed must still answer 200: '
            f'{response.status_code} {response.text[:400]}')
        ext = response.json().get('x-medkit', {})
        self.assertTrue(ext.get('partial'), f'the answer does not admit it is partial: {ext}')
        # R4: the existing key keeps its existing shape.
        self.assertEqual(
            ext.get('failed_peers'), ['remote_gateway'],
            f'failed_peers must stay a list of names: {ext}')
        failures = ext.get('peer_failures')
        self.assertIsInstance(failures, list, f'peer_failures missing from {ext}')
        self.assertEqual(len(failures), 1, f'one failed peer, one reason: {failures}')
        self.assertEqual(failures[0].get('peer'), 'remote_gateway', failures)
        return failures[0].get('reason')

    def test_1_a_peer_slower_than_the_budget_reads_as_timeout(self):
        """R1."""
        reason = self._fan_out_failure(TIGHT_URL, timeout=30)
        self.assertEqual(
            reason, 'timeout',
            f'the peer was working on the request when the {TIGHT_METADATA_MS}ms budget ran '
            f'out; that is a timeout, not {reason!r}')

    def test_2_the_same_peer_read_patiently_is_an_error_status(self):
        """R2: the discriminator. Same peer, same request, a different reason."""
        reason = self._fan_out_failure(PATIENT_URL, timeout=60)
        self.assertEqual(
            reason, 'error-status',
            f'an aggregator that waited for the peer saw the status the peer actually '
            f'sent; that is not {reason!r}')

    def test_3_a_peer_that_is_gone_reads_as_unreachable(self, peer_gateway):
        """R3: killed, not slow. The health check has not noticed yet."""
        pid = peer_gateway.process_details['pid']
        _KILLED_PIDS.add(pid)
        os.kill(pid, signal.SIGKILL)

        # The peer stays marked healthy until the next health check, which is
        # the window this reads in. Once it is marked unhealthy it drops out of
        # the fan-out entirely and reports nothing at all - so a run that
        # cannot observe a reason before then has proven nothing either way and
        # says so, rather than passing on an empty result.
        deadline = time.time() + 30
        reasons = []
        while time.time() < deadline:
            response = requests.get(_config_url(PATIENT_URL), timeout=30)
            if response.status_code != 200:
                time.sleep(0.2)
                continue
            ext = response.json().get('x-medkit', {})
            failures = ext.get('peer_failures') or []
            if failures:
                reasons.append(failures[0].get('reason'))
                break
            if not ext.get('partial'):
                # The peer has been dropped from the healthy set; nothing more
                # will be reported about it.
                break
            time.sleep(0.2)

        self.assertTrue(
            reasons,
            'the killed peer produced no peer_failures entry before it was dropped from the '
            'healthy set, so this run could not observe the reason at all')
        self.assertEqual(
            reasons[0], 'unreachable',
            f'a peer whose process is gone is unreachable, not {reasons[0]!r} - the two send '
            f'an operator to different boxes')


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        for info in proc_info:
            allowed = set(ALLOWED_EXIT_CODES)
            if info.pid in _KILLED_PIDS:
                allowed.add(-9)
            self.assertIn(
                info.returncode, allowed,
                f'{info.process_name} exited with code {info.returncode}')
