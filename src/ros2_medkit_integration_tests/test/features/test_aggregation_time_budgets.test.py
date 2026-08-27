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

"""End-to-end specification for an aggregating gateway's time budgets.

A peer forward carries the peer's own work: a synchronous service call runs
against THAT gateway's ``service_call_timeout_sec``, and a large
resource has to be serialised and transferred on top. One budget covering both
that and a metadata read gives the forward the budget of a listing, and the
answer a client gets is that the peer is unavailable - at the moment that peer
is serving the same request.

THE RULES

B1  An operation that takes longer than the metadata budget and less than the
    forward budget returns the peer's own result. Asserting a 200 alone would
    not show this: a fast operation returns 200 whatever the budgets are, so
    the call has to be held open past the metadata budget on purpose and the
    elapsed time asserted.
B2  A large resource arrives whole. Proven by comparing the array length
    against a read taken straight from the owning gateway, not by status. This
    rule is about completeness only: how long a given machine takes to
    serialise 180000 readings is a property of the machine, so requiring a
    timeout here would measure the host rather than the gateway.
B3  An operation against an aggregator whose forward budget is BELOW the work
    answers 504 not-responding. This is what makes B1 mean something: a test
    that passes under every budget is not measuring a budget. The two
    aggregators differ in exactly the key under test, and the peer is held for
    a controlled four seconds so the answer cannot depend on host speed.
B4  A timeout is not an unavailable peer. 504 ``not-responding`` and 502
    ``x-medkit-peer-unavailable`` are different answers to different events,
    and the timeout names the budget it exceeded so an operator can act on it.
"""

import os
import tempfile
import time
import unittest

from launch import LaunchDescription
from launch.actions import TimerAction
import launch_ros.actions
import launch_testing.actions
import requests
from ros2_medkit_test_utils.constants import (
    API_BASE_PATH,
    DISCOVERY_TIMEOUT,
    get_test_domain_id,
    get_test_port,
    get_time_scale,
)
from ros2_medkit_test_utils.launch_helpers import create_gateway_node

GENEROUS_PORT = get_test_port(0)
TIGHT_PORT = get_test_port(1)
PEER_PORT = get_test_port(2)

GENEROUS_URL = f'http://localhost:{GENEROUS_PORT}{API_BASE_PATH}'
TIGHT_URL = f'http://localhost:{TIGHT_PORT}{API_BASE_PATH}'
PEER_URL = f'http://localhost:{PEER_PORT}{API_BASE_PATH}'

GENEROUS_DOMAIN_ID = get_test_domain_id(0)
TIGHT_DOMAIN_ID = get_test_domain_id(1)
PEER_DOMAIN_ID = get_test_domain_id(2)

# Both aggregators read a peer's description with the same budget; they differ
# only in what a FORWARD gets. That is the key under test, and keeping every
# other parameter equal is what lets the two answers be attributed to it.
# NOT scaled. The metadata budget bounds discovery and fan-out over loopback,
# which a sanitizer barely touches, and B1 asserts the call outlived it - so
# scaling it to 6000 under a sanitizer would demand more than the four seconds
# of work the fixture holds, and the assertion would fail on a correct gateway.
METADATA_BUDGET_MS = 2000
# The generous budget covers the peer SERIALISING the scan, which is the work a
# sanitizer slows down most. Scaling only the client's patience leaves the
# gateway giving up first and answering 504 to a read that is merely slow.
GENEROUS_FORWARD_MS = int(20000 * get_time_scale())
# Below the four seconds of held work on purpose. Kept at or under the tight
# gateway's own metadata budget too, because a forward budget below the metadata
# budget is raised to it - scaling one without the other silently handed this
# aggregator a budget big enough to finish the work.
TIGHT_FORWARD_MS = 1000
TIGHT_METADATA_MS = 1000

# Longer than the metadata budget, shorter than the generous forward budget.
# The whole point of the fixture: under one budget for both, this call cannot
# succeed through an aggregator however healthy the peer is.
SLOW_SERVICE_DELAY_SEC = 4.0

# 360 / 0.01 = 36000 readings in each of two float arrays, about a megabyte of
# JSON. Large enough that a truncated or re-serialised payload shows up as a
# different reading count, which is what this measures. Deliberately not larger:
# at 180000 readings the serialisation alone outran a 60 s budget under ASan, so
# the test failed on instrumentation overhead rather than on gateway behaviour.
LIDAR_ANGULAR_RESOLUTION = 0.01
EXPECTED_LIDAR_READINGS = 36000

PEER_COMPONENT = 'remote-ecu'
SLOW_APP = 'remote_slow_calibration'
LIDAR_APP = 'remote_lidar'
PEER_NAMESPACE = '/chassis/sensors'

TIMEOUT = DISCOVERY_TIMEOUT * get_time_scale()

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
  - id: {SLOW_APP}
    name: "Slow Calibration Service"
    is_located_on: {PEER_COMPONENT}
    ros_binding:
      node_name: slow_calibration_service
      namespace: {PEER_NAMESPACE}
  - id: {LIDAR_APP}
    name: "Lidar Sensor"
    is_located_on: {PEER_COMPONENT}
    ros_binding:
      node_name: lidar_sensor
      namespace: {PEER_NAMESPACE}
"""


def _write_manifest(content):
    fd, path = tempfile.mkstemp(suffix='.yaml', prefix='test_agg_budgets_manifest_')
    with os.fdopen(fd, 'w') as handle:
        handle.write(content)
    return path


def generate_test_description():
    peer_manifest_path = _write_manifest(PEER_MANIFEST)
    peer_domain_env = {'ROS_DOMAIN_ID': str(PEER_DOMAIN_ID)}

    generous_gateway = create_gateway_node(
        name='generous_gateway_node',
        port=GENEROUS_PORT,
        extra_params={
            'aggregation.enabled': True,
            'aggregation.timeout_ms': METADATA_BUDGET_MS,
            'aggregation.forward_timeout_ms': GENEROUS_FORWARD_MS,
            'aggregation.peer_urls': [f'http://localhost:{PEER_PORT}'],
            'aggregation.peer_names': ['remote_gateway'],
        },
        extra_env={'ROS_DOMAIN_ID': str(GENEROUS_DOMAIN_ID)},
    )

    tight_gateway = create_gateway_node(
        name='tight_gateway_node',
        port=TIGHT_PORT,
        extra_params={
            'aggregation.enabled': True,
            'aggregation.timeout_ms': TIGHT_METADATA_MS,
            'aggregation.forward_timeout_ms': TIGHT_FORWARD_MS,
            'aggregation.peer_urls': [f'http://localhost:{PEER_PORT}'],
            'aggregation.peer_names': ['remote_gateway'],
        },
        extra_env={'ROS_DOMAIN_ID': str(TIGHT_DOMAIN_ID)},
    )

    peer_gateway = create_gateway_node(
        name='remote_gateway_node',
        port=PEER_PORT,
        extra_params={
            'discovery.mode': 'hybrid',
            'discovery.manifest_path': peer_manifest_path,
            'discovery.manifest_strict_validation': False,
        },
        extra_env=peer_domain_env,
    )

    slow_service = launch_ros.actions.Node(
        package='ros2_medkit_integration_tests',
        executable='demo_slow_calibration_service',
        name='slow_calibration_service',
        namespace=PEER_NAMESPACE,
        output='screen',
        additional_env=peer_domain_env,
    )
    lidar = launch_ros.actions.Node(
        package='ros2_medkit_integration_tests',
        executable='demo_lidar_sensor',
        name='lidar_sensor',
        namespace=PEER_NAMESPACE,
        output='screen',
        parameters=[{'angular_resolution': LIDAR_ANGULAR_RESOLUTION}],
        additional_env=peer_domain_env,
    )

    launch_description = LaunchDescription([
        generous_gateway,
        tight_gateway,
        peer_gateway,
        TimerAction(period=2.0, actions=[slow_service, lidar]),
        launch_testing.actions.ReadyToTest(),
    ])

    return (
        launch_description,
        {
            'gateway_node': generous_gateway,
            'tight_gateway': tight_gateway,
            'peer_gateway': peer_gateway,
        },
    )


class AggregationTimeBudgetsTest(unittest.TestCase):
    """Drives both aggregators; the peer is used to establish the truth."""

    @classmethod
    def setUpClass(cls):
        for url, label in ((PEER_URL, 'peer'), (GENEROUS_URL, 'generous'), (TIGHT_URL, 'tight')):
            cls._wait_for_apps(url, {SLOW_APP, LIDAR_APP}, label)
        # A manifest App exists before its node does, and an App with no live
        # binding has no operations at all - a 404 from an entity that is
        # merely early reads exactly like the routing failure under test.
        cls._wait_for_operation(PEER_URL, SLOW_APP, 'calibrate')

    @classmethod
    def _wait_for_apps(cls, base_url, expected, label):
        deadline = time.time() + TIMEOUT
        seen = set()
        while time.time() < deadline:
            try:
                response = requests.get(f'{base_url}/apps', timeout=10)
                if response.status_code == 200:
                    seen = {item['id'] for item in response.json().get('items', [])}
                    if expected.issubset(seen):
                        return
            except requests.RequestException:
                pass
            time.sleep(0.5)
        raise AssertionError(
            f'{label} gateway never listed {sorted(expected)}; saw {sorted(seen)}')

    @classmethod
    def _wait_for_operation(cls, base_url, app_id, operation_id):
        deadline = time.time() + TIMEOUT
        seen = []
        while time.time() < deadline:
            try:
                response = requests.get(f'{base_url}/apps/{app_id}/operations', timeout=10)
                if response.status_code == 200:
                    seen = [item['id'] for item in response.json().get('items', [])]
                    if operation_id in seen:
                        return
            except requests.RequestException:
                pass
            time.sleep(0.5)
        raise AssertionError(f'{app_id} never offered operation {operation_id}; saw {seen}')

    def _set_service_delay(self, seconds):
        response = requests.put(
            f'{PEER_URL}/apps/{SLOW_APP}/configurations/response_delay_sec',
            json={'data': seconds},
            timeout=15,
        )
        self.assertIn(
            response.status_code, (200, 204),
            f'could not set response_delay_sec on the peer: '
            f'{response.status_code} {response.text}')

    def test_b1_operation_past_the_metadata_budget_returns_the_peers_result(self):
        """B1: 4 s of work reaches the client through a 2 s metadata budget."""
        self._set_service_delay(SLOW_SERVICE_DELAY_SEC)
        self.addCleanup(self._set_service_delay, 0.0)

        started = time.monotonic()
        response = requests.post(
            f'{GENEROUS_URL}/apps/{SLOW_APP}/operations/calibrate/executions',
            json={},
            timeout=(GENEROUS_FORWARD_MS / 1000.0 + 10) * get_time_scale(),
        )
        elapsed = time.monotonic() - started

        self.assertEqual(
            response.status_code, 200,
            f'a peer-owned operation the peer itself serves in {SLOW_SERVICE_DELAY_SEC}s '
            f'failed through the aggregator after {elapsed:.3f}s: {response.text}')
        # Without this the test would pass against an instant service and prove
        # nothing about a budget.
        self.assertGreater(
            elapsed, METADATA_BUDGET_MS / 1000.0,
            f'the call returned in {elapsed:.3f}s, inside the metadata budget of '
            f'{METADATA_BUDGET_MS}ms - the fixture did not hold it open, so this '
            f'run cannot distinguish a correct forward budget from the old shared one')
        body = response.json()
        self.assertIn('parameters', body, f'peer result envelope missing: {body}')
        self.assertTrue(
            body['parameters'].get('success'),
            f'the peer reported failure through the aggregator: {body}')

    def test_b3_same_operation_times_out_under_a_tight_forward_budget(self):
        """B3+B4: the tight aggregator answers 504 not-responding, naming the budget."""
        self._set_service_delay(SLOW_SERVICE_DELAY_SEC)
        self.addCleanup(self._set_service_delay, 0.0)

        started = time.monotonic()
        response = requests.post(
            f'{TIGHT_URL}/apps/{SLOW_APP}/operations/calibrate/executions',
            json={},
            timeout=30 * get_time_scale(),
        )
        elapsed = time.monotonic() - started

        self.assertEqual(
            response.status_code, 504,
            f'an aggregator whose forward budget is {TIGHT_FORWARD_MS}ms answered '
            f'{response.status_code} to {SLOW_SERVICE_DELAY_SEC}s of work after '
            f'{elapsed:.3f}s: {response.text}')
        body = response.json()
        self.assertEqual(
            body.get('error_code'), 'not-responding',
            f'a timeout must be reported as a timeout, not as something else: {body}')
        # B4: the message has to name the budget, or an operator cannot act on it.
        self.assertIn(
            'forward_timeout_ms', body.get('message', ''),
            f'the timeout does not name the budget it exceeded: {body}')
        self.assertNotIn(
            'unavailable', body.get('message', '').lower(),
            f'the peer was answering; calling it unavailable points at the wrong box: {body}')

    def test_b2_large_resource_arrives_whole_through_the_aggregator(self):
        """B2: the payload #528 reports as timing out now arrives complete.

        This asserts completeness, not a timeout. Whether a given machine
        serialises 180000 readings faster or slower than any particular budget
        is a property of the machine, not of the gateway: an earlier version
        required the tight aggregator to answer 504 here and passed locally
        while failing on faster CI hardware, which measured the box rather than
        the code. The budget itself is proven by B3, where the peer is held for
        a controlled four seconds and the answer cannot depend on how fast the
        host is.
        """
        resource = self._find_scan_resource()

        direct = requests.get(f'{PEER_URL}/apps/{LIDAR_APP}/data/{resource}',
                              timeout=120 * get_time_scale())
        self.assertEqual(
            direct.status_code, 200,
            f'peer could not serve its own scan: {direct.text[:300]}')
        direct_ranges = self._ranges_of(direct.json())
        self.assertGreaterEqual(
            len(direct_ranges), EXPECTED_LIDAR_READINGS,
            f'the fixture produced {len(direct_ranges)} readings, not the '
            f'{EXPECTED_LIDAR_READINGS} the angular resolution asks for, so the payload '
            f'is not the size this test claims to be about')

        through = requests.get(
            f'{GENEROUS_URL}/apps/{LIDAR_APP}/data/{resource}',
            timeout=(GENEROUS_FORWARD_MS / 1000.0 + 20) * get_time_scale(),
        )
        self.assertEqual(
            through.status_code, 200,
            f'a large peer-owned resource failed through the aggregator: '
            f'{through.status_code} {through.text[:400]}')
        self.assertEqual(
            len(self._ranges_of(through.json())), len(direct_ranges),
            'the aggregator returned a different number of readings than the gateway that '
            'owns the topic, so the payload did not arrive whole')

    def _find_scan_resource(self):
        response = requests.get(f'{PEER_URL}/apps/{LIDAR_APP}/data', timeout=15)
        self.assertEqual(response.status_code, 200, response.text)
        ids = [item['id'] for item in response.json().get('items', [])]
        for candidate in ids:
            if 'scan' in candidate:
                return candidate
        raise AssertionError(f'{LIDAR_APP} offers no scan resource; saw {ids}')

    def _ranges_of(self, payload):
        """Pull the LaserScan ranges array out of a data read, whatever wraps it."""
        stack = [payload]
        while stack:
            node = stack.pop()
            if isinstance(node, dict):
                if isinstance(node.get('ranges'), list):
                    return node['ranges']
                stack.extend(node.values())
        raise AssertionError(f'no ranges array in the response: {str(payload)[:300]}')


@launch_testing.post_shutdown_test()
class TestProcessOutput(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info, allowable_exit_codes=[0, -2, -15])
