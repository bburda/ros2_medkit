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

"""Daisy-chain aggregation: primary -> peer_B -> peer_C.

Validates the extended aggregation model end-to-end:

- Hierarchical parent Component (``robot-x``) is served locally by every
  gateway in the chain; it is NEVER forwarded because the routing table
  excludes hierarchical parents.
- Leaf ECU Components (``ecu-a``, ``ecu-b``, ``ecu-c``) route to their
  owning peer. ``ecu-c`` requires a 2-hop forward from the primary, via
  ``peer_B``, to reach ``peer_C``.
- ``x-medkit.contributors`` on the primary's ``robot-x`` lists "local"
  plus "peer:peer_b"; each hop surfaces only its direct upstream.
- ``/health.warnings`` is an empty array when there are no deployment
  anomalies.

DDS isolation uses three distinct domain IDs so that the gateways cannot
discover each other's nodes via ROS 2 graph introspection - the only
channel is HTTP aggregation.
"""

import os
import tempfile
import textwrap
import time
import unittest

from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
import launch_testing
import launch_testing.actions
import requests

from ros2_medkit_test_utils.constants import (
    ALLOWED_EXIT_CODES,
    API_BASE_PATH,
    DISCOVERY_TIMEOUT,
    GATEWAY_STARTUP_INTERVAL,
    GATEWAY_STARTUP_TIMEOUT,
    get_test_domain_id,
    get_test_port,
)
from ros2_medkit_test_utils.launch_helpers import create_gateway_node

# Port layout: primary at base+0, peer_B at base+1, peer_C at base+2.
PRIMARY_PORT = get_test_port(0)
PEER_B_PORT = get_test_port(1)
PEER_C_PORT = get_test_port(2)
PRIMARY_URL = f'http://localhost:{PRIMARY_PORT}{API_BASE_PATH}'
PEER_B_URL = f'http://localhost:{PEER_B_PORT}{API_BASE_PATH}'
PEER_C_URL = f'http://localhost:{PEER_C_PORT}{API_BASE_PATH}'

# Secondary DDS domains: avoid collision with test_peer_aggregation (offset=1
# -> domain 230). Daisy chain uses offsets 2 and 3 -> domains 231 and 232.
PRIMARY_DOMAIN = get_test_domain_id(0)
PEER_B_DOMAIN = get_test_domain_id(2)
PEER_C_DOMAIN = get_test_domain_id(3)


def _manifest(name, ecu_id):
    """Return a minimal manifest for a daisy-chain gateway.

    Every manifest shares ``robot-x`` as the hierarchical parent and
    declares a single leaf ECU sub-component with one stub App. The
    App's ros_binding does not need a running node because the tests
    only exercise discovery endpoints and request forwarding - not
    live runtime sampling.
    """
    return textwrap.dedent(f"""
        manifest_version: "1.0"
        metadata:
          name: {name}
          description: Daisy-chain test
          version: "0.1.0"
        config:
          unmanifested_nodes: ignore
          inherit_runtime_resources: false
        components:
          - id: robot-x
            name: Robot X
            type: mobile-robot
            description: Shared hierarchical parent across all ECUs
          - id: {ecu_id}
            name: {ecu_id}
            type: compute-unit
            parent_component_id: robot-x
        apps:
          - id: {ecu_id}-app
            name: {ecu_id}-app
            category: infrastructure
            is_located_on: {ecu_id}
            ros_binding:
              node_name: {ecu_id}_app
              namespace: /{ecu_id}
    """).strip()


def _write_manifest(name, ecu_id):
    fd, path = tempfile.mkstemp(prefix=f'medkit_daisy_{name}_', suffix='.yaml')
    os.write(fd, _manifest(name, ecu_id).encode('utf-8'))
    os.close(fd)
    return path


PRIMARY_MANIFEST = _write_manifest('primary', 'ecu-a')
PEER_B_MANIFEST = _write_manifest('peer_b', 'ecu-b')
PEER_C_MANIFEST = _write_manifest('peer_c', 'ecu-c')
MANIFEST_FILES = [PRIMARY_MANIFEST, PEER_B_MANIFEST, PEER_C_MANIFEST]


def _gateway(port, name, manifest_path, domain, aggregation_peers=None):
    params = {
        'discovery.mode': 'manifest_only',
        'discovery.manifest_path': manifest_path,
        'discovery.manifest_strict_validation': False,
    }
    if aggregation_peers:
        urls = [url for url, _ in aggregation_peers]
        names = [nm for _, nm in aggregation_peers]
        params.update({
            'aggregation.enabled': True,
            'aggregation.timeout_ms': 5000,
            'aggregation.announce': False,
            'aggregation.discover': False,
            'aggregation.peer_urls': urls,
            'aggregation.peer_names': names,
        })
    return create_gateway_node(
        name=name,
        port=port,
        extra_params=params,
        extra_env={'ROS_DOMAIN_ID': str(domain)},
    )


def generate_test_description():
    primary = _gateway(
        PRIMARY_PORT, 'daisy_primary', PRIMARY_MANIFEST, PRIMARY_DOMAIN,
        aggregation_peers=[(f'http://localhost:{PEER_B_PORT}', 'peer_b')],
    )
    peer_b = _gateway(
        PEER_B_PORT, 'daisy_peer_b', PEER_B_MANIFEST, PEER_B_DOMAIN,
        aggregation_peers=[(f'http://localhost:{PEER_C_PORT}', 'peer_c')],
    )
    peer_c = _gateway(
        PEER_C_PORT, 'daisy_peer_c', PEER_C_MANIFEST, PEER_C_DOMAIN,
    )

    ld = LaunchDescription([
        SetEnvironmentVariable('ROS_DOMAIN_ID', str(PRIMARY_DOMAIN)),
        primary,
        peer_b,
        peer_c,
        launch_testing.actions.ReadyToTest(),
    ])
    return ld, {
        'primary': primary,
        'peer_b': peer_b,
        'peer_c': peer_c,
    }


class TestDaisyChainAggregation(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        # Ensure the tmp manifest files written at import time do not leak
        # past the test process, even if setUpClass fails later.
        for path in MANIFEST_FILES:
            cls.addClassCleanup(lambda p=path: os.path.exists(p) and os.unlink(p))

        for label, url in (('primary', PRIMARY_URL),
                           ('peer_b', PEER_B_URL),
                           ('peer_c', PEER_C_URL)):
            cls._wait_for_health(url, label)
        # Primary must see ecu-b (1 hop) AND ecu-c (2 hops) via aggregation.
        # Sub-components (with parent_component_id) appear under the parent's
        # /subcomponents endpoint, not the top-level /components list.
        cls._wait_for_subcomponents(
            PRIMARY_URL, 'robot-x', {'ecu-a', 'ecu-b', 'ecu-c'}, 'primary',
        )

    @classmethod
    def _wait_for_health(cls, base_url, label):
        deadline = time.monotonic() + GATEWAY_STARTUP_TIMEOUT
        while time.monotonic() < deadline:
            try:
                if requests.get(f'{base_url}/health', timeout=2).ok:
                    return
            except requests.exceptions.RequestException:
                pass
            time.sleep(GATEWAY_STARTUP_INTERVAL)
        raise AssertionError(f'{label} not healthy after {GATEWAY_STARTUP_TIMEOUT}s')

    @classmethod
    def _wait_for_subcomponents(cls, base_url, parent_id, required_ids, label):
        deadline = time.monotonic() + DISCOVERY_TIMEOUT
        last_ids = set()
        while time.monotonic() < deadline:
            try:
                r = requests.get(
                    f'{base_url}/components/{parent_id}/subcomponents',
                    timeout=5,
                )
                if r.ok:
                    last_ids = {c.get('id', '') for c in r.json().get('items', [])}
                    if required_ids.issubset(last_ids):
                        return
            except requests.exceptions.RequestException:
                pass
            time.sleep(1.0)
        raise AssertionError(
            f'{label}: subcomponents of {parent_id} expected {required_ids}, '
            f'last seen {last_ids}',
        )

    # --- Discovery across the chain --------------------------------------

    def test_primary_sees_all_ecus_as_subcomponents_of_robot_x(self):
        """@verifies REQ_INTEROP_003."""
        r = requests.get(f'{PRIMARY_URL}/components/robot-x/subcomponents', timeout=5)
        self.assertEqual(r.status_code, 200)
        sub_ids = {item['id'] for item in r.json().get('items', [])}
        self.assertEqual(sub_ids, {'ecu-a', 'ecu-b', 'ecu-c'})

    # --- Hierarchical parent served locally ------------------------------

    def test_primary_serves_robot_x_detail_locally_with_contributors(self):
        """@verifies REQ_INTEROP_003."""
        r = requests.get(f'{PRIMARY_URL}/components/robot-x', timeout=5)
        self.assertEqual(r.status_code, 200)
        body = r.json()
        self.assertEqual(body['id'], 'robot-x')
        contributors = body.get('x-medkit', {}).get('contributors', [])
        # Primary contributed locally; peer_b contributed via aggregation.
        # peer_c's contribution is hidden behind peer_b (daisy chain policy).
        self.assertIn('local', contributors)
        self.assertIn('peer:peer_b', contributors)

    # --- 1-hop and 2-hop leaf forwarding ---------------------------------

    def test_primary_forwards_ecu_b_detail_one_hop(self):
        """@verifies REQ_INTEROP_003."""
        r = requests.get(f'{PRIMARY_URL}/components/ecu-b', timeout=5)
        self.assertEqual(r.status_code, 200, r.text)
        self.assertEqual(r.json().get('id'), 'ecu-b')

    def test_primary_forwards_ecu_c_detail_two_hops(self):
        """@verifies REQ_INTEROP_003 (2-hop forward through daisy chain)."""
        # ecu-c lives on peer_c; primary reaches it via peer_b -> peer_c.
        r = requests.get(f'{PRIMARY_URL}/components/ecu-c', timeout=10)
        self.assertEqual(r.status_code, 200, r.text)
        self.assertEqual(r.json().get('id'), 'ecu-c')

    # --- /health ---------------------------------------------------------

    def test_primary_health_reports_peer_and_empty_warnings(self):
        r = requests.get(f'{PRIMARY_URL}/health', timeout=5)
        self.assertEqual(r.status_code, 200)
        body = r.json()
        self.assertIn('peers', body)
        self.assertIn('warnings', body)
        self.assertEqual(body['warnings'], [])
        # Schema version must be present whenever aggregation is active so
        # typed clients can feature-detect codes without string-matching.
        self.assertEqual(body.get('warning_schema_version'), 1)

    # --- Root capability flag --------------------------------------------

    def test_root_capabilities_flag_aggregation_enabled(self):
        # Clients should be able to feature-detect that /health.warnings
        # and x-medkit.contributors may be present via the root capabilities
        # object.
        r = requests.get(f'{PRIMARY_URL}/', timeout=5)
        self.assertEqual(r.status_code, 200)
        caps = r.json().get('capabilities', {})
        self.assertIs(caps.get('aggregation'), True)

    def test_peer_c_root_capabilities_reports_aggregation_disabled(self):
        # peer_c has no aggregation configured; capability must reflect that.
        r = requests.get(f'{PEER_C_URL}/', timeout=5)
        self.assertEqual(r.status_code, 200)
        caps = r.json().get('capabilities', {})
        self.assertIs(caps.get('aggregation'), False)

    # --- Contributors on non-Component entity types ----------------------

    def test_leaf_component_detail_contributors_single_peer(self):
        """@verifies REQ_INTEROP_003.

        GET /components/ecu-b through the primary is transparently forwarded
        to peer_b (1-hop). The response body is peer_b's own serialised view
        of ecu-b, where peer_b is the local gateway, so contributors starts
        with "local" under the stable "local first, then peer:* sorted"
        ordering. The assertion therefore validates that peer_b's
        x-medkit.contributors is present and proxied intact - a regression
        dropping the field on Area/App/Function detail handlers would flip
        this test. That the request is forwarded (not served with a merged
        primary view) is verified separately by
        test_primary_forwards_ecu_b_detail_one_hop.
        """
        r = requests.get(f'{PRIMARY_URL}/components/ecu-b', timeout=5)
        self.assertEqual(r.status_code, 200)
        contributors = r.json().get('x-medkit', {}).get('contributors', [])
        self.assertEqual(contributors[:1], ['local'])

    def test_app_detail_contributors_present(self):
        # Apps flow through x-medkit.contributors the same way Components do
        # after the detail-handler fix. ecu-a-app is the primary's local App.
        r = requests.get(f'{PRIMARY_URL}/apps/ecu-a-app', timeout=5)
        self.assertEqual(r.status_code, 200)
        contributors = r.json().get('x-medkit', {}).get('contributors', [])
        self.assertIn('local', contributors)


@launch_testing.post_shutdown_test()
class TestDaisyShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info, allowable_exit_codes=ALLOWED_EXIT_CODES)
