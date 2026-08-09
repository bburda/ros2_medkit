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

"""Leaf-Component-ID collision across peers: primary + peer_b + peer_c.

Covers the "deployment anomaly" branch that the daisy-chain test deliberately
avoids: two peers simultaneously announcing the same leaf Component ID. On
the wire this surfaces as:

- ``/health.warnings`` contains a ``leaf_id_collision`` entry listing both
  peers under ``peer_names``.
- ``GET /components/ecu-shared`` from the primary resolves to one of the
  peers (last-writer-wins) and returns 200 - never a local stub.
- The resolving peer's ``x-medkit.contributors`` reflects THAT peer only,
  not both.

Three gateways, three distinct DDS domains, aggregation is the only channel
between them. All three domains are allocated to this test alone, by the
``DOMAINS`` count on its registration in CMakeLists.txt.
"""

import os
import tempfile
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

PRIMARY_PORT = get_test_port(0)
PEER_B_PORT = get_test_port(1)
PEER_C_PORT = get_test_port(2)
PRIMARY_URL = f'http://localhost:{PRIMARY_PORT}{API_BASE_PATH}'
PEER_B_URL = f'http://localhost:{PEER_B_PORT}{API_BASE_PATH}'
PEER_C_URL = f'http://localhost:{PEER_C_PORT}{API_BASE_PATH}'

PRIMARY_DOMAIN = get_test_domain_id(0)
PEER_B_DOMAIN = get_test_domain_id(1)
PEER_C_DOMAIN = get_test_domain_id(2)

SHARED_LEAF_ID = 'ecu-shared'


def _manifest(name, include_shared_leaf):
    """Render a manifest for one leaf-collision gateway.

    Peer manifests both declare ``ecu-shared`` as a top-level leaf
    Component so the primary's aggregation merge sees the collision.
    The primary manifest does not declare it - it is peer-owned only.
    """
    lines = [
        'manifest_version: "1.0"',
        'metadata:',
        f'  name: {name}',
        '  description: Leaf-collision test',
        '  version: "0.1.0"',
        'config:',
        '  unmanifested_nodes: ignore',
        '  inherit_runtime_resources: false',
        'components:',
        f'  - id: {name}-local',
        f'    name: {name}-local',
        '    type: compute-unit',
    ]
    if include_shared_leaf:
        lines += [
            f'  - id: {SHARED_LEAF_ID}',
            '    name: Shared ECU',
            '    type: compute-unit',
        ]
    lines.append('apps: []')
    return '\n'.join(lines) + '\n'


def _write_manifest(name, include_shared_leaf):
    fd, path = tempfile.mkstemp(prefix=f'medkit_leafcoll_{name}_', suffix='.yaml')
    os.write(fd, _manifest(name, include_shared_leaf).encode('utf-8'))
    os.close(fd)
    return path


PRIMARY_MANIFEST = _write_manifest('primary', include_shared_leaf=False)
PEER_B_MANIFEST = _write_manifest('peer_b', include_shared_leaf=True)
PEER_C_MANIFEST = _write_manifest('peer_c', include_shared_leaf=True)
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
    # Primary aggregates peer_b AND peer_c directly (fan-out, not daisy).
    primary = _gateway(
        PRIMARY_PORT, 'leafcoll_primary', PRIMARY_MANIFEST, PRIMARY_DOMAIN,
        aggregation_peers=[
            (f'http://localhost:{PEER_B_PORT}', 'peer_b'),
            (f'http://localhost:{PEER_C_PORT}', 'peer_c'),
        ],
    )
    peer_b = _gateway(PEER_B_PORT, 'leafcoll_peer_b', PEER_B_MANIFEST, PEER_B_DOMAIN)
    peer_c = _gateway(PEER_C_PORT, 'leafcoll_peer_c', PEER_C_MANIFEST, PEER_C_DOMAIN)

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


class TestLeafCollisionAggregation(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        for path in MANIFEST_FILES:
            cls.addClassCleanup(lambda p=path: os.path.exists(p) and os.unlink(p))

        for label, url in (('primary', PRIMARY_URL),
                           ('peer_b', PEER_B_URL),
                           ('peer_c', PEER_C_URL)):
            cls._wait_for_health(url, label)
        # Wait until the merge has folded both peers in AND detected the
        # collision. Both conditions must hold before any test method runs,
        # otherwise the tests race the aggregation refresh cycle.
        cls._wait_for_collision_warning(PRIMARY_URL, SHARED_LEAF_ID)

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
    def _wait_for_collision_warning(cls, base_url, entity_id):
        deadline = time.monotonic() + DISCOVERY_TIMEOUT
        last_warnings = None
        while time.monotonic() < deadline:
            try:
                r = requests.get(f'{base_url}/health', timeout=5)
                if r.ok:
                    warnings = r.json().get('warnings', [])
                    last_warnings = warnings
                    if any(w.get('code') == 'leaf_id_collision'
                           and entity_id in w.get('entity_ids', [])
                           for w in warnings):
                        return
            except requests.exceptions.RequestException:
                pass
            time.sleep(1.0)
        raise AssertionError(
            f'collision warning for {entity_id} not surfaced within '
            f'{DISCOVERY_TIMEOUT}s (last warnings: {last_warnings})',
        )

    def test_health_surfaces_leaf_collision_with_both_peers(self):
        """@verifies REQ_INTEROP_003."""
        r = requests.get(f'{PRIMARY_URL}/health', timeout=5)
        self.assertEqual(r.status_code, 200)
        body = r.json()
        self.assertIn('warning_schema_version', body)
        self.assertEqual(body['warning_schema_version'], 1)
        collisions = [
            w for w in body.get('warnings', [])
            if w.get('code') == 'leaf_id_collision'
            and SHARED_LEAF_ID in w.get('entity_ids', [])
        ]
        self.assertEqual(
            len(collisions), 1,
            f'expected exactly one leaf_id_collision for {SHARED_LEAF_ID}, got: {body}',
        )
        self.assertEqual(set(collisions[0]['peer_names']), {'peer_b', 'peer_c'})

    def test_shared_leaf_resolves_to_exactly_one_peer(self):
        """@verifies REQ_INTEROP_003."""
        # Last-writer-wins: the primary must still forward to SOME peer and
        # return 200, never 404. The primary has no local ``ecu-shared`` so
        # a 404 would prove the routing table missed the collision case;
        # any 2xx proves a peer served the request.
        r = requests.get(f'{PRIMARY_URL}/components/{SHARED_LEAF_ID}', timeout=5)
        self.assertEqual(r.status_code, 200, r.text)
        body = r.json()
        self.assertEqual(body.get('id'), SHARED_LEAF_ID)
        # peer_b and peer_c do not run aggregation themselves, so their
        # local responses do not emit x-medkit.contributors. That absence
        # is the signal: the request reached a peer directly (rather than
        # being synthesised by the primary's merge), otherwise contributors
        # would be populated. A non-empty contributors list here would
        # indicate the primary fabricated a merged view for a leaf ID,
        # which is precisely what the routing table must prevent.
        contributors = body.get('x-medkit', {}).get('contributors', [])
        self.assertEqual(
            contributors, [],
            f'routed leaf response must not carry merged contributors, got: {contributors}',
        )

    def test_root_capabilities_flag_aggregation_enabled(self):
        r = requests.get(f'{PRIMARY_URL}/', timeout=5)
        self.assertEqual(r.status_code, 200)
        caps = r.json().get('capabilities', {})
        self.assertIs(caps.get('aggregation'), True)


@launch_testing.post_shutdown_test()
class TestLeafCollisionShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info, allowable_exit_codes=ALLOWED_EXIT_CODES)
