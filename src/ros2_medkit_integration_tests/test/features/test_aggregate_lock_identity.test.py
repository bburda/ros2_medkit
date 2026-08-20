#!/usr/bin/env python3
#
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

"""Whether a lock survives the hop to the gateway that owns the entity.

A lock names the client holding it, and every later request is judged against
that name: the owner passes, anyone else is refused. The name travels in
`X-Client-Id`. When the entity lives on a peer, the request crosses a gateway
boundary before it is judged, so the question here is whether the holder is
still recognisable on the other side.

Both failure directions are wrong and only one of them is loud. If the name is
lost, every request arrives anonymous - including the holder's - and anonymous
matches anonymous, so the lock stops refusing anybody while continuing to look
like it exists. That is why L2 asserts a refusal rather than an acceptance: a
lock that grants everything passes any test written the other way round.

L1  A lock taken on a peer-owned entity through the aggregating gateway is
    recorded on the peer as belonging to the client who took it. Read back
    from the peer directly - the aggregating gateway is what is under test, so
    its own view cannot be the witness.
L2  While that lock is held, a different client's write to the same entity is
    refused.
"""

import tempfile
import time
import unittest

from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, TimerAction
import launch_testing.actions
import requests

from ros2_medkit_test_utils.constants import (
    ALLOWED_EXIT_CODES,
    API_BASE_PATH,
    get_test_domain_id,
    get_test_port,
)
from ros2_medkit_test_utils.launch_helpers import (
    create_demo_nodes,
    create_gateway_node,
)

PRIMARY_PORT = get_test_port(0)
PEER_PORT = get_test_port(1)
PRIMARY_URL = f'http://localhost:{PRIMARY_PORT}{API_BASE_PATH}'
PEER_URL = f'http://localhost:{PEER_PORT}{API_BASE_PATH}'

PRIMARY_DOMAIN_ID = get_test_domain_id(0)
PEER_DOMAIN_ID = get_test_domain_id(1)

HOLDER = 'client-holder'
INTRUDER = 'client-intruder'

# The peer owns this app, so the primary knows it only through aggregation -
# which is what puts a gateway boundary between the lock and the request being
# judged against it.
PEER_APP = 'peer_sensor'
PEER_DATA_ITEM = 'chassis/brakes/pressure'

PEER_MANIFEST = f"""\
manifest_version: "1.0"
metadata:
  name: "Peer ECU"
  version: "1.0.0"
config:
  unmanifested_nodes: ignore
components:
  - id: peer-ecu
    name: "Peer ECU"
apps:
  - id: {PEER_APP}
    name: "Peer Sensor"
    is_located_on: peer-ecu
    ros_binding:
      node_name: pressure_sensor
      namespace: /chassis/brakes
"""


def _write_manifest(text):
    handle = tempfile.NamedTemporaryFile(
        mode='w', suffix='.yaml', delete=False, prefix='lock_identity_')
    handle.write(text)
    handle.close()
    return handle.name


def generate_test_description():
    """Launch an aggregating gateway and the peer that owns the entity."""
    peer_manifest_path = _write_manifest(PEER_MANIFEST)
    peer_domain_env = {'ROS_DOMAIN_ID': str(PEER_DOMAIN_ID)}

    locking = {
        'locking.enabled': True,
        'locking.default_max_expiration': 3600,
        'locking.cleanup_interval': 1,
    }

    primary_gateway = create_gateway_node(
        port=PRIMARY_PORT,
        extra_params={
            **locking,
            'aggregation.enabled': True,
            'aggregation.timeout_ms': 5000,
            'aggregation.announce': False,
            'aggregation.discover': False,
            'aggregation.peer_urls': [f'http://localhost:{PEER_PORT}'],
            'aggregation.peer_names': ['peer_gateway'],
        },
    )

    peer_gateway = create_gateway_node(
        name='peer_gateway_node',
        port=PEER_PORT,
        extra_params={
            **locking,
            'discovery.mode': 'hybrid',
            'discovery.manifest_path': peer_manifest_path,
            'discovery.manifest_strict_validation': False,
        },
        extra_env=peer_domain_env,
    )

    delayed = TimerAction(
        period=2.0,
        actions=create_demo_nodes(['pressure_sensor'], extra_env=peer_domain_env),
    )

    launch_description = LaunchDescription([
        SetEnvironmentVariable('ROS_DOMAIN_ID', str(PRIMARY_DOMAIN_ID)),
        primary_gateway,
        peer_gateway,
        delayed,
        launch_testing.actions.ReadyToTest(),
    ])
    return (
        launch_description,
        {'gateway_node': primary_gateway, 'peer_gateway': peer_gateway},
    )


class AggregateLockIdentityTest(unittest.TestCase):
    """Drives the aggregating gateway; the peer is only ever used to verify."""

    _lock_id = None

    def _wait_for_peer_app(self, timeout=60.0):
        """Block until the primary has merged the peer's app."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            try:
                response = requests.get(f'{PRIMARY_URL}/apps', timeout=5)
                if response.status_code == 200:
                    ids = [item.get('id') for item in response.json().get('items', [])]
                    if PEER_APP in ids:
                        return
            except requests.RequestException:
                pass
            time.sleep(0.25)
        self.fail(f'{PEER_APP} was never merged into the aggregating gateway')

    def test_a_lock_taken_through_the_aggregate_belongs_to_the_client_who_took_it(self):
        """L1: the holder's name reaches the gateway that records the lock."""
        self._wait_for_peer_app()

        acquired = requests.post(
            f'{PRIMARY_URL}/apps/{PEER_APP}/locks',
            headers={'X-Client-Id': HOLDER},
            json={'lock_expiration': 600},
            timeout=15,
        )
        self.assertIn(
            acquired.status_code, (200, 201),
            f'acquiring a lock on a peer-owned app through the aggregate failed: '
            f'{acquired.status_code} {acquired.text}',
        )
        type(self)._lock_id = acquired.json().get('id')

        # Asked of the PEER, not of the gateway under test. `owned` is the only
        # place the answer is visible: the wire never carries another client's
        # identity, so ownership is reported relative to whoever is asking.
        on_peer = requests.get(
            f'{PEER_URL}/apps/{PEER_APP}/locks',
            headers={'X-Client-Id': HOLDER},
            timeout=15,
        )
        self.assertEqual(on_peer.status_code, 200, on_peer.text)
        items = on_peer.json().get('items', [])
        self.assertTrue(items, 'the peer recorded no lock at all')
        self.assertTrue(
            any(item.get('owned') for item in items),
            f'the peer holds a lock that belongs to nobody it can recognise, so '
            f'the client that took it is not its owner there: {items}',
        )

    def test_b_another_client_is_refused_while_the_lock_is_held(self):
        """L2: the property a lock exists for, judged on the peer."""
        self.assertIsNotNone(
            type(self)._lock_id,
            'no lock was acquired, so this case would prove nothing',
        )

        refused = requests.put(
            f'{PRIMARY_URL}/apps/{PEER_APP}/data/{PEER_DATA_ITEM}',
            headers={'X-Client-Id': INTRUDER},
            json={'data': {'pressure': 1.0}},
            timeout=15,
        )
        self.assertEqual(
            refused.status_code, 409,
            f'a client holding no lock was allowed to write to a locked entity: '
            f'{refused.status_code} {refused.text}',
        )

    def test_c_the_holder_is_still_allowed_while_it_holds_the_lock(self):
        """The other half of L2: refusing everybody is not the fix either."""
        self.assertIsNotNone(
            type(self)._lock_id,
            'no lock was acquired, so this case would prove nothing',
        )

        allowed = requests.put(
            f'{PRIMARY_URL}/apps/{PEER_APP}/data/{PEER_DATA_ITEM}',
            headers={'X-Client-Id': HOLDER},
            json={'data': {'pressure': 2.0}},
            timeout=15,
        )
        self.assertNotEqual(
            allowed.status_code, 409,
            f'the client holding the lock was refused by its own lock: '
            f'{allowed.status_code} {allowed.text}',
        )


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):
    """Both gateways must come down cleanly."""

    def test_exit_codes(self, proc_info):
        """Check all processes exited cleanly."""
        for info in proc_info:
            self.assertIn(
                info.returncode,
                ALLOWED_EXIT_CODES,
                f'{info.process_name} exited with code {info.returncode}',
            )
