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

"""End-to-end specification for configurations on peer-owned members.

An entity whose members are not this gateway's to serve still has to answer for
them.

test_grouping_entity_aggregation covers the shape where the aggregating entity
has SEVERAL locally resolvable nodes. Two deployments it cannot express are the
ones this file exists for, and both are ordinary:

  AGGREGATOR-ONLY   a parent gateway that runs no ROS node of its own. Every
                    member of the entity belongs to a peer. The local walk
                    resolves NO node for it, so anything that decides what an
                    id means from the local node count decides it from zero.

  ONE LOCAL, N PEER a gateway that runs one node and aggregates the rest. The
                    local walk resolves exactly one node, so anything that
                    treats "more than one local node" as the mark of an
                    aggregate concludes this entity is not one.

Both deployments address a parameter the same way every other member-qualified
id is addressed - ``<member>:<param>`` - and the member half means exactly what
it means everywhere else: the entity that owns the parameter. Whether this
gateway happens to run that member's node decides WHERE the request is served,
never whether the id parses.

THE RULES

C1  An entity whose members all belong to peers still lists its members'
    parameters, and a read of one returns THAT member's value. Asserting the
    status alone cannot show this: the failure mode is a 404 raised before the
    id is even looked at, and the fix that removes it can still return a value
    read from the wrong node.
C2  A write through such an entity lands on the member's own node. Proven by
    reading the value back from the peer gateway directly, so the assertion
    never passes through the code path that performed the write.
C3  An entity with one local node and peer members reads the member half, for
    both halves it has: the local member and the peer-owned one.
C4  An id that works today keeps working and keeps its shape. A bare parameter
    name stays a bare parameter name, and the ids the list offers do not move.
    This is the constraint the other four are subordinate to: every deployment
    in the field addresses parameters by the ids these entities offer now.
C5  A colon in an id is only a separator when the half before it names a member
    of this entity. Anything else is part of the parameter name, which is what
    keeps a parameter whose name contains a colon addressable.
C6  Reset-all reports what it reset and what it did not. This gateway resets
    the nodes it runs; a member another gateway runs is not reached from here,
    and a response that says 204 to a caller whose peer-owned members were
    never touched reports a reset that did not happen.
"""

import os
import tempfile
import time
import unittest
from urllib.parse import quote

from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, TimerAction
import launch_ros.actions
import launch_testing.actions
import requests
from ros2_medkit_test_utils.constants import (
    ALLOWED_EXIT_CODES,
    API_BASE_PATH,
    get_test_domain_id,
    get_test_port,
)
from ros2_medkit_test_utils.launch_helpers import create_gateway_node

PRIMARY_PORT = get_test_port(0)
PEER_PORT = get_test_port(1)
PRIMARY_URL = f'http://localhost:{PRIMARY_PORT}{API_BASE_PATH}'
PEER_URL = f'http://localhost:{PEER_PORT}{API_BASE_PATH}'

PRIMARY_DOMAIN_ID = get_test_domain_id(0)
PEER_DOMAIN_ID = get_test_domain_id(1)

# Declared by the calibration demo node and writable, so a value put there by
# one request is observable by another.
CALIBRATION_PARAM = 'calibration_offset'

# The one node the primary runs. It exists so the ONE-LOCAL-N-PEER shape has a
# local half at all, and so the regression guard has a local entity whose ids
# must not move.
LOCAL_APP = 'local_calibration'
LOCAL_NAMESPACE = '/powertrain/engine'

# Members that live only on the peer.
PEER_CALIBRATION_APP = 'remote_calibration'
PEER_PRESSURE_APP = 'remote_pressure'
PEER_NAMESPACE = '/chassis/brakes'

# Declared on BOTH gateways, so it merges rather than being routed whole to the
# peer: a Function only one gateway declares gets a routing entry and the whole
# request is handed over, which is a different code path and not the one under
# test. The primary declares no hosts for it - its whole membership arrives
# from the peer, which is what makes it aggregator-only.
AGGREGATOR_ONLY_FUNCTION = 'remote_health'

# Declared on the primary alone, hosting one local App and one peer-owned one.
MIXED_FUNCTION = 'mixed_health'

PRIMARY_COMPONENT = 'primary-ecu'
PEER_COMPONENT = 'remote-ecu'

PRIMARY_MANIFEST = f"""\
manifest_version: "1.0"
metadata:
  name: "Aggregating ECU"
  version: "1.0.0"
config:
  unmanifested_nodes: ignore
components:
  - id: {PRIMARY_COMPONENT}
    name: "Primary ECU"
apps:
  - id: {LOCAL_APP}
    name: "Local Calibration Service"
    is_located_on: {PRIMARY_COMPONENT}
    ros_binding:
      node_name: calibration
      namespace: {LOCAL_NAMESPACE}
functions:
  # No hosts of its own. The peer declares the same id with its two Apps, the
  # two declarations merge, and every member of the merged Function is then
  # peer-owned.
  - id: {AGGREGATOR_ONLY_FUNCTION}
    name: "Remote Health Monitoring"
    category: monitoring
  - id: {MIXED_FUNCTION}
    name: "Mixed Health Monitoring"
    category: monitoring
    hosted_by:
      - {LOCAL_APP}
      - {PEER_CALIBRATION_APP}
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
  - id: {PEER_CALIBRATION_APP}
    name: "Remote Calibration Service"
    is_located_on: {PEER_COMPONENT}
    ros_binding:
      node_name: calibration
      namespace: {PEER_NAMESPACE}
  - id: {PEER_PRESSURE_APP}
    name: "Remote Brake Pressure Sensor"
    is_located_on: {PEER_COMPONENT}
    ros_binding:
      node_name: pressure_sensor
      namespace: {PEER_NAMESPACE}
functions:
  # The Component is a host alongside its own Apps. A Function is
  # cross-component by definition and hosting one is ordinary, and it puts a
  # member in the set that carries no parameters of its own - the case a
  # per-member report has to leave out, because its Apps already carry the
  # parameters and naming it too would report the same gap twice.
  - id: {AGGREGATOR_ONLY_FUNCTION}
    name: "Remote Health Monitoring"
    category: monitoring
    hosted_by:
      - {PEER_CALIBRATION_APP}
      - {PEER_PRESSURE_APP}
      - {PEER_COMPONENT}
"""


def _write_manifest(content):
    """Write manifest YAML to a temporary file and return its path."""
    fd, path = tempfile.mkstemp(suffix='.yaml', prefix='test_aggregator_only_manifest_')
    with os.fdopen(fd, 'w') as handle:
        handle.write(content)
    return path


def generate_test_description():
    primary_manifest_path = _write_manifest(PRIMARY_MANIFEST)
    peer_manifest_path = _write_manifest(PEER_MANIFEST)

    peer_domain_env = {'ROS_DOMAIN_ID': str(PEER_DOMAIN_ID)}

    primary_gateway = create_gateway_node(
        port=PRIMARY_PORT,
        extra_params={
            'discovery.mode': 'hybrid',
            'discovery.manifest_path': primary_manifest_path,
            'discovery.manifest_strict_validation': False,
            'aggregation.enabled': True,
            'aggregation.timeout_ms': 5000,
            'aggregation.announce': False,
            'aggregation.discover': False,
            'aggregation.peer_urls': [f'http://localhost:{PEER_PORT}'],
            'aggregation.peer_names': ['remote_gateway'],
        },
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

    # Built inline rather than through create_demo_nodes because the registry
    # binds each key to one fixed namespace, and both gateways need a
    # calibration node in a namespace of their own.
    local_calibration = launch_ros.actions.Node(
        package='ros2_medkit_integration_tests',
        executable='demo_calibration_service',
        name='calibration',
        namespace=LOCAL_NAMESPACE,
        output='screen',
    )
    peer_calibration = launch_ros.actions.Node(
        package='ros2_medkit_integration_tests',
        executable='demo_calibration_service',
        name='calibration',
        namespace=PEER_NAMESPACE,
        output='screen',
        additional_env=peer_domain_env,
    )
    peer_pressure = launch_ros.actions.Node(
        package='ros2_medkit_integration_tests',
        executable='demo_brake_pressure_sensor',
        name='pressure_sensor',
        namespace=PEER_NAMESPACE,
        output='screen',
        additional_env=peer_domain_env,
    )

    delayed = TimerAction(
        period=2.0,
        actions=[local_calibration, peer_calibration, peer_pressure],
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


class AggregatorOnlyConfigurationsTest(unittest.TestCase):
    """Drives the aggregating gateway; the peer is only ever used to verify."""

    @classmethod
    def setUpClass(cls):
        # A manifest App exists before its node does, and an App with no live
        # binding contributes no parameters, so presence alone would let a
        # collection read run against an entity that is legitimately empty.
        cls._wait_for_apps(PRIMARY_URL, {LOCAL_APP}, 'primary')
        cls._wait_for_apps(
            PEER_URL, {PEER_CALIBRATION_APP, PEER_PRESSURE_APP}, 'peer')
        cls._wait_until_merged()

    @classmethod
    def _wait_for_apps(cls, base_url, required, label):
        """Block until `required` Apps are present AND bound to a live node."""
        deadline = time.monotonic() + 60.0
        while time.monotonic() < deadline:
            try:
                response = requests.get(f'{base_url}/apps', timeout=5)
                if response.status_code == 200:
                    online = {
                        item.get('id')
                        for item in response.json().get('items', [])
                        if item.get('x-medkit', {}).get('is_online')
                    }
                    if required <= online:
                        return
            except requests.RequestException:
                pass
            time.sleep(1.0)
        raise AssertionError(f'{label}: {required} not online within 60s')

    @classmethod
    def _wait_until_merged(cls):
        """Block until the peer's members are visible on the primary."""
        deadline = time.monotonic() + 60.0
        while time.monotonic() < deadline:
            try:
                response = requests.get(f'{PRIMARY_URL}/apps', timeout=5)
                if response.status_code == 200:
                    ids = {
                        item.get('id') for item in response.json().get('items', [])
                    }
                    if {PEER_CALIBRATION_APP, PEER_PRESSURE_APP} <= ids:
                        return
            except requests.RequestException:
                pass
            time.sleep(0.5)
        raise AssertionError("the peer's Apps did not merge into the primary in 60s")

    # ------------------------------------------------------------------ helpers

    def _items(self, entity_path, collection):
        response = requests.get(
            f'{PRIMARY_URL}/{entity_path}/{collection}', timeout=15)
        self.assertEqual(response.status_code, 200, response.text)
        return response.json().get('items', [])

    @staticmethod
    def _config_url(base_url, entity_path, config_id):
        return (
            f'{base_url}/{entity_path}/configurations/{quote(config_id, safe="")}'
        )

    def _seed_on_peer(self, app_id, value):
        """Put a value on one peer App through THE PEER's own gateway.

        The value the aggregate is then asked for was written by a request that
        never went through the aggregate, so a read that returns it cannot have
        got it from the write path under test.
        """
        response = requests.put(
            self._config_url(PEER_URL, f'apps/{app_id}', CALIBRATION_PARAM),
            json={'data': value},
            timeout=15,
        )
        self.assertEqual(
            response.status_code, 200,
            f'could not seed {app_id}.{CALIBRATION_PARAM} on the peer: '
            f'{response.status_code} {response.text}',
        )

    def _read_on_peer(self, app_id):
        """Read a value from one peer App through THE PEER's own gateway."""
        response = requests.get(
            self._config_url(PEER_URL, f'apps/{app_id}', CALIBRATION_PARAM),
            timeout=15,
        )
        self.assertEqual(
            response.status_code, 200,
            f'could not read {app_id}.{CALIBRATION_PARAM} on the peer: '
            f'{response.status_code} {response.text}',
        )
        return response.json().get('data')

    def _seed_locally(self, value):
        """Put a value on the local App through its own route on the primary."""
        response = requests.put(
            self._config_url(PRIMARY_URL, f'apps/{LOCAL_APP}', CALIBRATION_PARAM),
            json={'data': value},
            timeout=15,
        )
        self.assertEqual(
            response.status_code, 200,
            f'could not seed {LOCAL_APP}.{CALIBRATION_PARAM}: '
            f'{response.status_code} {response.text}',
        )

    # ----------------------------------------------------------------- C1 (list)

    def test_a1_an_aggregator_only_entity_lists_its_peer_members_parameters(self):
        """C1, the listing half.

        The entity resolves no local node at all. Everything it can offer comes
        from members another gateway runs, so a listing that answers from the
        local node set alone has nothing to say and the collection looks empty -
        or, worse, refuses outright.
        """
        items = self._items(f'functions/{AGGREGATOR_ONLY_FUNCTION}', 'configurations')
        ids = [item.get('id') for item in items]
        self.assertIn(
            f'{PEER_CALIBRATION_APP}:{CALIBRATION_PARAM}', ids,
            f'an aggregator-only entity offered no parameter of its members: {ids}',
        )

    # ------------------------------------------------------------- C1 (single read)

    def test_a2_a_read_on_an_aggregator_only_entity_returns_the_members_value(self):
        """C1, the read half, asserted on the value and on who served it.

        A 200 says only that something answered. The seeded value says the
        answer came from the member's own node, and `x-medkit.entity_id` says
        which entity produced it - the member itself, because the member's own
        gateway answered on the member's own route.
        """
        self._seed_on_peer(PEER_CALIBRATION_APP, 4.25)

        config_id = f'{PEER_CALIBRATION_APP}:{CALIBRATION_PARAM}'
        response = requests.get(
            self._config_url(
                PRIMARY_URL, f'functions/{AGGREGATOR_ONLY_FUNCTION}', config_id),
            timeout=15,
        )
        self.assertEqual(response.status_code, 200, response.text)
        body = response.json()
        self.assertAlmostEqual(
            body.get('data'), 4.25, places=6,
            msg=f'the value did not come from the member that holds it: {body}',
        )
        self.assertEqual(
            body.get('x-medkit', {}).get('entity_id'), PEER_CALIBRATION_APP,
            f'the read was not served by the member that owns the parameter: {body}',
        )

    # ------------------------------------------------------------------ C2 (write)

    def test_a3_a_write_on_an_aggregator_only_entity_lands_on_the_peer(self):
        """C2: the write reaches the member's own node.

        Read back from the PEER gateway, not from the aggregate. A write that
        was accepted and dropped, or applied to some other node, still returns
        200 here and still reads back through the aggregate if the aggregate
        answers from whatever it wrote.
        """
        self._seed_on_peer(PEER_CALIBRATION_APP, 0.0)

        config_id = f'{PEER_CALIBRATION_APP}:{CALIBRATION_PARAM}'
        response = requests.put(
            self._config_url(
                PRIMARY_URL, f'functions/{AGGREGATOR_ONLY_FUNCTION}', config_id),
            json={'data': 7.5},
            timeout=15,
        )
        self.assertEqual(response.status_code, 200, response.text)

        self.assertAlmostEqual(
            self._read_on_peer(PEER_CALIBRATION_APP), 7.5, places=6,
            msg='the write did not reach the peer that runs the member',
        )

    def test_a3b_a_reset_of_one_parameter_reaches_the_peer_owned_member(self):
        """C2 for the third method the single-item route carries.

        Reset takes the same id and the same owner as read and write, so it has
        to reach the same node. Checked by reading the value back from the peer:
        the parameter is at the default its node declared, which is a state only
        a reset on that node produces.
        """
        self._seed_on_peer(PEER_CALIBRATION_APP, 9.75)

        config_id = f'{PEER_CALIBRATION_APP}:{CALIBRATION_PARAM}'
        response = requests.delete(
            self._config_url(
                PRIMARY_URL, f'functions/{AGGREGATOR_ONLY_FUNCTION}', config_id),
            timeout=15,
        )
        self.assertEqual(response.status_code, 204, response.text)

        self.assertAlmostEqual(
            self._read_on_peer(PEER_CALIBRATION_APP), 0.0, places=6,
            msg='the reset did not reach the peer that runs the member',
        )

    # ------------------------------------------------------------------------- C3

    def test_a4_one_local_node_plus_peer_members_still_reads_the_member_half(self):
        """C3: the shape whose local node count is one.

        One local node is not "not aggregating" - it is an aggregate with one
        local member. Both halves have to resolve: the peer-owned member, whose
        node this gateway cannot see, and the local one, whose node it runs.
        """
        self._seed_on_peer(PEER_CALIBRATION_APP, 3.5)
        self._seed_locally(-1.25)

        for member, expected in (
            (PEER_CALIBRATION_APP, 3.5),
            (LOCAL_APP, -1.25),
        ):
            with self.subTest(member=member):
                config_id = f'{member}:{CALIBRATION_PARAM}'
                response = requests.get(
                    self._config_url(
                        PRIMARY_URL, f'functions/{MIXED_FUNCTION}', config_id),
                    timeout=15,
                )
                self.assertEqual(response.status_code, 200, response.text)
                body = response.json()
                self.assertAlmostEqual(
                    body.get('data'), expected, places=6,
                    msg=f"{config_id} did not return {member}'s value: {body}",
                )

    # ------------------------------------------------------------------------- C4

    def test_a5_a_bare_parameter_name_and_the_ids_the_list_offers_do_not_move(self):
        """C4, the constraint every other rule here is subordinate to.

        An App is a single-node entity and its parameter ids are bare names.
        Nothing about member addressing may reach that: a client holding
        `calibration_offset` today must still be able to send it, and the list
        must still offer it in that form.
        """
        self._seed_locally(2.5)

        items = self._items(f'apps/{LOCAL_APP}', 'configurations')
        ids = [item.get('id') for item in items]
        self.assertIn(
            CALIBRATION_PARAM, ids,
            f'the App no longer offers its parameter under its bare name: {ids}',
        )
        self.assertFalse(
            [i for i in ids if ':' in str(i)],
            f'a single-node entity qualified its parameter ids: {ids}',
        )

        response = requests.get(
            self._config_url(PRIMARY_URL, f'apps/{LOCAL_APP}', CALIBRATION_PARAM),
            timeout=15,
        )
        self.assertEqual(response.status_code, 200, response.text)
        self.assertAlmostEqual(response.json().get('data'), 2.5, places=6)

    def test_a6_a_bare_name_on_an_entity_with_members_still_reads_its_local_node(self):
        """C4 on the aggregating entity, where the temptation to qualify is.

        The mixed Function resolves one local node, so the ids it offers for
        that node are bare today. Splitting on any colon, or qualifying every
        id because the entity has members, would move them.
        """
        self._seed_locally(6.75)

        items = self._items(f'functions/{MIXED_FUNCTION}', 'configurations')
        local_ids = [
            item.get('id') for item in items
            if str(item.get('id', '')).endswith(CALIBRATION_PARAM)
        ]
        self.assertIn(
            CALIBRATION_PARAM, local_ids,
            f"the local member's parameter id moved off its bare form: {local_ids}",
        )

        response = requests.get(
            self._config_url(
                PRIMARY_URL, f'functions/{MIXED_FUNCTION}', CALIBRATION_PARAM),
            timeout=15,
        )
        self.assertEqual(response.status_code, 200, response.text)
        self.assertAlmostEqual(response.json().get('data'), 6.75, places=6)

    # ------------------------------------------------------------------------- C5

    def test_a7_a_colon_that_names_no_member_stays_part_of_the_parameter_name(self):
        """C5: a prefix that is not a member is not a member half.

        `not_a_member` names nothing in this entity, so the whole string is a
        parameter name - one no node declares, hence a miss on the parameter
        and not on the member. The two are distinguishable on the wire and have
        to stay so: reporting a bad parameter name as an unreachable member
        sends the caller looking for a gateway that is not down.
        """
        config_id = f'not_a_member:{CALIBRATION_PARAM}'
        response = requests.get(
            self._config_url(PRIMARY_URL, f'functions/{MIXED_FUNCTION}', config_id),
            timeout=15,
        )
        self.assertEqual(response.status_code, 404, response.text)
        body = response.json()
        # The message is what discriminates. A split id that named nothing is
        # refused as a missing member or, for a member whose gateway is silent,
        # as `not-responding`; an unsplit one is refused as a parameter the
        # nodes do not declare, which is what this id is.
        self.assertEqual(
            body.get('message'), 'Parameter not found',
            f'the prefix was read as a member half: {body}',
        )
        self.assertEqual(
            body.get('parameters', {}).get('id'), config_id,
            f'the id was not carried back whole: {body}',
        )

    def test_a7a_a_member_half_with_no_parameter_after_it_addresses_nothing(self):
        """C5: a member half alone names no parameter.

        One path segment shorter is the member's own configurations COLLECTION.
        Read as a member half, this id would be re-addressed there and a caller
        that asked for one value would be handed a list - with 200 on it, so
        nothing downstream would notice.
        """
        config_id = f'{PEER_CALIBRATION_APP}:'
        response = requests.get(
            self._config_url(
                PRIMARY_URL, f'functions/{AGGREGATOR_ONLY_FUNCTION}', config_id),
            timeout=15,
        )
        self.assertNotEqual(
            response.status_code, 200,
            f'an id naming no parameter was answered: {response.text}',
        )
        self.assertNotIn(
            'items', response.json(),
            f'a single-value read was answered with a collection: {response.text}',
        )

    def test_a7b_an_entity_is_not_a_member_half_of_itself(self):
        """C5: the entity's own id separates nothing.

        `local_calibration:calibration_offset` on the App `local_calibration`
        would name the same entity twice. Reading it as a member half gives the
        parameter a second id that no listing offers, and every client that
        holds an id would then have two ways to spell it and one of them
        undocumented.
        """
        config_id = f'{LOCAL_APP}:{CALIBRATION_PARAM}'
        response = requests.get(
            self._config_url(PRIMARY_URL, f'apps/{LOCAL_APP}', config_id),
            timeout=15,
        )
        self.assertEqual(
            response.status_code, 404,
            f'an entity answered to its own id as a member half: {response.text}',
        )

    # ------------------------------------------------------------------------- C6

    def test_a8_reset_all_reports_the_members_it_did_not_reach(self):
        """C6: reset-all tells the truth about its own reach.

        This gateway resets the nodes it runs. A member another gateway runs is
        not reached from here, and the caller has to be told - a plain success
        for an entity whose peer-owned half was never touched is a reset the
        caller believes happened.

        Both claims are checked against what actually happened: the local
        member's parameter is read back and is at its default, and the peer
        member's is read back from the peer and is untouched.
        """
        self._seed_on_peer(PEER_CALIBRATION_APP, 5.5)
        self._seed_locally(5.5)

        response = requests.delete(
            f'{PRIMARY_URL}/functions/{MIXED_FUNCTION}/configurations', timeout=20)
        self.assertEqual(
            response.status_code, 207,
            f'reset-all reported plain success for an entity it could not fully '
            f'reset: {response.status_code} {response.text}',
        )
        results = response.json().get('results', [])
        by_member = {entry.get('app_id'): entry for entry in results}

        # The member this gateway runs is named, and named as attempted here:
        # its entry carries the ROS node the reset was addressed to.
        self.assertIn(
            LOCAL_APP, by_member,
            f'the member this gateway runs is not named: {results}')
        self.assertTrue(
            by_member[LOCAL_APP].get('node'),
            f"the local member's entry names no node: {by_member[LOCAL_APP]}",
        )
        # What the reset did on that node, per parameter. A verdict on its own
        # leaves the caller nothing to act on, and the entry is the only place
        # this ever appears.
        self.assertIsInstance(
            by_member[LOCAL_APP].get('details'), dict,
            f"the local member's entry reports no per-parameter outcome: "
            f'{by_member[LOCAL_APP]}',
        )

        # The member another gateway runs is named as not reset, and the entry
        # says who does own it - without that the caller has nowhere to go.
        self.assertIn(
            PEER_CALIBRATION_APP, by_member,
            f'the member this gateway did not reach is not named: {results}')
        peer_entry = by_member[PEER_CALIBRATION_APP]
        self.assertIs(peer_entry.get('success'), False, peer_entry)
        self.assertIn(
            'remote_gateway', str(peer_entry.get('error', '')),
            f'the entry does not say which gateway owns the member: {peer_entry}',
        )

        # Both claims are checked against reality, so the test fails if the
        # response lies in either direction.
        response = requests.get(
            self._config_url(PRIMARY_URL, f'apps/{LOCAL_APP}', CALIBRATION_PARAM),
            timeout=15,
        )
        self.assertEqual(response.status_code, 200, response.text)
        self.assertAlmostEqual(
            response.json().get('data'), 0.0, places=6,
            msg='the local member was named as reset here, but its value stands',
        )
        self.assertAlmostEqual(
            self._read_on_peer(PEER_CALIBRATION_APP), 5.5, places=6,
            msg='the response said the peer member was not reset, but it was',
        )

    def test_a9_reset_all_on_an_aggregator_only_entity_is_not_plain_success(self):
        """C6 where NOTHING is reachable from here.

        Every member belongs to a peer, so this gateway resets nothing at all.
        That is the case where a 204 is most misleading, because it reports a
        complete reset of an entity nothing was done to.
        """
        self._seed_on_peer(PEER_CALIBRATION_APP, 8.25)

        response = requests.delete(
            f'{PRIMARY_URL}/functions/{AGGREGATOR_ONLY_FUNCTION}/configurations',
            timeout=20,
        )
        self.assertEqual(
            response.status_code, 207,
            f'an entity with no locally resettable member reported a complete '
            f'reset: {response.status_code} {response.text}',
        )
        results = response.json().get('results', [])
        not_reached = {
            entry.get('app_id') for entry in results
            if entry.get('success') is False
        }
        self.assertEqual(
            not_reached, {PEER_CALIBRATION_APP, PEER_PRESSURE_APP},
            f'the members that were not reached are not named: {results}',
        )

        self.assertAlmostEqual(
            self._read_on_peer(PEER_CALIBRATION_APP), 8.25, places=6,
            msg='the response said nothing was reset, but the peer value moved',
        )

    def test_b1_a_peer_owned_operation_of_a_local_entity_is_not_reported_unreachable(self):
        """`available: false` is a statement about the member, not about the fan-out.

        Only this gateway declares the mixed Function, so no peer contributes
        the entity and the peer collection fan-out for it never runs - there is
        nobody to ask. That says nothing about the member: its gateway is up,
        and the request for the operation is dispatched to the member's own
        route and served. Marking the item unavailable there contradicts both
        the field's meaning and what the very next request does, so the two are
        checked together and the execution is asserted on its body - a status
        alone cannot tell a service that ran from one that was never called.
        """
        items = self._items(f'functions/{MIXED_FUNCTION}', 'operations')
        by_id = {item.get('id'): item for item in items}
        operation_id = f'{PEER_CALIBRATION_APP}:calibrate'
        self.assertIn(
            operation_id, by_id,
            f'the mixed Function does not offer its peer-owned operation: {sorted(by_id)}',
        )
        self.assertNotEqual(
            by_id[operation_id].get('x-medkit', {}).get('available'), False,
            f'a peer-owned operation was reported unreachable while its gateway '
            f'answers: {by_id[operation_id]}',
        )

        response = requests.post(
            f'{PRIMARY_URL}/functions/{MIXED_FUNCTION}/operations/'
            f'{quote(operation_id, safe="")}/executions',
            json={},
            timeout=15,
        )
        self.assertEqual(response.status_code, 200, response.text)
        parameters = response.json().get('parameters')
        self.assertIsInstance(
            parameters, dict, f'no service response came back: {response.text}')
        self.assertIs(
            parameters.get('success'), True,
            f"the member's service did not report success: {parameters}",
        )
        self.assertTrue(
            parameters.get('message'),
            f"the member's service answered with nothing to say: {parameters}",
        )


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        """Check all processes exited cleanly (SIGTERM allowed)."""
        for info in proc_info:
            self.assertIn(
                info.returncode, ALLOWED_EXIT_CODES,
                f'{info.process_name} exited with code {info.returncode}'
            )
