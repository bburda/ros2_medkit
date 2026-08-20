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

"""
End-to-end specification for one resource-addressing model across any entity.

An AGGREGATING entity draws its resources from members rather than owning them:
an Area, a Function (cross-component and cross-area by definition), or a
Component with subcomponents - and a subcomponent can live on its own gateway,
so a parent on the host with a child on another ECU is a normal deployment.

The model has two halves and they have to be built in this order.

IDENTITY FIRST. An item can only be addressed back to whatever serves it if
that thing has a name that is unique in the merged tree. Today Apps are
renamed on collision (`<peer>__<id>`) but Components are not - two peers
announcing one Component id resolve last-writer-wins - and member ids arriving
from a peer are re-emitted verbatim, so they can name something the aggregator
does not have. Until leaf identity is unique, no addressing scheme works,
because the qualifier itself is ambiguous.

ADDRESSING SECOND, and it has exactly two cases, decided per collection rather
than per handler:

  a LEAF-OWNED item  - a topic, an operation, a configuration key - belongs to
                       one leaf, and is addressed "<leaf>:<item>".
  an AGGREGATE item  - a fault, which has one code and a SET of reporting
                       sources - belongs to the aggregate itself, and is
                       addressed by its own id while naming its contributors.

Faults, logs and bulk-data are not exceptions to the model; they are the second
case. Which case a collection is, is declared once.

WHAT THIS SUITE EXISTS TO PREVENT, all three measured on this topology:

  * A grouping lists an item, then answers a read of it from the LOCAL graph
    only. An unknown topic returns 200 with status "metadata_only", so the
    client is told the read succeeded and handed an empty body.
  * Two members exposing one operation short name: the list shows both, and
    execution resolves against the local cache alone.
  * The fan-out concatenates peer items with no key, so one id can appear twice
    with nothing on the wire telling the copies apart.

THE RULES

R1  A leaf contributed by more than one gateway is addressable as more than one
    entity. Identity is unique after the merge.
R2  Every listed item names the leaf or leaves that contribute it.
R3  A leaf-owned item provided by MORE THAN ONE leaf is addressed
    "<leaf>:<item>". An item with a single provider keeps its bare id.
    Qualification follows ambiguity, not aggregation: in runtime discovery every
    App hangs off the one host Component, so "aggregating" is the ordinary
    entity, and qualifying everything there would change the ids of the most
    used entity in the product and refuse requests every current client sends.
R4  A bare id is refused only when it is ambiguous, and the refusal names the
    form to use. An unambiguous bare id keeps working.
R5  A compound id reaches its leaf, local or on a peer, and returns that leaf's
    data. Asserting the status alone cannot show this: the failure mode is a
    200 with an empty body.
R6  An item no leaf provides is refused, and is distinguishable from an item
    that exists and is empty.
R7  A Component both sides contribute to aggregates. Routing it wholesale to
    one peer would discard the other half, which is what happens today.
R8  A lock on a leaf is honoured by a request dispatched through an aggregate.
    Verified in test_aggregate_lock_identity, which needs a second gateway
    holding the lock and so cannot share this topology.
R9  Peered gateways terminate.
R10 A manifest-declared entity outlives the link that reported it: it stays in
    the tree, keeps the items it last reported, and says it cannot be reached.
    A runtime-discovered one vanishes, because it describes a graph this
    gateway can no longer observe. Availability belongs to what a request can
    be addressed to - an App or a Component. A grouping entity is a view over
    members and carries none of its own; a client asks the members.
"""

import os
import signal
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
    API_BASE_PATH,
    get_test_domain_id,
    get_test_port,
)
from ros2_medkit_test_utils.launch_helpers import (
    create_demo_nodes,
    create_fault_manager_node,
    create_gateway_node,
)

PRIMARY_PORT = get_test_port(0)
PEER_PORT = get_test_port(1)
PRIMARY_URL = f'http://localhost:{PRIMARY_PORT}{API_BASE_PATH}'
PEER_URL = f'http://localhost:{PEER_PORT}{API_BASE_PATH}'

PRIMARY_DOMAIN_ID = get_test_domain_id(0)
PEER_DOMAIN_ID = get_test_domain_id(1)

# `calibration` runs on BOTH domains deliberately. Each gateway binds it to an
# App of its own, so one operation short name - the id the HTTP API addresses -
# is exposed by a member on each side. That is R3's case and it is not
# hypothetical: deduplication keys on the full ROS path, the wire id is the
# short name, so both survive.
# The peer's calibration node runs in a DIFFERENT namespace on purpose. Both
# gateways expose an operation whose short name - the id the HTTP API addresses
# - is `calibrate`, but the full ROS paths differ. That is the case R3 and R4
# exist for. With the same namespace on both sides the full paths are identical
# and the local walk simply deduplicates the peer's copy away, which builds a
# dedup collapse rather than the ambiguity.
PRIMARY_NODES = ['temp_sensor', 'calibration', 'rpm_sensor']
PEER_NODES = ['pressure_sensor', 'actuator']
PEER_CALIBRATION_NAMESPACE = '/chassis/brakes'

# Declared with the SAME id on both gateways. Apps are renamed on collision,
# Components are not, so this is the case that shows whether leaf identity
# survives the merge - R1, the precondition for every addressing rule.
COLLIDING_LEAF = 'shared_sensor'

# Declared by the calibration demo node, so BOTH `primary_calibration` and
# `peer_calibration` expose it under one name. A member-qualified id is the only
# thing that separates the two copies, which is what makes this the parameter
# the configuration cases below address.
CALIBRATION_PARAM = 'calibration_offset'

MERGED_AREA = 'vehicle'
MERGED_FUNCTION = 'vehicle_health'
PARENT_COMPONENT = 'vehicle-ecu'
PEER_SUBCOMPONENT = 'brake-ecu'

# Both gateways contribute to one Area id and one Function id, and the peer's
# Component declares the primary's Component as its parent - so all three
# aggregating kinds span the pair.
PRIMARY_MANIFEST = f"""\
manifest_version: "1.0"
metadata:
  name: "Primary ECU"
  version: "1.0.0"
config:
  unmanifested_nodes: ignore
areas:
  - id: {MERGED_AREA}
    name: "Vehicle"
components:
  - id: {PARENT_COMPONENT}
    name: "Vehicle ECU"
    area: {MERGED_AREA}
apps:
  - id: temp_sensor
    name: "Engine Temperature Sensor"
    is_located_on: {PARENT_COMPONENT}
    ros_binding:
      node_name: temp_sensor
      namespace: /powertrain/engine
  - id: primary_calibration
    name: "Primary Calibration Service"
    is_located_on: {PARENT_COMPONENT}
    ros_binding:
      node_name: calibration
      namespace: /powertrain/engine
  - id: {COLLIDING_LEAF}
    name: "Shared Sensor (primary)"
    is_located_on: {PARENT_COMPONENT}
    ros_binding:
      node_name: rpm_sensor
      namespace: /powertrain/engine
functions:
  - id: {MERGED_FUNCTION}
    name: "Vehicle Health Monitoring"
    category: monitoring
    hosted_by:
      - temp_sensor
      - primary_calibration
      - {COLLIDING_LEAF}
"""

PEER_MANIFEST = f"""\
manifest_version: "1.0"
metadata:
  name: "Secondary ECU"
  version: "1.0.0"
config:
  # The peer exposes what it did not declare, so that its half of the tree has
  # both origins in it. Retention keeps the declared ones and drops these, and
  # a topology where everything is declared cannot show the difference.
  unmanifested_nodes: warn
areas:
  - id: {MERGED_AREA}
    name: "Vehicle"
components:
  # The parent is declared on BOTH gateways on purpose. A subcomponent may not
  # name a parent that is absent from its own manifest: rule R006 rejects it as
  # an ERROR, and an errored manifest is not loaded at all, so the peer would
  # contribute nothing. Declaring the parent on both sides is how a hierarchy
  # crosses a gateway boundary - the parent then merges, and the child stays
  # owned by the gateway that runs it.
  - id: {PARENT_COMPONENT}
    name: "Vehicle ECU"
    area: {MERGED_AREA}
  - id: {PEER_SUBCOMPONENT}
    name: "Brake ECU"
    area: {MERGED_AREA}
    parent_component_id: {PARENT_COMPONENT}
apps:
  - id: pressure_sensor
    name: "Brake Pressure Sensor"
    is_located_on: {PEER_SUBCOMPONENT}
    ros_binding:
      node_name: pressure_sensor
      namespace: /chassis/brakes
  - id: peer_calibration
    name: "Peer Calibration Service"
    is_located_on: {PEER_SUBCOMPONENT}
    ros_binding:
      node_name: calibration
      namespace: {PEER_CALIBRATION_NAMESPACE}
  - id: {COLLIDING_LEAF}
    name: "Shared Sensor (peer)"
    is_located_on: {PEER_SUBCOMPONENT}
    ros_binding:
      node_name: actuator
      namespace: /chassis/brakes
functions:
  - id: {MERGED_FUNCTION}
    name: "Vehicle Health Monitoring"
    category: monitoring
    hosted_by:
      - pressure_sensor
      - peer_calibration
      - {COLLIDING_LEAF}
"""


def _write_manifest(content):
    """Write manifest YAML to a temporary file and return its path."""
    fd, path = tempfile.mkstemp(suffix='.yaml', prefix='test_grouping_manifest_')
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
            'aggregation.peer_names': ['secondary_gateway'],
        },
    )

    peer_gateway = create_gateway_node(
        name='secondary_gateway_node',
        port=PEER_PORT,
        extra_params={
            'discovery.mode': 'hybrid',
            'discovery.manifest_path': peer_manifest_path,
            'discovery.manifest_strict_validation': False,
        },
        extra_env=peer_domain_env,
    )

    delayed = TimerAction(
        period=2.0,
        actions=(
            create_demo_nodes(PRIMARY_NODES, lidar_faulty=False)
            + create_demo_nodes(PEER_NODES, lidar_faulty=False, extra_env=peer_domain_env)
            # Built inline because create_demo_nodes takes its namespace from a
            # fixed registry and cannot place a node elsewhere.
            + [launch_ros.actions.Node(
                package='ros2_medkit_integration_tests',
                executable='demo_calibration_service',
                name='calibration',
                namespace=PEER_CALIBRATION_NAMESPACE,
                output='screen',
                additional_env=peer_domain_env,
            )]
            + [
                create_fault_manager_node(rosbag_enabled=False),
                create_fault_manager_node(rosbag_enabled=False, extra_env=peer_domain_env),
            ]
        ),
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


class GroupingAggregationTest(unittest.TestCase):
    """Drives the aggregating gateway; the peer is only ever used to verify."""

    @classmethod
    def setUpClass(cls):
        # Order matters. The entity merge is driven by HTTP from the peer and
        # completes while the local ROS graph is still binding Apps to nodes, so
        # a collection read between those two moments is legitimately empty and
        # would fail every rule below for a reason unrelated to the rule.
        cls._wait_for_apps(PRIMARY_URL, {'temp_sensor', 'primary_calibration'}, 'primary')
        cls._wait_for_apps(PEER_URL, {'pressure_sensor', 'peer_calibration'}, 'peer')
        cls._wait_until_merged()

    @classmethod
    def _wait_for_apps(cls, base_url, required, label):
        """Block until `required` Apps are present AND bound to a live node.

        Presence is not enough: a manifest App exists before its node does, and
        an App with no live binding contributes no resources.
        """
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
        """Block until the primary has merged the peer's half of the Function."""
        deadline = time.monotonic() + 60.0
        while time.monotonic() < deadline:
            try:
                response = requests.get(
                    f'{PRIMARY_URL}/functions/{MERGED_FUNCTION}', timeout=5,
                )
                if response.status_code == 200:
                    contributors = (
                        response.json().get('x-medkit', {}).get('contributors', [])
                    )
                    if 'local' in contributors and any(
                        c.startswith('peer:') for c in contributors
                    ):
                        return
            except requests.RequestException:
                pass
            time.sleep(0.5)
        raise AssertionError(f'{MERGED_FUNCTION} did not merge both contributors in 60s')

    # ------------------------------------------------------------------ helpers

    def _items(self, entity_path, collection):
        response = requests.get(f'{PRIMARY_URL}/{entity_path}/{collection}', timeout=10)
        self.assertEqual(response.status_code, 200, response.text)
        return response.json().get('items', [])

    @staticmethod
    def _set_offset(base_url, app_id, value):
        """Set `calibration_offset` on one App through ITS OWN gateway.

        The member's own route on the gateway that runs it, so the value the
        aggregate is then asked for was put there by a request that never went
        through the aggregate.
        """
        response = requests.put(
            f'{base_url}/apps/{app_id}/configurations/{CALIBRATION_PARAM}',
            json={'data': value},
            timeout=15,
        )
        if response.status_code != 200:
            raise AssertionError(
                f'could not seed {app_id}.{CALIBRATION_PARAM} on {base_url}: '
                f'{response.status_code} {response.text}'
            )
        return response

    @staticmethod
    def _offset_of(base_url, app_id):
        """Read `calibration_offset` from one App through ITS OWN gateway."""
        response = requests.get(
            f'{base_url}/apps/{app_id}/configurations/{CALIBRATION_PARAM}', timeout=15)
        if response.status_code != 200:
            raise AssertionError(
                f'could not read {app_id}.{CALIBRATION_PARAM} on {base_url}: '
                f'{response.status_code} {response.text}'
            )
        return response.json().get('data')

    # ---------------------------------------------------------------------- R1

    def test_a_leaf_contributed_by_both_gateways_stays_two_addressable_leaves(self):
        """R1, the precondition everything else rests on.

        Both gateways declare an App with the id `shared_sensor` bound to
        different nodes. They are two different things and the merged tree has
        to be able to name both, because every addressing rule below uses the
        leaf id as its qualifier. If the merge collapses them, a compound id
        built from that qualifier names two entities at once and the ambiguity
        the compound form exists to remove has simply moved up one level.
        """
        response = requests.get(f'{PRIMARY_URL}/apps', timeout=10)
        self.assertEqual(response.status_code, 200)
        ids = [item.get('id') for item in response.json().get('items', [])]

        matching = [i for i in ids if i == COLLIDING_LEAF or i.endswith(f'__{COLLIDING_LEAF}')]
        self.assertEqual(
            len(matching), 2,
            f'a leaf declared on both gateways is not addressable twice: {ids}',
        )
        # And each must actually resolve, not merely appear in the list.
        for leaf_id in matching:
            detail = requests.get(f'{PRIMARY_URL}/apps/{leaf_id}', timeout=10)
            self.assertEqual(detail.status_code, 200, f'{leaf_id} is listed but not addressable')

    # ---------------------------------------------------------------------- R7

    def test_a_component_both_sides_contribute_to_aggregates(self):
        """R7: a shared Component must fan in, not be handed to one peer.

        A Component whose id appears on both sides is put in the routing table
        today, so the whole request is forwarded and the local half is dropped.
        That is the same "local half vanishes" failure the design argues against
        for Areas and Functions, reached by a different route.
        """
        items = self._items(f'components/{PARENT_COMPONENT}', 'operations')
        members = set()
        for item in items:
            members.update(item.get('x-medkit', {}).get('member_ids') or [])
        self.assertIn(
            'primary_calibration', members,
            f'the local half of the shared Component was discarded: {members}',
        )

    # ---------------------------------------------------------------------- R2

    def test_every_item_of_an_aggregating_entity_names_its_member(self):
        """R1, for all three aggregating kinds.

        Without attribution nothing downstream can address an item, which is why
        every attempt to fix this by guessing has failed. The parent Component
        case matters most: its subcomponent is on the other gateway, so its
        members are not even reachable from the local graph.
        """
        for entity_path in (
            f'areas/{MERGED_AREA}',
            f'functions/{MERGED_FUNCTION}',
            f'components/{PARENT_COMPONENT}',
        ):
            with self.subTest(entity=entity_path):
                items = self._items(entity_path, 'operations')
                self.assertTrue(items, f'{entity_path} exposed no operations')
                for item in items:
                    self.assertTrue(
                        item.get('x-medkit', {}).get('member_ids'),
                        f"{entity_path}: {item.get('id')!r} names no member",
                    )

    def test_a_functions_members_span_components_and_gateways(self):
        """A Function is cross-component and cross-area by definition.

        This pins that the merged Function really does reach both sides, so the
        rules below are not being checked against a local-only view.
        """
        members = set()
        for item in self._items(f'functions/{MERGED_FUNCTION}', 'operations'):
            members.update(item.get('x-medkit', {}).get('member_ids') or [])
        self.assertIn('primary_calibration', members)
        self.assertIn('peer_calibration', members)

    # ---------------------------------------------------------------------- R2

    def test_only_an_ambiguous_item_is_qualified(self):
        """R3, both halves.

        Both members expose `calibrate`, so that id is ambiguous and each copy
        must carry its provider. `pressure` comes from one member only, so it
        keeps its bare id - qualifying it would change an id that was never
        ambiguous, on the entity type a default deployment uses most.
        """
        items = self._items(f'functions/{MERGED_FUNCTION}', 'operations')
        ids = [item.get('id') for item in items]

        # Counted, not set-ified. A set of ids collapses two items that share a
        # bare id into one entry, so a fix that emitted one compound id and
        # dropped the other member's would satisfy a membership assertion. Both
        # members expose `calibrate`, so both must survive as separate items.
        self.assertEqual(
            ids.count('primary_calibration:calibrate'), 1, f'ids were {ids}',
        )
        self.assertEqual(
            ids.count('peer_calibration:calibrate'), 1, f'ids were {ids}',
        )
        self.assertEqual(
            ids.count('calibrate'), 0,
            f'a bare id survived alongside the compound ones: {ids}',
        )

    # ---------------------------------------------------------------------- R3

    def test_an_unambiguous_bare_id_still_works(self):
        """R4, the half that protects every client that exists today.

        A single-provider item stays addressable by its bare id. Refusing it
        would break the web UI, the Foxglove panel and the MCP tools, which all
        send the bare name, and would contradict the OpenAPI document this
        gateway generates for itself.
        """
        items = self._items(f'functions/{MERGED_FUNCTION}', 'data')
        single = [
            item for item in items
            if len(item.get('x-medkit', {}).get('member_ids') or []) == 1
        ]
        self.assertTrue(single, 'no single-provider item to check')
        item_id = single[0]['id']
        self.assertNotIn(':', item_id, f'a single-provider item was qualified: {item_id}')
        url = f'{PRIMARY_URL}/functions/{MERGED_FUNCTION}/data/{quote(item_id, safe="")}'
        self.assertEqual(requests.get(url, timeout=15).status_code, 200)

    def test_an_ambiguous_bare_id_is_refused(self):
        """R4: refuse only where the bare form cannot mean one thing.

        Both members expose `calibrate`, so the bare form names two operations.
        Today it returns 200 and runs one of them without saying which.
        """
        response = requests.post(
            f'{PRIMARY_URL}/functions/{MERGED_FUNCTION}/operations/calibrate/executions',
            json={},
            timeout=15,
        )
        self.assertEqual(response.status_code, 400, response.text)
        body = response.json()
        message = (
            body.get('message', '') + ' ' + str(body.get('parameters', {}))
        ).lower()
        self.assertIn('member', message, f'refusal does not name the qualified form: {body}')

    # ---------------------------------------------------------------------- R4

    def test_a_compound_id_reaches_a_peer_owned_member(self):
        """R4, and the reason it asserts the body rather than the status.

        A read that misses samples the local ROS graph, finds nothing, and
        returns 200 with an empty body and status "metadata_only". Asserting
        only the status code passes on that false success, which is exactly how
        an earlier version of this suite failed to catch anything.
        """
        item_id = f'pressure_sensor:{"/chassis/brakes/pressure".lstrip("/")}'
        url = f'{PRIMARY_URL}/functions/{MERGED_FUNCTION}/data/{quote(item_id, safe="")}'
        response = requests.get(url, timeout=15)
        self.assertEqual(response.status_code, 200, response.text)
        body = response.json()
        self.assertEqual(
            body.get('x-medkit', {}).get('status'), 'data',
            f'read reported success with no data from the peer member: {body}',
        )
        self.assertTrue(body.get('data'), 'peer member returned an empty payload')

        # And it is THAT member's item, not merely some 200. The comparison is
        # against the peer's own answer for the same member and topic, so a read
        # that silently fell back to a local member - the other half of this
        # Function publishes a different message type on a different topic -
        # cannot satisfy it, and neither can a hand-built empty envelope.
        direct = requests.get(
            f'{PEER_URL}/apps/pressure_sensor/data/chassis/brakes/pressure', timeout=15)
        self.assertEqual(direct.status_code, 200, direct.text)
        direct_body = direct.json()
        self.assertEqual(
            body.get('x-medkit', {}).get('ros2', {}).get('topic'),
            '/chassis/brakes/pressure',
            f'the answer names a topic the member does not publish: {body}',
        )
        self.assertEqual(
            body.get('x-medkit', {}).get('ros2', {}).get('type'),
            direct_body.get('x-medkit', {}).get('ros2', {}).get('type'),
            f'the answer is not the message the member publishes: {body}',
        )
        self.assertEqual(
            sorted(body['data'].keys()), sorted(direct_body['data'].keys()),
            f"the payload is not shaped like the member's own: {body}",
        )
        # The answer says which entity produced it, and for a peer-owned member
        # that is the member itself - the request was served on the member's own
        # route, over there. Served here it would name the aggregating entity,
        # which is the shape a local sample of a topic this gateway cannot see
        # would also carry.
        self.assertEqual(
            body.get('x-medkit', {}).get('entity_id'), 'pressure_sensor',
            f'the aggregating entity answered for a member it does not run: {body}',
        )

    def test_a_compound_id_reaches_a_local_member(self):
        """R4 in the other direction: resolving members must not lose the local half.

        The payload is asserted, not the status, for the same reason as the peer
        case - and against the member's own App route on THIS gateway, so a
        dispatch that forwarded a local member to a peer that has never heard of
        it would fail here rather than pass as a 404 nobody looked at.
        """
        item_id = f'temp_sensor:{"/powertrain/engine/temperature".lstrip("/")}'
        url = f'{PRIMARY_URL}/functions/{MERGED_FUNCTION}/data/{quote(item_id, safe="")}'
        response = requests.get(url, timeout=15)
        self.assertEqual(response.status_code, 200, response.text)
        body = response.json()
        self.assertEqual(body.get('x-medkit', {}).get('status'), 'data')
        self.assertTrue(body.get('data'), 'local member returned an empty payload')

        direct = requests.get(
            f'{PRIMARY_URL}/apps/temp_sensor/data/powertrain/engine/temperature', timeout=15)
        self.assertEqual(direct.status_code, 200, direct.text)
        direct_body = direct.json()
        self.assertEqual(
            body.get('x-medkit', {}).get('ros2', {}).get('topic'),
            '/powertrain/engine/temperature',
            f'the answer names a topic the local member does not publish: {body}',
        )
        self.assertEqual(
            body.get('x-medkit', {}).get('ros2', {}).get('type'),
            direct_body.get('x-medkit', {}).get('ros2', {}).get('type'),
            f'the answer is not the message the local member publishes: {body}',
        )
        self.assertEqual(
            sorted(body['data'].keys()), sorted(direct_body['data'].keys()),
            f"the payload is not shaped like the local member's own: {body}",
        )
        # And this gateway answered: the entity named is the one the request
        # addressed. A dispatch that forwarded a locally owned member would come
        # back naming the member instead - or, more likely, not come back at all.
        self.assertEqual(
            body.get('x-medkit', {}).get('entity_id'), MERGED_FUNCTION,
            f'a locally owned member was not served here: {body}',
        )

    def test_a_compound_operation_id_runs_on_the_members_gateway(self):
        """R4 for an operation, asserted on the result rather than the status.

        The peer's calibration service is on another ROS domain, so this gateway
        cannot call it: served locally the request answers 500 service-unavailable.
        A 200 alone would still not say the service RAN, so the response payload
        is read - a service that answered is the only thing that can fill it.
        """
        response = requests.post(
            f'{PRIMARY_URL}/functions/{MERGED_FUNCTION}/operations/'
            f'{quote("peer_calibration:calibrate", safe="")}/executions',
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

    def test_a_write_through_an_aggregate_reaches_the_peer_owned_member(self):
        """R4 for a write: the topic is published where the member actually is.

        A write resolves its target exactly as a read does, so it lands on the
        same gateway. Served here it would publish onto a ROS graph the member is
        not on - a publisher nobody is listening to - and answer 200 with the
        same echo, which is why the status cannot decide this case and the
        serving entity is read instead. The 404 is asserted separately because it
        is the other way to get this wrong: consulting the local walk for a
        member another gateway runs finds no such topic and refuses a write that
        is perfectly valid.
        """
        direct = requests.get(
            f'{PEER_URL}/apps/pressure_sensor/data/chassis/brakes/pressure', timeout=15)
        self.assertEqual(direct.status_code, 200, direct.text)
        direct_body = direct.json()
        message_type = direct_body.get('x-medkit', {}).get('ros2', {}).get('type')
        self.assertTrue(message_type, f'the member does not name its message type: {direct.text}')

        response = requests.put(
            f'{PRIMARY_URL}/functions/{MERGED_FUNCTION}/data/'
            f'{quote("pressure_sensor:chassis/brakes/pressure", safe="")}',
            json={'type': message_type, 'data': direct_body['data']},
            timeout=15,
        )
        self.assertNotEqual(
            response.status_code, 404,
            f'a write to a peer-owned member was refused as an unknown item: {response.text}',
        )
        self.assertEqual(response.status_code, 200, response.text)
        body = response.json()
        self.assertEqual(
            body.get('x-medkit', {}).get('ros2', {}).get('topic'),
            '/chassis/brakes/pressure',
            f'the write echo names a topic the member does not publish: {response.text}',
        )
        # Served here the publish would succeed too - a publisher on this
        # gateway's graph, which the member is not on, and a 200 that looks
        # identical. What tells them apart is which entity answered.
        self.assertEqual(
            body.get('x-medkit', {}).get('entity_id'), 'pressure_sensor',
            f'the write was published by the aggregating gateway, not the member: {body}',
        )

    def test_a_configuration_id_the_list_offers_reads_back_from_its_member(self):
        """R5 for configurations, in both directions at once.

        The list of an aggregating entity is assembled from two places - this
        gateway's own nodes and the peer fan-out - so it offers ids for members
        this gateway cannot reach. Reading one back is what says the id is an
        address rather than a label. The value is asserted per member, and the
        two members are seeded to different values first, so an id answered by
        the wrong copy fails here instead of passing as a plausible 200.
        """
        expected = {'primary_calibration': 11.5, 'peer_calibration': 22.5}
        self._set_offset(PRIMARY_URL, 'primary_calibration', expected['primary_calibration'])
        self._set_offset(PEER_URL, 'peer_calibration', expected['peer_calibration'])

        items = self._items(f'functions/{MERGED_FUNCTION}', 'configurations')
        offered = sorted(
            item.get('id') for item in items
            if str(item.get('id', '')).endswith(f':{CALIBRATION_PARAM}')
        )
        self.assertEqual(
            offered,
            [f'peer_calibration:{CALIBRATION_PARAM}', f'primary_calibration:{CALIBRATION_PARAM}'],
            f"the aggregate does not offer both members' copies: "
            f"{[item.get('id') for item in items]}",
        )

        for config_id in offered:
            with self.subTest(configuration=config_id):
                member = config_id.split(':', 1)[0]
                response = requests.get(
                    f'{PRIMARY_URL}/functions/{MERGED_FUNCTION}/configurations/'
                    f'{quote(config_id, safe="")}',
                    timeout=15,
                )
                self.assertEqual(
                    response.status_code, 200,
                    f'the list offers {config_id!r} but reading it answered '
                    f'{response.status_code}: {response.text}',
                )
                self.assertAlmostEqual(
                    response.json().get('data'), expected[member], places=6,
                    msg=f"{config_id} did not answer with its own member's value: "
                        f'{response.text}',
                )

    def test_a_configuration_of_a_local_member_is_still_served_here(self):
        """R5 in the direction that a dispatch fix is most likely to break.

        A member this gateway runs must be answered from this gateway's own
        parameter service, with no hop. Both halves are pinned: the entity that
        answered is the one the request addressed, and the node behind the value
        is the local member's, not its peer namesake's - the two run the same
        executable under the same parameter name and differ only in namespace,
        so nothing shallower can tell them apart.
        """
        local_value = 2.5
        self._set_offset(PRIMARY_URL, 'primary_calibration', local_value)
        self._set_offset(PEER_URL, 'peer_calibration', -3.75)

        config_id = f'primary_calibration:{CALIBRATION_PARAM}'
        response = requests.get(
            f'{PRIMARY_URL}/functions/{MERGED_FUNCTION}/configurations/'
            f'{quote(config_id, safe="")}',
            timeout=15,
        )
        self.assertEqual(response.status_code, 200, response.text)
        body = response.json()
        self.assertAlmostEqual(
            body.get('data'), local_value, places=6,
            msg=f"the local member's value did not come back: {body}",
        )
        self.assertEqual(body.get('id'), config_id, body)
        x_medkit = body.get('x-medkit', {})
        self.assertEqual(
            x_medkit.get('entity_id'), MERGED_FUNCTION,
            f'a locally owned member was not served here: {body}',
        )
        self.assertEqual(
            x_medkit.get('source_app'), 'primary_calibration',
            f'the answer does not name the member it came from: {body}',
        )
        self.assertEqual(
            x_medkit.get('ros2', {}).get('node'), '/powertrain/engine/calibration',
            f'the value was read from a node the local member is not: {body}',
        )

    def test_a_configuration_of_a_peer_owned_member_is_read_from_that_member(self):
        """R5 for configurations, on the half the local walk cannot serve.

        A parameter lives on a node, and the node behind a peer-owned member is
        on a ROS graph this gateway is not on. Served here the lookup finds
        nothing and the request fails; served on the member's own gateway it
        returns that member's value. Both members expose the same parameter
        name, seeded to different values, and the answer is checked against the
        peer's - a fall-back to the local namesake would otherwise be a 200
        with a number in it.
        """
        peer_value = 4.25
        local_value = -1.5
        self._set_offset(PEER_URL, 'peer_calibration', peer_value)
        self._set_offset(PRIMARY_URL, 'primary_calibration', local_value)

        config_id = f'peer_calibration:{CALIBRATION_PARAM}'
        response = requests.get(
            f'{PRIMARY_URL}/functions/{MERGED_FUNCTION}/configurations/'
            f'{quote(config_id, safe="")}',
            timeout=15,
        )
        self.assertEqual(response.status_code, 200, response.text)
        body = response.json()
        self.assertAlmostEqual(
            body.get('data'), peer_value, places=6,
            msg=f"the peer member's value did not come back: {body}",
        )
        self.assertNotAlmostEqual(
            body.get('data'), local_value, places=6,
            msg=f'the local namesake answered for a peer-owned member: {body}',
        )
        x_medkit = body.get('x-medkit', {})
        # The answer says which entity produced it, and for a peer-owned member
        # that is the member itself - the request was served on the member's own
        # route, over there. Served here it would name the aggregating entity.
        self.assertEqual(
            x_medkit.get('entity_id'), 'peer_calibration',
            f'the aggregating entity answered for a member it does not run: {body}',
        )
        self.assertEqual(
            x_medkit.get('ros2', {}).get('node'), '/chassis/brakes/calibration',
            f'the value was read from a node the peer member is not: {body}',
        )

    def test_a_configuration_reset_through_an_aggregate_reaches_the_member(self):
        """R5 for a reset, the method whose answer carries no payload at all.

        DELETE returns 204 and nothing else, so there is no field in the response
        that could say where it landed. Both members are moved off their default
        first and the pair is read back from their own gateways afterwards: the
        member the id named is back at its default and the local namesake is not.
        A reset served here would satisfy neither, and a 204 alone both.
        """
        self._set_offset(PEER_URL, 'peer_calibration', 5.5)
        self._set_offset(PRIMARY_URL, 'primary_calibration', 6.5)

        config_id = f'peer_calibration:{CALIBRATION_PARAM}'
        response = requests.delete(
            f'{PRIMARY_URL}/functions/{MERGED_FUNCTION}/configurations/'
            f'{quote(config_id, safe="")}',
            timeout=15,
        )
        self.assertNotEqual(
            response.status_code, 404,
            f'a reset of a peer-owned member was refused as an unknown member: {response.text}',
        )
        self.assertEqual(response.status_code, 204, response.text)

        self.assertAlmostEqual(
            self._offset_of(PEER_URL, 'peer_calibration'), 0.0, places=6,
            msg='the reset never reached the member it named',
        )
        self.assertAlmostEqual(
            self._offset_of(PRIMARY_URL, 'primary_calibration'), 6.5, places=6,
            msg='the reset landed on the local namesake instead of the member it named',
        )

    def test_a_configuration_write_through_an_aggregate_lands_on_the_member(self):
        """R5 for a write: the parameter changes where the member actually is.

        A write echoes what it was handed, so its response cannot show where it
        landed. The proof is read back from the PEER's own gateway afterwards,
        and the local namesake is read too: a write served here would change
        that one, or nothing at all, and either way the echo would look the same.
        """
        written = 9.75
        self._set_offset(PEER_URL, 'peer_calibration', 0.0)
        self._set_offset(PRIMARY_URL, 'primary_calibration', 0.0)

        config_id = f'peer_calibration:{CALIBRATION_PARAM}'
        response = requests.put(
            f'{PRIMARY_URL}/functions/{MERGED_FUNCTION}/configurations/'
            f'{quote(config_id, safe="")}',
            json={'data': written},
            timeout=15,
        )
        self.assertNotEqual(
            response.status_code, 404,
            f'a write to a peer-owned member was refused as an unknown member: {response.text}',
        )
        self.assertEqual(response.status_code, 200, response.text)
        self.assertEqual(
            response.json().get('x-medkit', {}).get('entity_id'), 'peer_calibration',
            f'the write was applied by the aggregating gateway, not the member: {response.text}',
        )

        self.assertAlmostEqual(
            self._offset_of(PEER_URL, 'peer_calibration'), written, places=6,
            msg='the write never reached the member it named',
        )
        self.assertAlmostEqual(
            self._offset_of(PRIMARY_URL, 'primary_calibration'), 0.0, places=6,
            msg='the write landed on the local namesake instead of the member it named',
        )

    def test_an_unknown_member_in_a_compound_id_is_refused(self):
        """R5 for the member half: absent is said, not sampled for.

        A qualifier naming nothing must be refused before any gateway is asked,
        and the refusal must name the half that was wrong - otherwise a client
        cannot tell a mistyped member from a member whose item is missing. The
        second half of the case pins the distinction the refusal rests on: an id
        that DOES resolve answers 200 and states whether data arrived, so
        "present but empty" and "absent" are never the same response.
        """
        response = requests.get(
            f'{PRIMARY_URL}/functions/{MERGED_FUNCTION}/data/'
            f'{quote("no_such_member:chassis/brakes/pressure", safe="")}',
            timeout=10,
        )
        self.assertEqual(response.status_code, 404, response.text)
        body = response.json()
        self.assertEqual(
            body.get('parameters', {}).get('member_id'), 'no_such_member',
            f'the refusal does not name the member half that was wrong: {body}',
        )

        present = requests.get(
            f'{PRIMARY_URL}/functions/{MERGED_FUNCTION}/data/'
            f'{quote("pressure_sensor:chassis/brakes/pressure", safe="")}',
            timeout=15,
        )
        self.assertEqual(present.status_code, 200, present.text)
        self.assertIn(
            present.json().get('x-medkit', {}).get('status'), ('data', 'metadata_only'),
            f'a resolvable id does not say whether data arrived: {present.text}',
        )

    # ---------------------------------------------------------------------- R5

    def test_an_item_no_member_provides_is_refused(self):
        """R5: absent must be distinguishable from present-but-empty.

        Today an unknown topic answers 200 metadata-only on every entity, so a
        client cannot tell a typo from a silent sensor. On an aggregating entity
        the member set is known, so the answer can be exact.

        The reason is asserted, not just the status: a read that fell through to
        the ROS graph and failed to find the topic there also answers 404, so a
        bare status check cannot tell "the member does not provide this" from
        "nobody happened to be publishing".
        """
        response = requests.get(
            f'{PRIMARY_URL}/functions/{MERGED_FUNCTION}/data/'
            f'{quote("temp_sensor:no/such/topic", safe="")}',
            timeout=10,
        )
        self.assertEqual(response.status_code, 404, response.text)
        body = response.json()
        self.assertEqual(body.get('error_code'), 'resource-not-found', body)
        self.assertEqual(body.get('parameters', {}).get('member_id'), 'temp_sensor', body)
        self.assertEqual(
            body.get('parameters', {}).get('topic_name'), '/no/such/topic', body)

    def test_an_id_that_names_a_member_and_no_item_is_refused(self):
        """R5 at the boundary: naming a member is not naming an item.

        The member's own collection route is one trailing slash away from its
        item route, so an id that stops at the colon addresses the collection if
        it is carried any further - and the caller, having asked for one value,
        is handed a list and no sign that anything went wrong. Checked on a
        PEER-owned member because the local path never reaches that far.
        """
        response = requests.get(
            f'{PRIMARY_URL}/functions/{MERGED_FUNCTION}/data/'
            f'{quote("pressure_sensor:", safe="")}',
            timeout=15,
        )
        self.assertEqual(response.status_code, 404, response.text)
        self.assertNotIn(
            'items', response.json(),
            f'a request for one item was answered with a collection: {response.text}',
        )

    # ---------------------------------------------------------------------- R7

    def test_loop_suppression_is_carried_on_every_hop(self):
        """R7: without it a bidirectionally peered pair bounces a request.

        Asserted by the header's effect rather than by waiting for a hang, which
        would cost the full budget on every run.
        """
        response = requests.get(
            f'{PRIMARY_URL}/functions/{MERGED_FUNCTION}/operations',
            headers={'X-Medkit-No-Fan-Out': '1'},
            timeout=10,
        )
        self.assertEqual(response.status_code, 200)
        members = set()
        for item in response.json().get('items', []):
            members.update(item.get('x-medkit', {}).get('member_ids') or [])
        self.assertNotIn(
            'peer_calibration', members,
            'suppression header ignored, so a peered pair would not terminate',
        )

    def test_loop_suppression_does_not_stop_a_request_reaching_its_member(self):
        """R7 and R4 together: suppression bounds fan-out, it does not blind a read.

        A request addressed to ONE member is not a fan-out - it names its owner,
        and that owner serves it from its own tree without asking anyone else, so
        the chain is one hop whatever the header says. Turning the header into a
        refusal to dispatch would answer the same request two different ways
        depending on a hint about collection listing, and a client that sets it
        to keep listings local would silently start reading empty bodies.

        Termination is measured on the clock rather than inferred: a bounce
        between peered gateways shows up as a request that never comes back, and
        an assertion that only reads the status would report that as an error
        from the transport instead of as the loop it is.
        """
        started = time.monotonic()
        read = requests.get(
            f'{PRIMARY_URL}/functions/{MERGED_FUNCTION}/data/'
            f'{quote("pressure_sensor:chassis/brakes/pressure", safe="")}',
            headers={'X-Medkit-No-Fan-Out': '1'},
            timeout=15,
        )
        run = requests.post(
            f'{PRIMARY_URL}/functions/{MERGED_FUNCTION}/operations/'
            f'{quote("peer_calibration:calibrate", safe="")}/executions',
            headers={'X-Medkit-No-Fan-Out': '1'},
            json={},
            timeout=15,
        )
        elapsed = time.monotonic() - started

        self.assertEqual(read.status_code, 200, read.text)
        self.assertEqual(
            read.json().get('x-medkit', {}).get('status'), 'data',
            f'suppression turned a member read into an empty success: {read.text}',
        )
        self.assertIn(run.status_code, (200, 202), run.text)
        self.assertLess(
            elapsed, 10.0,
            f'two suppressed member requests took {elapsed:.1f}s, which is a bounce, not a hop',
        )

    # ------------------------------------------------------------------ R10
    # A PEER THAT STOPS ANSWERING.
    #
    # Everything above runs against a live pair. These run after it, because
    # unittest orders methods alphabetically within a class and these are the
    # only ones prefixed `test_z`. The peer is killed exactly once, by the
    # first of them, and the rest read the aggregator afterwards.
    #
    # The suite has never taken a peer down before, which is why "the entities
    # simply vanish" survived so long: every case measured a healthy pair, and
    # a tree that changes shape when a link drops looks identical to a tree
    # that never had the entity.

    #: Seconds the aggregator took to notice, filled in by the first case.
    _noticed_after_s = None
    #: Ids the PEER called runtime-discovered, captured before it was killed.
    _peer_runtime_ids = None
    #: Ids the PEER called manifest-declared, captured before it was killed.
    _peer_declared_ids = None

    @staticmethod
    def _peer_ids_by_declared_source():
        """Ask the PEER which of its entities are manifest- and which runtime-declared.

        The aggregator overwrites `source` with `peer:<name>` on arrival, so
        the peer's own answer is the only place the distinction is visible.
        Both collections are read: this topology manifests every app, so the
        runtime half of the split lives among the components.
        """
        by_source = {}

        def record(collection, item):
            source = item.get('x-medkit', {}).get('source', '')
            by_source.setdefault(source, []).append((collection, item.get('id')))

        for collection in ('apps', 'components'):
            response = requests.get(f'{PEER_URL}/{collection}', timeout=10)
            response.raise_for_status()
            for item in response.json().get('items', []):
                record(collection, item)
                if collection != 'components':
                    continue
                # A subcomponent is not in the flat list, and this topology puts
                # the peer's leaf Component exactly there.
                nested = requests.get(
                    f"{PEER_URL}/components/{item.get('id')}/subcomponents", timeout=10)
                if nested.status_code != 200:
                    continue
                for sub in nested.json().get('items', []):
                    record('components', sub)
        return by_source

    @staticmethod
    def _primary_entity(collection, entity_id):
        """One entity as the PRIMARY currently sees it, or None."""
        response = requests.get(f'{PRIMARY_URL}/{collection}', timeout=10)
        if response.status_code != 200:
            return None
        for item in response.json().get('items', []):
            if item.get('id') == entity_id:
                return item
        return None

    @classmethod
    def _primary_app(cls, app_id):
        """One app as the PRIMARY currently sees it, or None."""
        return cls._primary_entity('apps', app_id)

    @staticmethod
    def _primary_subcomponent(parent_id, subcomponent_id):
        """One subcomponent as the PRIMARY currently sees it, or None."""
        response = requests.get(
            f'{PRIMARY_URL}/components/{parent_id}/subcomponents', timeout=10)
        if response.status_code != 200:
            return None
        for item in response.json().get('items', []):
            if item.get('id') == subcomponent_id:
                return item
        return None

    def test_y_every_id_the_list_offers_runs_while_every_member_answers(self):
        """The same agreement as z6, but with nothing broken.

        z6 walks the list after the peer has been killed, so a peer-owned id is
        allowed to answer 504 and the case cannot tell a working dispatch from a
        missing one. Here every member is reachable, so the only honest answer
        is that the operation ran.
        """
        items = self._items(f'functions/{MERGED_FUNCTION}', 'operations')
        offered = [item.get('id') for item in items if item.get('id', '').endswith('calibrate')]
        self.assertTrue(offered, 'the list offered no calibrate operation to check')

        for operation_id in offered:
            with self.subTest(operation=operation_id):
                response = requests.post(
                    f'{PRIMARY_URL}/functions/{MERGED_FUNCTION}/operations/'
                    f'{quote(operation_id, safe="")}/executions',
                    json={},
                    timeout=15,
                )
                self.assertIn(
                    response.status_code, (200, 202),
                    f'every member answers, yet executing the offered id '
                    f'{operation_id!r} returned {response.status_code}: {response.text}',
                )

    def test_z1_a_declared_entity_survives_its_peer_going_silent(self, peer_gateway):
        """R10: what a peer DECLARED does not stop being true when it goes quiet.

        Retention is what makes every later rule stable. Without it the merged
        set is rebuilt from healthy peers only, so an entity - and the
        ambiguity it takes part in - disappears the moment a link drops.
        """
        cls = type(self)
        by_source = self._peer_ids_by_declared_source()
        cls._peer_declared_ids = by_source.get('manifest', [])
        # Anything the peer did not call "manifest" it worked out for itself.
        cls._peer_runtime_ids = [
            entry
            for source, entries in by_source.items() if source != 'manifest'
            for entry in entries
        ]
        self.assertIn(
            ('apps', 'peer_calibration'), cls._peer_declared_ids,
            f'the peer does not consider peer_calibration manifest-declared: {by_source}',
        )

        before = self._primary_app('peer_calibration')
        self.assertIsNotNone(before, 'peer_calibration was not merged while the peer was up')
        self.assertNotEqual(
            before.get('x-medkit', {}).get('available'), False,
            'peer_calibration was already marked unavailable before the peer was killed',
        )

        pid = peer_gateway.process_details['pid']
        os.kill(pid, signal.SIGKILL)

        deadline = time.monotonic() + 60.0
        started = time.monotonic()
        observed = None
        while time.monotonic() < deadline:
            observed = self._primary_app('peer_calibration')
            if observed is not None and observed.get('x-medkit', {}).get('available') is False:
                cls._noticed_after_s = time.monotonic() - started
                break
            time.sleep(0.25)

        self.assertIsNotNone(observed, 'peer_calibration vanished when its peer went silent')
        x_medkit = observed.get('x-medkit', {})
        self.assertIs(
            x_medkit.get('available'), False,
            f'a declared entity did not report itself unavailable within 60s: {observed}',
        )
        self.assertIs(
            x_medkit.get('is_online'), False,
            f'a retained app still claims to be running: {observed}',
        )
        # Detection is bounded by one discovery refresh (refresh_interval_ms,
        # 1000 ms for a test gateway) plus the failed fetch, because the health
        # check runs inside the refresh. Reported so a regression that pushes it
        # towards the 30 s production default is visible rather than merely slow.
        print(f'[retention] aggregator noticed the peer was gone in {cls._noticed_after_s:.1f}s')
        self.assertLess(
            cls._noticed_after_s, 30.0,
            f'took {cls._noticed_after_s:.1f}s to notice a dead peer',
        )

    def test_z2_a_runtime_discovered_peer_entity_is_not_retained(self):
        """R10, the other half: a live graph nobody can observe is not retained.

        Keeping a runtime-discovered entity would assert something this
        gateway can no longer see.
        """
        cls = type(self)
        self.assertTrue(
            cls._peer_runtime_ids,
            'the peer declared everything it exposes, so this case would prove nothing',
        )
        for collection, entity_id in cls._peer_runtime_ids:
            self.assertIsNone(
                self._primary_entity(collection, entity_id),
                f'{collection}/{entity_id} was runtime-discovered on the peer '
                'and must not be retained',
            )

    def test_z2a_availability_is_carried_by_what_a_request_can_reach(self):
        """R10: a Component answers for its reachability; a grouping has none.

        The peer's subcomponent is a real thing behind a link that is down, so
        it reports itself unreachable. The parent Component is declared on both
        sides and this gateway still serves its half, so it stays reachable. An
        Area and a Function are views over members: there is nothing to reach,
        so they carry no availability at all and a client must ask the members.
        """
        cls = type(self)
        self.assertIn(
            ('components', PEER_SUBCOMPONENT), cls._peer_declared_ids,
            f'the peer does not consider {PEER_SUBCOMPONENT} manifest-declared: '
            f'{cls._peer_declared_ids}',
        )

        retained = self._primary_subcomponent(PARENT_COMPONENT, PEER_SUBCOMPONENT)
        self.assertIsNotNone(
            retained, f'{PEER_SUBCOMPONENT} vanished when its peer went silent')
        self.assertIs(
            retained.get('x-medkit', {}).get('available'), False,
            f'a retained Component does not report itself unreachable: {retained}',
        )

        parent = self._primary_entity('components', PARENT_COMPONENT)
        self.assertIsNotNone(
            parent, f'{PARENT_COMPONENT} is declared here too and must remain')
        self.assertNotEqual(
            parent.get('x-medkit', {}).get('available'), False,
            f'a Component this gateway still serves was marked unreachable: {parent}',
        )

        for collection, entity_id in (
            ('areas', MERGED_AREA),
            ('functions', MERGED_FUNCTION),
        ):
            with self.subTest(entity=f'{collection}/{entity_id}'):
                grouping = self._primary_entity(collection, entity_id)
                self.assertIsNotNone(grouping, f'{collection}/{entity_id} vanished')
                # Read the block rather than defaulting it: "no available key"
                # is only evidence if there is a block that could have held one.
                self.assertIn(
                    'x-medkit', grouping,
                    f'{collection}/{entity_id} carries no x-medkit block, so this '
                    f'case would prove nothing: {grouping}',
                )
                self.assertNotIn(
                    'available', grouping['x-medkit'],
                    f'{collection}/{entity_id} groups members and has no '
                    f'availability of its own: {grouping}',
                )

    def test_z3_a_request_to_a_retained_entity_says_it_is_unavailable(self):
        """R10: an unreachable member is answered, not proxied into a 502.

        The body is asserted, not just the status: the failure this branch
        exists to remove is a success shape with nothing in it, and a bare
        status check passes straight over that.
        """
        response = requests.get(f'{PRIMARY_URL}/apps/peer_calibration', timeout=15)
        self.assertEqual(
            response.status_code, 504,
            f'expected 504 for a retained member, got {response.status_code}: {response.text}',
        )
        body = response.json()
        self.assertEqual(body.get('error_code'), 'not-responding', body)
        self.assertIn('peer_calibration', body.get('message', ''), body)
        self.assertEqual(body.get('parameters', {}).get('entity_id'), 'peer_calibration', body)

    def test_z4_a_retained_member_keeps_the_operations_it_reported(self):
        """R10: unreachable, not amnesiac.

        What the member exposed is part of what it declared. Forgetting it
        would make the tree change shape on a link drop all over again, one
        level down.
        """
        items = self._items(f'functions/{MERGED_FUNCTION}', 'operations')
        ids = [item.get('id') for item in items]

        # Counted, not merely present. A list that shrinks to one qualified
        # entry satisfies "the id is still qualified" and "the bare form is
        # still refused" while the client's view has quietly lost a member,
        # which is the failure this whole rule exists to make impossible.
        self.assertEqual(
            ids.count('primary_calibration:calibrate'), 1, f'ids were {ids}',
        )
        self.assertEqual(
            ids.count('peer_calibration:calibrate'), 1,
            f'a retained member forgot the operation it last reported: {ids}',
        )
        self.assertEqual(
            ids.count('calibrate'), 0,
            f'a bare id reappeared once a member became unreachable: {ids}',
        )

        # And the retained copy says it cannot be served, so a client can tell
        # "declared, unreachable" from "ready to run".
        by_id = {item.get('id'): item for item in items}
        self.assertIs(
            by_id['peer_calibration:calibrate'].get('x-medkit', {}).get('available'), False,
            'the retained operation does not report itself unavailable',
        )
        self.assertNotEqual(
            by_id['primary_calibration:calibrate'].get('x-medkit', {}).get('available'), False,
            'the local operation was marked unavailable',
        )

    def test_z5_ambiguity_does_not_move_when_a_peer_goes_silent(self):
        """R10, and the reason retention matters at all.

        Ambiguity is a property of the declared tree. If it tracked
        reachability instead, this same request would be refused while the
        peer answers and would quietly run the local member once it stopped -
        the same request, two different answers, decided by a link.
        """
        response = requests.post(
            f'{PRIMARY_URL}/functions/{MERGED_FUNCTION}/operations/calibrate/executions',
            json={},
            timeout=15,
        )
        self.assertEqual(
            response.status_code, 400,
            f'a bare ambiguous id stopped being refused once the peer went quiet: {response.text}',
        )
        body = response.json()
        message = (body.get('message', '') + ' ' + str(body.get('parameters', {}))).lower()
        self.assertIn('member', message, body)

    def test_z6_every_id_the_list_offers_is_executable(self):
        """The forward half of list/execution agreement.

        z4 and z5 together cover the other half - a refused id is not offered.
        This walks what the list actually offers and drives each one, because
        an agreement checked in one direction only is how the list came to
        advertise a bare id that execution refused.

        `not 400` is the property, not `200`: a member whose gateway is silent
        answers 504, which reports reachability rather than rejecting the id.
        """
        items = self._items(f'functions/{MERGED_FUNCTION}', 'operations')
        offered = [item.get('id') for item in items if item.get('id', '').endswith('calibrate')]
        self.assertTrue(offered, 'the list offered no calibrate operation to check')

        for operation_id in offered:
            with self.subTest(operation=operation_id):
                response = requests.post(
                    f'{PRIMARY_URL}/functions/{MERGED_FUNCTION}/operations/'
                    f'{quote(operation_id, safe="")}/executions',
                    json={},
                    timeout=15,
                )
                self.assertIn(
                    response.status_code, (200, 202, 504),
                    f'the list offers {operation_id!r} but executing it answered '
                    f'{response.status_code}: {response.text}',
                )

    def test_z6a_a_read_of_a_peer_owned_member_says_not_responding(self):
        """R10 for the dispatch path: a dead link is reported, never proxied.

        A forward to a peer that has stopped answering fails at the socket and
        comes back 502, which says this gateway broke. The member is retained
        precisely so the true answer is available without asking: it is declared,
        it is unreachable, and 504 not-responding is the SOVD code for that. The
        502 is asserted against explicitly because it is the regression this
        ordering exists to prevent, and a bare `assertEqual(504)` reads the same
        whichever wrong status arrives.
        """
        response = requests.get(
            f'{PRIMARY_URL}/functions/{MERGED_FUNCTION}/data/'
            f'{quote("pressure_sensor:chassis/brakes/pressure", safe="")}',
            timeout=15,
        )
        self.assertNotEqual(
            response.status_code, 502,
            f'a silent peer was forwarded to instead of answered for: {response.text}',
        )
        self.assertEqual(response.status_code, 504, response.text)
        body = response.json()
        self.assertEqual(body.get('error_code'), 'not-responding', body)
        self.assertIn('pressure_sensor', body.get('message', ''), body)
        self.assertEqual(
            body.get('parameters', {}).get('member_id'), 'pressure_sensor', body)

    def test_z6b_a_configuration_read_of_a_silent_peer_owned_member_says_not_responding(self):
        """R10 for the configuration dispatch path, for the same reason as z6a.

        Reachability is settled before anything is forwarded, so a member whose
        gateway has stopped answering is reported as unreachable rather than as
        a proxy hop that failed at the socket. The 502 is asserted against
        explicitly: it is the shape this ordering exists to prevent, and a bare
        assertEqual(504) reads the same whichever wrong status arrives.
        """
        response = requests.get(
            f'{PRIMARY_URL}/functions/{MERGED_FUNCTION}/configurations/'
            f'{quote(f"peer_calibration:{CALIBRATION_PARAM}", safe="")}',
            timeout=15,
        )
        self.assertNotEqual(
            response.status_code, 502,
            f'a silent peer was forwarded to instead of answered for: {response.text}',
        )
        self.assertEqual(response.status_code, 504, response.text)
        body = response.json()
        self.assertEqual(body.get('error_code'), 'not-responding', body)
        self.assertIn('peer_calibration', body.get('message', ''), body)
        self.assertEqual(
            body.get('parameters', {}).get('member_id'), 'peer_calibration', body)

    def test_z7_suppression_omits_the_peer_without_losing_ambiguity(self):
        """The loop-suppression guard, checked for the reason it was written.

        Suppression means the peers were never asked, so their items are not
        reported - that is what stops a bidirectionally peered pair bouncing a
        request. It does not mean the tree forgot they exist: the id stays
        qualified, so this response never offers a bare id that execution
        refuses either.
        """
        response = requests.get(
            f'{PRIMARY_URL}/functions/{MERGED_FUNCTION}/operations',
            headers={'X-Medkit-No-Fan-Out': '1'},
            timeout=10,
        )
        self.assertEqual(response.status_code, 200)
        items = response.json().get('items', [])
        ids = [item.get('id') for item in items]

        members = set()
        for item in items:
            members.update(item.get('x-medkit', {}).get('member_ids') or [])
        self.assertNotIn(
            'peer_calibration', members,
            f'a suppressed response reported a peer-owned member: {ids}',
        )
        self.assertEqual(
            ids.count('primary_calibration:calibrate'), 1,
            f'suppression dropped the qualification along with the peer: {ids}',
        )
        self.assertEqual(
            ids.count('calibrate'), 0,
            f'a suppressed response offered a bare id that execution refuses: {ids}',
        )
