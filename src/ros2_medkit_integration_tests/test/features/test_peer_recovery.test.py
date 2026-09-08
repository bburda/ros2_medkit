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

"""Feature test: an aggregated tree recovers when a silent peer answers again.

Retention is what makes an outage survivable. A peer's manifest-declared
entities stay in the merged tree marked unreachable and keep the items they
last reported, while the entities that peer only discovered at runtime drop
out, because they describe a graph this gateway can no longer observe.

Recovery is the other end of that same mechanism and it has no code of its own.
``mark_unreachable`` is one-way - it only ever sets ``available`` false, and
nothing ever sets it back. A peer that answers its health check again is simply
fetched again, and the live answer supersedes the retained declaration because
retained copies are replayed only for peers that could not be read this cycle.

Which is precisely why it needs a case of its own. If a refresh ever preferred
the retained copy over a live one - a merge order swapped, a retained entry
never dropped - one transient outage would poison the tree for the life of the
process: every entity behind that peer frozen as unreachable, every read of one
answering ``504`` forever. Nothing else in the suite restarts a peer, so every
other case measures either a healthy pair or a peer that stays dead, and both
stay green through that regression.

WHAT IS CHECKED, in the order the cases run

  1 The pair merges, and a read of a peer-owned member is served by the peer.
  2 The peer is killed; a DECLARED member reports ``x-medkit.available: false``.
  3 A member the peer only discovered at runtime is gone from the merged tree.
  4 A read addressed to a peer-owned member answers ``504 not-responding``.
  5 The replacement peer answers; the declared member reports itself reachable
    again - the flag is absent, which is how a reachable entity is emitted.
  6 The runtime-discovered member is merged again.
  7 The read that answered 504 answers 200, with the peer's own payload.
  8 Nothing lingers twice: the retained declaration is gone rather than sitting
    beside the live copy, under its own id or a collision-renamed one.

Case 5 is only evidence because case 2 watched the flag go false first, and
case 7 only because case 4 watched that same URL fail. Every case therefore
asserts that its predecessor recorded what it depends on, so a case that broke
earlier cannot leave a later one quietly proving nothing.

The peer is REPLACED rather than resurrected. A gate process waits on a file
the test writes, and its exit starts a second gateway carrying the first one's
configuration on the same port and the same DDS domain. The replacement
appears when the outage has been observed, not on a clock, so the window in
which the peer is provably gone is as long as the test says it is.
"""

import os
import signal
import tempfile
import time
import unittest
from urllib.parse import quote

from launch import LaunchDescription
from launch.actions import (
    ExecuteProcess,
    RegisterEventHandler,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
import launch_testing
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

# The name the aggregator files this peer under. It is also the prefix a
# collision rename would carry, which is what case 8 looks for.
PEER_NAME = 'secondary_gateway'

MERGED_AREA = 'vehicle'
MERGED_FUNCTION = 'vehicle_health'
PARENT_COMPONENT = 'vehicle-ecu'
PEER_SUBCOMPONENT = 'brake-ecu'

# A member on this gateway's own graph. It is the control for every case that
# asserts something about the peer's half: whatever the link does, this one is
# served here and never becomes unreachable.
LOCAL_APP = 'temp_sensor'
LOCAL_TOPIC = '/powertrain/engine/temperature'

# The peer's DECLARED member. Its manifest entry is what outlives the link, so
# it is the entity whose availability flag can flip in both directions, and the
# one whose topic the dispatch cases address.
PEER_DECLARED_APP = 'pressure_sensor'
PEER_TOPIC = '/chassis/brakes/pressure'

# A node the peer's manifest does NOT declare. The peer's policy is `warn`, so
# it exposes the node as a heuristic App; retention drops it when the link goes
# down, and only a completed fetch can bring it back. That makes it the half of
# the tree that cannot be faked by a stale copy.
PEER_RUNTIME_NODE = 'rpm_sensor'
PEER_RUNTIME_APP = 'rpm_sensor'

# The gate the test opens to start the peer's replacement.
GATE_REPLACEMENT = os.path.join(
    tempfile.gettempdir(),
    f'test_peer_recovery_replacement_{os.getpid()}',
)

# Detection and re-inclusion both ride the discovery refresh, 1000 ms for a
# test gateway, so both are seconds rather than tens of seconds. The budgets
# are wide enough that a loaded machine does not fail the case, and the
# measured latency is printed so a regression that pushes either towards the
# 30 s production default is visible rather than merely slow.
OUTAGE_TIMEOUT = 60.0
RECOVERY_TIMEOUT = 90.0
STARTUP_TIMEOUT = 90.0

# PIDs this test killed on purpose, so the post-shutdown exit-code check can
# tell a process the test destroyed from one that died on its own.
_KILLED_PIDS = set()

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
  - id: {LOCAL_APP}
    name: "Engine Temperature Sensor"
    is_located_on: {PARENT_COMPONENT}
    ros_binding:
      node_name: temp_sensor
      namespace: /powertrain/engine
functions:
  - id: {MERGED_FUNCTION}
    name: "Vehicle Health Monitoring"
    category: monitoring
    hosted_by:
      - {LOCAL_APP}
"""

PEER_MANIFEST = f"""\
manifest_version: "1.0"
metadata:
  name: "Secondary ECU"
  version: "1.0.0"
config:
  # The peer exposes what it did not declare, so its half of the tree has both
  # origins in it. Retention keeps the declared entity and drops the other, and
  # a peer that declares everything it runs cannot show the difference.
  unmanifested_nodes: warn
areas:
  - id: {MERGED_AREA}
    name: "Vehicle"
components:
  # The parent is declared on both gateways because a subcomponent may not name
  # a parent absent from its own manifest - the validator rejects that as an
  # error, and an errored manifest is not loaded, so the peer would contribute
  # nothing at all.
  - id: {PARENT_COMPONENT}
    name: "Vehicle ECU"
    area: {MERGED_AREA}
  - id: {PEER_SUBCOMPONENT}
    name: "Brake ECU"
    area: {MERGED_AREA}
    parent_component_id: {PARENT_COMPONENT}
apps:
  - id: {PEER_DECLARED_APP}
    name: "Brake Pressure Sensor"
    is_located_on: {PEER_SUBCOMPONENT}
    ros_binding:
      node_name: pressure_sensor
      namespace: /chassis/brakes
functions:
  - id: {MERGED_FUNCTION}
    name: "Vehicle Health Monitoring"
    category: monitoring
    hosted_by:
      - {PEER_DECLARED_APP}
"""


def _write_manifest(content):
    """Write manifest YAML to a temporary file and return its path."""
    fd, path = tempfile.mkstemp(suffix='.yaml', prefix='test_peer_recovery_manifest_')
    with os.fdopen(fd, 'w') as handle:
        handle.write(content)
    return path


def _gate_process(name, path):
    """Return a process that exits once ``path`` exists."""
    return ExecuteProcess(
        cmd=['sh', '-c', f'while [ ! -e "{path}" ]; do sleep 0.2; done'],
        name=name,
        output='screen',
    )


def _open_gate(path):
    """Write a gate file, releasing the process that waits on it."""
    with open(path, 'w', encoding='utf-8') as gate:
        gate.write('open')


def _remove_gate():
    """Drop the gate file so a rerun does not inherit an open gate."""
    if os.path.exists(GATE_REPLACEMENT):
        try:
            os.unlink(GATE_REPLACEMENT)
        except OSError:
            pass


def _peer_gateway_params(manifest_path):
    """Gateway parameters shared by the peer and by its replacement.

    The replacement is the same gateway again, not a similar one: same port,
    same manifest, same DDS domain. Anything that differed here would leave the
    recovery it demonstrates ambiguous.
    """
    return {
        'discovery.mode': 'hybrid',
        'discovery.manifest_path': manifest_path,
        'discovery.manifest_strict_validation': False,
    }


def generate_test_description():
    """Launch an aggregator, a peer, and the peer's gated replacement."""
    primary_manifest_path = _write_manifest(PRIMARY_MANIFEST)
    peer_manifest_path = _write_manifest(PEER_MANIFEST)

    peer_domain_env = {'ROS_DOMAIN_ID': str(PEER_DOMAIN_ID)}
    peer_params = _peer_gateway_params(peer_manifest_path)

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
            'aggregation.peer_names': [PEER_NAME],
        },
    )

    peer_gateway = create_gateway_node(
        name='secondary_gateway_node',
        port=PEER_PORT,
        extra_params=peer_params,
        extra_env=peer_domain_env,
    )

    # Distinct ROS node name only so the two processes are separable in the
    # launch output and in proc_info. The first is dead before this one starts.
    replacement_gateway = create_gateway_node(
        name='secondary_gateway_node_replacement',
        port=PEER_PORT,
        extra_params=peer_params,
        extra_env=peer_domain_env,
    )

    replacement_gate = _gate_process('peer_replacement_gate', GATE_REPLACEMENT)

    delayed = TimerAction(
        period=2.0,
        actions=(
            create_demo_nodes([LOCAL_APP], lidar_faulty=False)
            + create_demo_nodes(
                [PEER_DECLARED_APP, PEER_RUNTIME_NODE],
                lidar_faulty=False,
                extra_env=peer_domain_env,
            )
        ),
    )

    launch_description = LaunchDescription([
        SetEnvironmentVariable('ROS_DOMAIN_ID', str(PRIMARY_DOMAIN_ID)),
        primary_gateway,
        peer_gateway,
        replacement_gate,
        RegisterEventHandler(
            OnProcessExit(target_action=replacement_gate, on_exit=[replacement_gateway]),
        ),
        delayed,
        launch_testing.actions.ReadyToTest(),
    ])

    return (
        launch_description,
        {
            'gateway_node': primary_gateway,
            'peer_gateway': peer_gateway,
            'replacement_gateway': replacement_gateway,
        },
    )


def _wait_for_health(base_url, *, timeout):
    """Poll ``/health`` until it answers 200, or fail."""
    deadline = time.monotonic() + timeout
    last = None
    while time.monotonic() < deadline:
        try:
            response = requests.get(f'{base_url}/health', timeout=2)
            if response.status_code == 200:
                return
            last = response.status_code
        except requests.exceptions.RequestException as exc:
            last = repr(exc)
        time.sleep(0.25)
    raise AssertionError(f'{base_url} was not healthy within {timeout}s (last: {last})')


def _poll(predicate, *, timeout, interval=0.25):
    """Call ``predicate`` until it answers something; return that, or None.

    ``None`` from the predicate means "not yet"; every other value is the
    answer. Falsiness deliberately does not mean "not yet": a
    ``requests.Response`` is falsy for any status at or above 400, so a
    predicate handing back the 504 a case is waiting for would otherwise be
    polled straight past until the budget ran out.

    Polling rather than sleeping is what keeps the budgets above from becoming
    the thing under test: a case finishes as soon as the aggregator has caught
    up, and only a gateway that never catches up spends the whole budget.
    """
    deadline = time.monotonic() + timeout
    while True:
        value = predicate()
        if value is not None:
            return value
        if time.monotonic() >= deadline:
            return None
        time.sleep(interval)


class PeerRecoveryTest(unittest.TestCase):
    """Drives the aggregating gateway; the peer is only ever used to verify."""

    #: True once a case has watched the declared member go unavailable.
    _outage_observed = False
    #: Seconds the aggregator took to notice the peer was gone.
    _noticed_after_s = None
    #: Seconds the aggregator took to re-include the replacement.
    _recovered_after_s = None
    #: True once a case has watched the peer-owned read answer 504.
    _read_refused_while_down = False
    #: The peer's own answer for the addressed topic, read while it was healthy.
    _peer_payload_keys = None

    @classmethod
    def setUpClass(cls):
        """Wait until both gateways answer and the peer's half has merged.

        The merge is driven by HTTP from the peer while the local ROS graph is
        still binding Apps to nodes, so a case that starts on the first
        successful response can be reading a tree that is still filling in.
        """
        cls.addClassCleanup(_remove_gate)
        _remove_gate()

        _wait_for_health(PRIMARY_URL, timeout=STARTUP_TIMEOUT)
        _wait_for_health(PEER_URL, timeout=STARTUP_TIMEOUT)

        wanted = {LOCAL_APP, PEER_DECLARED_APP, PEER_RUNTIME_APP}
        merged = _poll(
            lambda: True if cls._app_ids_seen_by_primary() >= wanted else None,
            timeout=STARTUP_TIMEOUT,
        )
        if not merged:
            raise AssertionError(
                f'the peer half never merged; the aggregator lists '
                f'{sorted(cls._app_ids_seen_by_primary())}'
            )

        served = _poll(
            lambda: True if cls._aggregate_read_of_peer_topic().status_code == 200 else None,
            timeout=STARTUP_TIMEOUT,
        )
        if not served:
            raise AssertionError(
                'the aggregator never served the peer-owned topic while the peer '
                'was healthy, so the outage cases would prove nothing'
            )

    # ------------------------------------------------------------------
    # Reading the merged tree
    # ------------------------------------------------------------------

    @staticmethod
    def _primary_items(collection):
        """Every item of a top-level collection as the aggregator sees it."""
        response = requests.get(f'{PRIMARY_URL}/{collection}', timeout=10)
        if response.status_code != 200:
            return []
        return response.json().get('items', [])

    @classmethod
    def _app_ids_seen_by_primary(cls):
        """Return the set of App ids in the merged tree."""
        return {item.get('id') for item in cls._primary_items('apps')}

    @classmethod
    def _primary_apps_named(cls, app_id):
        """Every copy of one App id in the merged tree, so copies can be counted."""
        return [item for item in cls._primary_items('apps') if item.get('id') == app_id]

    @classmethod
    def _primary_app(cls, app_id):
        """One App as the aggregator currently sees it, or None."""
        found = cls._primary_apps_named(app_id)
        return found[0] if found else None

    @staticmethod
    def _primary_subcomponents(parent_id, subcomponent_id):
        """Every copy of one subcomponent id under a parent Component."""
        response = requests.get(
            f'{PRIMARY_URL}/components/{parent_id}/subcomponents', timeout=10)
        if response.status_code != 200:
            return []
        return [
            item for item in response.json().get('items', [])
            if item.get('id') == subcomponent_id
        ]

    @staticmethod
    def _function_items(collection):
        """Items of a resource collection on the merged Function."""
        response = requests.get(
            f'{PRIMARY_URL}/functions/{MERGED_FUNCTION}/{collection}', timeout=15)
        if response.status_code != 200:
            return []
        return response.json().get('items', [])

    @staticmethod
    def _aggregate_read_of_peer_topic():
        """Read the peer-owned topic through the merged Function.

        The compound form names the member, so the answer has to come from the
        gateway that runs it. This one URL is used by every dispatch case here:
        it is what answers 504 while the peer is down and what has to answer
        with the peer's own sample once the peer is back.
        """
        item_id = f'{PEER_DECLARED_APP}:{PEER_TOPIC.lstrip("/")}'
        return requests.get(
            f'{PRIMARY_URL}/functions/{MERGED_FUNCTION}/data/{quote(item_id, safe="")}',
            timeout=15,
        )

    @staticmethod
    def _peer_direct_read():
        """Read the same topic on the peer's own App route, on the peer itself."""
        return requests.get(
            f'{PEER_URL}/apps/{PEER_DECLARED_APP}/data{PEER_TOPIC}', timeout=15)

    @staticmethod
    def _peer_app_view(app_id):
        """Return one App as the PEER itself lists it, or None."""
        response = requests.get(f'{PEER_URL}/apps', timeout=10)
        if response.status_code != 200:
            return None
        for item in response.json().get('items', []):
            if item.get('id') == app_id:
                return item
        return None

    @staticmethod
    def _peer_source_of(app_id):
        """Ask the PEER what origin it gives one of its Apps, or None.

        The aggregator overwrites ``source`` with ``peer:<name>`` on arrival, so
        the peer's own answer is the only place the manifest/runtime split is
        visible.
        """
        response = requests.get(f'{PEER_URL}/apps', timeout=10)
        if response.status_code != 200:
            return None
        for item in response.json().get('items', []):
            if item.get('id') == app_id:
                return item.get('x-medkit', {}).get('source')
        return None

    # ------------------------------------------------------------------
    # Cases
    # ------------------------------------------------------------------

    def test_01_both_gateways_answer_and_the_peer_half_is_merged(self):
        """The baseline every later case is measured against.

        Two things are established here rather than assumed. The declared
        member is reachable, so case 5 can claim the flag moved instead of
        merely reporting where it ended up. And the peer really does call the
        other member runtime-discovered, so case 3's disappearance is the rule
        working rather than an entity that was never there.
        """
        cls = type(self)

        declared = self._primary_app(PEER_DECLARED_APP)
        self.assertIsNotNone(
            declared, f'{PEER_DECLARED_APP} was not merged while the peer answered')
        self.assertNotEqual(
            declared.get('x-medkit', {}).get('available'), False,
            f'{PEER_DECLARED_APP} was already unavailable before the peer was killed: '
            f'{declared}',
        )
        self.assertEqual(
            self._peer_source_of(PEER_DECLARED_APP), 'manifest',
            f'the peer does not call {PEER_DECLARED_APP} manifest-declared, so nothing '
            f'here is retained and the recovery this file measures is not the one '
            f'described',
        )

        runtime_source = self._peer_source_of(PEER_RUNTIME_APP)
        self.assertIsNotNone(
            runtime_source,
            f'the peer does not expose {PEER_RUNTIME_APP} at all, so the '
            f'runtime-discovered half of this test is missing',
        )
        self.assertNotEqual(
            runtime_source, 'manifest',
            f'the peer calls {PEER_RUNTIME_APP} manifest-declared, so it would be '
            f'retained through the outage and case 3 would prove nothing',
        )
        self.assertIsNotNone(
            self._primary_app(PEER_RUNTIME_APP),
            f'{PEER_RUNTIME_APP} was not merged while the peer answered',
        )

        # And the peer answers for its own topic, so case 7's success has
        # something to be compared against. A merged App is not a publishing
        # one yet: the sample carries no data until the peer's node is up and
        # has published, so this waits for the payload rather than reading once
        # and calling an empty answer a failure.
        def _read_carrying_data():
            reply = self._aggregate_read_of_peer_topic()
            if reply.status_code != 200:
                return None
            payload = reply.json()
            return payload if payload.get('data') else None

        body = _poll(_read_carrying_data, timeout=60.0)
        self.assertIsNotNone(
            body, 'the peer-owned topic never carried data while the peer answered')
        self.assertEqual(
            body.get('x-medkit', {}).get('entity_id'), PEER_DECLARED_APP,
            f'the aggregating entity answered for a member it does not run: {body}',
        )
        cls._peer_payload_keys = sorted(body['data'].keys())

    def test_02_the_declared_half_reports_itself_unreachable_when_the_peer_dies(
            self, peer_gateway):
        """The outage, and the flag whose reverse trip case 5 checks for.

        Watching the flag go false here is what makes case 5 evidence: without
        it, "the entity is reachable" describes a tree that never noticed
        anything happened.
        """
        cls = type(self)
        self.assertIsNotNone(
            cls._peer_payload_keys,
            'test_01 must establish the healthy baseline before test_02 runs',
        )

        pid = peer_gateway.process_details['pid']
        _KILLED_PIDS.add(pid)
        os.kill(pid, signal.SIGKILL)

        def unavailable():
            app = self._primary_app(PEER_DECLARED_APP)
            if app is not None and app.get('x-medkit', {}).get('available') is False:
                return app
            return None

        started = time.monotonic()
        observed = _poll(unavailable, timeout=OUTAGE_TIMEOUT)
        self.assertIsNotNone(
            observed,
            f'{PEER_DECLARED_APP} never reported itself unavailable within '
            f'{OUTAGE_TIMEOUT}s of its peer being killed; the aggregator now sees '
            f'{self._primary_app(PEER_DECLARED_APP)}',
        )
        cls._noticed_after_s = time.monotonic() - started
        cls._outage_observed = True

        self.assertIs(
            observed.get('x-medkit', {}).get('is_online'), False,
            f'a retained App still claims to be running: {observed}',
        )
        print(f'[recovery] aggregator noticed the peer was gone in '
              f'{cls._noticed_after_s:.1f}s')
        # Detection is bounded by one discovery refresh - 1000 ms for a test
        # gateway - plus the failed health check that runs inside it. Bounded
        # here so a regression that pushes it towards the 30 s production
        # default is a failure rather than a slower green run.
        self.assertLess(
            cls._noticed_after_s, 30.0,
            f'took {cls._noticed_after_s:.1f}s to notice a dead peer',
        )

        local = self._primary_app(LOCAL_APP)
        self.assertIsNotNone(local, f'{LOCAL_APP} is local and must not vanish')
        self.assertNotEqual(
            local.get('x-medkit', {}).get('available'), False,
            f'a locally owned App was marked unreachable by a peer outage: {local}',
        )

    def test_03_a_runtime_discovered_member_is_gone_while_the_peer_is_down(self):
        """The half that is NOT retained, so its return is not a stale replay.

        A retained copy could satisfy case 6 on its own. This case rules that
        out by requiring the entity to be absent first: whatever brings it back
        has to be a completed fetch from a gateway that is answering again.
        """
        self.assertTrue(
            self._outage_observed,
            'test_02 must observe the outage before test_03 runs',
        )
        vanished = _poll(
            lambda: True if self._primary_app(PEER_RUNTIME_APP) is None else None,
            timeout=OUTAGE_TIMEOUT,
        )
        self.assertTrue(
            vanished,
            f'{PEER_RUNTIME_APP} was runtime-discovered on the peer and must not '
            f'outlive the link that reported it: '
            f'{self._primary_app(PEER_RUNTIME_APP)}',
        )

    def test_04_a_peer_owned_read_says_not_responding_while_the_peer_is_down(self):
        """The request that has to start working again, failing first.

        Case 7 drives this same URL. Recording the refusal here is what lets
        that case claim a request recovered, rather than reporting that a
        request works - which it also did before anything was killed.
        """
        cls = type(self)
        self.assertTrue(
            self._outage_observed,
            'test_02 must observe the outage before test_04 runs',
        )

        def refused():
            answer = self._aggregate_read_of_peer_topic()
            return answer if answer.status_code != 200 else None

        response = _poll(refused, timeout=OUTAGE_TIMEOUT)
        if response is None:
            last = self._aggregate_read_of_peer_topic()
            self.fail(
                f'a topic on a dead gateway was still being served as a successful '
                f'read for {OUTAGE_TIMEOUT}s: {last.text}'
            )
        self.assertNotEqual(
            response.status_code, 502,
            f'a silent peer was forwarded to instead of answered for: {response.text}',
        )
        self.assertEqual(response.status_code, 504, response.text)
        body = response.json()
        self.assertEqual(body.get('error_code'), 'not-responding', body)
        self.assertEqual(
            body.get('parameters', {}).get('member_id'), PEER_DECLARED_APP, body)
        cls._read_refused_while_down = True

    def test_05_a_declared_member_is_reachable_again_once_the_peer_returns(self):
        """The promise: a peer that answers again is re-included automatically.

        Nothing resets the availability flag, so the only thing that can clear
        it is a completed fetch replacing the retained declaration wholesale. A
        merge that preferred the retained copy would leave this entity
        unreachable forever after one outage, with every other case in the
        suite still green.

        A reachable entity is emitted with no ``available`` field at all, so the
        x-medkit block is read rather than defaulted: "no key" is only evidence
        when there is a block that could have carried one.
        """
        cls = type(self)
        self.assertTrue(
            cls._outage_observed,
            'test_02 must watch the flag go false before test_05 can claim it moved',
        )

        _open_gate(GATE_REPLACEMENT)
        _wait_for_health(PEER_URL, timeout=RECOVERY_TIMEOUT)

        def reachable():
            app = self._primary_app(PEER_DECLARED_APP)
            if app is not None and app.get('x-medkit', {}).get('available') is not False:
                return app
            return None

        started = time.monotonic()
        recovered = _poll(reachable, timeout=RECOVERY_TIMEOUT)
        self.assertIsNotNone(
            recovered,
            f'{PEER_DECLARED_APP} was still unreachable {RECOVERY_TIMEOUT}s after its '
            f'peer started answering again: {self._primary_app(PEER_DECLARED_APP)}',
        )
        cls._recovered_after_s = time.monotonic() - started
        print(f'[recovery] aggregator re-included the peer in '
              f'{cls._recovered_after_s:.1f}s')

        x_medkit = recovered.get('x-medkit', {})
        self.assertTrue(
            x_medkit,
            f'{PEER_DECLARED_APP} carries no x-medkit block, so an absent '
            f'`available` proves nothing: {recovered}',
        )
        self.assertNotIn(
            'available', x_medkit,
            f'a reachable entity still emits an availability flag: {recovered}',
        )
        # Re-inclusion rides the same refresh as detection did. Reported so a
        # regression that pushes either towards the 30 s production default
        # shows up as a number rather than as a slow test.
        self.assertLess(
            cls._recovered_after_s, 30.0,
            f'took {cls._recovered_after_s:.1f}s to re-include a peer that answers',
        )

        # The second availability signal, waited for separately because it does
        # not come back at the same moment. Reachability is settled by the
        # peer's health check, but `is_online` is the peer's own account of
        # whether that App is bound to a running node - and a gateway that has
        # just started answering has not finished linking its ROS graph yet, so
        # it truthfully reports the App as offline for a refresh or two. What
        # the rule requires is that the aggregator ends up carrying what the
        # peer says, rather than the false that retention wrote over it.
        def online():
            app = self._primary_app(PEER_DECLARED_APP)
            if app is not None and app.get('x-medkit', {}).get('is_online') is True:
                return app
            return None

        back_online = _poll(online, timeout=RECOVERY_TIMEOUT)
        self.assertIsNotNone(
            back_online,
            f'{PEER_DECLARED_APP} never reported itself running again, while the peer '
            f'itself says {self._peer_app_view(PEER_DECLARED_APP)}',
        )
        self.assertIs(
            self._peer_app_view(PEER_DECLARED_APP).get('x-medkit', {}).get('is_online'),
            True,
            'the peer does not consider its own App online, so the aggregator '
            'agreeing with it proves nothing',
        )

    def test_06_a_runtime_discovered_member_is_merged_again(self):
        """The half a retained copy cannot account for.

        This entity was dropped when the link went down, so its presence now is
        a fetch that completed against a live peer - which is the difference
        between a tree that recovered and a tree that is replaying what it
        remembers.
        """
        self.assertIsNotNone(
            self._recovered_after_s,
            'test_05 must see the peer re-included before test_06 runs',
        )
        merged = _poll(
            lambda: self._primary_app(PEER_RUNTIME_APP),
            timeout=RECOVERY_TIMEOUT,
        )
        self.assertIsNotNone(
            merged,
            f'{PEER_RUNTIME_APP} was discovered on the peer, dropped with the link, '
            f'and never came back; the aggregator lists '
            f'{sorted(self._app_ids_seen_by_primary())}',
        )
        self.assertNotEqual(
            merged.get('x-medkit', {}).get('available'), False,
            f'a freshly fetched entity arrived marked unreachable: {merged}',
        )

    def test_07_a_peer_owned_read_succeeds_again_and_the_peer_answered_it(self):
        """The request that answered 504 answers with the member's own sample.

        Status alone cannot show this. The failure that matters is a 200 with
        an empty body - a local sample of a topic this gateway cannot see - so
        the answer is compared against the peer's own read of the same topic on
        the peer's own route, and it has to name the member as the entity that
        produced it.
        """
        self.assertTrue(
            self._read_refused_while_down,
            'test_04 must watch this URL fail before test_07 can claim it recovered',
        )

        # Poll for the condition this test actually asserts, not merely for a
        # 200. Recovery has two steps that finish at different times: the route
        # comes back, and then a sample arrives on the re-created subscription.
        # Between them the read answers 200 with status "metadata_only" and an
        # empty body, so a poll that stops at the status code hands the
        # assertions below a response taken from that window - and the wider
        # the machine's load, the wider the window.
        def served():
            answer = self._aggregate_read_of_peer_topic()
            if answer.status_code != 200:
                return None
            if answer.json().get('x-medkit', {}).get('status') != 'data':
                return None
            return answer

        response = _poll(served, timeout=RECOVERY_TIMEOUT)
        self.assertIsNotNone(
            response,
            f'a read of {PEER_DECLARED_APP} never recovered after its peer came back; '
            f'last answer was {self._aggregate_read_of_peer_topic().text}',
        )

        body = response.json()
        self.assertEqual(
            body.get('x-medkit', {}).get('status'), 'data',
            f'the read reported success with no data from the peer member: {body}',
        )
        self.assertTrue(body.get('data'), f'the peer member returned an empty payload: {body}')
        self.assertEqual(
            body.get('x-medkit', {}).get('entity_id'), PEER_DECLARED_APP,
            f'the aggregating entity answered for a member it does not run: {body}',
        )
        self.assertEqual(
            body.get('x-medkit', {}).get('ros2', {}).get('topic'), PEER_TOPIC,
            f'the answer names a topic the member does not publish: {body}',
        )

        direct = self._peer_direct_read()
        self.assertEqual(direct.status_code, 200, direct.text)
        direct_body = direct.json()
        self.assertEqual(
            body.get('x-medkit', {}).get('ros2', {}).get('type'),
            direct_body.get('x-medkit', {}).get('ros2', {}).get('type'),
            f'the answer is not the message the member publishes: {body}',
        )
        self.assertEqual(
            sorted(body['data'].keys()), sorted(direct_body['data'].keys()),
            f"the payload is not shaped like the member's own: {body}",
        )
        self.assertEqual(
            sorted(body['data'].keys()), self._peer_payload_keys,
            f'the recovered payload is not shaped like the one the same read '
            f'returned before the outage: {body}',
        )

    def test_08_the_retained_declaration_does_not_linger_beside_the_live_copy(self):
        """Recovery is a replacement, not an addition.

        A retained entry that is merged rather than superseded shows up either
        as the same id twice or - since the merge renames a leaf two
        contributors claim - as a peer-prefixed second copy. Both are counted,
        because "still listed" and "listed once" are different claims and the
        tree is only correct when both hold.
        """
        self.assertIsNotNone(
            self._recovered_after_s,
            'test_05 must see the peer re-included before test_08 runs',
        )

        copies = self._primary_apps_named(PEER_DECLARED_APP)
        self.assertEqual(
            len(copies), 1,
            f'{PEER_DECLARED_APP} is listed {len(copies)} times after its peer came '
            f'back: {copies}',
        )

        renamed = f'{PEER_NAME}__{PEER_DECLARED_APP}'
        self.assertEqual(
            self._primary_apps_named(renamed), [],
            f'the retained declaration was merged alongside the live copy and renamed '
            f'to {renamed}',
        )

        subcomponents = self._primary_subcomponents(PARENT_COMPONENT, PEER_SUBCOMPONENT)
        self.assertEqual(
            len(subcomponents), 1,
            f'{PEER_SUBCOMPONENT} is listed {len(subcomponents)} times under '
            f'{PARENT_COMPONENT} after its peer came back: {subcomponents}',
        )
        self.assertNotEqual(
            subcomponents[0].get('x-medkit', {}).get('available'), False,
            f'a re-included Component still reports itself unreachable: {subcomponents[0]}',
        )

        # And one level down: the items the retained copy carried through the
        # outage are not sitting beside the ones the live peer just reported.
        peer_topics = [
            item for item in self._function_items('data')
            if item.get('x-medkit', {}).get('ros2', {}).get('topic') == PEER_TOPIC
        ]
        self.assertEqual(
            len(peer_topics), 1,
            f'{PEER_TOPIC} is offered {len(peer_topics)} times after its peer came '
            f'back: {[item.get("id") for item in peer_topics]}',
        )
        self.assertNotEqual(
            peer_topics[0].get('x-medkit', {}).get('available'), False,
            f'a re-included topic still reports itself unavailable: {peer_topics[0]}',
        )
        local_topics = [
            item for item in self._function_items('data')
            if item.get('x-medkit', {}).get('ros2', {}).get('topic') == LOCAL_TOPIC
        ]
        self.assertEqual(
            len(local_topics), 1,
            f'the local half of the Function changed shape across the outage: '
            f'{local_topics}',
        )


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        """Check all processes exited cleanly.

        The peer this test kills is allowed to report SIGKILL, and only that
        one: it is matched by the pid the test killed, so a different process
        dying that way is still a failure.
        """
        for info in proc_info:
            allowed = set(ALLOWED_EXIT_CODES)
            if info.pid in _KILLED_PIDS:
                allowed.add(-9)
            self.assertIn(
                info.returncode, allowed,
                f'{info.process_name} exited with code {info.returncode}',
            )
        _remove_gate()
