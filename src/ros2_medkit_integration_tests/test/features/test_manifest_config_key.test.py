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

"""
Integration tests for the manifest's top-level discovery-configuration key.

Four gateways run side by side on one ROS graph, all in hybrid mode against
the same entity declarations, differing only in how the discovery
configuration block is spelled and how strict validation is set:

===================  ==========================  ======  ===========================
Gateway              Block                       Strict  Expected
===================  ==========================  ======  ===========================
config-key           ``config: ignore``          no      policy applies
alias-key            ``discovery: ignore``       no      identical entity tree
bad-policy-lenient   ``config: <unknown>``       no      loads, falls back to warn
bad-policy-strict    ``config: <unknown>``       yes     rejected -> runtime_only
===================  ==========================  ======  ===========================

The instrument is the entity tree from ``GET /apps``, not the gateway log: a
log line proves a message was emitted, not that the policy took effect.

Change, not steady state: the unmanifested node (``rpm_sensor``) and one
manifest-declared node (``pressure_sensor``) are both started well AFTER the
gateways are up, so the suppression has to hold for a node that appears
mid-run.
"""

import os
import tempfile
import time
import unittest

from launch import LaunchDescription
from launch.actions import TimerAction
import launch_testing
import launch_testing.actions
import requests

from ros2_medkit_test_utils.constants import (
    ALLOWED_EXIT_CODES,
    API_BASE_PATH,
    DISCOVERY_TIMEOUT,
    get_test_port,
    get_time_scale,
)
from ros2_medkit_test_utils.convergence import assert_documents_match
from ros2_medkit_test_utils.gateway_test_case import GatewayTestCase
from ros2_medkit_test_utils.launch_helpers import (
    create_demo_nodes,
    create_gateway_node,
)

# --------------------------------------------------------------------------
# Ports - one gateway per spelling / strictness combination
# --------------------------------------------------------------------------

CONFIG_KEY_PORT = get_test_port(0)
ALIAS_KEY_PORT = get_test_port(1)
BAD_POLICY_LENIENT_PORT = get_test_port(2)
BAD_POLICY_STRICT_PORT = get_test_port(3)


def _base_url(port):
    return f'http://localhost:{port}{API_BASE_PATH}'


CONFIG_KEY_URL = _base_url(CONFIG_KEY_PORT)
ALIAS_KEY_URL = _base_url(ALIAS_KEY_PORT)
BAD_POLICY_LENIENT_URL = _base_url(BAD_POLICY_LENIENT_PORT)
BAD_POLICY_STRICT_URL = _base_url(BAD_POLICY_STRICT_PORT)

# --------------------------------------------------------------------------
# Manifest fixture
# --------------------------------------------------------------------------

AREA_ID = 'cfgkey-area'
COMPONENT_ID = 'cfgkey-ecu'
TEMP_APP_ID = 'cfgkey-temp'
PRESSURE_APP_ID = 'cfgkey-pressure'
MANIFEST_APPS = {TEMP_APP_ID, PRESSURE_APP_ID}

# Node started up-front, bound by the manifest.
EARLY_NODE = 'temp_sensor'
# Nodes started long after the gateways are up. pressure_sensor is bound by the
# manifest and acts as the "this gateway has re-run discovery" witness;
# rpm_sensor is deliberately absent from the manifest and is the orphan under
# test.
LATE_MANIFESTED_NODE = 'pressure_sensor'
LATE_UNMANIFESTED_NODE = 'rpm_sensor'
LATE_UNMANIFESTED_FQN = '/powertrain/engine/rpm_sensor'
LATE_MANIFESTED_FQN = '/chassis/brakes/pressure_sensor'

# Long enough that every gateway has answered /health and completed at least
# one discovery pass before the late nodes join the graph.
# Wall-clock BUDGETS are scaled by MEDKIT_TEST_TIME_SCALE. The sanitizer jobs
# multiply every ctest timeout, but a deadline asserted inside a test is
# invisible to that rewrite, so an ASan/TSan gateway blows it and the failure
# reads as a regression rather than as instrumentation cost. Unscaled
# elsewhere, so the budgets keep their falsifying edge on normal jobs.
#
# Poll INTERVALS and soak durations are deliberately NOT scaled: polling more
# slowly would only notice the same answer later, and stretching a soak would
# hold the gateways longer under exactly the instrumentation that made them
# expensive - which is load this file would be adding, not absorbing.
TIME_SCALE = get_time_scale()

# The window the "before the late nodes" snapshot has to complete in. A budget:
# too short under instrumentation and the baseline is taken after the nodes
# already joined, which fails an assertion about the fixture, not the product.
LATE_NODE_DELAY_SEC = 10.0 * TIME_SCALE

# A value that is not one of ignore / warn / error / include_as_orphan.
UNKNOWN_POLICY = 'quiesce'

# How long the suppression must keep holding once it has been observed, in
# seconds. The gateways refresh every 1000 ms, so this spans several passes.
# Soak duration, not a budget - see the note above; left unscaled on purpose.
HOLD_SEC = 4.0
# Was a private 60.0; this is the shared discovery budget, scaled.
POLL_TIMEOUT_SEC = DISCOVERY_TIMEOUT * TIME_SCALE
POLL_INTERVAL_SEC = 0.5
# How long a single HTTP call has to come back.
HTTP_TIMEOUT_SEC = 5.0 * TIME_SCALE


def _manifest_yaml(config_key, policy):
    """Render the shared manifest under *config_key* with *policy*."""
    return f"""\
manifest_version: "1.0"
metadata:
  name: "Manifest config key test vehicle"
  version: "1.0.0"
{config_key}:
  unmanifested_nodes: "{policy}"
areas:
  - id: {AREA_ID}
    name: "Config Key Test Area"
components:
  - id: {COMPONENT_ID}
    name: "Config Key Test ECU"
    area: {AREA_ID}
apps:
  - id: {TEMP_APP_ID}
    name: "Engine Temperature Sensor"
    is_located_on: {COMPONENT_ID}
    ros_binding:
      node_name: {EARLY_NODE}
      namespace: /powertrain/engine
  - id: {PRESSURE_APP_ID}
    name: "Brake Pressure Sensor"
    is_located_on: {COMPONENT_ID}
    ros_binding:
      node_name: {LATE_MANIFESTED_NODE}
      namespace: /chassis/brakes
"""


# Manifests this module wrote, so the post-shutdown phase can remove them
# instead of leaving a file per run behind in the temp directory.
_MANIFEST_PATHS = []


def _remove_manifests():
    """Delete every manifest this module wrote."""
    while _MANIFEST_PATHS:
        try:
            os.unlink(_MANIFEST_PATHS.pop())
        except OSError:
            pass


def _write_manifest(name, content):
    """Write manifest YAML to a temporary file, return its path."""
    fd, path = tempfile.mkstemp(
        suffix='.yaml', prefix=f'test_manifest_config_key_{name}_',
    )
    with os.fdopen(fd, 'w') as handle:
        handle.write(content)
    _MANIFEST_PATHS.append(path)
    return path


# --------------------------------------------------------------------------
# Shared HTTP helpers (cross-gateway, so not GatewayTestCase methods)
# --------------------------------------------------------------------------

def _collection_items(base_url, path):
    response = requests.get(f'{base_url}{path}', timeout=HTTP_TIMEOUT_SEC)
    response.raise_for_status()
    return response.json().get('items', [])


def _app_ids(base_url):
    return {app['id'] for app in _collection_items(base_url, '/apps')}


def _comparable_entity(entity):
    """Return the part of a served entity two gateways must agree on exactly.

    Nothing is dropped. Both gateways run the same manifest against the same
    ROS graph and differ only in how the discovery-configuration block is
    spelled, so every served field - name, description, hierarchy, tags,
    bindings, provenance, online state, hyperlinks - has to be identical. The
    only fields that could carry a per-gateway value are the port-bearing
    ones, and the API serves relative hrefs, so there are none to exclude.
    """
    return entity


def _comparable_collection(base_url, path):
    """Return a collection keyed by id, so a diff names the entity involved."""
    return {
        item['id']: _comparable_entity(item)
        for item in _collection_items(base_url, path)
    }


# Convergence comparison lives in ros2_medkit_test_utils so there is one
# implementation: the obvious inline version passes when neither side was ever
# fetched. See convergence.py for why.
DOCUMENT_MATCH_TIMEOUT_SEC = 15.0 * TIME_SCALE


def _assert_documents_match(test, fetch_left, fetch_right, *, what):
    return assert_documents_match(
        test, fetch_left, fetch_right,
        what=what, timeout=DOCUMENT_MATCH_TIMEOUT_SEC,
        interval=POLL_INTERVAL_SEC,
    )


def _bound_nodes(base_url):
    """Return the set of ROS node FQNs any app on *base_url* is bound to."""
    nodes = set()
    for app in _collection_items(base_url, '/apps'):
        node = app.get('x-medkit', {}).get('ros2', {}).get('node')
        if node:
            nodes.add(node)
    return nodes


def _poll(predicate, *, what, timeout=POLL_TIMEOUT_SEC):
    """Poll *predicate* until it returns a truthy value, else fail loudly."""
    deadline = time.monotonic() + timeout
    last = None
    while time.monotonic() < deadline:
        try:
            last = predicate()
            if last:
                return last
        except requests.exceptions.RequestException as exc:
            last = exc
        time.sleep(POLL_INTERVAL_SEC)
    raise AssertionError(f'timed out after {timeout}s waiting for {what}; last={last!r}')


def _wait_for_orphan_witness():
    """Block until the warn-policy gateway shows the unmanifested node.

    That gateway shares the ROS graph with every other gateway in this file,
    so once it has surfaced ``rpm_sensor`` the node is demonstrably live and
    discoverable - which is what makes the *absence* of the same node on the
    ``ignore`` gateways evidence of the policy rather than of a slow start.
    """
    return _poll(
        lambda: LATE_UNMANIFESTED_FQN in _bound_nodes(BAD_POLICY_LENIENT_URL),
        what=f'{BAD_POLICY_LENIENT_URL} to surface {LATE_UNMANIFESTED_FQN}',
    )


def _wait_for_late_link(base_url):
    """Block until *base_url* has linked the late manifest-declared node.

    Proves this gateway has run a discovery pass that saw the late nodes, so a
    subsequent absence assertion is not just reading a pre-launch snapshot.
    """
    return _poll(
        lambda: LATE_MANIFESTED_FQN in _bound_nodes(base_url),
        what=f'{base_url} to link {LATE_MANIFESTED_FQN}',
    )


# --------------------------------------------------------------------------
# Launch description
# --------------------------------------------------------------------------

def generate_test_description():
    config_key_manifest = _write_manifest(
        'config', _manifest_yaml('config', 'ignore'),
    )
    alias_key_manifest = _write_manifest(
        'alias', _manifest_yaml('discovery', 'ignore'),
    )
    bad_policy_manifest = _write_manifest(
        'badpolicy', _manifest_yaml('config', UNKNOWN_POLICY),
    )

    def gateway(name, port, manifest_path, strict):
        return create_gateway_node(
            port=port,
            name=name,
            extra_params={
                'discovery.mode': 'hybrid',
                'discovery.manifest_path': manifest_path,
                'discovery.manifest_strict_validation': strict,
            },
        )

    gateways = [
        gateway('gw_config_key', CONFIG_KEY_PORT, config_key_manifest, False),
        gateway('gw_alias_key', ALIAS_KEY_PORT, alias_key_manifest, False),
        gateway(
            'gw_bad_policy_lenient', BAD_POLICY_LENIENT_PORT,
            bad_policy_manifest, False,
        ),
        gateway(
            'gw_bad_policy_strict', BAD_POLICY_STRICT_PORT,
            bad_policy_manifest, True,
        ),
    ]

    early_nodes = create_demo_nodes([EARLY_NODE])
    late_nodes = TimerAction(
        period=LATE_NODE_DELAY_SEC,
        actions=create_demo_nodes(
            [LATE_MANIFESTED_NODE, LATE_UNMANIFESTED_NODE],
        ),
    )

    return (
        LaunchDescription(
            gateways + early_nodes + [late_nodes, launch_testing.actions.ReadyToTest()],
        ),
        {'gateway_node': gateways[0]},
    )


# --------------------------------------------------------------------------
# U1: `config: unmanifested_nodes: ignore` actually hides unmanifested nodes
# --------------------------------------------------------------------------

# @verifies REQ_INTEROP_003
class TestConfigKeyAppliesUnmanifestedPolicy(GatewayTestCase):
    """The documented `config:` block drives the unmanifested-node policy."""

    BASE_URL = CONFIG_KEY_URL
    REQUIRED_APPS = MANIFEST_APPS

    def test_health_reports_hybrid_mode(self):
        """The manifest loaded, so the gateway stayed in hybrid mode."""
        # @verifies REQ_INTEROP_003
        health = self.get_json('/health')
        self.assertEqual(health.get('discovery', {}).get('mode'), 'hybrid')

    def test_late_unmanifested_node_never_enters_the_entity_tree(self):
        """A node that joins after startup and is not declared stays hidden."""
        # @verifies REQ_INTEROP_003
        _wait_for_orphan_witness()
        _wait_for_late_link(self.BASE_URL)

        deadline = time.monotonic() + HOLD_SEC
        while True:
            app_ids = _app_ids(self.BASE_URL)
            bound = _bound_nodes(self.BASE_URL)
            self.assertEqual(
                app_ids, MANIFEST_APPS,
                f'unmanifested_nodes=ignore must leave only the manifest apps; '
                f'extra: {app_ids - MANIFEST_APPS}, missing: {MANIFEST_APPS - app_ids}',
            )
            self.assertNotIn(
                LATE_UNMANIFESTED_FQN, bound,
                f'{LATE_UNMANIFESTED_FQN} is not declared in the manifest and '
                f'must not be bound to any app; bound nodes: {sorted(bound)}',
            )
            if time.monotonic() >= deadline:
                break
            time.sleep(POLL_INTERVAL_SEC)

    def test_manifest_declared_nodes_are_linked(self):
        """Both manifest apps bind to their runtime nodes, early and late."""
        # @verifies REQ_INTEROP_003
        _wait_for_late_link(self.BASE_URL)
        bound = _bound_nodes(self.BASE_URL)
        self.assertIn('/powertrain/engine/temp_sensor', bound)
        self.assertIn(LATE_MANIFESTED_FQN, bound)


# --------------------------------------------------------------------------
# U2: the deprecated `discovery:` alias produces the same entity tree
# --------------------------------------------------------------------------

# @verifies REQ_INTEROP_003
class TestDeprecatedDiscoveryKeyIsAnAlias(GatewayTestCase):
    """`discovery:` keeps working and behaves exactly like `config:`."""

    BASE_URL = ALIAS_KEY_URL
    REQUIRED_APPS = MANIFEST_APPS

    def test_health_reports_hybrid_mode(self):
        """The deprecated key is a deprecation, not a load failure."""
        # @verifies REQ_INTEROP_003
        health = self.get_json('/health')
        self.assertEqual(health.get('discovery', {}).get('mode'), 'hybrid')

    def test_entity_tree_matches_the_canonical_key(self):
        """Every collection matches the `config:` gateway, document for document.

        Comparing id sets alone would not notice the alias applying a
        different configuration value, or producing different names,
        hierarchy, bindings, provenance or online state - all of which are
        things a second parse path can get wrong. So the whole served item is
        compared, minus only the fields that cannot be equal by construction
        (see ``_comparable_entity``).
        """
        # @verifies REQ_INTEROP_003
        _wait_for_orphan_witness()
        _wait_for_late_link(self.BASE_URL)
        _wait_for_late_link(CONFIG_KEY_URL)

        for path in ('/areas', '/components', '/apps', '/functions'):
            _assert_documents_match(
                self,
                lambda path=path: _comparable_collection(self.BASE_URL, path),
                lambda path=path: _comparable_collection(CONFIG_KEY_URL, path),
                what=f'{path} between the alias and canonical gateways',
            )

    def test_entity_detail_documents_match_the_canonical_key(self):
        """The collection view can agree while the detail view does not."""
        # @verifies REQ_INTEROP_003
        _wait_for_late_link(self.BASE_URL)
        _wait_for_late_link(CONFIG_KEY_URL)

        def detail(base_url, app_id):
            response = requests.get(f'{base_url}/apps/{app_id}', timeout=HTTP_TIMEOUT_SEC)
            response.raise_for_status()
            return _comparable_entity(response.json())

        for app_id in sorted(MANIFEST_APPS):
            _assert_documents_match(
                self,
                lambda app_id=app_id: detail(self.BASE_URL, app_id),
                lambda app_id=app_id: detail(CONFIG_KEY_URL, app_id),
                what=f'/apps/{app_id} between the two gateways',
            )

    def test_late_unmanifested_node_never_enters_the_entity_tree(self):
        """The alias applies the policy, not just the entity declarations."""
        # @verifies REQ_INTEROP_003
        _wait_for_orphan_witness()
        _wait_for_late_link(self.BASE_URL)
        self.assertEqual(_app_ids(self.BASE_URL), MANIFEST_APPS)
        self.assertNotIn(LATE_UNMANIFESTED_FQN, _bound_nodes(self.BASE_URL))


# --------------------------------------------------------------------------
# U5: an unrecognised policy value
# --------------------------------------------------------------------------

# @verifies REQ_INTEROP_003
class TestUnknownPolicyValueNonStrict(GatewayTestCase):
    """Non-strict: the manifest loads and the default policy applies."""

    BASE_URL = BAD_POLICY_LENIENT_URL
    REQUIRED_APPS = MANIFEST_APPS

    def test_manifest_still_loads(self):
        """A bad value is a warning, so hybrid mode survives it."""
        # @verifies REQ_INTEROP_003
        health = self.get_json('/health')
        self.assertEqual(health.get('discovery', {}).get('mode'), 'hybrid')

    def test_policy_falls_back_to_warn_so_the_orphan_is_visible(self):
        """Fallback is `warn`, which keeps unmanifested nodes in the tree."""
        # @verifies REQ_INTEROP_003
        _wait_for_orphan_witness()
        bound = _bound_nodes(self.BASE_URL)
        self.assertIn(
            LATE_UNMANIFESTED_FQN, bound,
            'the fallback policy is warn, which must not hide orphans; '
            f'bound nodes: {sorted(bound)}',
        )


# @verifies REQ_INTEROP_003
class TestUnknownPolicyValueStrict(GatewayTestCase):
    """Strict validation must NOT reject a manifest over an advisory notice.

    ``R014`` is advisory for this release: it reaches the log, never
    ``ValidationResult``, so the strictness dial cannot act on it. The reason
    is upgrade safety - the discovery-config block was parsed but never read
    before, so a manifest carrying a bad policy value loaded fine, and making
    it a hard failure on upgrade would take hybrid discovery down to
    ``runtime_only`` for a deployment nobody edited. It becomes a validation
    warning in the next release, at which point this class flips back.
    """

    BASE_URL = BAD_POLICY_STRICT_URL
    REQUIRED_APPS = MANIFEST_APPS

    def test_manifest_still_loads_and_discovery_stays_hybrid(self):
        """The advisory notice does not fail the load, even under strict."""
        # @verifies REQ_INTEROP_003
        health = _poll(
            lambda: requests.get(f'{self.BASE_URL}/health', timeout=HTTP_TIMEOUT_SEC).json(),
            what=f'{self.BASE_URL} health document',
        )
        self.assertEqual(
            health.get('discovery', {}).get('mode'), 'hybrid',
            'an advisory R014 must not degrade discovery under strict validation',
        )

    def test_manifest_entities_are_served(self):
        """The manifest loaded, so its entities are in the tree."""
        # @verifies REQ_INTEROP_003
        _wait_for_orphan_witness()
        self.assertEqual(_app_ids(self.BASE_URL) & MANIFEST_APPS, MANIFEST_APPS)
        area_ids = {a['id'] for a in _collection_items(self.BASE_URL, '/areas')}
        self.assertIn(AREA_ID, area_ids)

    def test_the_documented_fallback_still_applies(self):
        """Advisory does not mean ignored: the value still falls back to warn."""
        # @verifies REQ_INTEROP_003
        health = _poll(
            lambda: requests.get(f'{self.BASE_URL}/health', timeout=HTTP_TIMEOUT_SEC).json(),
            what=f'{self.BASE_URL} linking block',
        )
        linking = health.get('discovery', {}).get('linking', {})
        self.assertEqual(
            linking.get('unmanifested_policy'), 'warn',
            f'the unrecognised value must fall back to the default: {linking}',
        )


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_manifests_are_cleaned_up(self):
        """The temp manifests this module wrote do not outlive the run."""
        _remove_manifests()

    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(
            proc_info, allowable_exit_codes=ALLOWED_EXIT_CODES
        )
