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
Integration tests for the settings inside the manifest's discovery-config block.

Six gateways run side by side on one ROS graph against the same entity
declarations, differing only in the discovery configuration under test:

=====================  ==============================================  =========================
Gateway                Configuration                                   Expected
=====================  ==============================================  =========================
baseline               defaults (inherit on, override allowed)         runtime resources
                                                                       inherited, manifest wins
no-inherit             ``inherit_runtime_resources: false``            no runtime topics or
                                                                       services on the app
no-override            ``allow_manifest_override: false``              runtime hosts join the
                                                                       hierarchy union;
                                                                       provenance unchanged
override-pinned        ``allow_manifest_override: false`` PLUS an      the explicit parameter
                       explicit ``layers.manifest.hierarchy``          wins: the manifest keeps
                       parameter                                       hosts exclusive
error-policy           ``unmanifested_nodes: error``                   gateway keeps serving and
                                                                       reports the orphans
orphan-policy          ``unmanifested_nodes: include_as_orphan``       entity tree identical to
                                                                       the baseline's (``warn``)
=====================  ==============================================  =========================

Every instrument is a served document: the app's own ``/data`` and
``/operations`` collections, the merged Function's ``x-medkit`` block, and the
``/health`` warnings array. None of them is a log line, and none of them is the
setting that was configured read back to itself.
"""

import os
import subprocess
import tempfile
import time
import unittest

from ament_index_python.packages import get_package_prefix

from launch import LaunchDescription
import launch_testing
import launch_testing.actions
import requests

from ros2_medkit_test_utils.constants import (
    ALLOWED_EXIT_CODES,
    API_BASE_PATH,
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
# Ports - one gateway per configuration under test
# --------------------------------------------------------------------------

BASELINE_PORT = get_test_port(0)
NO_INHERIT_PORT = get_test_port(1)
NO_OVERRIDE_PORT = get_test_port(2)
OVERRIDE_PINNED_PORT = get_test_port(3)
ERROR_POLICY_PORT = get_test_port(4)
ORPHAN_POLICY_PORT = get_test_port(5)


def _base_url(port):
    return f'http://localhost:{port}{API_BASE_PATH}'


BASELINE_URL = _base_url(BASELINE_PORT)
NO_INHERIT_URL = _base_url(NO_INHERIT_PORT)
NO_OVERRIDE_URL = _base_url(NO_OVERRIDE_PORT)
OVERRIDE_PINNED_URL = _base_url(OVERRIDE_PINNED_PORT)
ERROR_POLICY_URL = _base_url(ERROR_POLICY_PORT)
ORPHAN_POLICY_URL = _base_url(ORPHAN_POLICY_PORT)

# --------------------------------------------------------------------------
# Manifest fixture
# --------------------------------------------------------------------------

AREA_ID = 'mcfg-area'
COMPONENT_ID = 'mcfg-ecu'
TEMP_APP_ID = 'mcfg-temp'
CALIB_APP_ID = 'mcfg-calib'
ACTION_APP_ID = 'mcfg-action'
MANIFEST_APPS = {TEMP_APP_ID, CALIB_APP_ID, ACTION_APP_ID}

# The manifest declares a Function under the id the runtime layer derives from
# the /powertrain/engine namespace, so both layers contribute the same entity
# and the merge policies decide what the served document says.
SHARED_FUNCTION_ID = 'powertrain'
MANIFEST_FUNCTION_NAME = 'Powertrain (declared)'
# Runtime app id for the temp_sensor node - present in the runtime layer's
# view of the same Function, absent from the manifest's.
RUNTIME_HOST_ID = 'temp_sensor'

# Nodes present from the start; all are declared by the manifest. All three
# resource kinds enrich_app() copies are represented: a topic, a service and an
# action, because inherit_runtime_resources gates the copy of all three.
EARLY_NODES = ['temp_sensor', 'calibration', 'long_calibration']
TEMP_TOPIC = '/powertrain/engine/temperature'
CALIB_SERVICE = '/powertrain/engine/calibrate'
CALIB_ACTION = '/powertrain/engine/long_calibration'

# Nodes that join after the baseline snapshot, declared nowhere. Started by
# the test (see _start_late_nodes) rather than by the launch description, so
# they cannot appear before the snapshot that must not see them.
# (executable, node name, namespace) - mirrors DEMO_NODE_REGISTRY.
LATE_NODE_SPECS = [
    ('demo_rpm_sensor', 'rpm_sensor', '/powertrain/engine'),
    ('demo_door_status_sensor', 'status_sensor', '/body/door/front_left'),
]
LATE_RPM_FQN = '/powertrain/engine/rpm_sensor'
LATE_STATUS_FQN = '/body/door/front_left/status_sensor'

WARN_UNMANIFESTED_NODES = 'unmanifested_nodes'
WARNING_SCHEMA_VERSION = 2

# Wall-clock BUDGETS are scaled by MEDKIT_TEST_TIME_SCALE; poll INTERVALS and
# soak durations are not. See the note in test_manifest_config_key for why
# stretching a soak under sanitizers is the wrong direction.
TIME_SCALE = get_time_scale()
POLL_TIMEOUT_SEC = 45.0 * TIME_SCALE
POLL_INTERVAL_SEC = 0.5
# How long a single HTTP call has to come back.
HTTP_TIMEOUT_SEC = 5.0 * TIME_SCALE
# How long a helper node has to exit when the test reaps it.
PROCESS_REAP_TIMEOUT_SEC = 10.0 * TIME_SCALE
# How long the gateway must keep answering normally after the orphans appeared.
# Soak duration, NOT a budget: it is time this test spends holding six
# gateways up, and tripling it under sanitizers adds exactly the load that
# makes neighbouring tests miss their own deadlines. Unscaled on purpose.
HOLD_SEC = 3.0


def _manifest_yaml(policy='warn', inherit=True, allow_override=True):
    """Render the shared manifest with the discovery configuration under test."""
    return f"""\
manifest_version: "1.0"
metadata:
  name: "Discovery configuration policy test vehicle"
  version: "1.0.0"
config:
  unmanifested_nodes: "{policy}"
  inherit_runtime_resources: {str(inherit).lower()}
  allow_manifest_override: {str(allow_override).lower()}
areas:
  - id: {AREA_ID}
    name: "Policy Test Area"
components:
  - id: {COMPONENT_ID}
    name: "Policy Test ECU"
    area: {AREA_ID}
apps:
  - id: {TEMP_APP_ID}
    name: "Engine Temperature Sensor"
    is_located_on: {COMPONENT_ID}
    ros_binding:
      node_name: temp_sensor
      namespace: /powertrain/engine
  - id: {CALIB_APP_ID}
    name: "Engine Calibration Service"
    is_located_on: {COMPONENT_ID}
    ros_binding:
      node_name: calibration
      namespace: /powertrain/engine
  - id: {ACTION_APP_ID}
    name: "Engine Long Calibration Action"
    is_located_on: {COMPONENT_ID}
    ros_binding:
      node_name: long_calibration
      namespace: /powertrain/engine
functions:
  - id: {SHARED_FUNCTION_ID}
    name: "{MANIFEST_FUNCTION_NAME}"
    hosted_by:
      - {TEMP_APP_ID}
      - {CALIB_APP_ID}
      - {ACTION_APP_ID}
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
        suffix='.yaml', prefix=f'test_unmanifested_policy_{name}_',
    )
    with os.fdopen(fd, 'w') as handle:
        handle.write(content)
    _MANIFEST_PATHS.append(path)
    return path


# --------------------------------------------------------------------------
# Shared HTTP helpers (cross-gateway, so not GatewayTestCase methods)
# --------------------------------------------------------------------------

def _get_json(base_url, path):
    response = requests.get(f'{base_url}{path}', timeout=HTTP_TIMEOUT_SEC)
    response.raise_for_status()
    return response.json()


def _item_ids(base_url, path):
    return {item['id'] for item in _get_json(base_url, path).get('items', [])}


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


def _unmanifested_warning_in(health):
    """Return the unmanifested-node warning inside an already-read document.

    Separate from _unmanifested_warning so a caller that needs the warning AND
    another field of the same document can read both from one response.
    """
    for warning in health.get('warnings', []):
        if warning.get('code') == WARN_UNMANIFESTED_NODES:
            return warning
    return None


def _unmanifested_warning(base_url):
    """Return the unmanifested-node warning from /health, or None."""
    return _unmanifested_warning_in(_get_json(base_url, '/health'))


def _service_paths(base_url, app_id):
    """Return the ROS service paths the app's /operations collection exposes."""
    items = _get_json(base_url, f'/apps/{app_id}/operations').get('items', [])
    return {op.get('x-medkit', {}).get('ros2', {}).get('service') for op in items}


def _action_paths(base_url, app_id):
    """Return the ROS action paths the app's /operations collection exposes.

    Actions are the third resource kind ``enrich_app`` copies, alongside topics
    and services, so ``inherit_runtime_resources`` has to be checked on them
    too - a test that only looks at topics and services cannot tell whether the
    action leg of the copy is gated at all.
    """
    items = _get_json(base_url, f'/apps/{app_id}/operations').get('items', [])
    return {
        op.get('x-medkit', {}).get('ros2', {}).get('action')
        for op in items
        if op.get('x-medkit', {}).get('ros2', {}).get('kind') == 'action'
    }


# Convergence comparison lives in ros2_medkit_test_utils so there is one
# implementation: the obvious inline version passes when neither side was ever
# fetched. See convergence.py for why.
TREE_MATCH_TIMEOUT_SEC = 15.0 * TIME_SCALE


def _assert_trees_match(test, fetch_left, fetch_right, *, what):
    return assert_documents_match(
        test, fetch_left, fetch_right,
        what=what, timeout=TREE_MATCH_TIMEOUT_SEC,
        interval=POLL_INTERVAL_SEC,
    )


def _function_extension(base_url, function_id=SHARED_FUNCTION_ID):
    """Return the merged Function's x-medkit block."""
    return _get_json(base_url, f'/functions/{function_id}').get('x-medkit', {})


def _wait_for_manifest_apps(base_url):
    """Block until *base_url* serves both manifest apps."""
    return _poll(
        lambda: True if MANIFEST_APPS <= _item_ids(base_url, '/apps') else None,
        what=f'{base_url} to serve the manifest apps',
    )


_INITIAL_ORPHANS = None


def _initial_orphans():
    """Snapshot the error-policy gateway's orphan list before the late nodes.

    Taken by whichever test class runs first, so the "before" state does not
    depend on the order the classes happen to execute in. The gateways share
    one ROS graph, so each of them is an undeclared node in the others' view -
    the warning is therefore already present, and what the snapshot proves is
    that the LATE nodes are not yet in it.
    """
    global _INITIAL_ORPHANS
    if _INITIAL_ORPHANS is None:
        warning = _poll(
            lambda: _unmanifested_warning(ERROR_POLICY_URL),
            what=f'{ERROR_POLICY_URL} to report its first unmanifested nodes',
        )
        _INITIAL_ORPHANS = set(warning['ros_node_fqns'])
        # Only NOW may the late nodes appear. Starting them from the test
        # rather than a launch-time timer removes the race: a fixed delay has
        # to be longer than however long this snapshot takes to converge, and
        # when it is not, a correct gateway fails an assertion about its own
        # test fixture.
        _start_late_nodes()
    return _INITIAL_ORPHANS


_LATE_NODE_PROCESSES = []


def _start_late_nodes():
    """Put the undeclared late nodes on the graph, once."""
    if _LATE_NODE_PROCESSES:
        return
    lib_dir = os.path.join(
        get_package_prefix('ros2_medkit_integration_tests'),
        'lib', 'ros2_medkit_integration_tests',
    )
    for executable, node_name, namespace in LATE_NODE_SPECS:
        _LATE_NODE_PROCESSES.append(subprocess.Popen(  # noqa: S603
            [os.path.join(lib_dir, executable), '--ros-args',
             '-r', f'__ns:={namespace}', '-r', f'__node:={node_name}'],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        ))


def _stop_late_nodes():
    """Reap the helpers so they cannot outlive the test as orphans."""
    while _LATE_NODE_PROCESSES:
        proc = _LATE_NODE_PROCESSES.pop()
        proc.terminate()
        try:
            proc.wait(timeout=PROCESS_REAP_TIMEOUT_SEC)
        except subprocess.TimeoutExpired:
            proc.kill()
            proc.wait(timeout=PROCESS_REAP_TIMEOUT_SEC)


# --------------------------------------------------------------------------
# Launch description
# --------------------------------------------------------------------------

def generate_test_description():
    baseline_manifest = _write_manifest('baseline', _manifest_yaml())
    no_inherit_manifest = _write_manifest(
        'noinherit', _manifest_yaml(inherit=False),
    )
    no_override_manifest = _write_manifest(
        'nooverride', _manifest_yaml(allow_override=False),
    )
    error_manifest = _write_manifest('error', _manifest_yaml(policy='error'))
    orphan_manifest = _write_manifest(
        'orphan', _manifest_yaml(policy='include_as_orphan'),
    )

    def gateway(name, port, manifest_path, extra=None):
        params = {
            'discovery.mode': 'hybrid',
            'discovery.manifest_path': manifest_path,
            'discovery.manifest_strict_validation': False,
            # The runtime layer must contribute the namespace-derived Function
            # so that the manifest's Function of the same id has something to
            # be merged against.
            'discovery.merge_pipeline.gap_fill.allow_heuristic_functions': True,
        }
        if extra:
            params.update(extra)
        return create_gateway_node(port=port, name=name, extra_params=params)

    gateways = [
        gateway('gw_baseline', BASELINE_PORT, baseline_manifest),
        gateway('gw_no_inherit', NO_INHERIT_PORT, no_inherit_manifest),
        gateway('gw_no_override', NO_OVERRIDE_PORT, no_override_manifest),
        gateway(
            'gw_override_pinned', OVERRIDE_PINNED_PORT, no_override_manifest,
            extra={
                # A deployment that explicitly pins a group must beat the
                # manifest's blanket switch. `hierarchy` is the pin that can be
                # observed: it is the only group whose demotion changes the
                # served document against the shipped manifest+runtime pair, so
                # it is the only one that can tell a pinned gateway from an
                # unpinned one.
                'discovery.merge_pipeline.layers.manifest.identity': 'authoritative',
                'discovery.merge_pipeline.layers.manifest.hierarchy': 'authoritative',
            },
        ),
        gateway('gw_error_policy', ERROR_POLICY_PORT, error_manifest),
        gateway('gw_orphan_policy', ORPHAN_POLICY_PORT, orphan_manifest),
    ]

    early_nodes = create_demo_nodes(EARLY_NODES)

    return (
        LaunchDescription(
            gateways + early_nodes + [launch_testing.actions.ReadyToTest()],
        ),
        {'gateway_node': gateways[0]},
    )


# --------------------------------------------------------------------------
# U7 / U8: `unmanifested_nodes: error` reports, it does not stop the gateway
# --------------------------------------------------------------------------

class TestErrorPolicyHealthSurface(GatewayTestCase):
    """`error` produces a /health warning and keeps the gateway serving."""

    BASE_URL = ERROR_POLICY_URL
    REQUIRED_APPS = MANIFEST_APPS

    @classmethod
    def setUpClass(cls):
        super().setUpClass()
        _initial_orphans()

    def test_health_reports_the_configured_policy(self):
        """`discovery.linking` names the policy a monitor has to branch on."""
        linking = _poll(
            lambda: _get_json(self.BASE_URL, '/health').get('discovery', {}).get('linking'),
            what=f'{self.BASE_URL} to publish its linking block',
        )
        self.assertEqual(linking.get('unmanifested_policy'), 'error')

    def test_late_unmanifested_nodes_join_the_warning(self):
        """Nodes that appear after startup show up in a later /health poll."""
        before = _initial_orphans()
        self.assertNotIn(
            LATE_RPM_FQN, before,
            'the baseline must be taken before the late nodes start; '
            '_initial_orphans() starts them itself, so this can only fail if '
            'that ordering was broken',
        )
        self.assertNotIn(LATE_STATUS_FQN, before)

        late = {LATE_RPM_FQN, LATE_STATUS_FQN}
        warning = _poll(
            lambda: (w := _unmanifested_warning(self.BASE_URL)) and (
                w if late <= set(w['ros_node_fqns']) else None
            ),
            what=f'{self.BASE_URL} to list both late nodes as unmanifested',
        )

        # Scale: every orphan is listed, not just the first one. Both numbers
        # come from ONE response - six gateways share this graph and nodes join
        # late, so a count read from a second /health call can legitimately
        # describe a different discovery pass and the comparison would be
        # meaningless.
        health = _poll(
            lambda: (h := _get_json(self.BASE_URL, '/health')) and (
                h if _unmanifested_warning_in(h) else None
            ),
            what=f'{self.BASE_URL} to serve the warning and its counts together',
        )
        same_response_warning = _unmanifested_warning_in(health)
        self.assertEqual(
            len(same_response_warning['ros_node_fqns']),
            health['discovery']['linking']['orphan_count'],
            'ros_node_fqns must carry every orphan the linker counted',
        )
        warning = same_response_warning
        # The subjects are ROS nodes, so the SOVD-id field stays empty. A node
        # FQN is not addressable: the '/' it always contains is rejected in an
        # entity id because it conflicts with URL routing.
        self.assertEqual(
            warning['entity_ids'], [],
            'entity_ids carries addressable SOVD ids only; node names belong '
            f"in ros_node_fqns: {warning['entity_ids']}",
        )
        self.assertEqual(warning['peer_names'], [])
        self.assertTrue(warning['message'])

    def test_gateway_keeps_serving_after_the_orphans_appear(self):
        """R6: the `error` policy is a report, never an outage."""
        late = {LATE_RPM_FQN, LATE_STATUS_FQN}
        _poll(
            lambda: (w := _unmanifested_warning(self.BASE_URL)) and (
                w if late <= set(w['ros_node_fqns']) else None
            ),
            what=f'{self.BASE_URL} to list both late nodes as unmanifested',
        )

        deadline = time.monotonic() + HOLD_SEC
        while True:
            health = _get_json(self.BASE_URL, '/health')
            self.assertEqual(health['status'], 'healthy')
            self.assertEqual(health['discovery']['mode'], 'hybrid')
            self.assertTrue(MANIFEST_APPS <= _item_ids(self.BASE_URL, '/apps'))
            if time.monotonic() >= deadline:
                break
            time.sleep(POLL_INTERVAL_SEC)


class TestErrorPolicyWarningsContract(GatewayTestCase):
    """The warnings array is served whether or not aggregation is enabled."""

    BASE_URL = BASELINE_URL
    REQUIRED_APPS = MANIFEST_APPS

    @classmethod
    def setUpClass(cls):
        super().setUpClass()
        _initial_orphans()

    def test_warnings_and_schema_version_present_without_aggregation(self):
        """U8: no aggregation manager, still an array and a version."""
        root = _get_json(self.BASE_URL, '/')
        self.assertFalse(root['capabilities']['aggregation'])

        health = _get_json(self.BASE_URL, '/health')
        self.assertIn('warnings', health)
        self.assertIsInstance(health['warnings'], list)
        self.assertEqual(health.get('warning_schema_version'), WARNING_SCHEMA_VERSION)

    def test_warn_policy_raises_no_unmanifested_warning(self):
        """The baseline gateway has the same orphans and stays quiet."""
        _poll(
            lambda: _get_json(self.BASE_URL, '/health').get('discovery', {}).get('linking'),
            what=f'{self.BASE_URL} to publish its linking block',
        )
        health = _get_json(self.BASE_URL, '/health')
        self.assertGreater(health['discovery']['linking']['orphan_count'], 0)
        self.assertEqual(health['discovery']['linking']['unmanifested_policy'], 'warn')
        self.assertIsNone(
            _unmanifested_warning(self.BASE_URL),
            f'policy warn must not raise the warning: {health["warnings"]}',
        )


# --------------------------------------------------------------------------
# include_as_orphan: documented as "warn with a quieter log", so the entity
# tree - not the policy read back to itself - has to prove it
# --------------------------------------------------------------------------

class TestIncludeAsOrphanPolicyMatchesWarn(GatewayTestCase):
    """The documented promise is an identical tree; assert exactly that.

    Both gateways run the same manifest and differ only in this one setting,
    and they share one ROS graph, so every collection must agree entity for
    entity. Asserting the configured policy or the orphan count would only
    read the setting back - it would not notice `include_as_orphan` quietly
    hiding, renaming or unbinding anything.
    """

    BASE_URL = ORPHAN_POLICY_URL
    REQUIRED_APPS = MANIFEST_APPS

    @staticmethod
    def _app_documents(base_url):
        """Project /apps into a form two gateways on one graph can be compared on."""
        documents = {}
        for app in _get_json(base_url, '/apps').get('items', []):
            medkit = app.get('x-medkit', {})
            documents[app['id']] = {
                'name': app.get('name'),
                'type': app.get('type'),
                'source': medkit.get('source'),
                'node': medkit.get('ros2', {}).get('node'),
                'is_online': medkit.get('is_online'),
                'component_id': medkit.get('component_id'),
            }
        return documents

    def test_the_policy_is_actually_in_force(self):
        """Guards the fixture: an ignored setting would make the rest vacuous."""
        health = _poll(
            lambda: _get_json(self.BASE_URL, '/health'),
            what=f'{self.BASE_URL} health document',
        )
        linking = health.get('discovery', {}).get('linking', {})
        self.assertEqual(linking.get('unmanifested_policy'), 'include_as_orphan')
        self.assertGreater(
            linking.get('orphan_count', 0), 0,
            'the graph must contain undeclared nodes for this comparison to '
            f'mean anything: {linking}',
        )

    def test_the_app_tree_is_identical_to_the_warn_gateways(self):
        """Every app, with its name, binding, provenance and online state."""
        _assert_trees_match(
            self,
            lambda: self._app_documents(self.BASE_URL),
            lambda: self._app_documents(BASELINE_URL),
            what='the include_as_orphan and warn app trees',
        )

    def test_no_entity_is_tagged_with_an_orphan_provenance(self):
        """`include_as_orphan` does not invent a source value; nothing does."""
        sources = {
            doc['source'] for doc in self._app_documents(self.BASE_URL).values()
        }
        self.assertNotIn(
            'orphan', sources,
            f'no code assigns source="orphan"; got: {sorted(s for s in sources if s)}',
        )

    def test_the_other_collections_match_too(self):
        """Areas, components and functions, not just apps."""
        for path in ('/areas', '/components', '/functions'):
            _assert_trees_match(
                self,
                lambda path=path: _item_ids(self.BASE_URL, path),
                lambda path=path: _item_ids(BASELINE_URL, path),
                what=f'{path} between include_as_orphan and warn',
            )

    def test_the_policy_raises_no_unmanifested_warning(self):
        """Only `error` raises the warning; this policy is a log level."""
        health = _get_json(self.BASE_URL, '/health')
        self.assertIsNone(
            _unmanifested_warning(self.BASE_URL),
            f'include_as_orphan must stay quiet on /health: {health["warnings"]}',
        )


# --------------------------------------------------------------------------
# U6: inherit_runtime_resources gates what a linked app carries
# --------------------------------------------------------------------------

class TestInheritRuntimeResources(GatewayTestCase):
    """The linked app's own resource collections are the instrument."""

    BASE_URL = BASELINE_URL
    REQUIRED_APPS = MANIFEST_APPS

    @classmethod
    def setUpClass(cls):
        super().setUpClass()
        _initial_orphans()

    def test_inherit_true_fills_the_apps_data_and_operations(self):
        """With inheritance on, the runtime topic and service are served."""
        data_ids = _poll(
            lambda: _item_ids(self.BASE_URL, f'/apps/{TEMP_APP_ID}/data') or None,
            what=f'{TEMP_APP_ID} to inherit its runtime topics',
        )
        self.assertIn(TEMP_TOPIC, data_ids)

        op_paths = _poll(
            lambda: _service_paths(self.BASE_URL, CALIB_APP_ID) or None,
            what=f'{CALIB_APP_ID} to inherit its runtime services',
        )
        self.assertIn(CALIB_SERVICE, op_paths)

        action_paths = _poll(
            lambda: _action_paths(self.BASE_URL, ACTION_APP_ID) or None,
            what=f'{ACTION_APP_ID} to inherit its runtime actions',
        )
        self.assertIn(
            CALIB_ACTION, action_paths,
            f'enrich_app copies actions too; got: {sorted(action_paths)}',
        )

    def test_inherit_false_leaves_the_app_without_runtime_resources(self):
        """With inheritance off the same nodes contribute nothing."""
        # Barrier: the gateway has linked the same apps, so absence below is
        # the setting taking effect rather than discovery not having run.
        _wait_for_manifest_apps(NO_INHERIT_URL)
        _poll(
            lambda: True if any(
                app['id'] == TEMP_APP_ID
                and app.get('x-medkit', {}).get('ros2', {}).get('node')
                for app in _get_json(NO_INHERIT_URL, '/apps').get('items', [])
            ) else None,
            what=f'{NO_INHERIT_URL} to bind {TEMP_APP_ID} to its node',
        )
        # And the baseline gateway proves the resources are inheritable at all -
        # all three kinds, so an empty collection below means "gated" and not
        # "there was nothing to copy".
        _poll(
            lambda: _item_ids(BASELINE_URL, f'/apps/{TEMP_APP_ID}/data') or None,
            what='the baseline gateway to inherit the same topics',
        )
        _poll(
            lambda: _service_paths(BASELINE_URL, CALIB_APP_ID) or None,
            what='the baseline gateway to inherit the same services',
        )
        _poll(
            lambda: _action_paths(BASELINE_URL, ACTION_APP_ID) or None,
            what='the baseline gateway to inherit the same actions',
        )

        self.assertEqual(
            _item_ids(NO_INHERIT_URL, f'/apps/{TEMP_APP_ID}/data'), set(),
            'inherit_runtime_resources=false must not copy runtime topics',
        )
        self.assertEqual(
            _get_json(NO_INHERIT_URL, f'/apps/{CALIB_APP_ID}/operations').get('items', []), [],
            'inherit_runtime_resources=false must not copy runtime services',
        )
        self.assertEqual(
            _get_json(NO_INHERIT_URL, f'/apps/{ACTION_APP_ID}/operations').get('items', []), [],
            'inherit_runtime_resources=false must not copy runtime actions',
        )


# --------------------------------------------------------------------------
# U11 / U12: allow_manifest_override and its precedence against parameters
# --------------------------------------------------------------------------

class TestManifestOverridePrecedence(GatewayTestCase):
    """The merged Function document is the instrument, not the setting."""

    BASE_URL = BASELINE_URL
    REQUIRED_APPS = MANIFEST_APPS

    @classmethod
    def setUpClass(cls):
        super().setUpClass()
        _initial_orphans()

    @staticmethod
    def _merged_function(base_url):
        return _poll(
            lambda: _function_extension(base_url) or None,
            what=f'{base_url} to merge the {SHARED_FUNCTION_ID} function',
        )

    def test_override_allowed_lets_the_manifest_win(self):
        """Default: the manifest owns identity, hierarchy and metadata."""
        ext = self._merged_function(BASELINE_URL)
        self.assertEqual(ext.get('source'), 'manifest')
        self.assertEqual(set(ext.get('hosts', [])), MANIFEST_APPS)

    def test_override_denied_widens_the_hierarchy_union_not_provenance(self):
        """U11: the blanket switch drops the manifest's hierarchy exclusivity.

        It does NOT reassign provenance. `source` records which layer declared
        the entity, and the orphan filter decides deletion from it, so no
        field-group policy may rewrite it.
        """
        # Barrier: the baseline gateway has already merged the same Function,
        # so a difference below is the setting, not a slower gateway.
        baseline = self._merged_function(BASELINE_URL)
        ext = self._merged_function(NO_OVERRIDE_URL)

        self.assertEqual(
            ext.get('source'), 'manifest',
            'allow_manifest_override=false must not touch provenance: `source` '
            'is what the orphan filter keys its delete decision on',
        )
        self.assertEqual(
            baseline.get('source'), 'manifest',
            'the baseline gateway reports the same provenance, so the line '
            'above is a property of the entity, not of this one gateway',
        )
        self.assertIn(
            RUNTIME_HOST_ID, ext.get('hosts', []),
            'allow_manifest_override=false must let the runtime hosts through',
        )
        self.assertNotIn(
            RUNTIME_HOST_ID, baseline.get('hosts', []),
            'the baseline keeps the manifest exclusive over hosts, so the '
            'widening asserted above is the setting and not something every '
            'gateway does',
        )

    def test_explicit_layer_policy_beats_the_blanket_switch(self):
        """U12: a pinned group keeps manifest exclusivity, unpinned does not.

        `hierarchy` is the discriminator. Against the shipped manifest+runtime
        pair it is the only group whose demotion changes the served document,
        so the pinned and unpinned gateways are compared on it directly.
        """
        pinned = self._merged_function(OVERRIDE_PINNED_URL)
        unpinned = self._merged_function(NO_OVERRIDE_URL)

        # hierarchy was pinned authoritative in the gateway parameters, so the
        # manifest keeps exclusive ownership of the hosts list ...
        self.assertNotIn(
            RUNTIME_HOST_ID, pinned.get('hosts', []),
            'an explicit layers.manifest.hierarchy must beat '
            'allow_manifest_override=false',
        )
        self.assertEqual(set(pinned.get('hosts', [])), MANIFEST_APPS)
        # ... while the same manifest without that parameter lets the runtime
        # host in. This pair is what makes the test falsifiable: drop the pin
        # and the first assertion fails.
        self.assertIn(
            RUNTIME_HOST_ID, unpinned.get('hosts', []),
            'without the explicit parameter the blanket switch still widens '
            'the hosts union',
        )

        # Regression checks, not evidence of precedence. Provenance is never
        # policy-driven, and demoting identity is a no-op against a runtime
        # layer that is already fallback there - both read the same on the
        # unpinned gateway too.
        self.assertEqual(pinned.get('source'), 'manifest')
        detail = _get_json(OVERRIDE_PINNED_URL, f'/functions/{SHARED_FUNCTION_ID}')
        self.assertEqual(detail.get('name'), MANIFEST_FUNCTION_NAME)


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_helpers_and_manifests_are_cleaned_up(self):
        """The nodes and manifests this module owns do not outlive the run."""
        _stop_late_nodes()
        _remove_manifests()

    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(
            proc_info, allowable_exit_codes=ALLOWED_EXIT_CODES
        )
