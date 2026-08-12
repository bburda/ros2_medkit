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
Integration tests: a malformed discovery-configuration block costs the block only.

The block is addressed by key and then indexed by field, so a key whose value
is not a mapping cannot be read that way. Getting this wrong is expensive in
both directions: indexing a scalar throws, which abandons the entire manifest
over one bad key, and indexing a sequence yields nothing, which drops the
operator's configuration in silence.

The contract is that a malformed block is *reported* under rule ``R015`` and
costs only itself - the rest of the manifest still parses, the offending block
is ignored and its defaults apply.

``R015`` is ADVISORY for this release: it goes to the log only, so the
strictness dial does not act on it and the outcome is the same either way.
That is deliberate upgrade safety - this work is what makes the block read at
all, so a manifest carrying a malformed one loaded fine before and must not
start failing after. It becomes a validation warning in the next release.
Both strictness settings are pinned below:

====================  ==========================================  =====================
Gateway               Manifest                                    Expected
====================  ==========================================  =====================
lenient               ``config: ""`` (scalar), strict off         loads, serves its
                                                                  entities, defaults
                                                                  apply
alias-lenient         ``discovery:`` a sequence, strict off       identical - the
                                                                  deprecated key is
                                                                  guarded too
strict                ``config: ""`` (scalar), strict on          identical: R015 is
                                                                  advisory this
                                                                  release
====================  ==========================================  =====================

The instrument is the served entity tree and ``/health.discovery.mode``, not a
log line: a warning being logged is not evidence that the manifest survived.
"""

import os
import tempfile
import time
import unittest

from launch import LaunchDescription
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
from ros2_medkit_test_utils.gateway_test_case import GatewayTestCase
from ros2_medkit_test_utils.launch_helpers import (
    create_demo_nodes,
    create_gateway_node,
)

# --------------------------------------------------------------------------
# Ports
# --------------------------------------------------------------------------

LENIENT_PORT = get_test_port(0)
ALIAS_LENIENT_PORT = get_test_port(1)
STRICT_PORT = get_test_port(2)


def _base_url(port):
    return f'http://localhost:{port}{API_BASE_PATH}'


LENIENT_URL = _base_url(LENIENT_PORT)
ALIAS_LENIENT_URL = _base_url(ALIAS_LENIENT_PORT)
STRICT_URL = _base_url(STRICT_PORT)

# --------------------------------------------------------------------------
# Manifest fixture
# --------------------------------------------------------------------------

AREA_ID = 'mmc-area'
COMPONENT_ID = 'mmc-ecu'
APP_ID = 'mmc-temp'
DEMO_NODE = 'temp_sensor'
DEMO_FQN = '/powertrain/engine/temp_sensor'

# The node that is deliberately NOT declared. Under the default policy (warn)
# it stays visible, which is how the test shows the defaults took effect after
# the malformed block was discarded.
UNDECLARED_NODE = 'rpm_sensor'
UNDECLARED_FQN = '/powertrain/engine/rpm_sensor'

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

# Was a private 60.0; this is the shared discovery budget, scaled.
POLL_TIMEOUT_SEC = DISCOVERY_TIMEOUT * TIME_SCALE
POLL_INTERVAL_SEC = 0.5
# How long a single HTTP call has to come back.
HTTP_TIMEOUT_SEC = 5.0 * TIME_SCALE

# A scalar value: yaml-cpp throws when this is indexed by field name.
SCALAR_BLOCK = ' ""'
# A sequence value: indexing yields nothing, so it used to vanish silently.
SEQUENCE_BLOCK = '\n  - unmanifested_nodes: ignore'


def _manifest_yaml(key, block):
    """Render the shared manifest with a malformed *key* block."""
    return f"""\
manifest_version: "1.0"
metadata:
  name: "Malformed discovery config test vehicle"
  version: "1.0.0"
{key}:{block}
areas:
  - id: {AREA_ID}
    name: "Malformed Config Area"
components:
  - id: {COMPONENT_ID}
    name: "Malformed Config ECU"
    area: {AREA_ID}
apps:
  - id: {APP_ID}
    name: "Engine Temperature Sensor"
    is_located_on: {COMPONENT_ID}
    ros_binding:
      node_name: {DEMO_NODE}
      namespace: /powertrain/engine
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
        suffix='.yaml', prefix=f'test_manifest_malformed_config_{name}_',
    )
    with os.fdopen(fd, 'w') as handle:
        handle.write(content)
    _MANIFEST_PATHS.append(path)
    return path


# --------------------------------------------------------------------------
# Shared HTTP helpers
# --------------------------------------------------------------------------

def _get_json(base_url, path):
    response = requests.get(f'{base_url}{path}', timeout=HTTP_TIMEOUT_SEC)
    response.raise_for_status()
    return response.json()


def _item_ids(base_url, path):
    return {item['id'] for item in _get_json(base_url, path).get('items', [])}


def _bound_nodes(base_url):
    nodes = set()
    for app in _get_json(base_url, '/apps').get('items', []):
        node = app.get('x-medkit', {}).get('ros2', {}).get('node')
        if node:
            nodes.add(node)
    return nodes


def _poll(predicate, *, what, timeout=POLL_TIMEOUT_SEC):
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
    raise AssertionError(
        f'timed out after {timeout}s waiting for {what}; last={last!r}'
    )


# --------------------------------------------------------------------------
# Launch description
# --------------------------------------------------------------------------

def generate_test_description():
    scalar_manifest = _write_manifest(
        'scalar', _manifest_yaml('config', SCALAR_BLOCK),
    )
    alias_sequence_manifest = _write_manifest(
        'aliasseq', _manifest_yaml('discovery', SEQUENCE_BLOCK),
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
        gateway('gw_mmc_lenient', LENIENT_PORT, scalar_manifest, False),
        gateway(
            'gw_mmc_alias_lenient', ALIAS_LENIENT_PORT,
            alias_sequence_manifest, False,
        ),
        gateway('gw_mmc_strict', STRICT_PORT, scalar_manifest, True),
    ]

    demo_nodes = create_demo_nodes([DEMO_NODE, UNDECLARED_NODE])

    return (
        LaunchDescription(
            gateways + demo_nodes + [launch_testing.actions.ReadyToTest()],
        ),
        {'gateway_node': gateways[0]},
    )


# --------------------------------------------------------------------------
# Shared assertions for the two non-strict gateways
# --------------------------------------------------------------------------

class MalformedBlockCostsOnlyTheBlockMixin:
    """Non-strict: the block is discarded, the rest of the manifest is not."""

    REQUIRED_APPS = {APP_ID}

    def test_manifest_still_loads_and_discovery_stays_hybrid(self):
        """A malformed block must not abandon the manifest."""
        health = _poll(
            lambda: _get_json(self.BASE_URL, '/health'),
            what=f'{self.BASE_URL} health document',
        )
        self.assertEqual(health.get('discovery', {}).get('mode'), 'hybrid')

    def test_every_declared_entity_is_served(self):
        """Areas, components and apps all survive the malformed block."""
        self.assertIn(AREA_ID, _item_ids(self.BASE_URL, '/areas'))
        self.assertIn(COMPONENT_ID, _item_ids(self.BASE_URL, '/components'))
        self.assertIn(APP_ID, _item_ids(self.BASE_URL, '/apps'))

        detail = _get_json(self.BASE_URL, f'/apps/{APP_ID}')
        self.assertEqual(detail.get('id'), APP_ID)
        self.assertEqual(detail.get('x-medkit', {}).get('source'), 'manifest')

    def test_the_app_still_links_to_its_node(self):
        """The declared binding is intact, so the block was the only loss."""
        _poll(
            lambda: DEMO_FQN in _bound_nodes(self.BASE_URL),
            what=f'{self.BASE_URL} to link {DEMO_FQN}',
        )

    def test_the_discarded_block_falls_back_to_the_defaults(self):
        """Not "the block was applied anyway", and not "the load half-failed".

        The manifest's sequence form asked for ``unmanifested_nodes: ignore``.
        The default is ``warn``, which keeps undeclared nodes visible - so the
        undeclared node being present proves the defaults are in force rather
        than whatever the malformed block appeared to say.
        """
        _poll(
            lambda: UNDECLARED_FQN in _bound_nodes(self.BASE_URL),
            what=f'{self.BASE_URL} to surface the undeclared {UNDECLARED_FQN}',
        )
        health = _get_json(self.BASE_URL, '/health')
        linking = health.get('discovery', {}).get('linking', {})
        self.assertEqual(
            linking.get('unmanifested_policy'), 'warn',
            f'the defaults must apply after the block is discarded: {linking}',
        )


class TestMalformedConfigKeyIsLenient(
    MalformedBlockCostsOnlyTheBlockMixin, GatewayTestCase,
):
    """The canonical `config:` key holding a scalar."""

    BASE_URL = LENIENT_URL


class TestMalformedDiscoveryAliasIsLenient(
    MalformedBlockCostsOnlyTheBlockMixin, GatewayTestCase,
):
    """The deprecated `discovery:` alias holding a sequence.

    The alias is read by the same code path, so it carries the same hazard and
    must carry the same guard.
    """

    BASE_URL = ALIAS_LENIENT_URL


# --------------------------------------------------------------------------
# Strict: the same manifest is a load failure
# --------------------------------------------------------------------------

class TestMalformedConfigUnderStrictValidation(GatewayTestCase):
    """Strict validation must NOT reject a manifest over an advisory notice.

    ``R015`` is advisory for this release, for the same upgrade-safety reason
    as ``R014``: the block was parsed but never read before this work, so a
    manifest carrying a malformed one loaded fine, and refusing it after an
    upgrade would take a working deployment down to ``runtime_only``. It
    becomes a validation warning in the next release.
    """

    BASE_URL = STRICT_URL
    REQUIRED_APPS = {APP_ID}

    def test_discovery_stays_hybrid(self):
        """The malformed block is reported, not fatal - strict or not."""
        health = _poll(
            lambda: _get_json(self.BASE_URL, '/health'),
            what=f'{self.BASE_URL} health document',
        )
        self.assertEqual(
            health.get('discovery', {}).get('mode'), 'hybrid',
            'an advisory R015 must not degrade discovery under strict validation',
        )

    def test_every_declared_entity_is_still_served(self):
        """The rest of the manifest is untouched by the malformed block."""
        self.assertIn(AREA_ID, _item_ids(self.BASE_URL, '/areas'))
        self.assertIn(COMPONENT_ID, _item_ids(self.BASE_URL, '/components'))
        self.assertIn(APP_ID, _item_ids(self.BASE_URL, '/apps'))

    def test_the_defaults_apply_after_the_block_is_discarded(self):
        """Advisory does not mean ignored: the block still does not take effect."""
        health = _get_json(self.BASE_URL, '/health')
        linking = health.get('discovery', {}).get('linking', {})
        self.assertEqual(linking.get('unmanifested_policy'), 'warn', linking)

    def test_the_gateway_is_still_serving(self):
        """Malformed configuration, but not a dead gateway."""
        health = _get_json(self.BASE_URL, '/health')
        self.assertEqual(health.get('status'), 'healthy')


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_manifests_are_cleaned_up(self):
        """The temp manifests this module wrote do not outlive the run."""
        _remove_manifests()

    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(
            proc_info, allowable_exit_codes=ALLOWED_EXIT_CODES
        )
