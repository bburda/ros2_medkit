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
Integration test: `unmanifested_nodes: ignore` must not delete plugin entities.

The orphan sweep used to decide protection by matching an entity's
``x-medkit.source`` against a list of known strings. That string is chosen by
whoever produced the entity: a discovery plugin may set anything, and the
beacon mapper sets ``beacon``. A provider that picked a tag the list had never
heard of therefore had its entities swept away - and this branch is what makes
``ignore`` actually take effect, so the sweep is newly reachable.

Protection now follows the layer that OWNS the entity, which the pipeline knows
for certain, rather than a string carried in the data. This test pins that end
to end with the in-tree test plugin, which declares two apps:

* ``plugin_dev`` - ``source: "plugin"``, a tag the old whitelist knew
* ``plugin_unlisted_source`` - ``source: "beacon"``, a tag it did not

Both must survive. The second is the one that regressed; the first is here so a
failure tells you whether protection broke generally or only for unlisted tags.
"""

import os
import tempfile
import time
import unittest

from ament_index_python.packages import get_package_prefix
import launch_testing
import launch_testing.actions
import requests

from ros2_medkit_test_utils.constants import (
    ALLOWED_EXIT_CODES,
    DISCOVERY_TIMEOUT,
    get_time_scale,
)
from ros2_medkit_test_utils.gateway_test_case import GatewayTestCase
from ros2_medkit_test_utils.launch_helpers import create_test_launch

# Apps the test plugin declares. The tag each carries is the whole point.
PLUGIN_APP_LISTED_TAG = 'plugin_dev'
PLUGIN_APP_UNLISTED_TAG = 'plugin_unlisted_source'
PLUGIN_APPS = {PLUGIN_APP_LISTED_TAG, PLUGIN_APP_UNLISTED_TAG}

PLUGIN_AREA = 'test_plugin_area'
PLUGIN_COMPONENT = 'test_plugin_comp'

# A node nothing declares. Under `ignore` it must be swept - that is what shows
# the policy is in force rather than quietly doing nothing.
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
# The plugin's entities appear within a pass or two when protection works. A
# short budget for "is it there" keeps a genuine deletion failing fast with a
# diagnostic, instead of every test burning the long timeout and the whole
# launch test dying as an uninformative ctest timeout.
ENTITY_TIMEOUT_SEC = 20.0 * TIME_SCALE
POLL_INTERVAL_SEC = 0.5
# How long a single HTTP call has to come back.
HTTP_TIMEOUT_SEC = 5.0 * TIME_SCALE
# The policy must keep holding, not merely hold once: the sweep runs on every
# discovery pass, and the defect deleted the entity on a later pass.
# Soak duration, not a budget - see the note above; left unscaled on purpose.
HOLD_SEC = 5.0


_MANIFEST_PATHS = []


def _plugin_path():
    return os.path.join(
        get_package_prefix('ros2_medkit_integration_tests'),
        'lib', 'ros2_medkit_integration_tests', 'libtopics_test_plugin.so',
    )


def _manifest_yaml():
    """Render a manifest that declares nothing, with the sweep switched on.

    `ignore` drops every app that is not protected, so with no manifest apps
    at all the only things that may survive are the plugin's.
    """
    return """\
manifest_version: "1.0"
metadata:
  name: "Plugin survives ignore"
  version: "1.0.0"
config:
  unmanifested_nodes: "ignore"
areas:
  - id: pi-area
    name: "Plugin Ignore Area"
"""


def generate_test_description():
    fd, manifest_path = tempfile.mkstemp(
        suffix='.yaml', prefix='test_plugin_entity_survives_ignore_',
    )
    with os.fdopen(fd, 'w') as handle:
        handle.write(_manifest_yaml())
    _MANIFEST_PATHS.append(manifest_path)

    return create_test_launch(
        demo_nodes=[UNDECLARED_NODE],
        fault_manager=False,
        gateway_params={
            'discovery.mode': 'hybrid',
            'discovery.manifest_path': manifest_path,
            'discovery.manifest_strict_validation': False,
            'plugins': ['topics_test'],
            'plugins.topics_test.path': _plugin_path(),
        },
    )


def _remove_manifests():
    """Delete every manifest this module wrote."""
    while _MANIFEST_PATHS:
        try:
            os.unlink(_MANIFEST_PATHS.pop())
        except OSError:
            pass


class TestPluginEntitySurvivesIgnorePolicy(GatewayTestCase):
    """A plugin-owned entity is protected whatever `source` tag it carries."""

    MIN_EXPECTED_APPS = 0

    def _app_documents(self):
        response = requests.get(f'{self.BASE_URL}/apps', timeout=HTTP_TIMEOUT_SEC)
        response.raise_for_status()
        return {
            item['id']: item for item in response.json().get('items', [])
        }

    def _poll_until(self, predicate, *, what, timeout=POLL_TIMEOUT_SEC):
        deadline = time.monotonic() + timeout
        last = None
        while time.monotonic() < deadline:
            try:
                last = self._app_documents()
                if predicate(last):
                    return last
            except requests.exceptions.RequestException as exc:
                last = exc
            time.sleep(POLL_INTERVAL_SEC)
        served = sorted(last) if isinstance(last, dict) else last
        raise AssertionError(
            f'timed out after {timeout}s waiting for {what}; served apps: {served}'
        )

    def test_the_ignore_policy_is_actually_in_force(self):
        """Guards the fixture: without the sweep running this proves nothing."""
        health = requests.get(f'{self.BASE_URL}/health', timeout=HTTP_TIMEOUT_SEC).json()
        linking = health.get('discovery', {}).get('linking', {})
        self.assertEqual(linking.get('unmanifested_policy'), 'ignore', health)

        # The undeclared node must be gone - that is the sweep working.
        def swept(apps):
            return not any(
                a.get('x-medkit', {}).get('ros2', {}).get('node') == UNDECLARED_FQN
                for a in apps.values()
            )

        self._poll_until(swept, what=f'{UNDECLARED_FQN} to be swept by `ignore`')

    def test_both_plugin_apps_survive_and_keep_their_source_tags(self):
        """The unlisted tag is the regression; the listed one is the control."""
        apps = self._poll_until(
            lambda a: PLUGIN_APPS <= set(a),
            what='both plugin apps to be served',
            timeout=ENTITY_TIMEOUT_SEC,
        )

        self.assertEqual(
            apps[PLUGIN_APP_UNLISTED_TAG].get('x-medkit', {}).get('source'), 'beacon',
            'the fix must protect the entity WITHOUT rewriting the tag its '
            'provider chose - `source` is public API and feeds identity '
            'provenance',
        )
        self.assertEqual(
            apps[PLUGIN_APP_LISTED_TAG].get('x-medkit', {}).get('source'), 'plugin',
        )

    def test_the_plugin_apps_keep_surviving_across_passes(self):
        """The sweep runs every pass; surviving one pass is not enough."""
        self._poll_until(
            lambda a: PLUGIN_APPS <= set(a),
            what='both plugin apps to be served',
            timeout=ENTITY_TIMEOUT_SEC,
        )

        deadline = time.monotonic() + HOLD_SEC
        while True:
            served = set(self._app_documents())
            missing = PLUGIN_APPS - served
            self.assertEqual(
                missing, set(),
                f'a plugin app was swept on a later discovery pass: {sorted(missing)}',
            )
            if time.monotonic() >= deadline:
                break
            time.sleep(POLL_INTERVAL_SEC)

    def test_the_plugins_area_and_component_survive_too(self):
        """Areas and Components go through the same suppression path."""
        def collection_ids(path):
            response = requests.get(f'{self.BASE_URL}{path}', timeout=HTTP_TIMEOUT_SEC)
            response.raise_for_status()
            return {item['id'] for item in response.json().get('items', [])}

        deadline = time.monotonic() + ENTITY_TIMEOUT_SEC
        while time.monotonic() < deadline:
            if PLUGIN_AREA in collection_ids('/areas') and \
                    PLUGIN_COMPONENT in collection_ids('/components'):
                return
            time.sleep(POLL_INTERVAL_SEC)
        self.fail(
            f'{PLUGIN_AREA} / {PLUGIN_COMPONENT} did not survive; '
            f'areas={sorted(collection_ids("/areas"))}, '
            f'components={sorted(collection_ids("/components"))}'
        )


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_manifests_are_cleaned_up(self):
        """The temp manifest this module wrote does not outlive the run."""
        _remove_manifests()

    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(
            proc_info, allowable_exit_codes=ALLOWED_EXIT_CODES
        )
