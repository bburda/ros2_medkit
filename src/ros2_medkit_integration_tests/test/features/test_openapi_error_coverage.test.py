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

"""The document must declare every status the gateway actually serves.

Every other OpenAPI assertion in this suite compares the document against a
list somebody typed. This one compares it against a *run*: the gateway is
compiled with an emitted-status recorder (test builds only), the test drives
the whole documented route surface into its error branches, and then asserts
``declared`` is a superset of ``observed``. Nothing here enumerates statuses,
so nothing here can rot when a handler gains a new one.

The exercise is derived from the served document itself: every documented
operation whose path has at least one ``{param}`` is called twice - once with
a syntactically valid but nonexistent entity id (the not-found branch) and
once with a syntactically invalid one (the validation branch). Operations
without a path parameter are deliberately skipped: with no id to poison they
would act on real gateway state.

What the recorder can see, and what it cannot, is documented on
``StatusRecordingScope`` in
``include/ros2_medkit_gateway/http/detail/status_recorder.hpp``. The short
version: it observes the wire status of everything the route registry mounts,
and is blind to what answers ahead of routing (the rate limiter's 429, the
auth middleware's 401/403, the CORS reject), to anything cpp-httplib answers
by itself, and to routes a plugin mounts outside the registry. Those are
declared by hand; the plugin ones are marked ``x-medkit-plugin-served`` in the
document so the reachability assertion below can tell them apart from a
registry route the sweep genuinely failed to reach.
"""

import json
import tempfile
import unittest

import launch_testing
import launch_testing.actions
import requests

from ros2_medkit_test_utils.constants import ALLOWED_EXIT_CODES
from ros2_medkit_test_utils.gateway_test_case import GatewayTestCase
from ros2_medkit_test_utils.launch_helpers import (
    create_test_launch,
    full_feature_gateway_params,
)

HTTP_METHODS = {'get', 'post', 'put', 'delete', 'patch'}

# Statuses every operation declares anyway (the blanket 400/404/500, plus the
# success codes). Observing only these would mean the sweep never reached a
# branch the document had to be told about, so a run that sees nothing outside
# this set is treated as a broken sweep rather than a pass.
UNINTERESTING_STATUSES = {200, 201, 202, 204, 400, 404, 500}

# A well-formed entity id (alphanumeric + underscore) that no fixture creates,
# so the request survives id validation and dies in the entity lookup.
ABSENT_ID = 'zzz_no_such_entity_zzz'

# An id that fails the gateway's own id validation, so the request dies before
# the lookup. The two together cover both sides of every parameterised route.
INVALID_ID = 'bad id!*'

# The operations the sweep refuses to call, pinned rather than derived. Each is
# a state-changing verb on a path with no id to poison, so calling it would act
# on real gateway state instead of exercising an error path: DELETE /faults
# clears the store, POST /updates registers an update, and the three /auth
# posts mint or revoke tokens.
#
# Pinned as a literal on purpose. Deriving this set by shape - "parameterless
# and not a GET" - would excuse a *new* parameterless write route forever, so
# the rule that is supposed to catch an unreachable operation would grow a
# silent hole every time one is added. The shape rule is still checked below,
# as a cross-check that nothing reachable was smuggled into this list.
SKIPPED_BY_DESIGN = {
    ('delete', '/faults'),
    ('post', '/auth/authorize'),
    ('post', '/auth/revoke'),
    ('post', '/auth/token'),
    ('post', '/updates'),
}

# Floor on the number of `make_error` sites a run must reach. The fixture
# reaches 39 of the 281 sites in the tree; the floor sits below that so
# ordinary churn in the handlers does not need this number edited, while a
# collapse - a broken sweep, a recorder that stopped recording, a substitution
# that stopped producing bad ids - fails instead of quietly reporting "1 site".
MIN_ERROR_SITES = 30

_SCRIPTS_DIR = tempfile.mkdtemp(prefix='medkit-coverage-scripts-')


def generate_test_description():
    return create_test_launch(
        demo_nodes=['calibration', 'temp_sensor'],
        fault_manager=True,
        gateway_params=full_feature_gateway_params(_SCRIPTS_DIR),
    )


def path_params(template):
    """Return the ``{param}`` names of an OpenAPI path, in order."""
    names = []
    i = 0
    while i < len(template):
        if template[i] == '{':
            close = template.find('}', i)
            if close < 0:
                break
            names.append(template[i + 1:close])
            i = close + 1
        else:
            i += 1
    return names


def concrete_path(template, ids):
    """Substitute the ``{param}`` placeholders of *template* from *ids*."""
    out = []
    used = 0
    i = 0
    while i < len(template):
        if template[i] == '{':
            close = template.find('}', i)
            if close < 0:
                out.append(template[i:])
                break
            out.append(ids[min(used, len(ids) - 1)])
            used += 1
            i = close + 1
        else:
            out.append(template[i])
            i += 1
    return ''.join(out)


class TestOpenApiErrorCoverage(GatewayTestCase):
    """``declared`` must be a superset of ``observed``."""

    MIN_EXPECTED_APPS = 2
    REQUIRED_APPS = {'calibration', 'temp_sensor'}

    _spec = None
    _coverage = None
    _swept = 0

    @classmethod
    def setUpClass(cls):
        """Wait for the gateway, then drive the error paths before asserting.

        The sweep runs here rather than in a test so that ordering is not left
        to unittest's alphabetical method order: every assertion below reads a
        recorder that has already been filled.

        The fixture state is stored on ``TestOpenApiErrorCoverage`` by name,
        not on ``cls``: launch_testing runs a generated subclass, so ``cls`` is
        not the class the assertions read through.
        """
        super().setUpClass()
        owner = TestOpenApiErrorCoverage
        owner._spec = cls._fetch_spec()
        owner._swept = cls._sweep_error_paths()
        cls._drive_lock_conflict()
        owner._coverage = cls._fetch_coverage()

    # ------------------------------------------------------------------
    # Fixture helpers
    # ------------------------------------------------------------------

    @classmethod
    def _fetch_spec(cls):
        resp = requests.get(f'{cls.BASE_URL}/docs', timeout=15)
        resp.raise_for_status()
        return resp.json()

    @classmethod
    def _fetch_coverage(cls):
        resp = requests.get(
            f'{cls.BASE_URL}/x-medkit-status-coverage', timeout=15)
        resp.raise_for_status()
        return resp.json()

    @classmethod
    def _real_entity_ids(cls):
        """Return {collection: first discovered id} for the four entity types.

        Read from the running gateway rather than hard-coded, so the deep
        sweep below cannot silently degrade into the shallow one by naming an
        entity the fixture stopped creating.
        """
        found = {}
        for collection in ('areas', 'components', 'apps', 'functions'):
            resp = requests.get(f'{cls.BASE_URL}/{collection}', timeout=10)
            items = resp.json().get('items', []) if resp.status_code == 200 else []
            ids = sorted(item['id'] for item in items if item.get('id'))
            if ids:
                found[collection] = ids[0]
        return found

    @classmethod
    def _sweep_error_paths(cls):
        """Drive every parameterised operation into its error branches.

        Three substitutions per operation:

        1. every id absent-but-well-formed - the entity-lookup branch;
        2. every id malformed - the id-validation branch, which answers before
           the lookup;
        3. for paths with two or more parameters, a *real* leading entity with
           the rest absent - the per-resource branches (unknown data item,
           unknown execution, unknown fault) that (1) never reaches because it
           dies at the entity gate.

        The real-entity pass is restricted to two-or-more-parameter paths on
        purpose: a trailing absent id is what guarantees the request cannot
        change state. ``DELETE /apps/{app_id}/configurations`` has one
        parameter, and pointing it at a real app would wipe that app's
        configurations rather than exercise an error path.
        """
        real_ids = cls._real_entity_ids()
        swept = 0
        for path, item in TestOpenApiErrorCoverage._spec['paths'].items():
            params = path_params(path)
            if not params:
                # Nothing to poison here, so only the read-only verb is safe:
                # a POST or DELETE on a parameterless path acts on real state
                # (`DELETE /faults` clears them, `POST /updates` starts one).
                if 'get' in item:
                    try:
                        resp = requests.get(
                            f'{cls.BASE_URL}{path}', timeout=10, stream=True)
                        resp.close()
                        swept += 1
                    except requests.exceptions.RequestException as exc:
                        print(f'sweep: GET {path} raised {exc}')
                continue
            substitutions = [[ABSENT_ID], [INVALID_ID]]
            collection = path.strip('/').split('/')[0]
            if len(params) >= 2 and collection in real_ids:
                substitutions.append([real_ids[collection], ABSENT_ID])
            for method in item:
                if method not in HTTP_METHODS:
                    continue
                for ids in substitutions:
                    url = f'{cls.BASE_URL}{concrete_path(path, ids)}'
                    kwargs = {'timeout': 10, 'stream': True}
                    if method in ('post', 'put', 'patch'):
                        # An empty object is a valid JSON document and an
                        # invalid body for every route that takes one, so the
                        # body-validation branch is reached too.
                        kwargs['json'] = {}
                    try:
                        resp = requests.request(method, url, **kwargs)
                        resp.close()
                        swept += 1
                    except requests.exceptions.RequestException as exc:
                        print(f'sweep: {method.upper()} {url} raised {exc}')
        print(f'sweep: issued {swept} requests over {len(real_ids)} real entities')
        return swept

    @classmethod
    def _drive_lock_conflict(cls):
        """Make one write answer 409, a status no blanket rule adds.

        Raises rather than reports: this is the only 409 the run produces, and
        an acquire that quietly returned 501 (locking off) or 404 (the entity
        gone) would leave the interesting-status assertion passing on the
        sweep's 501s alone - a green run that proved less than it claims.
        """
        acquired = requests.post(
            f'{cls.BASE_URL}/apps/temp_sensor/locks',
            json={'lock_expiration': 300},
            headers={'X-Client-Id': 'coverage_client_a'},
            timeout=10,
        )
        if acquired.status_code != 201:
            raise AssertionError(
                f'lock acquire returned {acquired.status_code}, expected 201: '
                f'{acquired.text}')
        try:
            blocked = requests.put(
                f'{cls.BASE_URL}/apps/temp_sensor/data/engine_temperature',
                json={'type': 'std_msgs/msg/Float64', 'data': {'data': 42.0}},
                headers={'X-Client-Id': 'coverage_client_b'},
                timeout=10,
            )
            if blocked.status_code != 409:
                raise AssertionError(
                    f'write by the wrong client returned {blocked.status_code}, '
                    f'expected 409: {blocked.text}')
        finally:
            requests.delete(
                f'{cls.BASE_URL}/apps/temp_sensor/locks/{acquired.json()["id"]}',
                headers={'X-Client-Id': 'coverage_client_a'},
                timeout=10,
            )

    def spec(self):
        return TestOpenApiErrorCoverage._spec

    def coverage(self):
        return TestOpenApiErrorCoverage._coverage

    def operations(self):
        """Yield (path, method, operation) for every documented operation."""
        for path, item in self.spec()['paths'].items():
            for method, operation in item.items():
                if method in HTTP_METHODS:
                    yield path, method, operation

    # ------------------------------------------------------------------
    # Assertions
    # ------------------------------------------------------------------

    def test_every_emitted_status_is_declared(self):
        """No handler emits a status its operation does not declare."""
        spec = self.spec()
        observed = self.coverage()
        undeclared = []
        for entry in observed['emitted']:
            op = spec['paths'].get(entry['path'], {}).get(entry['method'].lower())
            if op is None:
                continue
            if str(entry['status']) not in op.get('responses', {}):
                undeclared.append(
                    f"{entry['method'].upper()} {entry['path']} -> {entry['status']}")
        self.assertEqual(
            sorted(undeclared), [], f'emitted but undeclared: {sorted(undeclared)}')

    def test_the_sweep_reached_the_documented_surface(self):
        """A recorder that saw nothing would make the rule above vacuous.

        The expectation is exact, not a percentage: the only operations the
        sweep is allowed to miss are the ones it deliberately skips - a
        state-changing verb on a path with no id to poison. Anything else
        going unreached means the substitution, the spec fetch or the recorder
        itself has quietly stopped working, and the superset rule above would
        then pass with an empty left-hand side.

        Operations marked ``x-medkit-plugin-served`` are excluded, and that
        exclusion is derived rather than listed: a plugin route is mounted by
        ``PluginManager::register_routes`` instead of by the ``RouteRegistry``,
        and the recorder attaches at the registry's mounting point, so no run
        can ever observe one. The sweep still *calls* them - what it cannot do
        is see the answer. The marker is stamped by the gateway when it folds
        the plugin's description into the document, so this cannot drift into
        excusing a registry route.
        """
        plugin_served = {(method, path) for path, method, op in self.operations()
                         if op.get('x-medkit-plugin-served')}
        documented = {(method, path) for path, method, _ in self.operations()} - plugin_served
        observed = {(e['method'].lower(), e['path'])
                    for e in self.coverage()['emitted']}
        reached = observed & documented
        print(f'coverage: {len(reached)}/{len(documented)} documented operations '
              f'reached, {self._swept} requests issued; '
              f'plugin-served (recorder-invisible): {sorted(plugin_served)}; '
              f'unreached: {sorted(documented - observed)}')
        # This fixture loads the graph provider, which is the only in-tree
        # plugin that describes a route. An empty exclusion set would mean the
        # fold stopped happening, and this test would then be passing on a
        # document that lost an operation rather than on one that never had it.
        self.assertTrue(
            plugin_served,
            'no plugin-served operation in the document; the fixture loads the '
            'graph provider, so the plugin route fold has broken')
        # The recorder's blind spot is structural, not per-route: nothing a
        # plugin serves may appear in `observed` at all. If one ever did, the
        # exclusion above would be silently hiding a real gap.
        self.assertEqual(
            sorted(plugin_served & observed), [],
            'the recorder observed a plugin-served route, so excluding them is wrong')
        # Cross-check first: every pinned entry must be one the sweep genuinely
        # cannot call. Without this the literal could be padded with a route
        # that is reachable, turning the pin into an exemption list.
        for method, path in sorted(SKIPPED_BY_DESIGN):
            self.assertIn((method, path), documented, f'{method} {path} is not documented')
            self.assertFalse(
                path_params(path),
                f'{method} {path} has a path parameter, so the sweep can reach it')
            self.assertNotEqual(
                method, 'get', f'{method} {path} is read-only, so the sweep can reach it')
        self.assertEqual(
            sorted(documented - observed), sorted(SKIPPED_BY_DESIGN),
            'unreached operations differ from the pinned set')

    def test_the_sweep_reached_a_status_no_blanket_rule_declares(self):
        """The run must contain at least one interesting status.

        400, 404, 500 and the 2xx codes are declared on every operation by
        construction, so a sweep that only ever saw those would satisfy the
        superset rule no matter how wrong the document was.
        """
        interesting = sorted({
            (e['method'].upper(), e['path'], e['status'])
            for e in self.coverage()['emitted']
            if e['status'] not in UNINTERESTING_STATUSES
        })
        print(f'coverage: interesting statuses observed: '
              f'{json.dumps(interesting)}')
        self.assertTrue(
            interesting, 'sweep observed only blanket-declared statuses')

    def test_the_recorder_reports_the_error_sites_it_reached(self):
        """The recorder names the ``make_error`` sites that fired.

        This is the honest measure of the mechanism: it is what lets a reader
        say how much of the ~281-site error surface a run actually exercised,
        instead of assuming the recorder saw everything because it saw
        something.
        """
        sites = self.coverage()['error_sites']
        print(f'coverage: {len(sites)} make_error sites fired: {json.dumps(sites)}')
        for site in sites:
            self.assertRegex(site, r'^.+:\d+ -> \d{3}$', f'malformed site {site}')
        self.assertGreaterEqual(
            len(sites), MIN_ERROR_SITES,
            f'only {len(sites)} make_error sites fired; the sweep or the '
            f'recorder has collapsed (see MIN_ERROR_SITES)')


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        """Check all processes exited cleanly (SIGTERM allowed)."""
        for info in proc_info:
            self.assertIn(
                info.returncode, ALLOWED_EXIT_CODES,
                f'{info.process_name} exited with code {info.returncode}'
            )
