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

"""The role an operation publishes is the role the gateway enforces.

The document and the permission table come from one declaration on each route
registration, so the claim under test is that the derivation reaches both ends
intact: what ``GET /docs`` says a caller needs is what the middleware actually
demands.

Its own launch file, and that is the point of it existing separately from
``test_auth.test.py``. Launch configuration is per file, and that fixture runs
``require_auth_for: write`` - under which a GET is served with no token at all,
so a cross-check over the document's GETs would pass without checking anything.
Here it is ``all``: every path except ``/auth/*`` reaches
``AuthManager::check_authorization``, which is the code this file is about.

@verifies REQ_INTEROP_086
"""

import re
import tempfile
import time
import unittest

import launch
import launch_testing
import launch_testing.actions
import pytest
import requests

from ros2_medkit_test_utils.constants import (
    ALLOWED_EXIT_CODES,
    API_BASE_PATH,
    get_test_port,
)
from ros2_medkit_test_utils.launch_helpers import (
    create_gateway_node,
    full_feature_gateway_params,
)

RBAC_PORT = get_test_port()
RBAC_BASE_URL = f'http://127.0.0.1:{RBAC_PORT}{API_BASE_PATH}'

HTTP_METHODS = {'get', 'post', 'put', 'delete', 'patch'}

# Weakest first. The gateway expands a declared role upward (a route declaring
# OPERATOR is granted to OPERATOR, CONFIGURATOR and ADMIN), so "one step down"
# is the cheapest token that must be refused.
ROLE_LADDER = ['viewer', 'operator', 'configurator', 'admin']

CLIENTS = {
    'viewer': 'viewer_secret',
    'operator': 'operator_secret',
    'configurator': 'configurator_secret',
    'admin': 'admin_secret',
}

# Substituted for every ``{param}`` when a documented path template is turned
# into a request. Nothing by this name exists, so a request that clears the
# middleware lands on a 404/501/503 from the handler - which is all this file
# needs, since it only ever asks whether the *middleware* refused.
PROBE_ID = 'rbacprobe'

# Path parameters the router lets span segments, probed with a value that
# actually contains a slash.
#
# HAND-MAINTAINED, and it has to be: the document publishes a path template,
# not the regex behind it, so nothing in the served spec says which parameter
# may carry a `/`. The three below are what `RouteRegistry::to_regex_path`
# compiles to `(.+)` - a ROS topic name under `/data/{data_id}`, a dotted
# parameter path under `/configurations/{config_id}`, and the whole entity
# path prefixing `<entity-path>/docs`. A single-segment probe passes on those
# routes whether or not the permission derivation widened the wildcard, so
# without this list the part of the derivation most likely to be wrong is the
# part nothing here would notice. A fourth such parameter added to the gateway
# and not added here is simply not covered.
SLASH_SPANNING_PARAMS = {'data_id', 'config_id', 'entity_path'}
NESTED_PROBE_ID = f'{PROBE_ID}/nested'

# The lifecycle policy, written out rather than inferred.
#
# Every other test in this file checks that the published role and the enforced
# role agree. That is a *consistency* property, and consistency cannot notice a
# policy change: both sides come from one declaration, so flipping
# `destructive_transition` in `rest_server.cpp` to always-OPERATOR would move
# the document and the permission table together and leave the whole suite
# green. This table is the second opinion - it says what the roles should be,
# not merely that the two halves match - and it exists for the one call here
# with a real blast radius: `shutdown` and `force-shutdown` tear an entity down,
# `start` / `restart` / `force-restart` bring it back.
EXPECTED_LIFECYCLE_ROLES = {
    'start': 'operator',
    'restart': 'operator',
    'force-restart': 'operator',
    'shutdown': 'configurator',
    'force-shutdown': 'configurator',
}

# Entity types SOVD gives lifecycle transitions to.
LIFECYCLE_ENTITY_TYPES = {'apps', 'components'}

# `/<type>/{<type>_id}/status/<action>` - the shape a lifecycle transition path
# takes. The test reads the served transitions through this rather than
# assuming the two tables above are exhaustive, so a sixth action or a third
# entity type has to be added here deliberately instead of passing in silence.
LIFECYCLE_PATH = re.compile(r'/(\w+)/\{\w+\}/status/([\w-]+)')

# Scripts need a writable directory before the routes are mounted at all.
_SCRIPTS_DIR = tempfile.mkdtemp(prefix='medkit-rbac-scripts-')


@pytest.mark.launch_test
def generate_test_description():
    """Launch one gateway with authentication on and enforced everywhere."""
    gateway_node = create_gateway_node(
        port=RBAC_PORT,
        extra_params={
            'server.host': '127.0.0.1',
            'auth.enabled': True,
            'auth.jwt_secret': 'test_secret_key_for_rbac_contract_integration_1234',
            'auth.jwt_algorithm': 'HS256',
            'auth.token_expiry_seconds': 3600,
            'auth.refresh_token_expiry_seconds': 86400,
            # `write` would leave every GET unauthenticated and make the
            # cross-check below vacuous - see this module's docstring.
            'auth.require_auth_for': 'all',
            'auth.issuer': 'test_gateway',
            'auth.clients': [
                f'{role}:{secret}:{role}' for role, secret in CLIENTS.items()
            ],
            # Every optional gate on, so the routes behind them are in the
            # document and in the table rather than absent from both.
            **full_feature_gateway_params(_SCRIPTS_DIR),
        },
    )

    return launch.LaunchDescription([
        gateway_node,
        launch_testing.actions.ReadyToTest(),
    ]), {'gateway_node': gateway_node}


def _declared_role(operation):
    """Return the role an operation publishes.

    ``''`` for an operation that publishes the empty requirement (``security:
    []``, reachable with no token), ``None`` when it publishes no requirement
    at all - which under ``auth.enabled`` is the defect this file reports.
    """
    requirement = operation.get('security')
    if requirement is None:
        return None
    if requirement == []:
        return ''
    return requirement[0]['bearerAuth'][0]


def _weaker_role(role):
    """Return the next role down the ladder, or ``None`` for the weakest."""
    index = ROLE_LADDER.index(role)
    return ROLE_LADDER[index - 1] if index > 0 else None


def _is_sse(operation):
    """Report whether this operation is a stream.

    Excluded from every request this file makes: an SSE handler holds the
    connection open for the stream's whole lifetime, so a probe would block
    until the client timeout rather than answer, and the file would spend its
    launch budget waiting on eight of them.
    """
    content = operation.get('responses', {}).get('200', {}).get('content', {})
    return 'text/event-stream' in content


def _auth_refusal(response):
    """Report whether the *middleware* refused, as opposed to a handler.

    Both write 4xx, and the distinction is the whole measurement here. The auth
    middleware serialises ``AuthErrorResponse`` - RFC 6749's ``{"error",
    "error_description"}`` - while a handler's own 401/403 (a lock owned by
    another client, a read-only parameter, a provider refusing a transition) is
    the SOVD ``GenericError`` shape with ``error_code``.
    """
    if response.status_code not in (401, 403):
        return False
    try:
        body = response.json()
    except ValueError:
        return True
    return 'error' in body and 'error_code' not in body


class TestRbacContract(unittest.TestCase):
    """The published role and the enforced role are the same role."""

    _spec = None

    @classmethod
    def setUpClass(cls):
        """Wait for the gateway, then collect one token per role.

        `GatewayTestCase` is deliberately not the base class: its readiness
        wait polls `/health` unauthenticated and expects 200, which under
        `require_auth_for: all` is a 401 forever.
        """
        cls.session = requests.Session()
        cls.tokens = {}
        for role, secret in CLIENTS.items():
            cls.tokens[role] = cls._acquire_token(role, secret)

        cls._spec = cls.session.get(
            f'{RBAC_BASE_URL}/docs',
            headers={'Authorization': f'Bearer {cls.tokens["admin"]}'},
            timeout=15,
        ).json()

    @classmethod
    def tearDownClass(cls):
        cls.session.close()

    @classmethod
    def _acquire_token(cls, role, secret):
        """Poll `/auth/authorize` until the gateway hands out a token."""
        deadline = time.time() + 30.0
        last = None
        while time.time() < deadline:
            try:
                response = cls.session.post(
                    f'{RBAC_BASE_URL}/auth/authorize',
                    json={
                        'grant_type': 'client_credentials',
                        'client_id': role,
                        'client_secret': secret,
                    },
                    timeout=5,
                )
                if response.status_code == 200:
                    return response.json()['access_token']
                last = f'{response.status_code}: {response.text[:200]}'
            except requests.RequestException as exc:
                last = str(exc)
            time.sleep(0.5)
        raise AssertionError(f'no token for {role} within 30s (last: {last})')

    def _auth_header(self, role):
        """Build the Authorization header carrying `role`'s token."""
        return {'Authorization': f'Bearer {self.tokens[role]}'}

    def operations(self):
        """Yield (path, method, operation) for every documented operation."""
        for path, item in self._spec['paths'].items():
            for method, operation in item.items():
                if method in HTTP_METHODS:
                    yield path, method, operation

    def _request(self, method, path, headers):
        """Send one probe request at a documented path template."""
        url = RBAC_BASE_URL + path
        while '{' in url:
            start = url.index('{')
            end = url.index('}', start)
            name = url[start + 1:end]
            probe = (NESTED_PROBE_ID if name in SLASH_SPANNING_PARAMS
                     else PROBE_ID)
            url = url[:start] + probe + url[end + 1:]
        kwargs = {'timeout': 10, 'headers': headers}
        if method in ('post', 'put', 'patch'):
            kwargs['json'] = {}
        return self.session.request(method.upper(), url, **kwargs)

    def test_every_operation_publishes_a_requirement(self):
        """With auth on, no operation leaves its access rule unstated.

        The gateway fails closed: a path the permission table does not match is
        403 for every role the residual list does not cover. An operation that
        publishes nothing is therefore not "unrestricted" - it is a route whose
        rule the caller has no way to learn and, on a route whose registration
        forgot to declare one, a 403 with no explanation anywhere.
        """
        silent = sorted(
            op.get('operationId', f'{m.upper()} {p}')
            for p, m, op in self.operations()
            if _declared_role(op) is None
        )
        self.assertEqual(
            silent, [],
            f'operations publishing no security requirement: {silent}')

    def test_published_roles_are_roles_the_gateway_knows(self):
        """Every published scope names one of the four configured roles."""
        total = 0
        for path, method, op in self.operations():
            role = _declared_role(op)
            total += 1
            if role == '':
                continue
            self.assertIn(
                role, ROLE_LADDER,
                f'{method.upper()} {path} publishes unknown role {role!r}')
        # Anti-vacuous: the fixture turns every optional gate on, so the
        # document is the maximal route surface rather than the handful of
        # always-on endpoints. Without this the loop above would pass on a
        # document that had collapsed to almost nothing.
        self.assertGreater(
            total, 200,
            f'only {total} operations documented; the fixture lost its gates')

    def test_only_the_auth_endpoints_are_published_as_public(self):
        """`security: []` is reserved for the paths the middleware exempts.

        `AllAuthRequirementPolicy` lets `/api/v1/auth/` through by prefix and
        nothing else. Publishing the empty requirement anywhere else would
        promise a token-free call the middleware would then answer 401.
        """
        public = sorted(
            path for path, _, op in self.operations()
            if _declared_role(op) == ''
        )
        self.assertEqual(
            public, ['/auth/authorize', '/auth/revoke', '/auth/token'])

    def test_the_declared_role_is_admitted(self):
        """A token of the published role clears the middleware, everywhere.

        Sent at the documented path template with a probe id substituted, so
        the handler behind it answers 404/501/503 - a status this test does not
        care about. What it reads is only whether the middleware refused, which
        it can tell from a handler's own 401/403 by the body shape.
        """
        checked = 0
        for path, method, op in self.operations():
            role = _declared_role(op)
            if role is None or _is_sse(op):
                continue
            headers = {} if role == '' else self._auth_header(role)
            response = self._request(method, path, headers)
            self.assertFalse(
                _auth_refusal(response),
                f'{method.upper()} {path} publishes {role or "public"!r} but '
                f'the middleware answered {response.status_code}: '
                f'{response.text[:200]}')
            checked += 1
        self.assertGreater(
            checked, 200, f'only {checked} operations probed')

    def test_a_weaker_role_is_refused(self):
        """One step down the ladder is refused wherever there is a step.

        Only the direction the derivation could get wrong by being too
        generous. A route declaring OPERATOR is expanded to OPERATOR and above,
        so VIEWER must not reach it; a route declaring VIEWER has nothing below
        it and is skipped, which is why the count is asserted - a derivation
        that granted everything to VIEWER would leave nothing to check here.
        """
        checked = 0
        for path, method, op in self.operations():
            role = _declared_role(op)
            if not role or _is_sse(op):
                continue
            weaker = _weaker_role(role)
            if weaker is None:
                continue
            response = self._request(method, path, self._auth_header(weaker))
            # `_auth_refusal`, not `status_code == 403`. Several handlers answer
            # their own 403 (a lock owned by another client, a read-only
            # parameter, a provider refusing a transition), and those carry the
            # SOVD `error_code` shape. Accepting a bare 403 would let a genuine
            # widening - the middleware waving the weaker token through to a
            # handler that then refused for its own reasons - pass as if the
            # permission had held.
            self.assertTrue(
                _auth_refusal(response),
                f'{method.upper()} {path} publishes {role!r} but a {weaker!r} '
                f'token was not refused by the middleware; got '
                f'{response.status_code}: {response.text[:200]}')
            checked += 1
        self.assertGreater(
            checked, 40,
            f'only {checked} operations sit above viewer; the derivation may '
            f'have granted everything to the weakest role')

    def test_lifecycle_transitions_publish_the_roles_the_policy_fixes(self):
        """Tearing an entity down needs configurator; restarting it needs operator.

        The one assertion in this file that is not a consistency check. Every
        other test compares the document with enforcement, and both come from a
        single declaration on the registration - so a change to that
        declaration moves them together and goes unnoticed. This states the
        policy independently, which is what makes an accidental loosening of
        the destructive transitions turn the suite red.

        Which transitions exist is read out of the document, not assumed, so a
        sixth action or a third entity type gaining transitions fails here
        rather than slipping past an expected set that never mentions it. Only
        the roles are a literal, and deliberately so: a value derived from
        nothing is what makes this a statement about policy instead of a second
        look at the same declaration.
        """
        served = {}
        for path, item in self._spec['paths'].items():
            found = LIFECYCLE_PATH.fullmatch(path)
            if found and 'put' in item:
                served[found.group(1), found.group(2)] = _declared_role(item['put'])

        self.assertEqual(
            {action for _, action in served}, set(EXPECTED_LIFECYCLE_ROLES),
            'the document serves a different set of lifecycle transitions than '
            'this test states a policy for')
        self.assertEqual(
            {entity for entity, _ in served}, LIFECYCLE_ENTITY_TYPES,
            'lifecycle transitions are served for a different set of entity '
            'types than this test states a policy for')

        for (entity_type, action), role in sorted(served.items()):
            self.assertEqual(
                role, EXPECTED_LIFECYCLE_ROLES[action],
                f'PUT /{entity_type}/.../status/{action} publishes {role!r}, '
                f'policy says {EXPECTED_LIFECYCLE_ROLES[action]!r}')

    def test_a_read_route_needs_a_token_and_accepts_the_viewer_one(self):
        """The derived table reached the enforcer, not just the document.

        Everything above compares one side of the gateway with the other. This
        compares the running gateway with the plain claim: no token is 401,
        and the weakest role the table grants gets through.
        """
        anonymous = self.session.get(f'{RBAC_BASE_URL}/health', timeout=10)
        self.assertEqual(anonymous.status_code, 401)

        authorized = self.session.get(
            f'{RBAC_BASE_URL}/health',
            headers=self._auth_header('viewer'),
            timeout=10,
        )
        self.assertEqual(authorized.status_code, 200)

    def test_a_plugin_route_stays_admin_only(self):
        """Routes the registry never sees are covered by the residual list.

        A plugin mounts its routes straight onto the HTTP server, so no
        `requires_role` on any registration describes them and only ADMIN's
        `**` entries reach them. The document says `admin`; this is the
        enforcement half of that, and it is the reason the residual list stops
        at ADMIN rather than granting plugin surface to a weaker role.
        """
        path = '/functions/{function_id}/x-medkit-graph'
        operation = self._spec['paths'].get(path, {}).get('get')
        self.assertIsNotNone(
            operation,
            'the graph provider route is not documented; the fixture must '
            'load the plugin for this test to mean anything')
        self.assertEqual(_declared_role(operation), 'admin')

        for role in ('viewer', 'operator', 'configurator'):
            response = self._request('get', path, self._auth_header(role))
            # `_auth_refusal` rather than a bare 403, and this is the route
            # where it matters most: it is the only one whose handler is
            # third-party, so a plugin answering its own 403 is the likeliest
            # way a widening could disguise itself as a refusal.
            self.assertTrue(
                _auth_refusal(response),
                f'a {role} token was not refused by the middleware on a '
                f'plugin route; got {response.status_code}: '
                f'{response.text[:200]}')

        admitted = self._request('get', path, self._auth_header('admin'))
        self.assertFalse(_auth_refusal(admitted))


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):
    """Post-shutdown tests."""

    def test_exit_codes(self, proc_info):
        """Check all processes exited cleanly (SIGTERM allowed)."""
        for info in proc_info:
            self.assertIn(
                info.returncode, ALLOWED_EXIT_CODES,
                f'{info.process_name} exited with code {info.returncode}'
            )
