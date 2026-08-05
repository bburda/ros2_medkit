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

"""What the document says about authentication is what the middleware does.

``test_rbac_contract.test.py`` asks whether the *role* an operation publishes is
the role the gateway demands, and it asks it under ``require_auth_for: all``.
That leaves the other half unasked: whether an operation should be publishing a
token requirement at all. ``auth.enabled`` is not that answer - enforcement is
``config_.enabled && auth_policy_->requires_authentication(method, path)``
(``auth_manager.cpp``), so under ``require_auth_for: write`` a GET is served to
an anonymous caller, and under ``none`` so is everything.

So this file launches three gateways, one per policy, and asks each the same
question about every operation it serves: does the published requirement, and
do the two middleware refusal statuses, match what the middleware does on the
wire?

Both halves are measured. The published half is read from that gateway's own
``GET /docs``; the enforced half is a real unauthenticated request at the
documented path template. Neither side is assumed from the policy name, so a
policy whose behaviour is not what its name suggests fails here rather than
passing by agreeing with a hard-coded table.

@verifies REQ_INTEROP_086
"""

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

HTTP_METHODS = {'get', 'post', 'put', 'delete', 'patch'}

# One gateway per `require_auth_for` value, each on its own port within this
# test's stride-of-10 allocation.
POLICIES = ('all', 'write', 'none')
PORTS = {policy: get_test_port(offset) for offset, policy in enumerate(POLICIES)}

JWT_SECRET = 'test_secret_key_for_auth_policy_contract_integration'
CLIENTS = {'admin': 'admin_secret'}

# Substituted for every `{param}`. Nothing by this name exists, so a probe that
# clears the middleware lands on a handler's 404/501/503 - which is all this
# file needs, since it only ever asks whether the *middleware* refused.
PROBE_ID = 'authpolicyprobe'

# The two response components the auth middleware owns. A route may declare its
# own 401 - `/auth/authorize` answers one for bad credentials whatever the
# policy - so the presence of the status is not the measurement; the presence of
# *these* components is.
MIDDLEWARE_401 = '#/components/responses/Unauthorized'
MIDDLEWARE_403 = '#/components/responses/Forbidden'

# `/auth/*` is exempted by every policy the gateway can be configured with, and
# its handlers answer their own RFC 6749 401 for bad credentials - the same body
# shape the middleware uses, which is exactly what `_middleware_refused` reads.
# Probing these paths would therefore measure the handler and call it the
# middleware, so they are checked structurally instead (see
# `test_auth_endpoints_are_published_as_reachable_without_a_token`).
AUTH_PATH_PREFIX = '/auth/'

_SCRIPTS_DIRS = {
    policy: tempfile.mkdtemp(prefix=f'medkit-authpolicy-{policy}-')
    for policy in POLICIES
}


@pytest.mark.launch_test
def generate_test_description():
    """Launch one authenticated gateway per `require_auth_for` value."""
    nodes = [
        create_gateway_node(
            port=PORTS[policy],
            name=f'gateway_auth_{policy}',
            extra_params={
                'server.host': '127.0.0.1',
                'auth.enabled': True,
                'auth.jwt_secret': JWT_SECRET,
                'auth.jwt_algorithm': 'HS256',
                'auth.token_expiry_seconds': 3600,
                'auth.require_auth_for': policy,
                'auth.issuer': 'test_gateway',
                'auth.clients': [
                    f'{role}:{secret}:admin' for role, secret in CLIENTS.items()
                ],
                # Every optional gate on, so the routes behind them are in the
                # document and on the wire rather than absent from both.
                **full_feature_gateway_params(_SCRIPTS_DIRS[policy]),
            },
        )
        for policy in POLICIES
    ]

    return launch.LaunchDescription([
        *nodes,
        launch_testing.actions.ReadyToTest(),
    ]), {f'gateway_{policy}': node for policy, node in zip(POLICIES, nodes)}


def _base_url(policy):
    """Return the API base URL of the gateway running `policy`."""
    return f'http://127.0.0.1:{PORTS[policy]}{API_BASE_PATH}'


def _is_sse(operation):
    """Report whether this operation is a stream.

    Excluded from every request this file makes: an SSE handler holds the
    connection open for the stream's whole lifetime, so a probe would block
    until the client timeout rather than answer.
    """
    content = operation.get('responses', {}).get('200', {}).get('content', {})
    return 'text/event-stream' in content


def _middleware_refused(response):
    """Report whether the *middleware* refused, as opposed to a handler.

    Both write 4xx, and the distinction is the whole measurement here. The auth
    middleware serialises RFC 6749's ``{"error", "error_description"}`` while a
    handler's own 401/403 is the SOVD ``GenericError`` shape with
    ``error_code``.
    """
    if response.status_code not in (401, 403):
        return False
    try:
        body = response.json()
    except ValueError:
        return True
    return 'error' in body and 'error_code' not in body


def _response_ref(operation, status):
    """Return the ``$ref`` target this operation gives `status`, or ``''``."""
    entry = operation.get('responses', {}).get(status)
    if not isinstance(entry, dict):
        return ''
    return entry.get('$ref', '')


class AuthPolicyContractBase:
    """Shared body; one concrete subclass per policy, so failures name it.

    Not a `TestCase` itself - unittest would collect and run it with
    `POLICY = None`. The subclasses below mix it with `TestCase`.
    """

    POLICY = None

    @classmethod
    def setUpClass(cls):
        """Take a token, read the document, and confirm the policy served."""
        cls.session = requests.Session()
        cls.base_url = _base_url(cls.POLICY)
        cls.token = cls._acquire_token()
        cls._spec = cls.session.get(
            f'{cls.base_url}/docs',
            headers={'Authorization': f'Bearer {cls.token}'},
            timeout=15,
        ).json()

        # The gateway states its own policy at the root. Read it rather than
        # trusting the launch parameter: without this, a gateway that ignored
        # `require_auth_for` would make every assertion below agree with the
        # wrong policy and pass.
        root = cls.session.get(
            cls.base_url,
            headers={'Authorization': f'Bearer {cls.token}'},
            timeout=15,
        ).json()
        served = root.get('auth', {}).get('require_auth_for')
        assert served == cls.POLICY, (
            f'asked for require_auth_for={cls.POLICY!r}, gateway on port '
            f'{PORTS[cls.POLICY]} reports {served!r}')

        # Probed once for the whole class. unittest builds a fresh instance per
        # test method, so an instance-level cache would re-send several hundred
        # requests for each of the four methods below and spend the launch
        # budget on it.
        cls.enforcement = {
            (path, method): _middleware_refused(cls._probe(method, path))
            for path, method, _ in cls._probeable()
        }

    @classmethod
    def tearDownClass(cls):
        cls.session.close()

    @classmethod
    def _acquire_token(cls):
        """Poll `/auth/authorize` until the gateway hands out an admin token."""
        deadline = time.time() + 60.0
        last = None
        while time.time() < deadline:
            try:
                response = cls.session.post(
                    f'{cls.base_url}/auth/authorize',
                    json={
                        'grant_type': 'client_credentials',
                        'client_id': 'admin',
                        'client_secret': CLIENTS['admin'],
                    },
                    timeout=5,
                )
                if response.status_code == 200:
                    return response.json()['access_token']
                last = f'{response.status_code}: {response.text[:200]}'
            except requests.RequestException as exc:
                last = str(exc)
            time.sleep(0.5)
        raise AssertionError(
            f'no token from the {cls.POLICY!r} gateway within 60s '
            f'(last: {last})')

    @classmethod
    def _operations(cls):
        """Yield (path, method, operation) for every documented operation."""
        for path, item in cls._spec['paths'].items():
            for method, operation in item.items():
                if method in HTTP_METHODS:
                    yield path, method, operation

    @classmethod
    def _probeable(cls):
        """Yield the operations this file sends a real request to."""
        for path, method, operation in cls._operations():
            if _is_sse(operation) or path.startswith(AUTH_PATH_PREFIX):
                continue
            yield path, method, operation

    def _published_requires_token(self, operation):
        """Read what this operation publishes about needing a token.

        Read the way a client reads it. An absent ``security`` inherits the
        document-level requirement, which this gateway emits whenever
        ``auth.enabled`` is on, so absent means *required*; the empty list is
        OpenAPI's explicit override for "reachable with no token".
        """
        requirement = operation.get('security')
        if requirement is None:
            return bool(self._spec.get('security'))
        return requirement != []

    @classmethod
    def _probe(cls, method, path):
        """Send one unauthenticated request at a documented path template."""
        url = cls.base_url + path
        while '{' in url:
            start = url.index('{')
            end = url.index('}', start)
            url = url[:start] + PROBE_ID + url[end + 1:]
        kwargs = {'timeout': 15}
        if method in ('post', 'put', 'patch'):
            kwargs['json'] = {}
        return cls.session.request(method.upper(), url, **kwargs)

    # -- the contract ------------------------------------------------------

    def test_a_published_token_requirement_is_one_the_gateway_enforces(self):
        """Publish a token requirement exactly where the middleware demands one.

        Both directions are defects and both are reported. Publishing where
        nothing is enforced sends a caller looking for credentials it does not
        need; not publishing where the middleware refuses leaves a caller with
        no way to learn why.
        """
        mismatched = sorted(
            f'{method.upper()} {path}: published='
            f'{self._published_requires_token(op)} enforced={enforced}'
            for (path, method), enforced in self.enforcement.items()
            for op in [self._spec['paths'][path][method]]
            if self._published_requires_token(op) != enforced
        )
        self.assertEqual(
            mismatched, [],
            f'under require_auth_for={self.POLICY!r}, '
            f'{len(mismatched)} operations publish a token requirement the '
            f'middleware does not match: {mismatched[:20]}')

    def test_the_middleware_never_claims_a_refusal_it_will_not_make(self):
        """The `Unauthorized`/`Forbidden` components appear only where enforced.

        Keyed on the component rather than on the status, and that is the
        point. A route may answer its own 401 or its own 403 for reasons that
        have nothing to do with authentication - a refresh token the handler
        rejects, a provider refusing a transition, a lock held by another
        client - and those shadow the middleware component by design
        (`route_registry.cpp`, first-wins). Reading the status alone would call
        every one of them a middleware claim.
        """
        overclaimed = sorted(
            f'{method.upper()} {path}: {status}'
            for (path, method), enforced in self.enforcement.items()
            if not enforced
            for status, component in (('401', MIDDLEWARE_401),
                                      ('403', MIDDLEWARE_403))
            if _response_ref(self._spec['paths'][path][method],
                             status) == component
        )
        self.assertEqual(
            overclaimed, [],
            f'under require_auth_for={self.POLICY!r}, '
            f'{len(overclaimed)} operation/status pairs name the middleware '
            f'refusal component on an operation the middleware admits: '
            f'{overclaimed[:20]}')

    def test_a_refused_operation_declares_both_refusal_statuses(self):
        """Where the middleware does refuse, the caller can read both statuses.

        Satisfied by the middleware's own component or by a route-declared
        response of the same status - the two are interchangeable to a caller
        reading which statuses an operation can answer, and only the first
        carries the middleware's headers.
        """
        undeclared = sorted(
            f'{method.upper()} {path}: {status}'
            for (path, method), enforced in self.enforcement.items()
            if enforced
            for status in ('401', '403')
            if status not in self._spec['paths'][path][method].get(
                'responses', {})
        )
        self.assertEqual(
            undeclared, [],
            f'under require_auth_for={self.POLICY!r}, '
            f'{len(undeclared)} operation/status pairs are reachable through '
            f'the middleware but undeclared: {undeclared[:20]}')

    def test_auth_endpoints_are_published_as_reachable_without_a_token(self):
        """`/auth/*` is exempt under every policy, and says so.

        Checked from the document rather than the wire: these handlers answer
        RFC 6749 errors themselves, in the body shape `_middleware_refused`
        reads, so a probe here cannot tell the two apart.
        """
        for path, method, operation in self._operations():
            if not path.startswith(AUTH_PATH_PREFIX):
                continue
            with self.subTest(path=path, method=method):
                self.assertEqual(
                    operation.get('security'), [],
                    f'{method.upper()} {path} is exempted by every policy but '
                    f'publishes {operation.get("security")!r}')
                self.assertNotEqual(_response_ref(operation, '401'),
                                    MIDDLEWARE_401)
                self.assertNotEqual(_response_ref(operation, '403'),
                                    MIDDLEWARE_403)

    def test_the_probe_covers_the_maximal_route_surface(self):
        """Anti-vacuous: the assertions above ran against a full gateway.

        The fixture turns every optional gate on, so a document that had
        collapsed to the handful of always-on endpoints - or a probe that
        skipped everything - would leave the loops above passing on nothing.
        """
        self.assertGreater(
            len(self.enforcement), 200,
            f'only {len(self.enforcement)} operations probed under '
            f'{self.POLICY!r}')

    def test_a_scoped_docs_item_publishes_the_role_of_the_route_it_names(self):
        """The `<entity-path>/docs` sub-documents follow the policy too.

        `test_openapi_contract` compares a built item against its templated
        sibling, but its fixture runs with authentication off, so `security` is
        `None` on both sides there and the comparison passes without ever
        testing the dimension it was written for. This is the only place in the
        suite that fetches a scoped sub-document from an authenticated gateway,
        and it runs once per `require_auth_for` value.

        `x-sovd-name` marks a built item; the projection at the same key has
        none. Without that check a discarded item would leave the projection
        behind and every assertion here would pass on it.
        """
        built = 0
        offenders = []
        for entity_type in ('apps', 'components'):
            listing = self.session.get(f'{self.base_url}/{entity_type}',
                                       headers={'Authorization':
                                                f'Bearer {self.token}'},
                                       timeout=15)
            if listing.status_code != 200:
                continue
            for entity in listing.json().get('items', []):
                for collection, parameter in (('data', 'data_id'),
                                              ('operations', 'operation_id')):
                    doc = self._authed_json(
                        f'/{entity_type}/{entity["id"]}/{collection}/docs')
                    if doc is None:
                        continue
                    paths = doc.get('paths', {})
                    template = (f'/{entity_type}/{entity["id"]}/{collection}/'
                                f'{{{parameter}}}')
                    sibling = paths.get(template, {})
                    for key, path_item in paths.items():
                        if 'x-sovd-name' not in path_item:
                            continue
                        built += 1
                        for method, operation in path_item.items():
                            if method not in HTTP_METHODS:
                                continue
                            expected = sibling.get(method, {}).get('security')
                            if operation.get('security') != expected:
                                offenders.append(
                                    f'{self.POLICY}: {method.upper()} {key} '
                                    f'security {operation.get("security")} != '
                                    f'{expected}')
        self.assertEqual(
            offenders, [],
            f'built items disagreeing with their route: {offenders[:10]}')
        self.assertGreater(
            built, 0,
            f'under {self.POLICY!r} no scoped sub-document published a '
            f'cache-derived item; this test checked nothing')

    def _authed_json(self, path):
        """GET `path` with this class's token, or None when not served."""
        response = self.session.get(
            f'{self.base_url}{path}',
            headers={'Authorization': f'Bearer {self.token}'}, timeout=15)
        if response.status_code != 200:
            return None
        return response.json()


class TestAuthPolicyAll(AuthPolicyContractBase, unittest.TestCase):
    """`require_auth_for: all` - every path outside `/auth/*` is checked."""

    POLICY = 'all'

    def test_every_probed_operation_is_enforced(self):
        """The policy's own behaviour, stated rather than inferred.

        Every other assertion in this file is a consistency check, and
        consistency cannot notice a policy change: a policy that stopped
        enforcing anything would move the document with it and leave the suite
        green. These three tests are the second opinion.
        """
        unenforced = sorted(f'{m.upper()} {p}'
                            for (p, m), e in self.enforcement.items()
                            if not e)
        self.assertEqual(unenforced, [], f'not refused under `all`: '
                                         f'{unenforced[:20]}')


class TestAuthPolicyWrite(AuthPolicyContractBase, unittest.TestCase):
    """`require_auth_for: write` - writes are checked, reads are not."""

    POLICY = 'write'

    def test_reads_are_open_and_writes_are_checked(self):
        """The split this policy exists for, measured on the wire."""
        by_method = {}
        for (path, method), enforced in self.enforcement.items():
            by_method.setdefault(method, set()).add(enforced)
        self.assertEqual(by_method.get('get'), {False},
                         'a GET was refused under `write`')
        for method in ('post', 'put', 'patch', 'delete'):
            if method in by_method:
                self.assertEqual(
                    by_method[method], {True},
                    f'a {method.upper()} was admitted under `write`')
        # Both halves non-empty, or the assertions above are vacuous.
        self.assertIn('get', by_method)
        self.assertTrue(by_method.keys() & {'post', 'put', 'patch', 'delete'})


class TestAuthPolicyNone(AuthPolicyContractBase, unittest.TestCase):
    """`require_auth_for: none` - `auth.enabled` on, nothing checked."""

    POLICY = 'none'

    def test_no_probed_operation_is_enforced(self):
        """Authentication configured and issuing tokens, enforcing nothing."""
        enforced = sorted(f'{m.upper()} {p}'
                          for (p, m), e in self.enforcement.items() if e)
        self.assertEqual(enforced, [], f'refused under `none`: {enforced[:20]}')

    def test_tokens_are_still_issued(self):
        """`none` disables enforcement, not the auth endpoints themselves.

        Without this the class above would also pass on a gateway that had
        turned authentication off altogether, which is a different
        configuration with a different document (no document-level
        requirement at all).
        """
        self.assertTrue(self.token)
        self.assertTrue(self._spec.get('security'),
                        'auth.enabled is on, so the document keeps its '
                        'document-level security requirement')


@launch_testing.post_shutdown_test()
class TestAuthPolicyShutdown(unittest.TestCase):
    """All three gateways exit cleanly."""

    def test_exit_codes(self, proc_info):
        """Check that every gateway process exited with an allowed code."""
        launch_testing.asserts.assertExitCodes(
            proc_info, allowable_exit_codes=ALLOWED_EXIT_CODES)
