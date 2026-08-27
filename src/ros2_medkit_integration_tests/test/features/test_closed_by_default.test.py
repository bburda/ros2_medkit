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
Every route refuses an uncredentialed request under the shipped defaults.

The gateway's own half of the closed-door acceptance. It does not check
configuration values: it asks the RUNNING gateway for its route table and then
probes every route in it. A test that asserted `require_auth_for == "all"`
would keep passing the day a route is registered outside the policy, which is
the failure this is here to catch.

The route table comes from RouteRegistry via `GET /api/v1/`, so a route added
next year is swept the day it is registered, with nothing here to update.

Two exemptions, and both are named with their reason in EXEMPT below.

@verifies REQ_INTEROP_086, REQ_INTEROP_087
"""

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
from ros2_medkit_test_utils.gateway_test_case import GatewayTestCase
from ros2_medkit_test_utils.launch_helpers import create_gateway_node

CLOSED_PORT = get_test_port()
CLOSED_BASE_URL = f'http://127.0.0.1:{CLOSED_PORT}{API_BASE_PATH}'
CLOSED_ROOT = f'http://127.0.0.1:{CLOSED_PORT}'

# At least 32 characters, or the gateway refuses to start under HS256.
JWT_SECRET = 'closed_by_default_integration_secret_key_0123456789'
CLIENT_ID = 'diagbox'
CLIENT_SECRET = 'diagbox_client_secret'

# A path parameter is filled with an id that exists on no gateway. A route that
# refuses for a nonexistent id refuses for a real one, and a probe that turns
# out to reach an OPEN route cannot mutate anything real.
PROBE_ID = 'closed-by-default-probe'


@pytest.mark.launch_test
def generate_test_description():
    """Gateway with the posture the shipped default config carries."""
    gateway_node = create_gateway_node(
        port=CLOSED_PORT,
        extra_params={
            'server.host': '127.0.0.1',
            # These three are what config/gateway_params.yaml now ships. The
            # secret and client cannot come from that file (a committed secret
            # is a secret every deployment shares), so they are supplied here
            # the way a deployment supplies them.
            'auth.enabled': True,
            'auth.require_auth_for': 'all',
            'auth.issuer': 'ros2_medkit_gateway',
            'auth.jwt_secret': JWT_SECRET,
            'auth.clients': [f'{CLIENT_ID}:{CLIENT_SECRET}:admin'],
        },
    )

    return launch.LaunchDescription([
        gateway_node,
        launch_testing.actions.ReadyToTest(),
    ]), {'gateway_node': gateway_node}


def _is_exempt(method, path):
    """Routes that are deliberately reachable without a credential.

    GET /health - a container supervisor and an upstream load balancer probe
    it to decide whether this process is alive, and neither holds a
    credential; requiring one turns a healthy gateway into a restart loop. The
    body is a fixed status document: no entity names, no topics, no data.

    /auth/* - token issuance. Authentication cannot bootstrap through a door
    that already demands the credential it exists to hand out.
    """
    if method == 'GET' and path == f'{API_BASE_PATH}/health':
        return True
    return path.startswith(f'{API_BASE_PATH}/auth/')


class TestClosedByDefault(GatewayTestCase):
    """The gateway refuses every route it serves, bar the named exemptions."""

    BASE_URL = CLOSED_BASE_URL

    @classmethod
    def setUpClass(cls):
        super().setUpClass()
        resp = requests.post(
            f'{CLOSED_BASE_URL}/auth/authorize',
            json={
                'grant_type': 'client_credentials',
                'client_id': CLIENT_ID,
                'client_secret': CLIENT_SECRET,
            },
            timeout=10,
        )
        assert resp.status_code == 200, f'could not obtain a token: {resp.status_code} {resp.text}'
        cls.token = resp.json()['access_token']
        cls.auth = {'Authorization': f'Bearer {cls.token}'}

        # The route table, read from the gateway itself. A hardened gateway
        # does not list its routes anonymously, so this read authenticates.
        root = requests.get(f'{CLOSED_BASE_URL}/', headers=cls.auth, timeout=10)
        assert root.status_code == 200, f'route table unreadable: {root.status_code}'
        cls.endpoints = root.json().get('endpoints', [])
        assert cls.endpoints, 'gateway reported no endpoints - nothing would be proven'

    @staticmethod
    def _fill(path):
        out, depth = [], 0
        for ch in path:
            if ch == '{':
                depth += 1
                if depth == 1:
                    out.append(PROBE_ID)
            elif ch == '}':
                depth -= 1
            elif depth == 0:
                out.append(ch)
        return ''.join(out)

    def test_01_route_table_is_substantial(self):
        """A sweep over three routes would prove almost nothing."""
        self.assertGreater(
            len(self.endpoints), 50,
            f'expected the full gateway surface, got {len(self.endpoints)} routes'
        )

    def test_02_no_route_answers_without_a_credential(self):
        """Sweep EVERY registered route. This is the acceptance."""
        answered = []
        for entry in self.endpoints:
            method, _, raw = entry.partition(' ')
            if not raw:
                continue
            if _is_exempt(method, raw):
                continue
            path = self._fill(raw)
            # A write method needs a body: without Content-Length the server
            # waits for one that never arrives and the probe times out with no
            # status, measuring nothing at all.
            kwargs = {'timeout': 15}
            if method in ('POST', 'PUT', 'PATCH'):
                kwargs['json'] = {}
            try:
                resp = requests.request(method, f'{CLOSED_ROOT}{path}', **kwargs)
            except requests.RequestException as exc:
                answered.append(f'{method} {path} -> transport error {exc}')
                continue
            # 401/403 only. A 404 is an ANSWER: the gateway parsed the request
            # and told an anonymous caller what does not exist here.
            if resp.status_code not in (401, 403):
                answered.append(f'{method} {path} -> {resp.status_code}')

        self.assertEqual(
            [], answered,
            'these routes answered an uncredentialed request:\n  '
            + '\n  '.join(answered)
        )

    def test_03_a_wrong_credential_is_refused_everywhere(self):
        """A token this gateway never issued gets no further than none at all."""
        bad = {'Authorization': 'Bearer not.a.real.token'}
        answered = []
        for entry in self.endpoints:
            method, _, raw = entry.partition(' ')
            if not raw or _is_exempt(method, raw):
                continue
            path = self._fill(raw)
            kwargs = {'timeout': 15, 'headers': bad}
            if method in ('POST', 'PUT', 'PATCH'):
                kwargs['json'] = {}
            try:
                resp = requests.request(method, f'{CLOSED_ROOT}{path}', **kwargs)
            except requests.RequestException as exc:
                answered.append(f'{method} {path} -> transport error {exc}')
                continue
            if resp.status_code not in (401, 403):
                answered.append(f'{method} {path} -> {resp.status_code}')

        self.assertEqual(
            [], answered,
            'these routes accepted a forged credential:\n  ' + '\n  '.join(answered)
        )

    def test_04_reads_are_refused_not_just_writes(self):
        """The require_auth_for="write" hole, pinned directly.

        Under "write" every one of these answers 200 to an anonymous caller,
        and they are the disclosure: the entity tree names the machines.
        """
        for path in ('/', '/areas', '/components', '/apps', '/functions', '/version-info'):
            with self.subTest(path=path):
                resp = requests.get(f'{CLOSED_BASE_URL}{path}', timeout=15)
                self.assertIn(resp.status_code, (401, 403))

    def test_05_the_exempt_health_route_still_answers(self):
        """The mirror of the sweeps.

        Without this, a gateway that refused EVERYTHING would pass every test
        above while being uniformly broken - the container supervisor could
        not tell it was alive, and it would restart forever.
        """
        resp = requests.get(f'{CLOSED_BASE_URL}/health', timeout=15)
        self.assertEqual(resp.status_code, 200, 'health must stay probe-able')

    def test_06_anonymous_health_is_liveness_and_nothing_else(self):
        """The exemption rests on the body saying nothing, so check the body.

        An allowlist, not a denylist. Listing the fields known to leak today
        would pass the day a new section is added, and the whole reason this
        route is public is that a probe needs no more than "am I alive".
        """
        body = requests.get(f'{CLOSED_BASE_URL}/health', timeout=15).json()
        # warnings and its schema version are always serialised - they are part
        # of the /health contract whether or not anything produced one - so the
        # allowlist includes them and the assertion below covers the content.
        self.assertEqual(
            set(body), {'status', 'timestamp', 'warnings', 'warning_schema_version'},
            f'an anonymous /health returned more than liveness: {body}'
        )
        self.assertEqual(body['status'], 'healthy')
        # The array is the leak vector: a linking warning reads like
        # "App 'engine_ecu' cannot bind to '/nav/controller'", naming an entity
        # and a ROS node FQN. Empty is the only safe value for a caller that
        # presented nothing.
        self.assertEqual(
            body['warnings'], [],
            f'an anonymous /health carried warning text: {body["warnings"]}'
        )

    def test_06b_the_full_health_document_names_entities(self):
        """The reason test_06 matters, pinned so it cannot be argued away.

        With a credential the same route returns discovery state and entity
        cache counts. That is a legitimate operator surface, and it is exactly
        what an anonymous caller must not receive - so if this ever stops being
        true, the narrowing above has become pointless and should be revisited
        rather than left as dead weight.
        """
        body = requests.get(
            f'{CLOSED_BASE_URL}/health', headers=self.auth, timeout=15
        ).json()
        self.assertIn('discovery', body)
        self.assertIn('x-medkit-entity-cache', body)
        self.assertGreater(
            len(set(body)), 2,
            'the authenticated /health is now as bare as the anonymous one'
        )

    def test_06c_a_forged_credential_gets_the_bare_body(self):
        """A token this gateway never issued is an anonymous caller."""
        body = requests.get(
            f'{CLOSED_BASE_URL}/health',
            headers={'Authorization': 'Bearer not.a.real.token'},
            timeout=15,
        ).json()
        self.assertEqual(
            set(body), {'status', 'timestamp', 'warnings', 'warning_schema_version'},
            f'a forged credential unlocked the full health document: {body}'
        )
        self.assertEqual(body['warnings'], [])

    def test_07_a_valid_credential_gets_through(self):
        """Otherwise the sweeps above would pass on a gateway that serves nobody."""
        resp = requests.get(f'{CLOSED_BASE_URL}/areas', headers=self.auth, timeout=15)
        self.assertEqual(resp.status_code, 200)

    def test_08_head_on_health_is_not_a_way_in(self):
        """The exemption is GET-only, deliberately.

        Widening it to "any method on /health" is the natural next edit and it
        would be wrong, so the narrowness is pinned here.
        """
        # HEAD first, and separately, because it is the method that matters and
        # the one an earlier version of this test left out despite its name.
        # cpp-httplib dispatches HEAD into the GET handler table, so if the
        # exemption stopped checking the method, HEAD would return the status
        # document to an anonymous caller. The others have no handler at all on
        # this path, so they answer 404 either way and cannot show the
        # difference on their own.
        head = requests.head(f'{CLOSED_BASE_URL}/health', timeout=15)
        self.assertIn(
            head.status_code, (401, 403),
            f'HEAD /health answered {head.status_code} to an anonymous caller'
        )

        for method in ('POST', 'PUT', 'DELETE', 'PATCH'):
            with self.subTest(method=method):
                resp = requests.request(
                    method, f'{CLOSED_BASE_URL}/health', json={}, timeout=15
                )
                self.assertNotEqual(
                    resp.status_code, 200,
                    f'{method} /health answered 200 to an anonymous caller'
                )


@launch_testing.post_shutdown_test()
class TestClosedByDefaultShutdown(unittest.TestCase):
    """Gateway exits cleanly."""

    def test_exit_codes(self, proc_info, gateway_node):
        launch_testing.asserts.assertExitCodes(
            proc_info, allowable_exit_codes=ALLOWED_EXIT_CODES, process=gateway_node
        )
