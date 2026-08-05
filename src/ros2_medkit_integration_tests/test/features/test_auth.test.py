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
Integration tests for JWT authentication and authorization.

Launches the gateway on port 8085 with auth enabled (JWT HS256, four client
roles). Tests authentication flows (authorize, refresh, revoke) and RBAC
enforcement (viewer, operator, configurator, admin).

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
from ros2_medkit_test_utils.launch_helpers import (
    create_gateway_node,
    graph_provider_params,
)

AUTH_PORT = get_test_port()
AUTH_BASE_URL = f'http://127.0.0.1:{AUTH_PORT}{API_BASE_PATH}'


@pytest.mark.launch_test
def generate_test_description():
    """Generate launch description with gateway node with auth enabled."""
    gateway_node = create_gateway_node(
        port=AUTH_PORT,
        extra_params={
            'server.host': '127.0.0.1',
            'auth.enabled': True,
            'auth.jwt_secret': 'test_secret_key_for_jwt_signing_integration_test_12345',
            'auth.jwt_algorithm': 'HS256',
            'auth.token_expiry_seconds': 3600,
            'auth.refresh_token_expiry_seconds': 86400,
            'auth.require_auth_for': 'write',
            'auth.issuer': 'test_gateway',
            'auth.clients': [
                'admin:admin_secret:admin',
                'operator:operator_secret:operator',
                'viewer:viewer_secret:viewer',
                'configurator:configurator_secret:configurator',
            ],
            # The auth-on half of the per-operation `security` pair. This is
            # the only fixture in the suite with authentication enabled AND a
            # plugin that describes a route, so it is the only place the
            # published-requirement direction can be driven.
            **graph_provider_params(),
        },
    )

    return launch.LaunchDescription([
        gateway_node,
        launch_testing.actions.ReadyToTest(),
    ]), {'gateway_node': gateway_node}


class TestAuthenticationIntegration(GatewayTestCase):
    """Integration tests for authentication endpoints."""

    BASE_URL = AUTH_BASE_URL

    def test_01_health_endpoint_no_auth_required(self):
        """Health endpoint should be accessible without authentication."""
        response = requests.get(f'{self.BASE_URL}/health', timeout=5)
        self.assertEqual(response.status_code, 200)
        data = response.json()
        self.assertEqual(data['status'], 'healthy')

    def test_02_root_endpoint_shows_auth_enabled(self):
        """Root endpoint should indicate auth is enabled."""
        response = requests.get(f'{self.BASE_URL}/', timeout=5)
        self.assertEqual(response.status_code, 200)
        data = response.json()
        self.assertTrue(data['capabilities']['authentication'])
        self.assertIn('auth', data)
        self.assertTrue(data['auth']['enabled'])
        self.assertEqual(data['auth']['algorithm'], 'HS256')

    def test_02b_plugin_operation_publishes_its_role_when_auth_is_on(self):
        """With auth enabled the document publishes the role an operation needs.

        The other half of this pair is
        ``test_openapi_contract::test_no_operation_publishes_a_role_when_auth_is_off``:
        the same gateway code, the same plugin, the opposite ``auth.enabled``.
        Splitting it across two fixtures is not a convenience - a single
        gateway has one value of ``auth.enabled``, and the whole point of the
        rule is that the document follows it.

        The role is ``admin`` because ``AuthConfig``'s ``*`` matches a single
        path segment, so no ``viewer`` entry under ``/functions/*`` reaches
        this collection and only ADMIN's ``GET:/api/v1/**`` does.
        """
        spec = requests.get(f'{self.BASE_URL}/docs', timeout=10).json()
        op = spec['paths'].get(
            '/functions/{function_id}/x-medkit-graph', {}).get('get')
        self.assertIsNotNone(
            op, 'the graph provider route is not documented; the fixture must '
                'load the plugin for this test to mean anything')
        self.assertEqual(op.get('security'), [{'bearerAuth': ['admin']}])
        # A requirement may only name a scheme the document defines.
        self.assertIn(
            'bearerAuth',
            spec.get('components', {}).get('securitySchemes', {}))

    def test_03_authenticate_valid_credentials(self):
        """@verifies REQ_INTEROP_086 - Authentication with valid credentials."""
        response = requests.post(
            f'{self.BASE_URL}/auth/authorize',
            json={
                'grant_type': 'client_credentials',
                'client_id': 'admin',
                'client_secret': 'admin_secret',
            },
            timeout=5,
        )
        self.assertEqual(response.status_code, 200)
        data = response.json()
        self.assertIn('access_token', data)
        self.assertIn('refresh_token', data)
        self.assertEqual(data['token_type'], 'Bearer')
        self.assertEqual(data['scope'], 'admin')
        self.assertEqual(data['expires_in'], 3600)

    def test_04_authenticate_invalid_credentials(self):
        """Authentication with invalid credentials should fail."""
        response = requests.post(
            f'{self.BASE_URL}/auth/authorize',
            json={
                'grant_type': 'client_credentials',
                'client_id': 'admin',
                'client_secret': 'wrong_secret',
            },
            timeout=5,
        )
        self.assertEqual(response.status_code, 401)
        data = response.json()
        self.assertEqual(data['error'], 'invalid_client')

    def test_05_authenticate_unknown_client(self):
        """Authentication with unknown client should fail."""
        response = requests.post(
            f'{self.BASE_URL}/auth/authorize',
            json={
                'grant_type': 'client_credentials',
                'client_id': 'unknown_client',
                'client_secret': 'some_secret',
            },
            timeout=5,
        )
        self.assertEqual(response.status_code, 401)
        data = response.json()
        self.assertEqual(data['error'], 'invalid_client')

    def test_06_authenticate_form_urlencoded(self):
        """@verifies REQ_INTEROP_086 - Authentication with form-urlencoded."""
        response = requests.post(
            f'{self.BASE_URL}/auth/authorize',
            data='grant_type=client_credentials&client_id=viewer&client_secret=viewer_secret',
            headers={'Content-Type': 'application/x-www-form-urlencoded'},
            timeout=5,
        )
        self.assertEqual(response.status_code, 200)
        data = response.json()
        self.assertEqual(data['scope'], 'viewer')

    def test_07_refresh_token(self):
        """@verifies REQ_INTEROP_087 - Token refresh flow."""
        # First, authenticate to get tokens
        auth_response = requests.post(
            f'{self.BASE_URL}/auth/authorize',
            json={
                'grant_type': 'client_credentials',
                'client_id': 'operator',
                'client_secret': 'operator_secret',
            },
            timeout=5,
        )
        self.assertEqual(auth_response.status_code, 200)
        tokens = auth_response.json()
        refresh_token = tokens['refresh_token']

        # Use refresh token to get new access token
        refresh_response = requests.post(
            f'{self.BASE_URL}/auth/token',
            json={
                'grant_type': 'refresh_token',
                'refresh_token': refresh_token,
            },
            timeout=5,
        )
        self.assertEqual(refresh_response.status_code, 200)
        new_tokens = refresh_response.json()
        self.assertIn('access_token', new_tokens)
        self.assertNotEqual(new_tokens['access_token'], tokens['access_token'])
        self.assertEqual(new_tokens['scope'], 'operator')

    def test_08_refresh_invalid_token(self):
        """Refresh with invalid token should fail."""
        response = requests.post(
            f'{self.BASE_URL}/auth/token',
            json={
                'grant_type': 'refresh_token',
                'refresh_token': 'invalid_token',
            },
            timeout=5,
        )
        self.assertEqual(response.status_code, 401)
        data = response.json()
        self.assertEqual(data['error'], 'invalid_grant')

    def test_09_revoke_token(self):
        """Token revocation flow."""
        # First, authenticate
        auth_response = requests.post(
            f'{self.BASE_URL}/auth/authorize',
            json={
                'grant_type': 'client_credentials',
                'client_id': 'admin',
                'client_secret': 'admin_secret',
            },
            timeout=5,
        )
        tokens = auth_response.json()
        refresh_token = tokens['refresh_token']

        # Revoke the refresh token
        revoke_response = requests.post(
            f'{self.BASE_URL}/auth/revoke',
            json={'token': refresh_token},
            timeout=5,
        )
        self.assertEqual(revoke_response.status_code, 200)
        data = revoke_response.json()
        self.assertEqual(data['status'], 'revoked')

        # Try to use revoked refresh token
        refresh_response = requests.post(
            f'{self.BASE_URL}/auth/token',
            json={
                'grant_type': 'refresh_token',
                'refresh_token': refresh_token,
            },
            timeout=5,
        )
        self.assertEqual(refresh_response.status_code, 401)

    def test_10_unsupported_grant_type(self):
        """Unsupported grant type should fail."""
        response = requests.post(
            f'{self.BASE_URL}/auth/authorize',
            json={
                'grant_type': 'authorization_code',
                'client_id': 'admin',
                'client_secret': 'admin_secret',
            },
            timeout=5,
        )
        self.assertEqual(response.status_code, 400)
        data = response.json()
        self.assertEqual(data['error'], 'unsupported_grant_type')


class TestAuthorizationIntegration(GatewayTestCase):
    """Integration tests for RBAC authorization."""

    BASE_URL = AUTH_BASE_URL

    @classmethod
    def setUpClass(cls):
        """Wait for gateway to be ready and get tokens."""
        cls._wait_for_gateway_health()

        # Get tokens for different roles
        cls.tokens = {}
        for role, client_id, client_secret in [
            ('admin', 'admin', 'admin_secret'),
            ('operator', 'operator', 'operator_secret'),
            ('viewer', 'viewer', 'viewer_secret'),
            ('configurator', 'configurator', 'configurator_secret'),
        ]:
            response = requests.post(
                f'{cls.BASE_URL}/auth/authorize',
                json={
                    'grant_type': 'client_credentials',
                    'client_id': client_id,
                    'client_secret': client_secret,
                },
                timeout=5,
            )
            if response.status_code == 200:
                cls.tokens[role] = response.json()['access_token']

        missing = [r for r in ('admin', 'operator', 'viewer', 'configurator')
                   if r not in cls.tokens]
        if missing:
            raise AssertionError(
                f'Could not acquire tokens for: {", ".join(missing)}'
            )

    def _auth_header(self, role):
        """Get Authorization header for a role."""
        return {'Authorization': f'Bearer {self.tokens.get(role, "")}'}

    def test_01_read_endpoints_no_auth_required(self):
        """GET endpoints should work without authentication (require_auth_for=write)."""
        # These should work without auth
        endpoints = [
            '/health',
            '/version-info',
            '/areas',
            '/components',
        ]
        for endpoint in endpoints:
            response = requests.get(f'{self.BASE_URL}{endpoint}', timeout=5)
            self.assertEqual(
                response.status_code, 200,
                f'GET {endpoint} failed without auth: {response.status_code}'
            )

    def test_02_write_without_auth_returns_401(self):
        """Write operations without auth should return 401."""
        # Note: These endpoints may not exist but should still require auth
        response = requests.post(
            f'{self.BASE_URL}/components/test/operations/calibrate',
            json={},
            timeout=5,
        )
        self.assertEqual(response.status_code, 401)

        response = requests.put(
            f'{self.BASE_URL}/components/test/configurations/param',
            json={'value': 123},
            timeout=5,
        )
        self.assertEqual(response.status_code, 401)

    def test_03_write_with_invalid_token_returns_401(self):
        """Write operations with invalid token should return 401."""
        response = requests.post(
            f'{self.BASE_URL}/components/test/operations/calibrate',
            json={},
            headers={'Authorization': 'Bearer invalid_token'},
            timeout=5,
        )
        self.assertEqual(response.status_code, 401)

    def test_04_viewer_cannot_write(self):
        """Viewer role should not be able to perform write operations."""
        headers = self._auth_header('viewer')

        # POST operation should be forbidden
        response = requests.post(
            f'{self.BASE_URL}/components/test/operations/calibrate',
            json={},
            headers=headers,
            timeout=5,
        )
        self.assertEqual(response.status_code, 403)

        # PUT configuration should be forbidden
        response = requests.put(
            f'{self.BASE_URL}/components/test/configurations/param',
            json={'value': 123},
            headers=headers,
            timeout=5,
        )
        self.assertEqual(response.status_code, 403)

    def test_05_operator_can_trigger_operations(self):
        """Operator role should be able to trigger operations (auth passes)."""
        headers = self._auth_header('operator')

        # POST operation execution - auth should pass for operator role
        response = requests.post(
            f'{self.BASE_URL}/components/test_component/operations/test_op/executions',
            json={},
            headers=headers,
            timeout=5,
        )
        # Auth passes (not 401/403), entity doesn't exist -> 404
        self.assertEqual(
            response.status_code, 404,
            f'Operator should be authorized (not 401/403), got {response.status_code}'
        )

    def test_06_operator_cannot_modify_configurations(self):
        """Operator role should not be able to modify configurations."""
        headers = self._auth_header('operator')

        response = requests.put(
            f'{self.BASE_URL}/components/test/configurations/param',
            json={'value': 123},
            headers=headers,
            timeout=5,
        )
        self.assertEqual(response.status_code, 403)

    def test_07_configurator_can_modify_configurations(self):
        """Configurator role should be able to modify configurations (auth passes)."""
        headers = self._auth_header('configurator')

        # PUT configuration - auth should pass for configurator role
        response = requests.put(
            f'{self.BASE_URL}/components/test_component/configurations/param',
            json={'value': 123},
            headers=headers,
            timeout=5,
        )
        # Auth passes (not 401/403), entity doesn't exist -> 404
        self.assertEqual(
            response.status_code, 404,
            f'Configurator should be authorized (not 401/403), got {response.status_code}'
        )

    def test_08_admin_has_full_access(self):
        """Admin role should have access to all operations."""
        headers = self._auth_header('admin')

        # All write operations should be authorized
        endpoints = [
            ('POST', '/components/test/operations/calibrate', {}),
            ('PUT', '/components/test/configurations/param', {'value': 123}),
            ('DELETE', '/components/test/faults/F001', None),
        ]

        for method, endpoint, body in endpoints:
            if method == 'POST':
                response = requests.post(
                    f'{self.BASE_URL}{endpoint}',
                    json=body,
                    headers=headers,
                    timeout=5,
                )
            elif method == 'PUT':
                response = requests.put(
                    f'{self.BASE_URL}{endpoint}',
                    json=body,
                    headers=headers,
                    timeout=5,
                )
            elif method == 'DELETE':
                response = requests.delete(
                    f'{self.BASE_URL}{endpoint}',
                    headers=headers,
                    timeout=5,
                )

            # Auth passes (not 401/403), entity doesn't exist -> 404
            self.assertEqual(
                response.status_code, 404,
                f'{method} {endpoint}: expected 404 (auth passed, entity not found), '
                f'got {response.status_code}'
            )

    def test_09_www_authenticate_header_on_401(self):
        """401 responses should include WWW-Authenticate header."""
        response = requests.post(
            f'{self.BASE_URL}/components/test/operations/calibrate',
            json={},
            timeout=5,
        )
        self.assertEqual(response.status_code, 401)
        self.assertIn('WWW-Authenticate', response.headers)
        self.assertIn('Bearer', response.headers['WWW-Authenticate'])


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
