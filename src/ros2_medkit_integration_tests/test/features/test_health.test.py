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

"""Feature tests for health, version, root, and docs endpoints.

Validates the gateway's informational endpoints that do not depend on
discovered entities. Only a single lightweight demo node (temp_sensor)
is launched so the test starts quickly.

"""

import unittest

import launch_testing
import launch_testing.actions

from ros2_medkit_test_utils.constants import ALLOWED_EXIT_CODES, API_BASE_PATH
from ros2_medkit_test_utils.gateway_test_case import GatewayTestCase
from ros2_medkit_test_utils.launch_helpers import create_test_launch


def generate_test_description():
    return create_test_launch(
        demo_nodes=['temp_sensor'],
        fault_manager=False,
    )


class TestHealth(GatewayTestCase):
    """Health, version, root endpoint, and docs feature tests."""

    MIN_EXPECTED_APPS = 1
    REQUIRED_APPS = {'temp_sensor'}

    def test_health_endpoint_returns_200(self):
        """GET /health returns 200 with status field.

        @verifies REQ_INTEROP_010
        """
        data = self.get_json('/health')
        self.assertIn('status', data)

    def test_root_endpoint_returns_api_info(self):
        """GET / returns server capabilities and entry points.

        @verifies REQ_INTEROP_010
        """
        data = self.get_json('/')
        self.assertIn('name', data)
        self.assertIn('version', data)
        self.assertIn('endpoints', data)
        self.assertIn('capabilities', data)

        self.assertEqual(data['name'], 'ROS 2 Medkit Gateway')
        self.assertRegex(
            data['version'],
            r'^\d+\.\d+\.\d+$',
            f'Version should be semver format, got: {data["version"]}'
        )

        # Verify endpoints list
        self.assertIsInstance(data['endpoints'], list)
        self.assertIn('GET /api/v1/health', data['endpoints'])
        self.assertIn('GET /api/v1/version-info', data['endpoints'])
        self.assertIn('GET /api/v1/areas', data['endpoints'])
        self.assertIn('GET /api/v1/components', data['endpoints'])
        self.assertIn(
            'PUT /api/v1/components/{component_id}/data/{data_id}', data['endpoints']
        )

        # Verify api_base field
        self.assertIn('api_base', data)
        self.assertEqual(data['api_base'], API_BASE_PATH)

        # Verify capabilities
        self.assertIn('discovery', data['capabilities'])
        self.assertIn('data_access', data['capabilities'])
        self.assertTrue(data['capabilities']['discovery'])
        self.assertTrue(data['capabilities']['data_access'])

    def test_version_endpoint(self):
        """GET /version-info returns valid format and data.

        @verifies REQ_INTEROP_001
        """
        data = self.get_json('/version-info')

        # Check items array (SOVD-standard wrapper key)
        self.assertIn('items', data)
        self.assertIsInstance(data['items'], list)
        self.assertGreaterEqual(len(data['items']), 1)

        # Check first items entry
        info = data['items'][0]
        self.assertIn('version', info)
        self.assertIn('base_uri', info)
        self.assertIn('vendor_info', info)
        self.assertIn('version', info['vendor_info'])
        self.assertIn('name', info['vendor_info'])
        self.assertEqual(info['vendor_info']['name'], 'ros2_medkit')

    def test_root_includes_apps_endpoints(self):
        """GET / includes apps endpoints in the endpoints list.

        @verifies REQ_INTEROP_010
        """
        data = self.get_json('/')
        self.assertIn('endpoints', data)

        endpoints = data['endpoints']
        self.assertIn('GET /api/v1/apps', endpoints)
        self.assertIn('GET /api/v1/apps/{app_id}', endpoints)
        self.assertIn('GET /api/v1/apps/{app_id}/is-located-on', endpoints)
        self.assertIn('GET /api/v1/apps/{app_id}/depends-on', endpoints)
        self.assertIn('GET /api/v1/apps/{app_id}/data', endpoints)
        self.assertIn('GET /api/v1/apps/{app_id}/operations', endpoints)
        self.assertIn('GET /api/v1/apps/{app_id}/configurations', endpoints)

    def test_docs_endpoint(self):
        """GET /docs returns OpenAPI 3.1.0 spec.

        @verifies REQ_INTEROP_002
        """
        data = self.poll_endpoint_until(
            '/docs',
            lambda d: d if 'openapi' in d else None,
        )
        self.assertEqual(data['openapi'], '3.1.0')
        self.assertIn('info', data)
        self.assertIn('paths', data)

    def test_docs_spec_completeness(self):
        """Verify OpenAPI spec has response schemas, request bodies, and named types.

        Validates that every endpoint has proper type contracts for client
        code generation - no bare "Successful response" without schema.

        @verifies REQ_INTEROP_002
        """
        spec = self.poll_endpoint_until(
            '/docs',
            lambda d: d if d.get('paths') else None,
        )

        # Must have named component schemas
        schemas = spec.get('components', {}).get('schemas', {})
        self.assertGreater(len(schemas), 30, f'Expected 30+ component schemas, got {len(schemas)}')

        # Must have GenericError response component
        responses = spec.get('components', {}).get('responses', {})
        self.assertIn('GenericError', responses)

        # Must have global tags matching all used tags
        global_tags = {t['name'] for t in spec.get('tags', [])}
        used_tags = set()
        for path_item in spec.get('paths', {}).values():
            for op in path_item.values():
                if isinstance(op, dict):
                    for t in op.get('tags', []):
                        used_tags.add(t)
        missing = used_tags - global_tags
        self.assertEqual(missing, set(), f'Tags used but not globally defined: {missing}')

        # Every non-DELETE operation must have a response with schema or $ref
        issues = []
        for path, path_item in spec.get('paths', {}).items():
            for method, op in path_item.items():
                if not isinstance(op, dict) or method == 'delete':
                    continue
                op_id = op.get('operationId', f'{method} {path}')
                has_schema = False
                for code, resp in op.get('responses', {}).items():
                    if not code.startswith('2'):
                        continue
                    # 204 No Content never has a body
                    if code == '204':
                        has_schema = True
                    # 202 Accepted without body is OK (async operations)
                    if code == '202' and 'content' not in resp:
                        has_schema = True
                    if '$ref' in resp:
                        has_schema = True
                    elif 'content' in resp:
                        for media_type, ct in resp['content'].items():
                            if 'schema' in ct:
                                has_schema = True
                            # A non-JSON body is fully described by its media
                            # type. `format: binary` is an OpenAPI 3.0 idiom
                            # 3.1 dropped, and the SSE families put three
                            # different shapes in `data:`, so there is nothing
                            # truthful to put in a schema here - the absence is
                            # the declaration. Deliberately keyed on the media
                            # type and not on 'content' being present: a JSON
                            # response still owes a schema, which is the
                            # coverage this rule exists for.
                            elif media_type != 'application/json':
                                has_schema = True
                # An operation that declares no 2xx at all cannot return a
                # success body to describe - the data-categories / data-groups
                # stubs declare only their 501. Deliberately narrow: an
                # operation that DOES declare a 2xx still owes a schema, which
                # is the coverage this rule exists for.
                declared = op.get('responses', {})
                if declared and not any(code.startswith('2') for code in declared):
                    has_schema = True
                if not has_schema:
                    issues.append(f'{op_id}: no response schema')
        self.assertEqual(issues, [], f'Operations missing response schema: {issues}')

        # operationIds must be unique
        op_ids = []
        for path_item in spec.get('paths', {}).values():
            for op in path_item.values():
                if isinstance(op, dict) and 'operationId' in op:
                    op_ids.append(op['operationId'])
        duplicates = [oid for oid in op_ids if op_ids.count(oid) > 1]
        self.assertEqual(duplicates, [], f'Duplicate operationIds: {set(duplicates)}')

        # Error responses must use $ref (not inline)
        for path, path_item in spec.get('paths', {}).items():
            for method, op in path_item.items():
                if not isinstance(op, dict):
                    continue
                for code in ('400', '404', '500'):
                    resp = op.get('responses', {}).get(code)
                    if resp and '$ref' not in resp and 'description' in resp:
                        self.fail(
                            f'{op.get("operationId", method + " " + path)} '
                            f'{code} response should use $ref to GenericError'
                        )


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        """Check all processes exited cleanly (SIGTERM allowed)."""
        for info in proc_info:
            self.assertIn(
                info.returncode, ALLOWED_EXIT_CODES,
                f'{info.process_name} exited with code {info.returncode}'
            )
