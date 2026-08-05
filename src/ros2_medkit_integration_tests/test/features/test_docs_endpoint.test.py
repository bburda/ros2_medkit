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

"""Feature tests for GET /{any-path}/docs capability description endpoint.

Validates the OpenAPI 3.1.0 spec generation for root, entity collections,
specific entities, and resource collections. Verifies error handling for
nonexistent paths.

"""

import unittest

import launch_testing
import launch_testing.actions
import requests

from ros2_medkit_test_utils.constants import ALLOWED_EXIT_CODES
from ros2_medkit_test_utils.gateway_test_case import GatewayTestCase
from ros2_medkit_test_utils.launch_helpers import create_test_launch


def generate_test_description():
    return create_test_launch(
        demo_nodes=['calibration', 'temp_sensor'],
        fault_manager=False,
    )


class TestDocsEndpoint(GatewayTestCase):
    """Integration tests for GET /{any-path}/docs capability description."""

    MIN_EXPECTED_APPS = 2
    REQUIRED_APPS = {'calibration', 'temp_sensor'}

    def _assert_valid_openapi_spec(self, data):
        """Assert the response is a valid OpenAPI 3.1.0 spec."""
        self.assertEqual(data['openapi'], '3.1.0')
        self.assertIn('info', data)
        self.assertIn('paths', data)
        self.assertIn('servers', data)
        self.assertIn('components', data)

    def test_root_docs_returns_openapi_spec(self):
        """GET /docs returns valid OpenAPI 3.1.0 spec with SOVD version.

        @verifies REQ_INTEROP_002
        """
        data = self.poll_endpoint_until(
            '/docs',
            lambda d: d if 'openapi' in d else None,
        )
        self._assert_valid_openapi_spec(data)
        self.assertIn('x-sovd-version', data['info'])

        # Root spec should include server-level endpoints
        paths = data['paths']
        self.assertIn('/health', paths)
        self.assertIn('/version-info', paths)
        self.assertIn('/', paths)

        # Should include entity collection endpoints
        self.assertTrue(
            any('/apps' in p for p in paths),
            f'Root spec should include /apps path. Paths: {list(paths.keys())}'
        )

        # Lifecycle status operationIds use the singular entity name
        # (getAppStatus), not the plural collection name (getAppsStatus).
        self.assertEqual(
            paths['/apps/{app_id}/status']['get']['operationId'],
            'getAppStatus')
        self.assertEqual(
            paths['/components/{component_id}/status']['get']['operationId'],
            'getComponentStatus')
        self.assertEqual(
            paths['/apps/{app_id}/status/restart']['put']['operationId'],
            'putAppStatusRestart')

    def test_apps_docs_returns_entity_collection_spec(self):
        """GET /apps/docs returns spec for apps collection.

        @verifies REQ_INTEROP_002
        """
        data = self.poll_endpoint_until(
            '/apps/docs',
            lambda d: d if 'paths' in d else None,
        )
        self._assert_valid_openapi_spec(data)

        paths = data['paths']
        self.assertTrue(
            any('/apps' in p for p in paths),
            f'Apps docs should include /apps path. Paths: {list(paths.keys())}'
        )

    def test_components_docs_returns_entity_collection_spec(self):
        """GET /components/docs returns spec for components collection.

        @verifies REQ_INTEROP_002
        """
        data = self.poll_endpoint_until(
            '/components/docs',
            lambda d: d if 'paths' in d else None,
        )
        self._assert_valid_openapi_spec(data)

        paths = data['paths']
        self.assertTrue(
            any('/components' in p for p in paths),
            f'Components docs should include /components path. Paths: {list(paths.keys())}'
        )

    def test_areas_docs_returns_entity_collection_spec(self):
        """GET /areas/docs returns spec for areas collection.

        @verifies REQ_INTEROP_002
        """
        data = self.poll_endpoint_until(
            '/areas/docs',
            lambda d: d if 'paths' in d else None,
        )
        self._assert_valid_openapi_spec(data)

        paths = data['paths']
        self.assertTrue(
            any('/areas' in p for p in paths),
            f'Areas docs should include /areas path. Paths: {list(paths.keys())}'
        )

    def test_specific_app_docs(self):
        """GET /apps/{id}/docs returns spec with resource collections.

        @verifies REQ_INTEROP_002
        """
        # First get a known app ID
        apps_data = self.poll_endpoint_until(
            '/apps',
            lambda d: d if d.get('items') else None,
        )
        app_id = apps_data['items'][0]['id']

        data = self.poll_endpoint_until(
            f'/apps/{app_id}/docs',
            lambda d: d if 'paths' in d else None,
        )
        self._assert_valid_openapi_spec(data)

        # Verify paths dict contains a key with '/data' resource collection
        paths = data['paths']
        data_paths = [p for p in paths.keys() if '/data' in p]
        self.assertGreater(
            len(data_paths), 0,
            f'Expected at least one path containing /data. Paths: {list(paths.keys())}'
        )

    def test_data_collection_docs(self):
        """GET /apps/{id}/data/docs returns spec with data item schemas.

        @verifies REQ_INTEROP_002
        """
        data = self.poll_endpoint_until(
            '/apps/temp_sensor/data/docs',
            lambda d: d if 'paths' in d else None,
        )
        self._assert_valid_openapi_spec(data)

        paths = data['paths']
        self.assertGreater(len(paths), 0, 'Expected at least one path in data docs')

        # Verify at least one path has a GET operation with a response schema
        has_get_with_schema = False
        for path_key, path_item in paths.items():
            if 'get' in path_item:
                responses = path_item['get'].get('responses', {})
                if '200' in responses and 'content' in responses['200']:
                    has_get_with_schema = True
                    break
        self.assertTrue(
            has_get_with_schema,
            f'Expected at least one data path with GET + 200 response schema. '
            f'Paths: {list(paths.keys())}'
        )

    def test_operations_collection_docs(self):
        """GET /apps/{id}/operations/docs returns spec for operations.

        @verifies REQ_INTEROP_002
        """
        data = self.poll_endpoint_until(
            '/apps/calibration/operations/docs',
            lambda d: d if 'paths' in d else None,
        )
        self._assert_valid_openapi_spec(data)
        self.assertTrue(len(data['paths']) > 0)

    def test_logs_configuration_schema_field_names(self):
        """GET /apps/{id}/logs/docs spec has correct config schema fields.

        The log configuration schema must use 'severity_filter' and 'max_entries',
        not 'level' (which was a bug in the initial implementation).

        @verifies REQ_INTEROP_002
        """
        data = self.poll_endpoint_until(
            '/apps/temp_sensor/logs/docs',
            lambda d: d if 'paths' in d else None,
        )
        self._assert_valid_openapi_spec(data)

        # Find the /configuration sub-path
        config_paths = [p for p in data['paths'].keys() if '/configuration' in p]
        self.assertGreater(
            len(config_paths), 0,
            f'Expected a logs/configuration path. Paths: {list(data["paths"].keys())}'
        )

        config_path = config_paths[0]
        path_item = data['paths'][config_path]

        # Verify GET response schema references LogConfiguration component
        self.assertIn('get', path_item)
        get_schema = path_item['get']['responses']['200']['content']['application/json']['schema']
        if '$ref' in get_schema:
            # Entity-scoped docs use $ref to shared LogConfiguration schema
            self.assertIn('LogConfiguration', get_schema['$ref'])
            # Verify the referenced component schema has correct fields
            schemas = data.get('components', {}).get('schemas', {})
            if 'LogConfiguration' in schemas:
                props = schemas['LogConfiguration'].get('properties', {})
                self.assertIn('severity_filter', props)
                self.assertIn('max_entries', props)
                self.assertNotIn('level', props)
        else:
            # Inline schema (root docs)
            props = get_schema.get('properties', {})
            self.assertIn('severity_filter', props, f'Missing severity_filter: {props}')
            self.assertIn('max_entries', props, f'Missing max_entries: {props}')
            self.assertNotIn('level', props, f'Old "level" field present: {props}')

        # Verify PUT request body schema references LogConfiguration
        self.assertIn('put', path_item)
        put_schema = path_item['put']['requestBody']['content']['application/json']['schema']
        if '$ref' in put_schema:
            self.assertIn('LogConfiguration', put_schema['$ref'])
        else:
            put_props = put_schema.get('properties', {})
            self.assertIn('severity_filter', put_props)
            self.assertNotIn('level', put_props)

    def test_nested_entity_docs_describes_nothing_rather_than_fabricating(self):
        """A nested entity path the gateway does not route publishes no paths.

        `/components/{id}/apps/{id}` resolves - the component really does host
        the app - but no route is registered under it, and the nested entity
        itself answers 404. Since the sub-document became a projection of the
        route registry, its spec is a valid OpenAPI document with an empty
        `paths` object. The hand-written producer described 15 paths there that
        no route answers: the detail endpoint, one per entry of
        `EntityCapabilities::for_type(APP).collections()` - 13 of them, none
        skipped - and `/logs/configuration`.

        Pinned so that emptiness is the intended answer rather than an
        accident of the filter.

        @verifies REQ_INTEROP_002
        """
        comp_id = self.poll_endpoint_until(
            '/components',
            lambda d: d if d.get('items') else None,
        )['items'][0]['id']
        app_id = self.poll_endpoint_until(
            '/apps',
            lambda d: d if d.get('items') else None,
        )['items'][0]['id']
        nested = f'/components/{comp_id}/apps/{app_id}'

        # The nested entity is not itself a route the gateway serves ...
        self.assertEqual(
            requests.get(f'{self.BASE_URL}{nested}', timeout=10).status_code, 404,
            f'{nested} is routed after all - this case no longer proves anything'
        )

        # ... so its spec describes nothing, and invents nothing.
        data = self.poll_endpoint_until(
            f'{nested}/docs',
            lambda d: d if 'paths' in d else None,
        )
        self._assert_valid_openapi_spec(data)
        self.assertEqual(
            data['paths'], {},
            f'expected no paths, got {list(data["paths"])}'
        )

    def test_nonexistent_entity_docs_returns_404(self):
        """GET /apps/nonexistent_entity_xyz/docs returns 404.

        @verifies REQ_INTEROP_002
        """
        response = requests.get(
            f'{self.BASE_URL}/apps/nonexistent_entity_xyz/docs', timeout=10
        )
        self.assertEqual(response.status_code, 404)

        data = response.json()
        self.assertIn('error_code', data)

    def test_nonexistent_path_docs_returns_404(self):
        """GET /totally_invalid_path/docs returns 404.

        @verifies REQ_INTEROP_002
        """
        response = requests.get(
            f'{self.BASE_URL}/totally_invalid_path/docs', timeout=10
        )
        self.assertEqual(response.status_code, 404)

        data = response.json()
        self.assertIn('error_code', data)

    def test_spec_contains_server_info(self):
        """GET /docs spec includes server URL information.

        @verifies REQ_INTEROP_002
        """
        data = self.poll_endpoint_until(
            '/docs',
            lambda d: d if 'servers' in d else None,
        )
        servers = data['servers']
        self.assertIsInstance(servers, list)
        self.assertGreater(len(servers), 0)
        self.assertIn('url', servers[0])

    def test_spec_contains_components(self):
        """GET /docs spec includes components with GenericError response.

        @verifies REQ_INTEROP_002
        """
        data = self.poll_endpoint_until(
            '/docs',
            lambda d: d if 'components' in d else None,
        )
        components = data['components']
        self.assertIn('responses', components)
        self.assertIn('GenericError', components['responses'])


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        """Check all processes exited cleanly (SIGTERM allowed)."""
        for info in proc_info:
            self.assertIn(
                info.returncode, ALLOWED_EXIT_CODES,
                f'{info.process_name} exited with code {info.returncode}'
            )
