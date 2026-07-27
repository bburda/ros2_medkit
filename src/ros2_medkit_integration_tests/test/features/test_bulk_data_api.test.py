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

"""Feature tests for bulk-data API.

Validates bulk-data category listing, descriptor listing, empty results,
unknown categories, download 404, and nested entity paths.

NOTE: Tests that require rosbag downloads (test_127, test_129, test_137)
are in scenario tests, not here.

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
        demo_nodes=['lidar_sensor'],
        fault_manager=True,
        lidar_faulty=True,
    )


class TestBulkDataApi(GatewayTestCase):
    """Bulk-data API tests."""

    MIN_EXPECTED_APPS = 1
    REQUIRED_APPS = {'lidar_sensor'}

    def test_bulk_data_list_categories_success(self):
        """GET /apps/{app}/bulk-data returns categories.

        @verifies REQ_INTEROP_071
        """
        response = requests.get(
            f'{self.BASE_URL}/apps/lidar_sensor/bulk-data',
            timeout=10
        )
        self.assertEqual(response.status_code, 200)

        data = response.json()
        self.assertIn('items', data)
        self.assertIsInstance(data['items'], list)
        # Should include rosbags category
        self.assertIn('rosbags', data['items'])

    def test_bulk_data_list_categories_all_entity_types(self):
        """Bulk-data endpoint works for apps and components.

        As a ros2_medkit extension, these entity types support bulk-data.
        Uses the host-derived default component (SOVD-aligned entity model).

        @verifies REQ_INTEROP_071
        """
        # Get host component ID dynamically
        comp_data = self.get_json('/components')
        components = comp_data.get('items', [])
        self.assertGreater(len(components), 0, 'Expected at least one component')
        comp_id = components[0]['id']

        supported_endpoints = [
            '/apps/lidar_sensor/bulk-data',
            f'/components/{comp_id}/bulk-data',
        ]

        for endpoint in supported_endpoints:
            response = requests.get(f'{self.BASE_URL}{endpoint}', timeout=10)
            self.assertEqual(
                response.status_code, 200,
                f'Expected 200 for {endpoint}, got {response.status_code}'
            )

            data = response.json()
            self.assertIn('items', data)
            self.assertIsInstance(data['items'], list)

    def test_bulk_data_list_categories_entity_not_found(self):
        """Bulk-data returns 404 for nonexistent entity.

        @verifies REQ_INTEROP_071
        """
        response = requests.get(
            f'{self.BASE_URL}/apps/nonexistent_app/bulk-data',
            timeout=10
        )
        self.assertEqual(response.status_code, 404)

        data = response.json()
        self.assertIn('error_code', data)

    def test_bulk_data_list_descriptors_structure(self):
        """GET /apps/{app}/bulk-data/rosbags returns BulkDataDescriptor[].

        @verifies REQ_INTEROP_072
        """
        # Poll until rosbag descriptors appear (fault triggers capture)
        data = self.poll_endpoint_until(
            '/apps/lidar_sensor/bulk-data/rosbags',
            lambda d: d if d.get('items') else None,
            timeout=10.0,
            interval=1.0,
        )
        self.assertIsInstance(data['items'], list)
        self.assertGreater(
            len(data['items']), 0, 'Expected at least one rosbag descriptor',
        )

        descriptor = data['items'][0]
        self.assertIn('id', descriptor)
        self.assertIn('name', descriptor)
        self.assertIn('size', descriptor)
        self.assertIn('mimetype', descriptor)  # SOVD uses 'mimetype'
        self.assertIn('creation_date', descriptor)
        # Verify x-medkit extension
        self.assertIn('x-medkit', descriptor)
        x_medkit = descriptor['x-medkit']
        self.assertIn('fault_code', x_medkit)
        # recording_id (bag directory basename) groups the descriptors of
        # faults that share one recording.
        self.assertIn('recording_id', x_medkit)
        self.assertTrue(
            x_medkit['recording_id'].startswith('fault_'),
            f'recording_id should be the bag directory name, '
            f'got: {x_medkit["recording_id"]}',
        )

    def test_bulk_data_list_descriptors_empty_result(self):
        """Bulk-data returns empty or non-empty array for component rosbags.

        @verifies REQ_INTEROP_072
        """
        # Use the host-derived default component
        comp_data = self.get_json('/components')
        components = comp_data.get('items', [])
        self.assertGreater(len(components), 0, 'Expected at least one component')
        comp_id = components[0]['id']

        response = requests.get(
            f'{self.BASE_URL}/components/{comp_id}/bulk-data/rosbags',
            timeout=10
        )
        self.assertEqual(response.status_code, 200)

        data = response.json()
        self.assertIn('items', data)
        self.assertIsInstance(data['items'], list)

    def test_bulk_data_component_aggregates_child_apps(self):
        """GET /components/{id}/bulk-data/rosbags aggregates from hosted apps.

        Synthetic / runtime-discovered components have an empty fqn /
        namespace_path, so the legacy fall-through path returned zero source
        filters and produced empty descriptor lists. The handler now resolves
        hosted apps via the entity cache (mirrors the FUNCTION branch).

        @verifies REQ_INTEROP_072
        """
        comp_data = self.get_json('/components')
        components = comp_data.get('items', [])
        # Find the component that hosts lidar_sensor (the only demo app here).
        comp_id = None
        for c in components:
            hosts = self.get_json(
                f"/components/{c['id']}/hosts").get('items', [])
            if any(h.get('id') == 'lidar_sensor' for h in hosts):
                comp_id = c['id']
                break
        self.assertIsNotNone(
            comp_id, 'No component hosts lidar_sensor')

        # Poll - rosbag capture is fault-triggered and runs after launch.
        data = self.poll_endpoint_until(
            f'/components/{comp_id}/bulk-data/rosbags',
            lambda d: d if d.get('items') else None,
            timeout=15.0,
            interval=1.0,
        )
        self.assertGreater(
            len(data['items']), 0,
            'Component bulk-data aggregation returned zero descriptors',
        )

    def test_bulk_data_unknown_category_returns_404(self):
        """Bulk-data returns 404 for unknown category.

        @verifies REQ_INTEROP_072
        """
        response = requests.get(
            f'{self.BASE_URL}/apps/lidar_sensor/bulk-data/unknown_category',
            timeout=10
        )
        self.assertEqual(response.status_code, 404)

        data = response.json()
        self.assertIn('error_code', data)

    def test_bulk_data_download_not_found(self):
        """Bulk-data download returns 404 for invalid UUID.

        @verifies REQ_INTEROP_073
        """
        response = requests.get(
            f'{self.BASE_URL}/apps/lidar_sensor/bulk-data/rosbags/nonexistent-uuid',
            timeout=10
        )
        self.assertEqual(response.status_code, 404)

        # Response should be JSON error
        data = response.json()
        self.assertIn('error_code', data)

    def test_bulk_data_nested_entity_path(self):
        """Bulk-data endpoints work for component entities.

        Components support bulk-data. Uses the host-derived default component.

        @verifies REQ_INTEROP_071
        """
        # Use the host-derived default component
        comp_data = self.get_json('/components')
        components = comp_data.get('items', [])
        self.assertGreater(len(components), 0, 'Expected at least one component')
        comp_id = components[0]['id']

        response = requests.get(
            f'{self.BASE_URL}/components/{comp_id}/bulk-data',
            timeout=10
        )
        self.assertEqual(response.status_code, 200)

        data = response.json()
        self.assertIn('items', data)
        self.assertIsInstance(data['items'], list)


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        """Check all processes exited cleanly (SIGTERM allowed)."""
        for info in proc_info:
            self.assertIn(
                info.returncode, ALLOWED_EXIT_CODES,
                f'{info.process_name} exited with code {info.returncode}'
            )
