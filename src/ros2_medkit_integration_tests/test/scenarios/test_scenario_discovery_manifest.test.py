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

"""Scenario: Manifest-only discovery mode validation.

Validates discovery endpoints when the gateway is configured with
discovery_mode: manifest_only using demo_nodes_manifest.yaml.

Tests verify:
- Areas are loaded from manifest (not runtime discovery)
- Components are loaded from manifest with correct types
- Apps are loaded from manifest with correct ros_binding
- Functions are loaded from manifest with hosted_by relationships
- Subareas and related-components relationships work
- Entity details, capabilities, and HATEOAS links are correct
- Error handling for invalid and nonexistent entity IDs

Demo nodes are launched to verify apps become online when their
bound ROS 2 nodes are running.
"""

import os
import unittest

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction
import launch_testing
import launch_testing.actions
import requests

from ros2_medkit_test_utils.constants import (
    ALLOWED_EXIT_CODES,
    PARAM_SERVICE_TIMEOUT,
)
from ros2_medkit_test_utils.gateway_test_case import GatewayTestCase
from ros2_medkit_test_utils.launch_helpers import create_demo_nodes, create_gateway_node


def generate_test_description():
    pkg_share = get_package_share_directory('ros2_medkit_gateway')
    manifest_path = os.path.join(
        pkg_share, 'config', 'examples', 'demo_nodes_manifest.yaml'
    )

    gateway = create_gateway_node(extra_params={
        'discovery.mode': 'manifest_only',
        'discovery.manifest_path': manifest_path,
        'discovery.manifest_strict_validation': False,
    })

    # Launch a subset of demo nodes to verify apps become online
    demo_nodes = create_demo_nodes(
        ['temp_sensor', 'rpm_sensor', 'pressure_sensor', 'calibration', 'lidar_sensor'],
        lidar_faulty=False,
    )

    delayed = TimerAction(period=2.0, actions=demo_nodes)

    return (
        LaunchDescription([
            gateway,
            delayed,
            launch_testing.actions.ReadyToTest(),
        ]),
        {'gateway_node': gateway},
    )


class TestScenarioDiscoveryManifest(GatewayTestCase):
    """Scenario: Gateway in manifest_only discovery mode.

    The gateway loads entity definitions exclusively from
    demo_nodes_manifest.yaml. Runtime ROS 2 graph discovery is disabled.
    Demo nodes are launched to verify online status linking.

    Steps:
    1. Verify all manifest-defined areas are present
    2. Verify area details, subareas, and area-component relationships
    3. Verify all manifest-defined components are present with details
    4. Verify subcomponents endpoint works
    5. Verify all manifest-defined apps are present
    6. Verify app data, operations, and configurations endpoints
    7. Verify app data item retrieval
    8. Verify all manifest-defined functions are present
    9. Verify function hosts and aggregation endpoints
    10. Verify discovery stats report manifest_only mode
    11. Verify error handling for invalid and nonexistent entity IDs
    """

    # Manifest entities are available immediately; no runtime discovery needed.
    # We set a small required set to ensure the gateway is fully initialized.
    MIN_EXPECTED_APPS = 5
    REQUIRED_AREAS = {'powertrain', 'chassis', 'body', 'perception'}

    # =========================================================================
    # Areas
    # =========================================================================

    def test_01_list_areas(self):
        """GET /areas returns only top-level areas (subareas are filtered out).

        Sub-entities (areas with parent_area_id) are only accessible via
        GET /areas/{id}/subareas, not in the top-level listing.

        @verifies REQ_INTEROP_003
        """
        data = self.get_json('/areas')
        self.assertIn('items', data)
        self.assertIn('total_count', data.get('x-medkit', {}))

        area_ids = [a['id'] for a in data['items']]

        # Top-level areas
        for area_id in ['powertrain', 'chassis', 'body', 'perception']:
            self.assertIn(area_id, area_ids, f'Missing top-level area: {area_id}')

        # Subareas must NOT appear in top-level listing
        for area_id in ['engine', 'brakes', 'lidar']:
            self.assertNotIn(
                area_id, area_ids,
                f'Subarea {area_id} should not appear in top-level /areas listing'
            )

    def test_02_get_area_details(self):
        """GET /areas/{id} returns area with capabilities and links.

        @verifies REQ_INTEROP_003
        """
        area = self.assert_entity_exists('areas', 'powertrain')
        self.assertEqual(area['id'], 'powertrain')
        self.assertEqual(area['name'], 'Powertrain')
        self.assertIn('capabilities', area)
        self.assertIn('_links', area)

    def test_03_get_area_not_found(self):
        """GET /areas/{id} returns 404 for unknown area."""
        self.assert_entity_not_found('areas', 'nonexistent')

    def test_04_area_subareas(self):
        """GET /areas/{id}/subareas returns nested areas.

        @verifies REQ_INTEROP_004
        """
        data = self.get_json('/areas/powertrain/subareas')
        self.assertIn('items', data)
        subarea_ids = [s['id'] for s in data['items']]
        self.assertIn('engine', subarea_ids)

    def test_05_area_components(self):
        """GET /areas/{id}/components returns components in area.

        The area's `capabilities` array must advertise the same route. Areas were
        the one entity type whose relationship endpoint was served and named in
        `AreaDetail.components` but missing from the capability array, so a
        client reading only that array could not find it.

        @verifies REQ_INTEROP_006
        """
        data = self.get_json('/areas/engine/components')
        self.assertIn('items', data)

        area = self.get_json('/areas/engine')
        advertised = {c['name']: c['href'] for c in area['capabilities']}
        self.assertIn('components', advertised)
        self.assertEqual(
            advertised['components'], '/api/v1/areas/engine/components')

    # =========================================================================
    # Components
    # =========================================================================

    def test_06_list_components(self):
        """GET /components returns all manifest-defined components.

        @verifies REQ_INTEROP_003
        """
        data = self.get_json('/components')
        self.assertIn('items', data)
        self.assertIn('total_count', data.get('x-medkit', {}))

        component_ids = [c['id'] for c in data['items']]
        for comp_id in ['engine-ecu', 'temp-sensor-hw', 'brake-ecu', 'lidar-unit']:
            self.assertIn(comp_id, component_ids, f'Missing component: {comp_id}')

    def test_07_get_component_details(self):
        """GET /components/{id} returns component with capabilities in x-medkit.

        @verifies REQ_INTEROP_003
        """
        component = self.assert_entity_exists('components', 'engine-ecu')
        self.assertEqual(component['id'], 'engine-ecu')
        self.assertEqual(component['name'], 'Engine ECU')
        self.assertIn('x-medkit', component)
        self.assertIn('capabilities', component['x-medkit'])
        self.assertIn('_links', component)

    def test_08_get_component_not_found(self):
        """GET /components/{id} returns 404 for unknown component."""
        self.assert_entity_not_found('components', 'nonexistent')

    def test_09_component_subcomponents(self):
        """GET /components/{id}/subcomponents returns list (possibly empty).

        @verifies REQ_INTEROP_005
        """
        data = self.get_json('/components/engine-ecu/subcomponents')
        self.assertIn('items', data)
        self.assertIsInstance(data['items'], list)

    def test_10_component_subcomponents_not_found(self):
        """GET /components/{id}/subcomponents returns 404 for unknown component.

        @verifies REQ_INTEROP_005
        """
        self.get_json(
            '/components/nonexistent/subcomponents', expected_status=404
        )

    # =========================================================================
    # Apps
    # =========================================================================

    def test_11_list_apps(self):
        """GET /apps returns all manifest-defined apps.

        @verifies REQ_INTEROP_003
        """
        data = self.get_json('/apps')
        self.assertIn('items', data)
        self.assertIn('total_count', data.get('x-medkit', {}))

        app_ids = [a['id'] for a in data['items']]
        for app_id in [
            'engine-temp-sensor', 'engine-rpm-sensor',
            'brake-pressure-sensor', 'lidar-sensor',
        ]:
            self.assertIn(app_id, app_ids, f'Missing app: {app_id}')

    def test_12_get_app_details(self):
        """GET /apps/{id} returns app with capabilities and links.

        @verifies REQ_INTEROP_003
        """
        app = self.assert_entity_exists('apps', 'engine-temp-sensor')
        self.assertEqual(app['id'], 'engine-temp-sensor')
        self.assertEqual(app['name'], 'Engine Temperature Sensor')
        self.assertIn('capabilities', app)
        self.assertIn('_links', app)

    def test_13_get_app_not_found(self):
        """GET /apps/{id} returns 404 for unknown app."""
        self.assert_entity_not_found('apps', 'nonexistent')

    def test_14_app_online_status(self):
        """Apps with running demo nodes should exist in the listing.

        In manifest-only mode apps always exist from the manifest.
        Online status depends on runtime linking timing.
        """
        data = self.poll_endpoint_until(
            '/apps',
            lambda d: d if any(
                a['id'] == 'engine-temp-sensor' for a in d.get('items', [])
            ) else None,
            timeout=15.0,
        )
        apps_by_id = {a['id']: a for a in data['items']}
        self.assertIn('engine-temp-sensor', apps_by_id)

    def test_15_app_data_endpoint(self):
        """GET /apps/{id}/data returns topic list.

        @verifies REQ_INTEROP_003
        """
        data = self.get_json('/apps/engine-temp-sensor/data')
        self.assertIn('items', data)

    def test_16_app_operations_endpoint(self):
        """GET /apps/{id}/operations returns services/actions."""
        data = self.get_json('/apps/engine-calibration-service/operations')
        self.assertIn('items', data)

    def test_17_app_configurations_endpoint(self):
        """GET /apps/{id}/configurations returns parameters when node is running.

        App is defined in manifest with bound_fqn. Configurations are retrieved
        from the ROS 2 parameter service on the node, which may not be ready the
        instant the app is discovered - the endpoint returns 503 until it is.
        Poll until it responds rather than asserting on a single early GET.

        @verifies REQ_INTEROP_003
        """
        data = self.poll_endpoint_until(
            '/apps/lidar-sensor/configurations',
            lambda d: d if 'items' in d else None,
            timeout=PARAM_SERVICE_TIMEOUT,
        )
        self.assertIn('items', data)
        self.assertIn('x-medkit', data)

    def test_18_app_data_item_endpoint(self):
        """GET /apps/{id}/data returns valid response; if items present, drill in.

        In manifest_only mode, data items depend on runtime graph discovery
        via inherit_runtime_resources. The endpoint always works (200), but
        items may be empty when runtime linking has not enriched the app
        with topic information.

        @verifies REQ_INTEROP_003
        """
        data = self.get_json('/apps/engine-temp-sensor/data')
        self.assertIn('items', data)
        self.assertIsInstance(data['items'], list)

        if data['items']:
            data_id = data['items'][0]['id']
            item = self.get_json(f'/apps/engine-temp-sensor/data/{data_id}')
            self.assertIn('id', item)
            self.assertIn('direction', item)

    # =========================================================================
    # Functions
    # =========================================================================

    def test_19_list_functions(self):
        """GET /functions returns all manifest-defined functions.

        @verifies REQ_INTEROP_003
        """
        data = self.get_json('/functions')
        self.assertIn('items', data)
        self.assertIn('total_count', data.get('x-medkit', {}))

        function_ids = [f['id'] for f in data['items']]
        for func_id in [
            'engine-monitoring', 'engine-calibration',
            'brake-system', 'body-electronics', 'perception-system',
        ]:
            self.assertIn(func_id, function_ids, f'Missing function: {func_id}')

    def test_20_get_function_details(self):
        """GET /functions/{id} returns function with capabilities and links.

        @verifies REQ_INTEROP_003
        """
        func = self.assert_entity_exists('functions', 'engine-monitoring')
        self.assertEqual(func['id'], 'engine-monitoring')
        self.assertEqual(func['name'], 'Engine Monitoring')
        self.assertIn('capabilities', func)
        self.assertIn('_links', func)

    def test_21_get_function_not_found(self):
        """GET /functions/{id} returns 404 for unknown function."""
        self.assert_entity_not_found('functions', 'nonexistent')

    def test_22_function_hosts(self):
        """GET /functions/{id}/hosts returns hosting apps.

        @verifies REQ_INTEROP_007
        """
        data = self.get_json('/functions/engine-monitoring/hosts')
        self.assertIn('items', data)
        host_ids = [h['id'] for h in data['items']]
        self.assertIn('engine-temp-sensor', host_ids)
        self.assertIn('engine-rpm-sensor', host_ids)

    def test_23_function_data(self):
        """GET /functions/{id}/data aggregates data from hosts."""
        data = self.get_json('/functions/engine-monitoring/data')
        self.assertIn('items', data)

    def test_24_function_operations(self):
        """GET /functions/{id}/operations aggregates operations from hosts."""
        data = self.get_json('/functions/engine-calibration/operations')
        self.assertIn('items', data)

    # =========================================================================
    # Regression: no synthetic/topic components leak into manifest_only mode
    # =========================================================================

    def test_25_no_topic_components_in_manifest_only(self):
        """Components list contains only manifest-defined IDs.

        Regression test: in manifest_only mode, runtime-discovered
        topic-based components must not appear in the entity cache.
        All component IDs must come from the manifest.
        """
        manifest_component_ids = {
            'engine-ecu', 'temp-sensor-hw', 'rpm-sensor-hw',
            'brake-ecu', 'brake-pressure-sensor-hw', 'brake-actuator-hw',
            'door-sensor-hw', 'light-module', 'lidar-unit',
            # HATEOAS edge-case fixture (component without area assignment).
            'hateoas-component-no-area',
        }
        data = self.get_json('/components')
        actual_ids = {c['id'] for c in data['items']}
        extra = actual_ids - manifest_component_ids
        self.assertEqual(
            extra, set(),
            f'Non-manifest components found in manifest_only mode: {extra}'
        )

    # =========================================================================
    # Function + Area logs (ros2_medkit extension)
    # =========================================================================

    def test_26_function_logs_returns_200(self):
        """GET /functions/{id}/logs returns 200 with items array.

        Function logs aggregate from all hosted apps. engine-monitoring
        is hosted by engine-temp-sensor and engine-rpm-sensor.
        """
        data = self.get_json('/functions/engine-monitoring/logs')
        self.assertIn('items', data)
        self.assertIsInstance(data['items'], list)

    def test_27_function_logs_has_entries_from_hosts(self):
        """GET /functions/{id}/logs returns entries from hosted apps.

        Poll until log entries appear from at least one hosted app.
        """
        data = self.poll_endpoint_until(
            '/functions/engine-monitoring/logs',
            condition=lambda d: d if d.get('items') else None,
            timeout=30.0,
        )
        self.assertGreater(
            len(data['items']), 0,
            'Expected log entries aggregated from hosted apps'
        )

    def test_28_function_logs_configuration_returns_200(self):
        """GET /functions/{id}/logs/configuration returns default config."""
        data = self.get_json('/functions/engine-monitoring/logs/configuration')
        self.assertIn('severity_filter', data)
        self.assertIn('max_entries', data)

    def test_29_area_logs_returns_200(self):
        """GET /areas/{id}/logs returns 200 with items array.

        Area logs use namespace prefix matching. powertrain area should
        capture logs from nodes in /powertrain/* namespace tree.
        """
        data = self.get_json('/areas/powertrain/logs')
        self.assertIn('items', data)
        self.assertIsInstance(data['items'], list)

    def test_30_area_logs_has_entries(self):
        """GET /areas/{id}/logs returns entries from nodes in area namespace."""
        data = self.poll_endpoint_until(
            '/areas/powertrain/logs',
            condition=lambda d: d if d.get('items') else None,
            timeout=15.0,
        )
        self.assertGreater(
            len(data['items']), 0,
            'Expected log entries from nodes in powertrain namespace'
        )

    # =========================================================================
    # Error Cases
    # =========================================================================

    def test_31_invalid_area_id(self):
        """GET /areas/{id} with invalid ID format returns 400.

        Entity IDs only allow alphanumeric, underscore, and hyphen characters.
        Dots are not allowed, so 'invalid..id' is rejected with 400 Bad Request.
        """
        response = requests.get(
            f'{self.BASE_URL}/areas/invalid..id', timeout=5
        )
        self.assertEqual(response.status_code, 400)

    def test_32_invalid_component_id(self):
        """GET /components/{id} with path traversal returns 404."""
        response = requests.get(
            f'{self.BASE_URL}/components/../etc/passwd', timeout=5
        )
        self.assertEqual(response.status_code, 404)


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        """Check all processes exited cleanly (SIGTERM allowed)."""
        for info in proc_info:
            self.assertIn(
                info.returncode, ALLOWED_EXIT_CODES,
                f'{info.process_name} exited with code {info.returncode}'
            )
