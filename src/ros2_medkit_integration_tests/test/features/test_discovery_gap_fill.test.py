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
Integration tests for hybrid mode gap-fill configuration.

Tests that gap-fill controls restrict which heuristic entities the
runtime layer can create when a manifest is present.

Note: Areas and Components are never created by runtime discovery.
Gap-fill only controls heuristic Apps and Functions.
"""

import os
import unittest

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction
import launch_testing
import launch_testing.actions

from ros2_medkit_test_utils.constants import ALLOWED_EXIT_CODES
from ros2_medkit_test_utils.gateway_test_case import GatewayTestCase
from ros2_medkit_test_utils.launch_helpers import create_demo_nodes, create_gateway_node


def generate_test_description():
    pkg_share = get_package_share_directory('ros2_medkit_gateway')
    manifest_path = os.path.join(
        pkg_share, 'config', 'examples', 'demo_nodes_manifest.yaml'
    )

    # Hybrid mode with gap-fill:
    # - Apps allowed (for linking)
    # - Areas and Components come from manifest only (runtime never creates them)
    gateway_node = create_gateway_node(extra_params={
        'discovery.mode': 'hybrid',
        'discovery.manifest_path': manifest_path,
        'discovery.manifest_strict_validation': False,
        'discovery.merge_pipeline.gap_fill.allow_heuristic_apps': True,
    })

    # Launch a subset of demo nodes (some in manifest, some not)
    demo_nodes = create_demo_nodes(
        ['temp_sensor', 'rpm_sensor', 'pressure_sensor', 'calibration'],
    )
    delayed = TimerAction(period=2.0, actions=demo_nodes)

    return (
        LaunchDescription([
            gateway_node,
            delayed,
            launch_testing.actions.ReadyToTest(),
        ]),
        {'gateway_node': gateway_node},
    )


# @verifies REQ_INTEROP_003
class TestGapFillConfig(GatewayTestCase):
    """Test gap-fill restrictions in hybrid mode."""

    def test_only_manifest_areas_present(self):
        """Areas come from manifest only - no heuristic areas from runtime.

        GET /areas only returns top-level areas; subareas are filtered
        and accessible via GET /areas/{id}/subareas.
        """
        data = self.poll_endpoint_until(
            '/areas',
            lambda d: d if len(d.get('items', [])) >= 1 else None,
            timeout=30.0,
        )
        area_ids = [a['id'] for a in data['items']]

        # Only top-level manifest areas should appear in GET /areas
        # Subareas (engine, brakes, lidar, door, lights, front-left-door)
        # are filtered from the top-level listing
        for area_id in area_ids:
            self.assertIn(area_id, [
                'powertrain', 'chassis', 'body', 'perception',
                # HATEOAS edge-case fixture, manifest-only.
                'hateoas-edge-area',
            ], f'Unexpected area found in top-level listing: {area_id}')

    def test_only_manifest_components_present(self):
        """Components come from manifest only - runtime never creates components."""
        data = self.poll_endpoint_until(
            '/components',
            lambda d: d if len(d.get('items', [])) >= 1 else None,
            timeout=30.0,
        )
        component_ids = [c['id'] for c in data['items']]

        # Only manifest-defined components should be present
        manifest_components = [
            'engine-ecu', 'temp-sensor-hw', 'rpm-sensor-hw',
            'brake-ecu', 'brake-pressure-sensor-hw', 'brake-actuator-hw',
            'door-sensor-hw', 'light-module',
            'lidar-unit',
            # HATEOAS edge-case fixture (component without area assignment).
            'hateoas-component-no-area',
        ]
        for comp_id in component_ids:
            self.assertIn(
                comp_id, manifest_components,
                f'Unexpected heuristic component found: {comp_id}',
            )

    def test_manifest_apps_present_and_linked(self):
        """With allow_heuristic_apps=true, manifest apps matching launched nodes are present."""
        # 4 demo nodes launched: temp_sensor, rpm_sensor, pressure_sensor, calibration
        # These match manifest apps: engine-temp-sensor, engine-rpm-sensor,
        # brake-pressure-sensor, engine-calibration-service
        expected_linked = {
            'engine-temp-sensor', 'engine-rpm-sensor',
            'brake-pressure-sensor', 'engine-calibration-service',
        }
        data = self.poll_endpoint_until(
            '/apps',
            lambda d: d if len(d.get('items', [])) >= len(expected_linked) else None,
            timeout=30.0,
        )
        app_ids = {a['id'] for a in data['items']}
        for app_id in expected_linked:
            self.assertIn(
                app_id, app_ids,
                f'Expected manifest app {app_id} not found in apps list',
            )

    def test_health_shows_discovery_info(self):
        """Health endpoint should show discovery information."""
        health = self.poll_endpoint_until(
            '/health',
            lambda data: data if 'discovery' in data else None,
            timeout=30.0,
        )
        discovery = health.get('discovery', {})
        pipeline = discovery.get('pipeline', {})

        # Pipeline info should be present in hybrid mode
        self.assertIn('total_entities', pipeline)


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(
            proc_info, allowable_exit_codes=ALLOWED_EXIT_CODES
        )
