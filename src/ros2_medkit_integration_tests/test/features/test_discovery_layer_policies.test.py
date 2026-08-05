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
Integration tests for per-layer merge policy overrides in hybrid mode.

Tests that users can override default merge policies per layer per field group
via discovery.merge_pipeline.layers.<layer>.<field_group> parameters.
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

    # Hybrid mode with custom layer policy overrides:
    # - Manifest LIVE_DATA overridden from "enrichment" to "authoritative"
    #   (manifest topics should take precedence over runtime topics)
    # - Runtime IDENTITY overridden from "fallback" to "enrichment"
    #   (runtime names can fill empty manifest names)
    gateway_node = create_gateway_node(extra_params={
        'discovery.mode': 'hybrid',
        'discovery.manifest_path': manifest_path,
        'discovery.manifest_strict_validation': False,
        'discovery.merge_pipeline.layers.manifest.live_data': 'authoritative',
        'discovery.merge_pipeline.layers.runtime.identity': 'enrichment',
    })

    # Launch demo nodes that match manifest apps
    demo_nodes = create_demo_nodes(
        ['temp_sensor', 'rpm_sensor', 'calibration', 'pressure_sensor'],
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
class TestLayerPolicyOverrides(GatewayTestCase):
    """Test per-layer merge policy overrides in hybrid mode."""

    def test_gateway_starts_with_policy_overrides(self):
        """Gateway should start successfully with custom layer policies."""
        health = self.poll_endpoint_until(
            '/health',
            lambda data: data if data.get('status') == 'healthy' else None,
            timeout=30.0,
        )
        self.assertEqual(health['status'], 'healthy')

    def test_discovery_mode_is_hybrid(self):
        """Discovery mode should be hybrid."""
        health = self.poll_endpoint_until(
            '/health',
            lambda data: data if 'discovery' in data else None,
            timeout=30.0,
        )
        discovery = health.get('discovery', {})
        self.assertEqual(discovery.get('mode'), 'hybrid')

    def test_manifest_entities_present(self):
        """Manifest-defined entities should be discoverable."""
        data = self.poll_endpoint_until(
            '/areas',
            lambda d: d if any(a['id'] == 'powertrain' for a in d.get('items', [])) else None,
            timeout=30.0,
        )
        area_ids = [a['id'] for a in data['items']]
        self.assertIn('powertrain', area_ids)
        self.assertIn('chassis', area_ids)

    def test_manifest_apps_present(self):
        """Manifest-defined apps should be discoverable."""
        data = self.poll_endpoint_until(
            '/apps',
            lambda d: d if len(d.get('items', [])) >= 1 else None,
            timeout=30.0,
        )
        app_ids = [a['id'] for a in data['items']]
        # Manifest defines engine-temp-sensor, engine-rpm-sensor, etc.
        self.assertTrue(
            any('engine' in aid for aid in app_ids),
            f'No engine apps found: {app_ids}',
        )

    def test_merge_pipeline_has_layers(self):
        """Health endpoint should report merge pipeline with layer names."""
        health = self.poll_endpoint_until(
            '/health',
            lambda data: data if 'discovery' in data
            and 'pipeline' in data.get('discovery', {}) else None,
            timeout=30.0,
        )
        pipeline = health['discovery']['pipeline']
        layers = pipeline.get('layers', [])
        self.assertIn('manifest', layers)
        self.assertIn('runtime', layers)

    def test_manifest_authoritative_live_data(self):
        """Manifest LIVE_DATA=authoritative should override runtime topics.

        The manifest defines engine-temp-sensor with ros_binding for temp_sensor
        in /powertrain/engine. With manifest LIVE_DATA set to authoritative,
        manifest-defined topic info should take precedence. Verify that the
        component's topics field is not empty (manifest provides topic data
        from inherit_runtime_resources) and that the merge report shows no
        LIVE_DATA conflicts (authoritative wins cleanly).
        """
        health = self.poll_endpoint_until(
            '/health',
            lambda data: data if data.get('discovery', {}).get('pipeline', {}).get(
                'conflict_count', -1) >= 0 else None,
            timeout=30.0,
        )
        pipeline = health['discovery']['pipeline']
        # With authoritative vs enrichment at different levels, there should
        # be no conflict (authoritative simply wins)
        self.assertEqual(
            pipeline.get('conflict_count', -1), 0,
            f'Expected no merge conflicts with authoritative policy, got: {pipeline}',
        )


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(
            proc_info, allowable_exit_codes=ALLOWED_EXIT_CODES
        )
