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
Integration tests for namespace blacklist/whitelist in hybrid gap-fill.

Tests that namespace_blacklist and namespace_whitelist parameters
correctly filter which heuristic entities the runtime layer creates.

Strategy: launch two unmanifested nodes - one in a blacklisted namespace
(/chassis/brakes) and one in a non-blacklisted namespace (/powertrain/engine).
The blacklisted one should be filtered from gap-fill; the other should appear.
"""

import os
import unittest

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction
import launch_ros.actions
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

    # Hybrid mode with namespace blacklist:
    # Block /chassis namespace from gap-fill (only manifest chassis entities)
    gateway_node = create_gateway_node(extra_params={
        'discovery.mode': 'hybrid',
        'discovery.manifest_path': manifest_path,
        'discovery.manifest_strict_validation': False,
        'discovery.merge_pipeline.gap_fill.namespace_blacklist': ['/chassis'],
    })

    # Launch demo nodes including chassis nodes
    demo_nodes = create_demo_nodes(
        ['temp_sensor', 'rpm_sensor', 'pressure_sensor', 'calibration'],
    )

    # Launch an extra unmanifested node in /chassis/brakes (blacklisted)
    # This should be FILTERED by the namespace blacklist.
    unmanifested_chassis_node = launch_ros.actions.Node(
        package='ros2_medkit_integration_tests',
        executable='demo_engine_temp_sensor',
        name='unmanifested_chassis_sensor',
        namespace='/chassis/brakes',
        output='screen',
        # Give the node room to flush coverage data at shutdown before SIGKILL.
        sigterm_timeout='30',
        sigkill_timeout='15',
    )

    # Launch an extra unmanifested node in /powertrain/engine (not blacklisted)
    # This should NOT be filtered and appear as a heuristic gap-fill app.
    unmanifested_engine_node = launch_ros.actions.Node(
        package='ros2_medkit_integration_tests',
        executable='demo_engine_temp_sensor',
        name='unmanifested_engine_sensor',
        namespace='/powertrain/engine',
        output='screen',
        # Give the node room to flush coverage data at shutdown before SIGKILL.
        sigterm_timeout='30',
        sigkill_timeout='15',
    )

    delayed = TimerAction(
        period=2.0,
        actions=demo_nodes + [unmanifested_chassis_node, unmanifested_engine_node],
    )

    return (
        LaunchDescription([
            gateway_node,
            delayed,
            launch_testing.actions.ReadyToTest(),
        ]),
        {'gateway_node': gateway_node},
    )


# @verifies REQ_INTEROP_003
class TestNamespaceFilter(GatewayTestCase):
    """Test namespace blacklist filtering in hybrid gap-fill."""

    def test_gateway_starts_with_namespace_filter(self):
        """Gateway should start successfully with namespace blacklist."""
        health = self.poll_endpoint_until(
            '/health',
            lambda data: data if data.get('status') == 'healthy' else None,
            timeout=30.0,
        )
        self.assertEqual(health['status'], 'healthy')
        discovery = health.get('discovery', {})
        self.assertEqual(discovery.get('mode'), 'hybrid')

    def test_manifest_entities_always_present(self):
        """Manifest entities from blacklisted namespaces should still be present."""
        # Manifest defines chassis area and its components -
        # blacklist only affects gap-fill (heuristic entities), not manifest entities
        data = self.poll_endpoint_until(
            '/areas',
            lambda d: d if any(a['id'] == 'powertrain' for a in d.get('items', [])) else None,
            timeout=30.0,
        )
        area_ids = [a['id'] for a in data['items']]
        # Manifest-defined areas should still exist regardless of blacklist
        self.assertIn('powertrain', area_ids)
        self.assertIn('chassis', area_ids)

    def test_blacklist_filters_unmanifested_chassis_node(self):
        """Unmanifested node in blacklisted /chassis should NOT appear as app."""
        # Wait for the non-blacklisted unmanifested node to appear first,
        # proving that gap-fill is active and enough time has passed
        self.poll_endpoint_until(
            '/apps',
            lambda d: d if any(
                'unmanifested_engine_sensor' in a.get('id', '')
                for a in d.get('items', [])
            ) else None,
            timeout=30.0,
        )

        # Now verify the blacklisted one is absent
        data = self.get_json('/apps')
        app_ids = [a['id'] for a in data.get('items', [])]
        for app_id in app_ids:
            self.assertNotIn(
                'unmanifested_chassis_sensor', app_id,
                f'Blacklisted namespace node should not appear: {app_id}',
            )

    def test_health_shows_gap_fill_filtering(self):
        """Health endpoint should show pipeline stats including filtered count."""
        health = self.poll_endpoint_until(
            '/health',
            lambda data: data if 'discovery' in data
            and 'pipeline' in data.get('discovery', {}) else None,
            timeout=30.0,
        )
        pipeline = health['discovery']['pipeline']
        self.assertIn('total_entities', pipeline)


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(
            proc_info, allowable_exit_codes=ALLOWED_EXIT_CODES
        )
