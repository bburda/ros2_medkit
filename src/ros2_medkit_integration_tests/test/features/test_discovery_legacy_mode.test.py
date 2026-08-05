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
Integration tests for runtime discovery without HostInfoProvider.

When default_component is disabled, runtime discovery returns no components
(Components come from HostInfoProvider or manifest only).
"""

import unittest

from launch import LaunchDescription
from launch.actions import TimerAction
import launch_testing
import launch_testing.actions

from ros2_medkit_test_utils.constants import ALLOWED_EXIT_CODES
from ros2_medkit_test_utils.gateway_test_case import GatewayTestCase
from ros2_medkit_test_utils.launch_helpers import create_demo_nodes, create_gateway_node


def generate_test_description():
    gateway_node = create_gateway_node(
        extra_params={
            'discovery.runtime.default_component.enabled': False,
        },
    )

    demo_nodes = create_demo_nodes(
        ['temp_sensor', 'rpm_sensor', 'pressure_sensor'],
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
class TestNoHostInfoProviderMode(GatewayTestCase):
    """Test runtime discovery without HostInfoProvider (no components)."""

    def test_no_components_without_host_provider(self):
        """No components should exist when HostInfoProvider is disabled."""
        self.poll_endpoint_until(
            '/apps',
            lambda d: d if len(d.get('items', [])) >= 3 else None,
            timeout=60.0,
        )
        # Now check components - should be empty
        comp_data = self.get_json('/components')
        components = comp_data.get('items', [])
        self.assertEqual(
            len(components), 0,
            f'Expected no components without HostInfoProvider, '
            f"got: {[c.get('id') for c in components]}",
        )

    def test_apps_still_discovered(self):
        """Apps should still be discovered even without HostInfoProvider."""
        expected = ['temp_sensor', 'rpm_sensor', 'pressure_sensor']
        data = self.poll_endpoint_until(
            '/apps',
            lambda d: d if all(
                any(name in a.get('id', '') for a in d.get('items', []))
                for name in expected
            ) else None,
            timeout=60.0,
        )
        app_ids = [a['id'] for a in data['items']]
        for name in expected:
            self.assertTrue(
                any(name in aid for aid in app_ids),
                f'{name} not found in apps: {app_ids}',
            )


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(
            proc_info, allowable_exit_codes=ALLOWED_EXIT_CODES
        )
