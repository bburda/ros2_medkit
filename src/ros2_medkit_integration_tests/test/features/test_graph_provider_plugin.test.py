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

"""Integration tests for graph provider plugin loaded dynamically.

Validates that the graph provider plugin loads via its .so path, registers
the x-medkit-graph capability on Function entities, and serves graph data
at GET /functions/{id}/x-medkit-graph.

Requires hybrid discovery mode with a manifest that defines functions
so that the plugin has entities to operate on.
"""

import os
import unittest

from ament_index_python.packages import get_package_prefix, get_package_share_directory
import launch_testing
import requests

from ros2_medkit_test_utils.constants import ALLOWED_EXIT_CODES
from ros2_medkit_test_utils.gateway_test_case import GatewayTestCase
from ros2_medkit_test_utils.launch_helpers import (
    create_test_launch,
    SENSOR_NODES,
)


def _get_graph_plugin_path():
    """Get path to the graph provider plugin .so."""
    pkg_prefix = get_package_prefix('ros2_medkit_graph_provider')
    return os.path.join(
        pkg_prefix, 'lib', 'ros2_medkit_graph_provider',
        'libros2_medkit_graph_provider_plugin.so',
    )


def generate_test_description():
    """Launch gateway with graph provider plugin and demo nodes."""
    plugin_path = _get_graph_plugin_path()
    pkg_share = get_package_share_directory('ros2_medkit_gateway')
    manifest_path = os.path.join(
        pkg_share, 'config', 'examples', 'demo_nodes_manifest.yaml',
    )

    return create_test_launch(
        demo_nodes=SENSOR_NODES,
        fault_manager=False,
        gateway_params={
            'discovery.mode': 'hybrid',
            'discovery.manifest_path': manifest_path,
            'discovery.manifest_strict_validation': False,
            'plugins': ['graph_provider'],
            'plugins.graph_provider.path': plugin_path,
        },
    )


class TestGraphProviderPlugin(GatewayTestCase):
    """Graph provider plugin integration tests.

    @verifies REQ_INTEROP_003
    """

    MIN_EXPECTED_APPS = 3
    REQUIRED_APPS = {'engine-temp-sensor', 'engine-rpm-sensor', 'brake-pressure-sensor'}

    def _get_any_function_id(self):
        """Get the first discovered function ID."""
        data = self.get_json('/functions')
        items = data.get('items', [])
        self.assertGreater(len(items), 0, 'No functions discovered')
        return items[0]['id']

    def test_01_function_detail_includes_graph_capability(self):
        """Function detail includes x-medkit-graph capability.

        When the graph provider plugin is loaded, all Function entities
        should advertise the x-medkit-graph capability with a proper href.
        """
        func_id = self._get_any_function_id()
        data = self.get_json(f'/functions/{func_id}')

        capabilities = data.get('capabilities', [])
        cap_names = [c['name'] for c in capabilities]
        self.assertIn(
            'x-medkit-graph',
            cap_names,
            f'x-medkit-graph not in capabilities: {cap_names}',
        )

        # Verify href points to the correct path
        graph_cap = next(
            c for c in capabilities if c['name'] == 'x-medkit-graph'
        )
        self.assertIn(
            f'/functions/{func_id}/x-medkit-graph',
            graph_cap['href'],
        )

        # The top-level `x-medkit-graph` link is a second, independent
        # surface: it is emitted off the capability list above, and only
        # while a plugin serves the collection. Its absence without the
        # plugin is pinned by
        # `test_discovery_handlers.cpp::GetFunctionOmitsGraphLinkWithoutAPlugin`;
        # this is the other half - with the plugin loaded, it is there and it
        # matches the capability href.
        self.assertEqual(
            data.get('x-medkit-graph'), graph_cap['href'],
            'the detail link and the capability href must name the same URI')

    def test_02_graph_endpoint_returns_valid_response(self):
        """GET /functions/{id}/x-medkit-graph returns a valid graph document.

        The graph response should contain the x-medkit-graph wrapper with
        schema_version, graph_id, nodes, edges, and topics arrays.
        """
        func_id = self._get_any_function_id()

        data = self.poll_endpoint_until(
            f'/functions/{func_id}/x-medkit-graph',
            lambda d: d if 'x-medkit-graph' in d else None,
            timeout=15.0,
        )

        graph = data['x-medkit-graph']
        self.assertIn('schema_version', graph)
        self.assertEqual(graph['schema_version'], '2.0.0')
        self.assertIn('graph_id', graph)
        self.assertIn('timestamp', graph)
        self.assertIn('scope', graph)
        self.assertEqual(graph['scope']['type'], 'function')
        self.assertEqual(graph['scope']['entity_id'], func_id)
        self.assertIn('pipeline_status', graph)
        self.assertIn('nodes', graph)
        self.assertIn('edges', graph)
        self.assertIn('topics', graph)
        self.assertIsInstance(graph['nodes'], list)
        self.assertIsInstance(graph['edges'], list)
        self.assertIsInstance(graph['topics'], list)

    def test_03_graph_endpoint_nonexistent_function_returns_404(self):
        """GET /functions/nonexistent/x-medkit-graph returns 404."""
        r = requests.get(
            f'{self.BASE_URL}/functions/nonexistent-function/x-medkit-graph',
            timeout=5,
        )
        self.assertEqual(r.status_code, 404)

    def test_04_graph_nodes_reflect_discovered_apps(self):
        """Graph nodes correspond to apps hosted by the function.

        The engine-monitoring function hosts engine-temp-sensor and
        engine-rpm-sensor. Their entity IDs should appear in the graph nodes.
        """
        # Use engine-monitoring which hosts temp_sensor and rpm_sensor
        data = self.poll_endpoint_until(
            '/functions/engine-monitoring/x-medkit-graph',
            lambda d: d if 'x-medkit-graph' in d else None,
            timeout=15.0,
        )

        graph = data['x-medkit-graph']
        node_ids = [n['entity_id'] for n in graph['nodes']]
        self.assertIn('engine-temp-sensor', node_ids)
        self.assertIn('engine-rpm-sensor', node_ids)


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):
    """Verify gateway exits cleanly."""

    def test_exit_codes(self, proc_info):
        for info in proc_info:
            self.assertIn(
                info.returncode,
                ALLOWED_EXIT_CODES,
                f'Process {info.process_name} exited with {info.returncode}',
            )
