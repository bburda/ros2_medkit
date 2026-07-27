#!/usr/bin/env python3
# Copyright 2026 mfaferek93
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

"""Zero-config freeze-frames: the faulting entity's own data, no snapshot config.

The fault manager runs WITHOUT any snapshot configuration (empty
``snapshots.default_topics``, no config file). Three entity-default paths are
proven end to end:

1. Plugin-backed entity with a DataProvider: a fault reported under the demo
   data-provider plugin's bare app id carries a freeze-frame of that entity's
   current data values, captured by the gateway from the plugin's DataProvider
   at confirm time.
2. Plugin-backed entity WITHOUT a DataProvider (the commercial PLC bridge
   shape): the demo route-data plugin exports only introspection + fault
   providers and serves live values through its registered x-plc-data route.
   The gateway captures the freeze-frame by dispatching that route in-process.
   The plugin's second entity reports its link down (connected=false) while
   still serving last known values - the loss-of-comms fault must freeze
   those values and mark the frame disconnected in x-medkit.
3. ROS-backed entity: a fault reported under a demo node's FQN carries a
   freeze-frame of that node's own published topics, captured by the fault
   manager's entity-default fallback.
"""

import os
import time
import unittest

from ament_index_python.packages import get_package_prefix
import launch_testing
import rclpy
from rclpy.node import Node
import requests
from ros2_medkit_msgs.msg import Fault
from ros2_medkit_msgs.srv import ReportFault

from ros2_medkit_test_utils.constants import ALLOWED_EXIT_CODES
from ros2_medkit_test_utils.gateway_test_case import GatewayTestCase
from ros2_medkit_test_utils.launch_helpers import create_test_launch


PLUGIN_APP = 'test_plc_app'
PLUGIN_FAULT_CODE = 'PLC_OVERPRESSURE'
ROUTE_PLUGIN_APP = 'test_route_plc_app'
ROUTE_PLUGIN_FAULT_CODE = 'PLC_ROUTE_LEVEL_HIGH'
DOWN_PLUGIN_APP = 'test_route_plc_down_app'
DOWN_PLUGIN_FAULT_CODE = 'PLC_ROUTE_COMMS_LOST'
ROS_APP = 'temp_sensor'
ROS_SOURCE = '/powertrain/engine/temp_sensor'
ROS_FAULT_CODE = 'ENGINE_TEMP_SENSOR_DEGRADED'


def _get_plugin_path(so_name):
    """Get path to a demo plugin .so."""
    pkg_prefix = get_package_prefix('ros2_medkit_gateway')
    return os.path.join(pkg_prefix, 'lib', 'ros2_medkit_gateway', so_name)


def generate_test_description():
    return create_test_launch(
        demo_nodes=['temp_sensor'],
        fault_manager=True,
        # No snapshot config at all: entity-default capture is the only path
        # that can produce a freeze-frame.
        fault_manager_params={
            # [''] is the launch-file spelling of "no default topics"
            # (dropped by the node); entity-default is the only capture path.
            'snapshots.default_topics': [''],
            'snapshots.rosbag.enabled': False,
            # Generous on-demand sampling window for slow demo publishers.
            'snapshots.timeout_sec': 5.0,
        },
        gateway_params={
            'plugins': ['test_data_provider', 'test_route_data'],
            'plugins.test_data_provider.path':
                _get_plugin_path('libtest_data_provider_plugin.so'),
            'plugins.test_route_data.path':
                _get_plugin_path('libtest_route_data_plugin.so'),
        },
    )


class TestEntityFreezeFrame(GatewayTestCase):
    """Faults carry the faulting entity's own data with zero snapshot config."""

    MIN_EXPECTED_APPS = 4
    REQUIRED_APPS = {PLUGIN_APP, ROUTE_PLUGIN_APP, DOWN_PLUGIN_APP, ROS_APP}

    @classmethod
    def setUpClass(cls):
        super().setUpClass()
        rclpy.init()
        cls._reporter = Node('entity_freeze_frame_fault_reporter')
        cls._report_client = cls._reporter.create_client(
            ReportFault, '/fault_manager/report_fault'
        )
        assert cls._report_client.wait_for_service(timeout_sec=15.0), \
            'report_fault service not available'

    @classmethod
    def tearDownClass(cls):
        cls._reporter.destroy_node()
        rclpy.shutdown()

    def _report_fault(self, fault_code, source_id, times=4):
        """Report a fault, fire-and-forget (see test_external_app_fault_rollup)."""
        for _ in range(times):
            req = ReportFault.Request()
            req.fault_code = fault_code
            req.event_type = ReportFault.Request.EVENT_FAILED
            req.severity = Fault.SEVERITY_ERROR
            req.description = 'entity freeze-frame test fault'
            req.source_id = source_id
            self._report_client.call_async(req)
            rclpy.spin_once(self._reporter, timeout_sec=0.1)

    def _wait_for_freeze_frame(self, entity_endpoint, fault_code, *, max_wait=30.0):
        """Poll the fault detail until a freeze_frame snapshot appears."""
        deadline = time.monotonic() + max_wait
        last = None
        while time.monotonic() < deadline:
            try:
                response = requests.get(
                    f'{self.BASE_URL}{entity_endpoint}/faults/{fault_code}',
                    timeout=5,
                )
                if response.status_code == 200:
                    last = response.json()
                    snapshots = last.get(
                        'environment_data', {}).get('snapshots', [])
                    frames = [s for s in snapshots
                              if s.get('type') == 'freeze_frame']
                    if frames:
                        return frames
            except requests.exceptions.RequestException:
                pass
            time.sleep(0.5)
        raise AssertionError(
            f'No freeze_frame snapshot for {fault_code} at {entity_endpoint} '
            f'within {max_wait}s; last response: {last}'
        )

    def test_plugin_entity_fault_carries_entity_values(self):
        """A PLC-style plugin fault carries the entity's own data values.

        @verifies REQ_INTEROP_088
        """
        self._report_fault(PLUGIN_FAULT_CODE, PLUGIN_APP)

        frames = self._wait_for_freeze_frame(
            f'/apps/{PLUGIN_APP}', PLUGIN_FAULT_CODE)
        frame = next(
            (f for f in frames if f.get('name') == PLUGIN_APP), None)
        self.assertIsNotNone(
            frame, f'No frame named {PLUGIN_APP} in {frames}')

        # The frame holds the plugin entity's current values as served by
        # its DataProvider (compact {resource_id: value} dict).
        self.assertEqual(frame['data'].get('temperature'), 42.5)
        self.assertEqual(frame['data'].get('pressure'), 3.2)

        # Standard freeze_frame x-medkit metadata block (fleet_ui contract).
        x_medkit = frame.get('x-medkit', {})
        self.assertIn('full_data', x_medkit)
        self.assertIn('captured_at', x_medkit)

    def test_route_only_plugin_fault_carries_entity_values(self):
        """Freeze-frame a commercial-bridge-shaped plugin entity.

        With no DataProvider and no FaultProvider the entity still carries
        its own values, captured via in-process x-plc-data dispatch and
        served through the fault_manager fall-through.

        @verifies REQ_INTEROP_088
        """
        self._report_fault(ROUTE_PLUGIN_FAULT_CODE, ROUTE_PLUGIN_APP)

        frames = self._wait_for_freeze_frame(
            f'/apps/{ROUTE_PLUGIN_APP}', ROUTE_PLUGIN_FAULT_CODE)
        frame = next(
            (f for f in frames if f.get('name') == ROUTE_PLUGIN_APP), None)
        self.assertIsNotNone(
            frame, f'No frame named {ROUTE_PLUGIN_APP} in {frames}')

        # The frame holds the canned values the plugin serves exclusively
        # through its registered x-plc-data route (keyed by item "name").
        self.assertEqual(frame['data'].get('level'), 87.5)
        self.assertEqual(frame['data'].get('alarm'), True)

        # Standard freeze_frame x-medkit metadata block (fleet_ui contract).
        x_medkit = frame.get('x-medkit', {})
        self.assertIn('full_data', x_medkit)
        self.assertIn('captured_at', x_medkit)

        # The plugin has no FaultProvider: the entity fault list and clear
        # must be served by the fault_manager fall-through, not 404.
        list_resp = requests.get(
            f'{self.BASE_URL}/apps/{ROUTE_PLUGIN_APP}/faults', timeout=5)
        self.assertEqual(list_resp.status_code, 200)
        codes = [i.get('fault_code', i.get('code'))
                 for i in list_resp.json().get('items', [])]
        self.assertIn(ROUTE_PLUGIN_FAULT_CODE, codes)

        clear_resp = requests.delete(
            f'{self.BASE_URL}/apps/{ROUTE_PLUGIN_APP}/faults/'
            f'{ROUTE_PLUGIN_FAULT_CODE}',
            timeout=5)
        self.assertIn(clear_resp.status_code, (200, 204))

    def test_route_only_plugin_disconnected_entity_carries_last_known_values(self):
        """Freeze-frame a route-only plugin entity that reports its link down.

        The last known values it serves are the point of a loss-of-comms
        fault, and x-medkit marks the frame disconnected with the payload's
        own timestamp.

        @verifies REQ_INTEROP_088
        """
        self._report_fault(DOWN_PLUGIN_FAULT_CODE, DOWN_PLUGIN_APP)

        frames = self._wait_for_freeze_frame(
            f'/apps/{DOWN_PLUGIN_APP}', DOWN_PLUGIN_FAULT_CODE)
        frame = next(
            (f for f in frames if f.get('name') == DOWN_PLUGIN_APP), None)
        self.assertIsNotNone(
            frame, f'No frame named {DOWN_PLUGIN_APP} in {frames}')

        # The plugin serves these as its last known values with connected=false.
        self.assertEqual(frame['data'].get('level'), 42.0)
        self.assertEqual(frame['data'].get('alarm'), False)

        x_medkit = frame.get('x-medkit', {})
        self.assertIn('full_data', x_medkit)
        self.assertIn('captured_at', x_medkit)
        self.assertEqual(x_medkit.get('connected'), False)
        self.assertEqual(x_medkit.get('source_timestamp'), 1234567800)

    def test_ros_entity_fault_carries_own_topic_data(self):
        """A ROS-node fault carries the node's own published topic data.

        @verifies REQ_INTEROP_088
        """
        self._report_fault(ROS_FAULT_CODE, ROS_SOURCE)

        frames = self._wait_for_freeze_frame(
            f'/apps/{ROS_APP}', ROS_FAULT_CODE)

        # The fault manager's entity-default capture aggregates the source
        # node's own topics; temp_sensor publishes .../temperature.
        all_topics = set()
        for frame in frames:
            x_medkit = frame.get('x-medkit', {})
            topic = x_medkit.get('topic', '')
            if topic:
                all_topics.add(topic)
            full_data = x_medkit.get('full_data', {})
            if isinstance(full_data, dict):
                all_topics.update(full_data.keys())
        self.assertTrue(
            any('temperature' in t for t in all_topics),
            f'temp_sensor topic not captured; topics seen: {all_topics}',
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
