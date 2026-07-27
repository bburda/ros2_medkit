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

"""Startup catch-up for zero-config entity freeze-frames.

The fault manager is started first and a fault is confirmed under the demo
data-provider plugin's app id BEFORE the gateway launches (the seeder process
exits only once ListFaults shows the fault CONFIRMED, and the gateway is
launched on that exit). The gateway therefore never sees the confirm event -
its freeze-frame path must recover the standing fault via the startup
catch-up, which lists confirmed faults at init and snapshots the entity's
current values. The frame is marked ``capture_origin: startup`` because the
values are read at gateway start, not at confirm time.
"""

import os
import sys
import time
import unittest

from ament_index_python.packages import get_package_prefix
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
import launch_testing
import launch_testing.actions
import requests

from ros2_medkit_test_utils.constants import ALLOWED_EXIT_CODES
from ros2_medkit_test_utils.gateway_test_case import GatewayTestCase
from ros2_medkit_test_utils.launch_helpers import (
    create_fault_manager_node,
    create_gateway_node,
)

PLUGIN_APP = 'test_plc_app'
FAULT_CODE = 'PLC_STANDING_AT_BOOT'


def _get_plugin_path(so_name):
    """Get path to a demo plugin .so."""
    pkg_prefix = get_package_prefix('ros2_medkit_gateway')
    return os.path.join(pkg_prefix, 'lib', 'ros2_medkit_gateway', so_name)


def generate_test_description():
    fault_manager = create_fault_manager_node(
        extra_params={
            # [''] is the launch-file spelling of "no default topics"
            # (dropped by the node); the gateway's entity catch-up is the
            # only path that can produce a freeze-frame here.
            'snapshots.default_topics': [''],
            'snapshots.rosbag.enabled': False,
        },
    )

    seeder = ExecuteProcess(
        cmd=[
            sys.executable,
            os.path.join(
                os.path.dirname(os.path.abspath(__file__)),
                'standing_fault_seeder.py',
            ),
        ],
        name='standing_fault_seeder',
        output='screen',
    )

    gateway = create_gateway_node(
        extra_params={
            'plugins': ['test_data_provider'],
            'plugins.test_data_provider.path':
                _get_plugin_path('libtest_data_provider_plugin.so'),
        },
    )

    return (
        LaunchDescription([
            fault_manager,
            TimerAction(period=1.0, actions=[seeder]),
            # The gateway starts only after the fault stands CONFIRMED, so
            # the confirm event is guaranteed to predate its subscription.
            RegisterEventHandler(
                OnProcessExit(target_action=seeder, on_exit=[gateway])
            ),
            launch_testing.actions.ReadyToTest(),
        ]),
        {'gateway_node': gateway},
    )


class TestStandingFaultFreezeFrame(GatewayTestCase):
    """A fault confirmed before gateway start still gets a freeze-frame."""

    MIN_EXPECTED_APPS = 1
    REQUIRED_APPS = {PLUGIN_APP}

    @classmethod
    def setUpClass(cls):
        # The gateway launches only after the seeder has confirmed the fault,
        # so the boot chain (fault manager + seeder + gateway) needs more
        # headroom than the default health budget, which starts counting at
        # ReadyToTest. Pre-wait here; the base class waits then pass quickly.
        deadline = time.monotonic() + 60.0
        while time.monotonic() < deadline:
            try:
                response = requests.get(f'{cls.BASE_URL}/health', timeout=2)
                if response.status_code == 200:
                    break
            except requests.exceptions.RequestException:
                pass
            time.sleep(1.0)
        super().setUpClass()

    def test_standing_fault_snapshot_contains_freeze_frame(self):
        """The standing fault's detail carries the entity's own values.

        @verifies REQ_INTEROP_088
        """
        endpoint = f'/apps/{PLUGIN_APP}/faults/{FAULT_CODE}'
        deadline = time.monotonic() + 30.0
        frame = None
        last = None
        while time.monotonic() < deadline and frame is None:
            try:
                response = requests.get(
                    f'{self.BASE_URL}{endpoint}', timeout=5)
                if response.status_code == 200:
                    last = response.json()
                    snapshots = last.get(
                        'environment_data', {}).get('snapshots', [])
                    frame = next(
                        (s for s in snapshots
                         if s.get('type') == 'freeze_frame'
                         and s.get('name') == PLUGIN_APP),
                        None,
                    )
            except requests.exceptions.RequestException:
                pass
            if frame is None:
                time.sleep(0.5)
        self.assertIsNotNone(
            frame,
            f'No freeze_frame for {FAULT_CODE} within 30s; last: {last}',
        )

        # The frame holds the entity's current values from its DataProvider.
        self.assertEqual(frame['data'].get('temperature'), 42.5)
        self.assertEqual(frame['data'].get('pressure'), 3.2)

        # Catch-up frames are marked: values were read at gateway start, not
        # when the fault confirmed (which predates the gateway process).
        x_medkit = frame.get('x-medkit', {})
        self.assertEqual(x_medkit.get('capture_origin'), 'startup')
        self.assertIn('full_data', x_medkit)
        self.assertIn('captured_at', x_medkit)


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        """Check all processes exited cleanly (SIGTERM allowed)."""
        for info in proc_info:
            self.assertIn(
                info.returncode, ALLOWED_EXIT_CODES,
                f'{info.process_name} exited with code {info.returncode}'
            )
