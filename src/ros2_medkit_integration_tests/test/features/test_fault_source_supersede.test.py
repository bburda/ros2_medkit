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

"""End-to-end regression test for provisional-source re-attribution (#467).

A reporter that does not yet know the entity's node FQN (e.g. the
action_status_bridge before DDS discovery resolves the server node) reports a
fault under a provisional source - an action interface name that maps to no
SOVD entity. Under the strict-AND per-entity /faults scope filter that fault is
correctly kept out of every node entity.

ReportFault.supersedes_source_id lets the reporter correct the attribution: it
re-reports under the resolved FQN and names the provisional source to drop, so
the FaultManager ends with reporting_sources = {FQN} and the fault resolves to
the server node's entity. This drives that full path through the real gateway
scope filter: provisional -> 404 under the entity, then supersede -> 200.

@verifies REQ_INTEROP_013
"""

import os
import unittest

from ament_index_python.packages import get_package_share_directory
import launch_testing
import rclpy
from rclpy.node import Node
import requests

from ros2_medkit_msgs.msg import Fault
from ros2_medkit_msgs.srv import ReportFault
from ros2_medkit_test_utils.constants import ALLOWED_EXIT_CODES
from ros2_medkit_test_utils.gateway_test_case import GatewayTestCase
from ros2_medkit_test_utils.launch_helpers import create_test_launch


# The manifest maps this node FQN to the 'lidar-sensor' app on the 'lidar-unit'
# component (same mapping the scope-isolation regression test relies on).
SERVER_FQN = '/perception/lidar/lidar_sensor'
OWNER_APP = 'lidar-sensor'
# A provisional source that maps to no SOVD entity - stands in for the action
# interface name the action_status_bridge falls back to before discovery.
PROVISIONAL_SOURCE = '/navigate_to_pose'
FAULT_CODE = 'ACTION_NAVIGATE_TO_POSE_ABORTED'


def generate_test_description():
    manifest_path = os.path.join(
        get_package_share_directory('ros2_medkit_gateway'),
        'config', 'examples', 'demo_nodes_manifest.yaml',
    )
    return create_test_launch(
        demo_nodes=['lidar_sensor'],
        fault_manager=True,
        # Negative threshold: a fault confirms after a couple of FAILED events.
        fault_manager_params={'confirmation_threshold': -2},
        gateway_params={
            'discovery.mode': 'hybrid',
            'discovery.manifest_path': manifest_path,
            'discovery.manifest_strict_validation': False,
            'discovery.merge_pipeline.gap_fill.allow_heuristic_apps': True,
        },
    )


class TestFaultSourceSupersede(GatewayTestCase):
    """A provisional source is corrected to the server FQN via supersede."""

    MIN_EXPECTED_APPS = 1
    REQUIRED_APPS = {OWNER_APP}

    @classmethod
    def setUpClass(cls):
        super().setUpClass()
        rclpy.init()
        cls._reporter = Node('supersede_fault_reporter')
        cls._report_client = cls._reporter.create_client(
            ReportFault, '/fault_manager/report_fault'
        )
        assert cls._report_client.wait_for_service(timeout_sec=15.0), \
            'report_fault service not available'

    @classmethod
    def tearDownClass(cls):
        cls._reporter.destroy_node()
        rclpy.shutdown()

    def _report(self, source_id, supersedes='', times=4):
        """Fire-and-forget FAILED reports.

        Drop the reply future to avoid sanitizer-timing flakes; report several
        times so the negative confirmation_threshold latches CONFIRMED.
        """
        for _ in range(times):
            req = ReportFault.Request()
            req.fault_code = FAULT_CODE
            req.event_type = ReportFault.Request.EVENT_FAILED
            req.severity = Fault.SEVERITY_ERROR
            req.description = 'Action /navigate_to_pose aborted'
            req.source_id = source_id
            req.supersedes_source_id = supersedes
            self._report_client.call_async(req)
            rclpy.spin_once(self._reporter, timeout_sec=0.1)

    def _fault_status_on_owner(self):
        return requests.get(
            f'{self.BASE_URL}/apps/{OWNER_APP}/faults/{FAULT_CODE}', timeout=10,
        ).status_code

    def test_supersede_moves_fault_to_server_entity(self):
        """Provisional attribution is invisible to the entity; supersede fixes it.

        @verifies REQ_INTEROP_013
        """
        # 1. Report under the provisional (action-name) source. It maps to no
        #    entity, so the strict-AND scope filter keeps it off the owning app.
        self._report(PROVISIONAL_SOURCE)
        self._poll_until(
            lambda: self._fault_status_on_owner() == 404,
            'fault reported under a provisional source must not surface under '
            'the server entity',
        )

        # 2. Re-report under the resolved FQN, superseding the provisional
        #    source. The FaultManager drops the provisional source, leaving
        #    reporting_sources = {FQN}, so the fault now resolves to the entity.
        self._report(SERVER_FQN, supersedes=PROVISIONAL_SOURCE)
        self._poll_until(
            lambda: self._fault_status_on_owner() == 200,
            'after supersede the fault must resolve to the server entity',
        )

        # The corrected fault detail carries the FQN and not the provisional
        # source. reporting_sources lives under the x-medkit extension object,
        # not the SOVD "item" block (which only carries code/name/severity/status).
        detail = self.get_json(f'/apps/{OWNER_APP}/faults/{FAULT_CODE}')
        sources = detail.get('x-medkit', {}).get('reporting_sources', [])
        self.assertIn(SERVER_FQN, sources)
        self.assertNotIn(PROVISIONAL_SOURCE, sources)

    def _poll_until(self, predicate, message, timeout=20.0, interval=0.5):
        import time
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            try:
                if predicate():
                    return
            except requests.RequestException:
                pass
            rclpy.spin_once(self._reporter, timeout_sec=0.05)
            time.sleep(interval)
        self.fail(f'Timed out after {timeout}s: {message}')


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):
    """Confirm the gateway and fault_manager exited cleanly."""

    def test_exit_codes(self, proc_info):
        """Check all processes exited cleanly (SIGTERM allowed)."""
        for info in proc_info:
            self.assertIn(
                info.returncode, ALLOWED_EXIT_CODES,
                f'{info.process_name} exited with code {info.returncode}'
            )
