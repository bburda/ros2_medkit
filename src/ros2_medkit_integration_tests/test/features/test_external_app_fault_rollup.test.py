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

"""End-to-end regression for #517: external-app faults across every rollup.

An external app (``external: true`` - e.g. a PLC bridged into SOVD by a protocol
plugin) reports faults to the fault_manager under its own entity id, not a ROS
FQN. Fault-scope resolution must recognise that bare id so the app's faults
surface on the app route AND on every aggregate route that rolls it up
(component, function, area). #517 was the fault scope resolving to a live FQN
instead of the bare id, which silently emptied those rollups.

The fixture app deliberately also carries a ``ros_binding`` (the #517 "neighbor"
case): ``effective_fqn()`` derives a live FQN, so a fault scope that used it
would drop the app's bare-id faults. This test therefore fails against the
pre-fix fault scope and passes only when the external classification wins.

The merge-level preservation is unit-covered
(``MergePipelineTest.PluginExternalClassificationSurvivesManifestMetadataMerge``)
and the per-route scope resolution is unit-covered
(``ResolveEntitySourceFqnsTest.*OwnsItsFaults``). This test pins the full
HTTP-stack behaviour those two disjoint unit layers never exercise together:
report a fault under the bare id, then observe it on every rollup route.
"""

import os
import time
import unittest

from ament_index_python.packages import get_package_share_directory
import launch_testing
import rclpy
from rclpy.node import Node
from ros2_medkit_msgs.msg import Fault
from ros2_medkit_msgs.srv import ReportFault

from ros2_medkit_test_utils.constants import ALLOWED_EXIT_CODES
from ros2_medkit_test_utils.gateway_test_case import GatewayTestCase
from ros2_medkit_test_utils.launch_helpers import create_test_launch

from std_msgs.msg import Float32


EXTERNAL_APP = 'plc-process'
HOST_COMPONENT = 's7-plc'
HOST_FUNCTION = 'level-control'
HOST_AREA = 'plc-cell'
FAULT_CODE = 'PLC_LEVEL_OVERFLOW'
# Topic the test publishes so the fault_manager's rosbag ring buffer has data
# to flush when the fault confirms (no demo nodes run in this launch).
LEVEL_TOPIC = '/plc_cell/level'


def generate_test_description():
    manifest_path = os.path.join(
        get_package_share_directory('ros2_medkit_gateway'),
        'config', 'examples', 'external_app_fault_manifest.yaml',
    )
    return create_test_launch(
        demo_nodes=[],
        fault_manager=True,
        # Negative threshold: the fault confirms after a couple of FAILED
        # events (see _report_fault, which reports several times).
        fault_manager_params={
            'confirmation_threshold': -2,
            'snapshots.rosbag.include_topics': [LEVEL_TOPIC],
        },
        gateway_params={
            'discovery.mode': 'hybrid',
            'discovery.manifest_path': manifest_path,
            'discovery.manifest_strict_validation': False,
        },
    )


class TestExternalAppFaultRollup(GatewayTestCase):
    """An external app owns its faults on the app route and every rollup."""

    MIN_EXPECTED_APPS = 1
    REQUIRED_APPS = {EXTERNAL_APP}
    REQUIRED_AREAS = {HOST_AREA}
    REQUIRED_FUNCTIONS = {HOST_FUNCTION}

    @classmethod
    def setUpClass(cls):
        # Create the publisher BEFORE the gateway wait: the fault_manager's
        # explicit-topic rosbag capture retries type discovery for only ~10s
        # after startup, so LEVEL_TOPIC must be on the graph early.
        rclpy.init()
        cls._reporter = Node('external_app_fault_reporter')
        cls._level_pub = cls._reporter.create_publisher(Float32, LEVEL_TOPIC, 10)
        cls._report_client = cls._reporter.create_client(
            ReportFault, '/fault_manager/report_fault'
        )
        # Wait for the gateway + discovery (the external app, area and function
        # must exist before we exercise the rollup routes).
        super().setUpClass()
        assert cls._report_client.wait_for_service(timeout_sec=15.0), \
            'report_fault service not available'

    @classmethod
    def tearDownClass(cls):
        cls._reporter.destroy_node()
        rclpy.shutdown()

    def _report_fault(self, times=4):
        """Report the external app's fault under its bare entity id.

        Fire-and-forget, exactly like the production ``fault_reporter`` client
        (``async_send_request`` with the reply discarded). We deliberately do
        NOT wait on or assert the service reply: ``wait_for_service`` only
        confirms the *request* path (client->server) is discovery-matched, not
        the *reply* path (server->client), so an early round-trip can lose its
        response even though the fault_manager already recorded the fault
        ("failed to send response ... client will not receive response").
        Asserting on that reply made this test flaky under the sanitizer builds'
        slower discovery. The behavioural assertion is the HTTP rollup poll in
        the test body, which retries for up to 30s and absorbs that timing.

        Reported several times so the negative ``confirmation_threshold``
        latches the fault to CONFIRMED regardless of debounce timing.
        """
        for _ in range(times):
            req = ReportFault.Request()
            req.fault_code = FAULT_CODE
            req.event_type = ReportFault.Request.EVENT_FAILED
            req.severity = Fault.SEVERITY_ERROR
            req.description = 'PLC tank level exceeded high limit'
            req.source_id = EXTERNAL_APP
            # Fire-and-forget: the request is sent synchronously by call_async;
            # spin briefly to flush and pace the reports for the debounce, and
            # intentionally drop the reply future (see docstring).
            self._report_client.call_async(req)
            rclpy.spin_once(self._reporter, timeout_sec=0.1)

    def _publish_level(self, seconds, period=0.05):
        """Keep LEVEL_TOPIC flowing so the capture ring buffer is not empty."""
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            self._level_pub.publish(Float32(data=42.0))
            time.sleep(period)

    def test_external_app_bulk_data_uris_serve_the_captured_bag(self):
        """The advertised bulk_data_uri serves the bag on app AND area routes.

        The rosbag captured when the external app's fault confirms is keyed by
        the fault's reporting source - the app's bare entity id. Every route
        that lists the fault must also serve its bag. The area case is the
        regression: ``GET /areas/plc-cell/faults/<code>`` advertises
        ``/areas/plc-cell/bulk-data/rosbags/<code>``, which 404'd while the
        area resolved its bulk-data scope through the namespace path instead
        of its hosted apps' reporting sources. Since #620 the ``<code>`` in
        those URLs is a recording id; the bare fault code still resolves
        through the compatibility path and is checked here too.

        Runs before the rollup test (method order is alphabetical) and reports
        the same fault code; the report is idempotent for both tests.

        @verifies REQ_INTEROP_072
        """
        # Fill the capture buffer around the confirmation edge: the bag is
        # flushed once, on CONFIRMED, from whatever the buffer holds.
        self._publish_level(seconds=0.5)
        self._report_fault()
        self._publish_level(seconds=1.0)

        self.wait_for_fault(f'/apps/{EXTERNAL_APP}', FAULT_CODE)
        bag_id = self.wait_for_fault_with_rosbag(f'/apps/{EXTERNAL_APP}')
        # A bag is addressed by its recording id, which names the fault it was
        # captured for without being equal to it - one fault can hold several.
        self.assertTrue(
            bag_id.startswith(f'fault_{FAULT_CODE}_'),
            f'unexpected recording id: {bag_id}')

        # App-level download: 200 + bytes.
        app_uri = f'/apps/{EXTERNAL_APP}/bulk-data/rosbags/{bag_id}'
        resp = self.get_raw(app_uri)
        self.assertGreater(len(resp.content), 0, f'{app_uri} returned no bytes')

        # The pre-#620 address - the bare fault code - still serves the same
        # bag, so anything built against the documented URL keeps working.
        legacy_uri = f'/apps/{EXTERNAL_APP}/bulk-data/rosbags/{FAULT_CODE}'
        legacy = self.get_raw(legacy_uri)
        self.assertEqual(legacy.content, resp.content,
                         f'{legacy_uri} no longer serves the same bytes')

        # Area rollup advertises an area-scoped URI for the same bag.
        detail = self.wait_for_fault_detail(
            f'/areas/{HOST_AREA}', snapshot_types={'rosbag'})
        rosbag_snaps = [
            s for s in detail['environment_data']['snapshots']
            if s.get('type') == 'rosbag'
        ]
        self.assertTrue(
            rosbag_snaps, 'area fault detail lost the rosbag snapshot')
        area_uri = rosbag_snaps[0].get('bulk_data_uri')
        self.assertEqual(
            area_uri, f'/areas/{HOST_AREA}/bulk-data/rosbags/{bag_id}')

        # The area listing shows the bag and the advertised URI downloads.
        listing = self.get_json(f'/areas/{HOST_AREA}/bulk-data/rosbags')
        listed_ids = [item.get('id') for item in listing.get('items', [])]
        self.assertIn(bag_id, listed_ids)
        resp = self.get_raw(area_uri)
        self.assertGreater(len(resp.content), 0, f'{area_uri} returned no bytes')

        # Function rollup resolves the same scope.
        listing = self.get_json(f'/functions/{HOST_FUNCTION}/bulk-data/rosbags')
        listed_ids = [item.get('id') for item in listing.get('items', [])]
        self.assertIn(bag_id, listed_ids)

    def test_external_app_fault_appears_on_every_rollup(self):
        """The external app's fault surfaces on app, component, function, area.

        @verifies REQ_INTEROP_012
        """
        self._report_fault()

        # Precondition: the owning app sees the fault. If this fails the setup
        # is wrong (not the rollup), so assert it first for a clear signal.
        self.wait_for_fault(f'/apps/{EXTERNAL_APP}', FAULT_CODE)

        # #517: the same fault must roll up to every aggregate route that
        # includes the external app. Before the fix these rollups were empty
        # because the app's bare-id fault scope was dropped.
        for endpoint in (
            f'/components/{HOST_COMPONENT}',
            f'/functions/{HOST_FUNCTION}',
            f'/areas/{HOST_AREA}',
        ):
            with self.subTest(rollup=endpoint):
                fault = self.wait_for_fault(endpoint, FAULT_CODE)
                self.assertEqual(fault.get('fault_code'), FAULT_CODE)


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        """Check all processes exited cleanly (SIGTERM allowed)."""
        for info in proc_info:
            self.assertIn(
                info.returncode, ALLOWED_EXIT_CODES,
                f'{info.process_name} exited with code {info.returncode}'
            )
