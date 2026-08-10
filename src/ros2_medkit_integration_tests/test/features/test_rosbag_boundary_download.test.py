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

"""End-to-end regression for #574: the boundary fault's bag over HTTP.

A burst's second fault confirming right AFTER the first fault's post-fault
recording window closed used to get silently nothing: the ring buffer was
empty (drained by the flush, refills diverted by the post-roll), so no bag and
no metadata row ever appeared and every retrieval failed forever. It now gets
its own post-fault-only bag, and this test pins the full HTTP surface of that
artifact: bulk-data listing metadata, fault-detail snapshot metadata, and the
actual download through the gateway.

The test drives the burst itself (deterministic timeline): the test class owns
the only publisher on the captured topic, so going silent before a report pins
the empty-buffer boundary state. The lidar demo node reports
``LIDAR_CALIBRATION_REQUIRED`` on its own until something calibrates it, and
that fault opens a recording of its own; the burst therefore starts only once
that recording has finalised, or a fault of this test could attach to it and
the boundary would never be exercised.
"""

import os
import tempfile
import time
import unittest

import launch_testing
import rclpy
from rclpy.node import Node
from ros2_medkit_msgs.msg import Fault
from ros2_medkit_msgs.srv import ReportFault

from ros2_medkit_test_utils.constants import ALLOWED_EXIT_CODES
from ros2_medkit_test_utils.gateway_test_case import GatewayTestCase
from ros2_medkit_test_utils.launch_helpers import create_test_launch

from std_msgs.msg import Float32


# The issue's configuration: 2s pre-fault buffer, 0.5s post-fault window.
DURATION_SEC = 2.0
DURATION_AFTER_SEC = 0.5
# Topic the test publishes; the only traffic the rosbag capture records.
PROBE_TOPIC = '/e2e/boundary_probe'
# The demo node the faults are attributed to (its FQN is the fault source).
APP_ENDPOINT = '/apps/lidar_sensor'
SOURCE_FQN = '/perception/lidar/lidar_sensor'
FAULT_A = 'E2E_BOUNDARY_A'
FAULT_B = 'E2E_BOUNDARY_B'
# The demo node's own fault. It confirms shortly after startup and opens a
# recording; the burst below must not overlap it (see the module docstring).
# Nothing has ever been published on the only captured topic at that point, so
# its recording is a zero-message post-fault-only bag - the artifact the config
# docs promise is still listed and downloadable.
LIDAR_OWN_FAULT = 'LIDAR_CALIBRATION_REQUIRED'


def post_only_log_line(fault_code):
    """
    Return the log line the fault manager emits on the boundary path.

    It must carry the fault code: ``proc_output.assertWaitFor`` scans all output
    received so far, and the demo node's own fault takes the same branch long
    before the burst starts, so a code-less pattern matches a stale line and can
    never fail.
    """
    return f"fault '{fault_code}' - recording post-fault window only"


# Bag storage for this test run, removed in the post-shutdown test.
ROSBAG_STORAGE_PATH = tempfile.mkdtemp(prefix='rosbag_e2e_boundary_')


def generate_test_description():
    return create_test_launch(
        demo_nodes=['lidar_sensor'],
        # Healthy lidar: the test drives every fault itself, so the demo node
        # must not trigger captures at unpredictable times.
        lidar_faulty=False,
        fault_manager=True,
        fault_manager_params={
            'confirmation_threshold': -1,  # Single report confirms immediately
            'snapshots.rosbag.duration_sec': DURATION_SEC,
            'snapshots.rosbag.duration_after_sec': DURATION_AFTER_SEC,
            'snapshots.rosbag.include_topics': [PROBE_TOPIC],
            'snapshots.rosbag.format': 'mcap',
            'snapshots.rosbag.storage_path': ROSBAG_STORAGE_PATH,
        },
    )


class TestRosbagBoundaryDownload(GatewayTestCase):
    """The post-fault-only bag is listed, described, and downloadable."""

    MIN_EXPECTED_APPS = 1
    REQUIRED_APPS = {'lidar_sensor'}

    @classmethod
    def setUpClass(cls):
        # Publisher and clients BEFORE the gateway/discovery wait: the fault
        # manager's explicit-topic rosbag capture retries type discovery for
        # only ~10s after startup, so PROBE_TOPIC must be on the graph early.
        rclpy.init()
        cls.driver = Node('boundary_burst_driver')
        cls.probe_pub = cls.driver.create_publisher(Float32, PROBE_TOPIC, 10)
        cls.report_client = cls.driver.create_client(
            ReportFault, '/fault_manager/report_fault'
        )
        super().setUpClass()
        assert cls.report_client.wait_for_service(timeout_sec=20.0), \
            'report_fault service not available'
        # The capture subscription must exist before published messages count.
        deadline = time.time() + 15.0
        while (not cls.driver.get_subscriptions_info_by_topic(PROBE_TOPIC)
               and time.time() < deadline):
            time.sleep(0.2)
        assert cls.driver.get_subscriptions_info_by_topic(PROBE_TOPIC), \
            'fault_manager never subscribed to the probe topic'

    @classmethod
    def tearDownClass(cls):
        cls.driver.destroy_node()
        rclpy.shutdown()
        super().tearDownClass()

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _report_fault(self, fault_code):
        request = ReportFault.Request()
        request.fault_code = fault_code
        request.event_type = ReportFault.Request.EVENT_FAILED
        request.severity = Fault.SEVERITY_ERROR
        request.description = 'boundary e2e fault'
        request.source_id = SOURCE_FQN
        future = self.report_client.call_async(request)
        rclpy.spin_until_future_complete(self.driver, future, timeout_sec=10.0)
        self.assertIsNotNone(future.result(), 'report_fault call timed out')
        self.assertTrue(future.result().accepted)

    def _publish_for(self, duration_sec, rate_hz=20.0):
        """Publish probe messages for *duration_sec*, then go silent."""
        msg = Float32()
        msg.data = 1.0
        deadline = time.time() + duration_sec
        period = 1.0 / rate_hz
        while time.time() < deadline:
            self.probe_pub.publish(msg)
            time.sleep(period)

    def _wait_for_bag_descriptor(self, fault_code, timeout=20.0):
        """Poll the bulk-data rosbag listing until *fault_code* has an item.

        Returns ``None`` on timeout rather than failing, so each call site can
        say what its own missing descriptor means.
        """
        deadline = time.time() + timeout
        while time.time() < deadline:
            listing = self.get_json(f'{APP_ENDPOINT}/bulk-data/rosbags')
            for item in listing.get('items', []):
                if item.get('id') == fault_code:
                    return item
            time.sleep(0.5)
        return None

    # ------------------------------------------------------------------
    # Test
    # ------------------------------------------------------------------

    def test_01_boundary_fault_bag_via_gateway(self, proc_output):
        """Burst over the boundary; B's bag served end to end.

        @verifies REQ_INTEROP_073
        """
        # The demo node's own fault opens a recording of its own. Its descriptor
        # exists only once that recording finalised (rows are written at
        # finalize), so waiting for it is what guarantees the burst below runs
        # against an idle state machine instead of attaching to it.
        desc_lidar = self._wait_for_bag_descriptor(LIDAR_OWN_FAULT, timeout=25.0)
        self.assertIsNotNone(
            desc_lidar,
            f"the demo node's own {LIDAR_OWN_FAULT} recording never finalised; "
            'the burst below would attach to it instead of exercising the '
            'window boundary')

        # That recording is a zero-message post-fault-only bag: nothing has ever
        # been published on the only captured topic. The config docs promise such
        # a bag is still listed and downloadable, so pin it over HTTP rather than
        # only at the fault manager's service.
        empty_bag = self.get_raw(
            f'{APP_ENDPOINT}/bulk-data/rosbags/{LIDAR_OWN_FAULT}', timeout=10)
        self.assertEqual(empty_bag.content[:5], b'\x89MCAP',
                         'a zero-message bag must still download as a valid mcap')

        # Fill the ring buffer, then go silent BEFORE reporting so no message
        # can slip into the buffer between A's finalize and B's flush.
        self._publish_for(DURATION_SEC + 0.5)
        self._report_fault(FAULT_A)

        # A's descriptor appearing means its recording finalised (rows are
        # written at finalize); the ring buffer is empty by construction.
        desc_a = self._wait_for_bag_descriptor(FAULT_A, timeout=15.0)
        self.assertIsNotNone(desc_a, 'fault A must get a full bag')

        # The boundary case: B confirms right after A's window closed.
        self._report_fault(FAULT_B)

        # Wait until B's OWN post-fault-only recording opens, then resume
        # publishing so its window has content. The pattern carries the fault
        # code deliberately: assertWaitFor scans everything received so far, and
        # the demo node's fault logged the same sentence before this test began.
        proc_output.assertWaitFor(
            expected_output=post_only_log_line(FAULT_B), timeout=10.0,
        )
        self._publish_for(1.0)

        # --- Bulk-data listing metadata describes the post-only bag ---
        desc_b = self._wait_for_bag_descriptor(FAULT_B, timeout=15.0)
        self.assertIsNotNone(
            desc_b,
            'the boundary fault must get its own bag descriptor')
        self.assertEqual(desc_b['mimetype'], 'application/x-mcap')
        x_medkit = desc_b.get('x-medkit', {})
        self.assertEqual(x_medkit.get('format'), 'mcap')
        # duration_sec honesty: ~the post-fault window, never pre+post (2.5s).
        self.assertGreaterEqual(x_medkit.get('duration_sec', -1.0), 0.3)
        self.assertLessEqual(x_medkit.get('duration_sec', -1.0), 1.5)
        # Its own recording: neither A's (which finalised before B confirmed)
        # nor the demo node's (which finalised before the burst started).
        self.assertNotEqual(x_medkit.get('recording_id'),
                            desc_a.get('x-medkit', {}).get('recording_id'))
        self.assertNotEqual(x_medkit.get('recording_id'),
                            desc_lidar.get('x-medkit', {}).get('recording_id'))

        # --- Fault-detail snapshot metadata shows the bag ---
        detail = self.poll_endpoint_until(
            f'{APP_ENDPOINT}/faults/{FAULT_B}',
            lambda d: next(
                (s for s in d.get('environment_data', {}).get('snapshots', [])
                 if s.get('type') == 'rosbag'),
                None,
            ),
            timeout=10.0,
            interval=0.5,
        )
        self.assertIsNotNone(
            detail, "fault B's detail must list a rosbag snapshot")
        self.assertTrue(
            detail['bulk_data_uri'].endswith(
                f'/bulk-data/rosbags/{FAULT_B}'),
            f'unexpected bulk_data_uri: {detail["bulk_data_uri"]}')

        # --- Download through the gateway ---
        response = self.get_raw(
            f'{APP_ENDPOINT}/bulk-data/rosbags/{FAULT_B}',
            timeout=10,
        )
        self.assertIn('application/x-mcap',
                      response.headers.get('Content-Type', ''))
        self.assertEqual(response.content[:5], b'\x89MCAP')
        # A non-empty payload proves nothing on its own - the zero-message bag
        # downloaded at the top of this test is non-empty too (header + footer).
        # B recorded a full second of a 20 Hz topic, so it must be the larger of
        # the two by a wide margin.
        self.assertGreater(
            len(response.content), len(empty_bag.content),
            "the boundary fault's bag is no bigger than a bag with no messages "
            'in it, so its post-fault window was not recorded')


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):
    """Post-shutdown checks."""

    def test_exit_codes(self, proc_info):
        """Gateway and fault manager exit cleanly."""
        launch_testing.asserts.assertExitCodes(
            proc_info, allowable_exit_codes=ALLOWED_EXIT_CODES,
        )

    def test_cleanup_storage_directory(self):
        """Remove the bag storage directory."""
        import shutil
        if os.path.exists(ROSBAG_STORAGE_PATH):
            shutil.rmtree(ROSBAG_STORAGE_PATH, ignore_errors=True)
