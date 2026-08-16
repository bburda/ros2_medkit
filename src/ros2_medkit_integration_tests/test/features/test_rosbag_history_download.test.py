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

"""End-to-end HTTP surface of a fault that keeps several recordings (#620).

An intermittent fault used to leave exactly one black box no matter how often
it came back: the newest recording overwrote the previous one, so the recording
of the occurrence an engineer actually wanted was already gone by the time they
looked. This suite drives that scenario through the whole stack and asserts the
evidence is all still reachable over SOVD.

The fault is a real one, detected by the node itself. ``lidar_sensor`` checks
``min_range >= max_range`` on every parameter change and on a 2s timer, and
reports ``LIDAR_RANGE_INVALID`` on its own; the test only moves the parameters
that make the condition true, the way a misconfiguration would. Nothing here
calls ``ReportFault`` by hand, so the recordings are triggered by the same code
path a deployed sensor would take.

Every assertion goes through the gateway's HTTP API rather than the fault
manager's ROS services - the bulk-data descriptor list, the fault detail's
snapshot URIs and the binary downloads are what a diagnostic client actually
consumes, and the recording-id addressing this change introduces only exists
there.
"""

import os
import tempfile
import time
import unittest

import launch_testing
from rcl_interfaces.srv import SetParameters
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

from ros2_medkit_test_utils.constants import ALLOWED_EXIT_CODES
from ros2_medkit_test_utils.gateway_test_case import GatewayTestCase
from ros2_medkit_test_utils.launch_helpers import create_test_launch


APP_ENDPOINT = '/apps/lidar_sensor'
LIDAR_NAMESPACE = '/perception/lidar'
FAULT_CODE = 'LIDAR_RANGE_INVALID'

# The recording the fault manager keeps per fault code. 3 leaves headroom above
# the two occurrences driven below, so a failure means "history was lost", not
# "the cap trimmed it".
MAX_BAGS_PER_FAULT = 3

# The lidar's own scan output. Recording a topic the node really publishes is
# what makes the downloaded bags non-trivial.
CAPTURED_TOPIC = f'{LIDAR_NAMESPACE}/scan'

ROSBAG_STORAGE_PATH = tempfile.mkdtemp(prefix='rosbag_e2e_history_')


def generate_test_description():
    return create_test_launch(
        demo_nodes=['lidar_sensor'],
        # Start the lidar healthy on the range check. The test drives the fault
        # itself, so a node already faulting at startup would blur which
        # occurrence produced which recording.
        lidar_faulty=False,
        fault_manager=True,
        fault_manager_params={
            'confirmation_threshold': -1,  # Single report confirms immediately
            'snapshots.rosbag.duration_sec': 2.0,
            'snapshots.rosbag.duration_after_sec': 0.5,
            'snapshots.rosbag.include_topics': [CAPTURED_TOPIC],
            'snapshots.rosbag.format': 'mcap',
            'snapshots.rosbag.storage_path': ROSBAG_STORAGE_PATH,
            # The feature under test.
            'snapshots.rosbag.max_bags_per_fault': MAX_BAGS_PER_FAULT,
            # LEFT ON, which is the shipped default and the point: acknowledging
            # used to delete every recording of the fault, so the history the cap
            # above was raised to collect was wiped by the first acknowledgement.
            'snapshots.rosbag.auto_cleanup': True,

        },
    )


class TestRosbagHistoryDownload(GatewayTestCase):
    """Two occurrences of one fault, both recordings reachable over SOVD."""

    MIN_EXPECTED_APPS = 1
    REQUIRED_APPS = {'lidar_sensor'}

    @classmethod
    def setUpClass(cls):
        super().setUpClass()
        rclpy.init()
        cls._param_node = Node('rosbag_history_param_client')
        # Direct SetParameters client rather than AsyncParameterClient, which is
        # Jazzy+ only - same choice as test_graph_provider_stale, so this suite
        # also loads on Humble.
        cls._param_client = cls._param_node.create_client(
            SetParameters, f'{LIDAR_NAMESPACE}/lidar_sensor/set_parameters',
        )
        assert cls._param_client.wait_for_service(timeout_sec=20.0), \
            'lidar_sensor parameter services not available'

    @classmethod
    def tearDownClass(cls):
        cls._param_node.destroy_node()
        rclpy.shutdown()
        super().tearDownClass()

    # ------------------------------------------------------------------
    # Driving the real fault
    # ------------------------------------------------------------------

    def _set_range(self, min_range, max_range):
        """Live-set the lidar's range window, asserting the node accepted it."""
        request = SetParameters.Request()
        request.parameters = [
            Parameter('min_range', Parameter.Type.DOUBLE, min_range).to_parameter_msg(),
            Parameter('max_range', Parameter.Type.DOUBLE, max_range).to_parameter_msg(),
        ]
        future = self._param_client.call_async(request)
        rclpy.spin_until_future_complete(self._param_node, future, timeout_sec=10.0)
        result = future.result()
        self.assertIsNotNone(result, 'set_parameters call to lidar_sensor timed out')
        for outcome in result.results:
            self.assertTrue(outcome.successful,
                            f'lidar_sensor rejected the range change: {outcome.reason}')

    def _break_the_range(self):
        """Make min_range >= max_range, which the node reports on its own."""
        self._set_range(10.0, 5.0)

    def _repair_the_range(self):
        """Back to a valid window so the node stops reporting."""
        self._set_range(0.1, 30.0)

    def _clear_fault(self):
        """Acknowledge the fault over SOVD, the way a technician would."""
        # The ROS-backed clear path answers 204 No Content; the 200 variant of
        # this route only exists for plugin-provided faults.
        self.delete_request(f'{APP_ENDPOINT}/faults/{FAULT_CODE}', expected_status=204)

    # ------------------------------------------------------------------
    # Reading the evidence back over HTTP
    # ------------------------------------------------------------------

    def _recordings_of_the_fault(self):
        """Descriptor ids the bulk-data listing attributes to our fault code."""
        listing = self.get_json(f'{APP_ENDPOINT}/bulk-data/rosbags')
        return [
            item['id'] for item in listing.get('items', [])
            if FAULT_CODE in item.get('x-medkit', {}).get('fault_codes', [])
        ]

    def _wait_for_recording_count(self, expected, *, timeout=25.0):
        """Poll the listing until the fault has *expected* recordings."""
        deadline = time.time() + timeout
        ids = []
        while time.time() < deadline:
            ids = self._recordings_of_the_fault()
            if len(ids) >= expected:
                return ids
            time.sleep(0.5)
        return ids

    def _record_one_occurrence(self, expected_total):
        """Break the range, wait for the new bag, repair and acknowledge."""
        self._break_the_range()
        self.wait_for_fault(APP_ENDPOINT, FAULT_CODE)

        ids = self._wait_for_recording_count(expected_total)
        self.assertEqual(
            len(ids), expected_total,
            f'expected {expected_total} recording(s) for {FAULT_CODE} after this '
            f'occurrence, got {sorted(ids)}')

        # Stop the condition, then acknowledge, so the next occurrence is a
        # genuine new confirmation rather than a continuation of this one.
        self._repair_the_range()
        self._clear_fault()
        return ids

    # ------------------------------------------------------------------
    # Test
    # ------------------------------------------------------------------

    def test_01_two_occurrences_leave_two_downloadable_recordings(self):
        """The reported bug, end to end over SOVD.

        @verifies REQ_INTEROP_072
        """
        first_ids = self._record_one_occurrence(1)
        all_ids = self._record_one_occurrence(2)

        self.assertEqual(len(all_ids), 2,
                         'the second occurrence overwrote the first recording')
        self.assertEqual(len(set(all_ids)), 2,
                         'both occurrences share one recording id, so only one '
                         'of them is addressable')
        self.assertIn(first_ids[0], all_ids,
                      "the first occurrence's recording is gone - this is #620")

        # --- Every recording downloads as a real bag ---
        payloads = {}
        for recording_id in all_ids:
            response = self.get_raw(
                f'{APP_ENDPOINT}/bulk-data/rosbags/{recording_id}', timeout=15)
            self.assertIn('application/x-mcap',
                          response.headers.get('Content-Type', ''))
            self.assertEqual(response.content[:5], b'\x89MCAP',
                             f'{recording_id} did not download as a valid mcap')
            payloads[recording_id] = response.content

        # Distinct bytes, not the same bag served under two names. Without this
        # the test would pass on a build that resolved both ids to one bag.
        self.assertEqual(len(set(payloads.values())), 2,
                         'both recording ids served identical bytes')

        # --- The fault itself advertises both ---
        detail = self.get_json(f'{APP_ENDPOINT}/faults/{FAULT_CODE}')
        rosbag_snaps = [
            s for s in detail.get('environment_data', {}).get('snapshots', [])
            if s.get('type') == 'rosbag'
        ]
        self.assertEqual(len(rosbag_snaps), 2,
                         'the fault detail lists a different number of rosbag '
                         f'snapshots than the {len(all_ids)} recordings it has')
        # Each recording carries its own capture time. The fault manager has
        # always stamped one, but the transport dropped it on the rosbag branch,
        # so every bag reached a client with none and the UIs rendered "N/A".
        # With one recording per fault that was cosmetic; with several it removes
        # the only field an engineer can use to pick an occurrence.
        captured = [s.get('x-medkit', {}).get('captured_at') for s in rosbag_snaps]
        for value in captured:
            self.assertIsNotNone(value, 'a recording reached the client with no capture time')
            self.assertRegex(value, r'^\d{4}-\d{2}-\d{2}T\d{2}:\d{2}:\d{2}\.\d{3}Z$')
        self.assertEqual(len(set(captured)), 2,
                         'both recordings report the same instant, so neither can be placed')

        advertised = {s['bulk_data_uri'] for s in rosbag_snaps}
        self.assertEqual(
            advertised,
            {f'{APP_ENDPOINT}/bulk-data/rosbags/{rid}' for rid in all_ids},
            'the URIs the fault advertises do not match its recordings')

        # Every advertised URI is one a client can actually follow.
        for uri in advertised:
            self.assertEqual(self.get_raw(uri, timeout=15).content[:5], b'\x89MCAP')

    def test_02_the_fault_code_url_still_serves_the_newest_recording(self):
        """The compatibility window, over HTTP.

        The repo's own docs and the SSE payload tell clients to build
        ``/{entity}/bulk-data/rosbags/{fault_code}``. That address has to keep
        working, and to mean what it always meant: this fault's latest bag.

        @verifies REQ_INTEROP_072
        """
        ids = self._recordings_of_the_fault()
        self.assertEqual(len(ids), 2, 'test_01 must run first (alphabetical order)')

        legacy = self.get_raw(
            f'{APP_ENDPOINT}/bulk-data/rosbags/{FAULT_CODE}', timeout=15)
        self.assertEqual(legacy.content[:5], b'\x89MCAP',
                         'the pre-#620 fault-code URL stopped serving a bag')

        # "Newest" is not "either one": the detail lists snapshots newest first,
        # so the legacy URL must serve exactly the first of them.
        detail = self.get_json(f'{APP_ENDPOINT}/faults/{FAULT_CODE}')
        newest_uri = next(
            s['bulk_data_uri']
            for s in detail['environment_data']['snapshots']
            if s.get('type') == 'rosbag'
        )
        self.assertEqual(
            legacy.content, self.get_raw(newest_uri, timeout=15).content,
            'the fault-code URL served something other than the newest recording')

    def test_03_a_shared_burst_recording_is_listed_once(self):
        """One descriptor per recording, not per attached fault.

        The lidar's calibration fault confirms at startup and its own recording
        exists alongside the range fault's. Whatever the burst structure turned
        out to be, no bag may appear twice in the listing: a repeated id would
        report the same bytes as several items and inflate the storage the
        operator sees.

        @verifies REQ_INTEROP_072
        """
        listing = self.get_json(f'{APP_ENDPOINT}/bulk-data/rosbags')
        items = listing.get('items', [])
        self.assertGreater(len(items), 0, 'no rosbag descriptors at all')

        ids = [item['id'] for item in items]
        self.assertEqual(len(ids), len(set(ids)),
                         f'the listing repeats a recording id: {sorted(ids)}')

        for item in items:
            x_medkit = item.get('x-medkit', {})
            # The descriptor id IS the recording id, and the faults it covers are
            # a list because a burst shares one bag.
            self.assertEqual(item['id'], x_medkit.get('recording_id'))
            self.assertIsInstance(x_medkit.get('fault_codes'), list)
            self.assertGreater(len(x_medkit['fault_codes']), 0)
            self.assertEqual(
                len(x_medkit['fault_codes']), len(set(x_medkit['fault_codes'])),
                f'{item["id"]} lists a fault code twice')


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):
    """Post-shutdown checks."""

    def test_exit_codes(self, proc_info):
        """Gateway and fault manager exit cleanly."""
        launch_testing.asserts.assertExitCodes(
            proc_info, allowable_exit_codes=ALLOWED_EXIT_CODES)

    def test_cleanup_temp_directory(self):
        """Remove this run's bag storage."""
        import shutil
        if os.path.exists(ROSBAG_STORAGE_PATH):
            shutil.rmtree(ROSBAG_STORAGE_PATH, ignore_errors=True)
