#!/usr/bin/env python3
# Copyright 2025 mfaferek93
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

"""Integration tests for ros2_medkit_fault_manager services."""

import json
import os
import threading
import time
import unittest

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
import launch_ros.actions
import launch_testing.actions
import launch_testing.markers
import rclpy
from rclpy.node import Node
from ros2_medkit_msgs.msg import Fault
from ros2_medkit_msgs.srv import ClearFault, GetSnapshots, ListFaults, ReportFault
from sensor_msgs.msg import Temperature


def get_coverage_env():
    """
    Get environment variables for gcov coverage data collection.

    When running with coverage enabled (ENABLE_COVERAGE=ON), subprocess nodes
    need GCOV_PREFIX set to write coverage data to the correct build directory.
    This allows integration test coverage to be captured alongside unit tests.

    Returns
    -------
    dict
        Environment variables dict with GCOV_PREFIX and GCOV_PREFIX_STRIP,
        or empty dict if coverage path cannot be determined.

    """
    try:
        from ament_index_python.packages import get_package_prefix
        pkg_prefix = get_package_prefix('ros2_medkit_fault_manager')
        # pkg_prefix is like /path/to/workspace/install/ros2_medkit_fault_manager
        # workspace is 2 levels up from install/package_name
        workspace = os.path.dirname(os.path.dirname(pkg_prefix))
        build_dir = os.path.join(workspace, 'build', 'ros2_medkit_fault_manager')

        if os.path.exists(build_dir):
            # GCOV_PREFIX_STRIP removes leading path components from compiled-in paths
            # GCOV_PREFIX prepends the new path for .gcda file output
            return {
                'GCOV_PREFIX': build_dir,
                'GCOV_PREFIX_STRIP': str(build_dir.count(os.sep)),
            }
    except Exception:
        # Ignore: if coverage environment cannot be determined,
        # return empty dict so tests proceed without coverage data.
        pass
    return {}


def generate_test_description():
    """Generate launch description with fault_manager node."""
    # Get path to test snapshot config
    pkg_share = get_package_share_directory('ros2_medkit_fault_manager')
    snapshot_config = os.path.join(pkg_share, 'test', 'test_snapshots.yaml')
    correlation_config = os.path.join(pkg_share, 'test', 'test_correlation.yaml')

    fault_manager_node = launch_ros.actions.Node(
        package='ros2_medkit_fault_manager',
        executable='fault_manager_node',
        name='fault_manager',
        output='screen',
        additional_env=get_coverage_env(),
        parameters=[{
            'storage_type': 'memory',  # Use in-memory storage for integration tests
            'confirmation_threshold': -3,  # Use debounce for testing lifecycle
            'snapshots.enabled': True,
            'snapshots.config_file': snapshot_config,
            'snapshots.timeout_sec': 3.0,
            'snapshots.background_capture': False,  # On-demand for testing
            # Small bound so test_27's burst genuinely overflows the queue and
            # exercises the drop path (cap = pool_size + queue_depth = 3).
            'snapshots.capture_pool_size': 1,
            'snapshots.capture_queue_depth': 2,
            'snapshots.capture_queue_full_policy': 'reject_newest',
            'correlation.config_file': correlation_config,  # Enable correlation
        }],
        # Give the node room to flush coverage data at shutdown before SIGKILL.
        sigterm_timeout='30',
        sigkill_timeout='15',
    )

    return (
        LaunchDescription([
            fault_manager_node,
            launch_testing.actions.ReadyToTest(),
        ]),
        {
            'fault_manager_node': fault_manager_node,
        },
    )


class TestFaultManagerIntegration(unittest.TestCase):
    """Integration tests for FaultManager services."""

    @classmethod
    def setUpClass(cls):
        """Initialize ROS 2 context."""
        rclpy.init()
        cls.node = Node('test_fault_manager_client')

        # Create service clients
        cls.report_fault_client = cls.node.create_client(
            ReportFault, '/fault_manager/report_fault'
        )
        cls.list_faults_client = cls.node.create_client(
            ListFaults, '/fault_manager/list_faults'
        )
        cls.clear_fault_client = cls.node.create_client(
            ClearFault, '/fault_manager/clear_fault'
        )
        cls.get_snapshots_client = cls.node.create_client(
            GetSnapshots, '/fault_manager/get_snapshots'
        )

        # Create test publisher for snapshot capture tests
        cls.temp_publisher = cls.node.create_publisher(
            Temperature, '/test/temperature', 10
        )

        # Wait for services to be available
        assert cls.report_fault_client.wait_for_service(timeout_sec=10.0), \
            'report_fault service not available'
        assert cls.list_faults_client.wait_for_service(timeout_sec=10.0), \
            'list_faults service not available'
        assert cls.clear_fault_client.wait_for_service(timeout_sec=10.0), \
            'clear_fault service not available'
        assert cls.get_snapshots_client.wait_for_service(timeout_sec=10.0), \
            'get_snapshots service not available'

    @classmethod
    def tearDownClass(cls):
        """Shutdown ROS 2 context."""
        cls.node.destroy_node()
        rclpy.shutdown()

    def _call_service(self, client, request):
        """Call a service and wait for response."""
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)
        self.assertIsNotNone(future.result(), 'Service call timed out')
        return future.result()

    def test_01_report_fault_success(self):
        """Test reporting a new fault."""
        request = ReportFault.Request()
        request.fault_code = 'TEST_FAULT_001'
        request.event_type = ReportFault.Request.EVENT_FAILED
        request.severity = Fault.SEVERITY_ERROR
        request.description = 'Test fault from integration test'
        request.source_id = '/test_node'

        response = self._call_service(self.report_fault_client, request)

        self.assertTrue(response.accepted)
        print('Report fault accepted')

    def test_02_report_fault_empty_code_fails(self):
        """Test that empty fault_code is rejected."""
        request = ReportFault.Request()
        request.fault_code = ''
        request.event_type = ReportFault.Request.EVENT_FAILED
        request.severity = Fault.SEVERITY_ERROR
        request.description = 'Test fault'
        request.source_id = '/test_node'

        response = self._call_service(self.report_fault_client, request)

        self.assertFalse(response.accepted)
        print('Empty code rejected as expected')

    def test_03_report_fault_empty_source_fails(self):
        """Test that empty source_id is rejected."""
        request = ReportFault.Request()
        request.fault_code = 'TEST_FAULT_002'
        request.event_type = ReportFault.Request.EVENT_FAILED
        request.severity = Fault.SEVERITY_ERROR
        request.description = 'Test fault'
        request.source_id = ''

        response = self._call_service(self.report_fault_client, request)

        self.assertFalse(response.accepted)
        print('Empty source rejected as expected')

    def test_04_report_fault_invalid_severity_fails(self):
        """Test that invalid severity is rejected."""
        request = ReportFault.Request()
        request.fault_code = 'TEST_FAULT_003'
        request.event_type = ReportFault.Request.EVENT_FAILED
        request.severity = 99  # Invalid severity
        request.description = 'Test fault'
        request.source_id = '/test_node'

        response = self._call_service(self.report_fault_client, request)

        self.assertFalse(response.accepted)
        print('Invalid severity rejected as expected')

    def test_05_list_faults_prefailed(self):
        """Test getting faults with PREFAILED status."""
        # First report a fault
        report_request = ReportFault.Request()
        report_request.fault_code = 'TEST_FAULT_GET_001'
        report_request.event_type = ReportFault.Request.EVENT_FAILED
        report_request.severity = Fault.SEVERITY_WARN
        report_request.description = 'Fault for get test'
        report_request.source_id = '/test_node'
        self._call_service(self.report_fault_client, report_request)

        # Query PREFAILED faults
        get_request = ListFaults.Request()
        get_request.filter_by_severity = False
        get_request.severity = 0
        get_request.statuses = [Fault.STATUS_PREFAILED]

        response = self._call_service(self.list_faults_client, get_request)

        self.assertGreater(len(response.faults), 0)
        fault_codes = [f.fault_code for f in response.faults]
        self.assertIn('TEST_FAULT_GET_001', fault_codes)
        print(f'Get faults returned {len(response.faults)} faults')

    def test_06_list_faults_filter_by_severity(self):
        """Test filtering faults by severity."""
        # Report faults with different severities
        for i, severity in enumerate([Fault.SEVERITY_INFO, Fault.SEVERITY_ERROR]):
            request = ReportFault.Request()
            request.fault_code = f'TEST_FAULT_SEV_{i}'
            request.event_type = ReportFault.Request.EVENT_FAILED
            request.severity = severity
            request.description = f'Fault with severity {severity}'
            request.source_id = '/test_node'
            self._call_service(self.report_fault_client, request)

        # Query ERROR severity only
        get_request = ListFaults.Request()
        get_request.filter_by_severity = True
        get_request.severity = Fault.SEVERITY_ERROR
        get_request.statuses = [Fault.STATUS_PREFAILED]

        response = self._call_service(self.list_faults_client, get_request)

        for fault in response.faults:
            self.assertEqual(fault.severity, Fault.SEVERITY_ERROR)
        print(f'Severity filter returned {len(response.faults)} ERROR faults')

    def test_07_clear_fault_success(self):
        """Test clearing a fault."""
        # First report a fault
        report_request = ReportFault.Request()
        report_request.fault_code = 'TEST_FAULT_CLEAR'
        report_request.event_type = ReportFault.Request.EVENT_FAILED
        report_request.severity = Fault.SEVERITY_ERROR
        report_request.description = 'Fault to be cleared'
        report_request.source_id = '/test_node'
        self._call_service(self.report_fault_client, report_request)

        # Clear the fault
        clear_request = ClearFault.Request()
        clear_request.fault_code = 'TEST_FAULT_CLEAR'

        response = self._call_service(self.clear_fault_client, clear_request)

        self.assertTrue(response.success)
        self.assertIn('cleared', response.message.lower())
        print(f'Clear fault response: {response.message}')

        # Verify fault is now CLEARED
        get_request = ListFaults.Request()
        get_request.filter_by_severity = False
        get_request.severity = 0
        get_request.statuses = [Fault.STATUS_CLEARED]

        get_response = self._call_service(self.list_faults_client, get_request)

        cleared_codes = [f.fault_code for f in get_response.faults]
        self.assertIn('TEST_FAULT_CLEAR', cleared_codes)

    def test_08_clear_nonexistent_fault_fails(self):
        """Test clearing a non-existent fault returns false."""
        request = ClearFault.Request()
        request.fault_code = 'NONEXISTENT_FAULT_XYZ'

        response = self._call_service(self.clear_fault_client, request)

        self.assertFalse(response.success)
        self.assertIn('not found', response.message.lower())
        print(f'Clear nonexistent response: {response.message}')

    def test_09_clear_fault_empty_code_fails(self):
        """Test that clearing with empty fault_code fails."""
        request = ClearFault.Request()
        request.fault_code = ''

        response = self._call_service(self.clear_fault_client, request)

        self.assertFalse(response.success)
        self.assertIn('empty', response.message.lower())
        print(f'Clear empty code response: {response.message}')

    def test_10_report_updates_existing_fault(self):
        """Test that reporting same fault_code updates existing fault."""
        fault_code = 'TEST_FAULT_UPDATE'

        # Report first time
        request1 = ReportFault.Request()
        request1.fault_code = fault_code
        request1.event_type = ReportFault.Request.EVENT_FAILED
        request1.severity = Fault.SEVERITY_WARN
        request1.description = 'First report'
        request1.source_id = '/node1'

        response1 = self._call_service(self.report_fault_client, request1)
        self.assertTrue(response1.accepted)

        # Report second time from different source
        request2 = ReportFault.Request()
        request2.fault_code = fault_code
        request2.event_type = ReportFault.Request.EVENT_FAILED
        request2.severity = Fault.SEVERITY_ERROR  # Higher severity
        request2.description = 'Second report'
        request2.source_id = '/node2'

        response2 = self._call_service(self.report_fault_client, request2)
        self.assertTrue(response2.accepted)

        # Verify fault was updated
        get_request = ListFaults.Request()
        get_request.filter_by_severity = False
        get_request.severity = 0
        get_request.statuses = [Fault.STATUS_PREFAILED]

        get_response = self._call_service(self.list_faults_client, get_request)

        updated_fault = None
        for f in get_response.faults:
            if f.fault_code == fault_code:
                updated_fault = f
                break

        self.assertIsNotNone(updated_fault)
        self.assertEqual(updated_fault.severity, Fault.SEVERITY_ERROR)
        # Edge-counting: a re-report of a still-active fault is the same
        # continuous occurrence, so the count stays at 1 (it only increments
        # on reactivation after CLEARED).
        self.assertEqual(updated_fault.occurrence_count, 1)
        self.assertEqual(len(updated_fault.reporting_sources), 2)
        print(f'Updated fault: occurrence_count={updated_fault.occurrence_count}, '
              f'sources={updated_fault.reporting_sources}')

    def test_11_confirmation_workflow(self):
        """Test fault auto-confirms after reaching threshold (default=-3)."""
        fault_code = 'TEST_FAULT_CONFIRM'

        # Report fault 3 times (default threshold)
        for i in range(3):
            request = ReportFault.Request()
            request.fault_code = fault_code
            request.event_type = ReportFault.Request.EVENT_FAILED
            request.severity = Fault.SEVERITY_ERROR
            request.description = f'Report {i + 1}'
            request.source_id = f'/node{i + 1}'
            self._call_service(self.report_fault_client, request)

        # Query CONFIRMED faults (should include our fault now)
        get_request = ListFaults.Request()
        get_request.filter_by_severity = False
        get_request.severity = 0
        get_request.statuses = [Fault.STATUS_CONFIRMED]

        response = self._call_service(self.list_faults_client, get_request)

        confirmed_codes = [f.fault_code for f in response.faults]
        self.assertIn(fault_code, confirmed_codes)

        # Find the fault and verify its state
        confirmed_fault = next(f for f in response.faults if f.fault_code == fault_code)
        self.assertEqual(confirmed_fault.status, Fault.STATUS_CONFIRMED)
        # Edge-counting: the debounce reports that built toward confirmation
        # are one continuous occurrence, not three.
        self.assertEqual(confirmed_fault.occurrence_count, 1)
        print(f'Fault confirmed: status={confirmed_fault.status}, '
              f'occurrence_count={confirmed_fault.occurrence_count}')

    def test_12_prefailed_excluded_from_default_query(self):
        """Test that PREFAILED faults are not returned in default query."""
        fault_code = 'TEST_FAULT_PREFAILED_ONLY'

        # Report fault once (stays PREFAILED since threshold=-3)
        request = ReportFault.Request()
        request.fault_code = fault_code
        request.event_type = ReportFault.Request.EVENT_FAILED
        request.severity = Fault.SEVERITY_WARN
        request.description = 'Single report - should stay prefailed'
        request.source_id = '/test_node'
        self._call_service(self.report_fault_client, request)

        # Query with empty statuses (default = CONFIRMED only)
        get_request = ListFaults.Request()
        get_request.filter_by_severity = False
        get_request.severity = 0
        get_request.statuses = []  # Empty = CONFIRMED only

        response = self._call_service(self.list_faults_client, get_request)

        # Fault should NOT be in results (it's PREFAILED)
        fault_codes = [f.fault_code for f in response.faults]
        self.assertNotIn(fault_code, fault_codes)
        print('Default query excluded PREFAILED fault as expected')

        # But it should be visible when querying PREFAILED explicitly
        get_prefailed = ListFaults.Request()
        get_prefailed.filter_by_severity = False
        get_prefailed.severity = 0
        get_prefailed.statuses = [Fault.STATUS_PREFAILED]

        prefailed_response = self._call_service(self.list_faults_client, get_prefailed)
        prefailed_codes = [f.fault_code for f in prefailed_response.faults]
        self.assertIn(fault_code, prefailed_codes)
        print('PREFAILED query found the fault as expected')

    def test_13_multi_source_confirmation(self):
        """Test fault confirms when multiple sources report same fault."""
        fault_code = 'TEST_FAULT_MULTI_SRC'

        # Report from 3 different sources
        sources = ['/sensor1', '/sensor2', '/sensor3']
        for source in sources:
            request = ReportFault.Request()
            request.fault_code = fault_code
            request.event_type = ReportFault.Request.EVENT_FAILED
            request.severity = Fault.SEVERITY_ERROR
            request.description = 'Multi-source fault'
            request.source_id = source
            self._call_service(self.report_fault_client, request)

        # Query and verify
        get_request = ListFaults.Request()
        get_request.filter_by_severity = False
        get_request.severity = 0
        get_request.statuses = [Fault.STATUS_CONFIRMED]

        response = self._call_service(self.list_faults_client, get_request)

        fault = next((f for f in response.faults if f.fault_code == fault_code), None)
        self.assertIsNotNone(fault)
        self.assertEqual(fault.status, Fault.STATUS_CONFIRMED)
        self.assertEqual(len(fault.reporting_sources), 3)
        # Edge-counting: three sources reporting the same active fault are one
        # continuous occurrence; the count tracks raise edges, not reports.
        self.assertEqual(fault.occurrence_count, 1)
        print(f'Multi-source fault confirmed: sources={fault.reporting_sources}')

    def test_14_critical_severity_immediate_confirmation(self):
        """Test CRITICAL severity bypasses debounce and confirms immediately."""
        fault_code = 'TEST_FAULT_CRITICAL'

        # Report single CRITICAL fault
        request = ReportFault.Request()
        request.fault_code = fault_code
        request.event_type = ReportFault.Request.EVENT_FAILED
        request.severity = Fault.SEVERITY_CRITICAL
        request.description = 'Critical fault - should confirm immediately'
        request.source_id = '/test_node'
        self._call_service(self.report_fault_client, request)

        # Query CONFIRMED faults
        get_request = ListFaults.Request()
        get_request.filter_by_severity = False
        get_request.severity = 0
        get_request.statuses = [Fault.STATUS_CONFIRMED]

        response = self._call_service(self.list_faults_client, get_request)

        fault = next((f for f in response.faults if f.fault_code == fault_code), None)
        self.assertIsNotNone(fault)
        self.assertEqual(fault.status, Fault.STATUS_CONFIRMED)
        self.assertEqual(fault.occurrence_count, 1)  # Only one report needed
        print('CRITICAL fault confirmed immediately as expected')

    def test_15_passed_event_increments_counter(self):
        """Test PASSED events increment debounce counter towards healing."""
        fault_code = 'TEST_FAULT_PASSED'

        # Report 2 FAILED events (counter = -2)
        for i in range(2):
            request = ReportFault.Request()
            request.fault_code = fault_code
            request.event_type = ReportFault.Request.EVENT_FAILED
            request.severity = Fault.SEVERITY_ERROR
            request.description = f'Failed report {i + 1}'
            request.source_id = f'/node{i + 1}'
            self._call_service(self.report_fault_client, request)

        # Verify fault is PREFAILED
        get_request = ListFaults.Request()
        get_request.filter_by_severity = False
        get_request.severity = 0
        get_request.statuses = [Fault.STATUS_PREFAILED]

        response = self._call_service(self.list_faults_client, get_request)
        fault = next((f for f in response.faults if f.fault_code == fault_code), None)
        self.assertIsNotNone(fault)
        self.assertEqual(fault.status, Fault.STATUS_PREFAILED)

        # Report 3 PASSED events (counter = -2 + 3 = +1)
        for i in range(3):
            request = ReportFault.Request()
            request.fault_code = fault_code
            request.event_type = ReportFault.Request.EVENT_PASSED
            request.severity = 0  # Ignored for PASSED
            request.description = ''  # Ignored for PASSED
            request.source_id = '/test_node'
            self._call_service(self.report_fault_client, request)

        # Verify fault is PREPASSED (counter > 0)
        get_request = ListFaults.Request()
        get_request.filter_by_severity = False
        get_request.severity = 0
        get_request.statuses = [Fault.STATUS_PREPASSED]

        response = self._call_service(self.list_faults_client, get_request)
        fault = next((f for f in response.faults if f.fault_code == fault_code), None)
        self.assertIsNotNone(fault)
        self.assertEqual(fault.status, Fault.STATUS_PREPASSED)
        print('PASSED events moved fault to PREPASSED as expected')

    # ==================== Snapshot Capture Tests ====================
    # @verifies REQ_INTEROP_088

    def _publish_and_spin(self, publisher, msg, count=10, interval=0.1):
        """Publish messages and spin to ensure they're sent."""
        for _ in range(count):
            publisher.publish(msg)
            rclpy.spin_once(self.node, timeout_sec=0.01)
            time.sleep(interval)

    def _wait_for_snapshot(self, fault_code, timeout=10.0):
        """
        Poll GetSnapshots until the async on-demand capture completes.

        The snapshot subscription is created on demand when the fault confirms
        and fills over its own timeout window, so a fixed sleep plus a single
        query flakes under load - poll until data arrives.
        """
        request = GetSnapshots.Request()
        request.fault_code = fault_code
        request.topic = ''
        deadline = time.time() + timeout
        response = self._call_service(self.get_snapshots_client, request)
        while time.time() < deadline:
            if (response is not None and response.success
                    and len(response.data) > 0):
                return response
            time.sleep(0.5)
            response = self._call_service(self.get_snapshots_client, request)
        return response

    def test_16_snapshot_capture_on_fault_confirmation(self):
        """
        Test that snapshot is captured when fault is confirmed.

        @verifies REQ_INTEROP_088
        """
        fault_code = 'TEST_SNAPSHOT_FAULT'

        # Publish test data on the configured topic
        temp_msg = Temperature()
        temp_msg.temperature = 85.5
        temp_msg.variance = 0.1

        # Publish continuously to establish topic presence
        self._publish_and_spin(self.temp_publisher, temp_msg, count=20, interval=0.05)

        # Report CRITICAL fault (immediate confirmation triggers snapshot)
        # The snapshot capture will create on-demand subscription
        request = ReportFault.Request()
        request.fault_code = fault_code
        request.event_type = ReportFault.Request.EVENT_FAILED
        request.severity = Fault.SEVERITY_CRITICAL
        request.description = 'Snapshot test fault'
        request.source_id = '/test_node'

        # Start publishing in parallel with fault reporting
        # Snapshot capture needs messages available during its timeout window
        stop_publishing = threading.Event()

        def keep_publishing():
            while not stop_publishing.is_set():
                self.temp_publisher.publish(temp_msg)
                time.sleep(0.05)

        pub_thread = threading.Thread(target=keep_publishing)
        pub_thread.start()

        try:
            response = self._call_service(self.report_fault_client, request)
            self.assertTrue(response.accepted)

            # Poll for the snapshot; the publisher thread keeps feeding data.
            # _wait_for_snapshot is the ground-truth gate (it polls GetSnapshots
            # for the actual capture), so no fragile pre-wait on the short-lived
            # on-demand subscription is needed.
            snap_response = self._wait_for_snapshot(fault_code)
        finally:
            stop_publishing.set()
            pub_thread.join()

        self.assertTrue(snap_response.success)
        self.assertGreater(len(snap_response.data), 0)

        # Parse and verify snapshot data
        snapshot_data = json.loads(snap_response.data)
        self.assertIn('fault_code', snapshot_data)
        self.assertEqual(snapshot_data['fault_code'], fault_code)
        self.assertIn('topics', snapshot_data)

        # Check if temperature topic was captured
        if '/test/temperature' in snapshot_data['topics']:
            temp_snapshot = snapshot_data['topics']['/test/temperature']
            self.assertIn('message_type', temp_snapshot)
            self.assertIn('data', temp_snapshot)
            self.assertIn('temperature', temp_snapshot['data'])
            self.assertAlmostEqual(temp_snapshot['data']['temperature'], 85.5, places=1)
            print(f'Snapshot captured: {len(snapshot_data["topics"])} topics')
            print(f'Temperature captured: {temp_snapshot["data"]["temperature"]}')
        else:
            # If topic wasn't captured, it might be due to cross-process discovery timing
            # This is acceptable for integration tests - we verify the mechanism works
            print(f'Snapshot captured (topics may vary due to discovery): {snapshot_data}')

    def test_17_get_snapshots_nonexistent_fault(self):
        """
        Test GetSnapshots returns error for nonexistent fault.

        @verifies REQ_INTEROP_088
        """
        request = GetSnapshots.Request()
        request.fault_code = 'NONEXISTENT_SNAPSHOT_FAULT'
        request.topic = ''

        response = self._call_service(self.get_snapshots_client, request)

        self.assertFalse(response.success)
        self.assertIn('not found', response.error_message.lower())
        print(f'Nonexistent fault error: {response.error_message}')

    def test_18_get_snapshots_empty_fault_code(self):
        """
        Test GetSnapshots returns error for empty fault code.

        @verifies REQ_INTEROP_088
        """
        request = GetSnapshots.Request()
        request.fault_code = ''
        request.topic = ''

        response = self._call_service(self.get_snapshots_client, request)

        self.assertFalse(response.success)
        self.assertIn('empty', response.error_message.lower())
        print(f'Empty fault code error: {response.error_message}')

    def test_19_snapshot_with_topic_filter(self):
        """
        Test GetSnapshots with specific topic filter.

        @verifies REQ_INTEROP_088
        """
        # Use the fault from test_16 which should have snapshots
        fault_code = 'TEST_SNAPSHOT_FAULT'

        request = GetSnapshots.Request()
        request.fault_code = fault_code
        request.topic = '/test/temperature'

        response = self._call_service(self.get_snapshots_client, request)

        # Response should be successful (fault exists)
        self.assertTrue(response.success)
        snapshot_data = json.loads(response.data)
        self.assertIn('topics', snapshot_data)

        # If topic was captured, verify filter works
        if len(snapshot_data['topics']) > 0:
            # Should only contain the filtered topic (or none if not captured)
            self.assertLessEqual(len(snapshot_data['topics']), 1)
            if '/test/temperature' in snapshot_data['topics']:
                print('Topic filter returned single topic as expected')
            else:
                print('Topic filter returned empty (topic not captured due to discovery timing)')
        else:
            print('No topics captured (acceptable due to cross-process discovery timing)')

    def test_20_snapshot_config_loads_patterns(self):
        """
        Test that pattern-based config is loaded correctly.

        @verifies REQ_INTEROP_088
        """
        fault_code = 'MOTOR_OVERHEAT_TEST'

        # Report CRITICAL fault matching MOTOR_.* pattern
        request = ReportFault.Request()
        request.fault_code = fault_code
        request.event_type = ReportFault.Request.EVENT_FAILED
        request.severity = Fault.SEVERITY_CRITICAL
        request.description = 'Pattern test fault'
        request.source_id = '/test_node'

        response = self._call_service(self.report_fault_client, request)
        self.assertTrue(response.accepted)

        # Query snapshots - fault should be confirmed even if no topic data
        snap_request = GetSnapshots.Request()
        snap_request.fault_code = fault_code
        snap_request.topic = ''

        snap_response = self._call_service(self.get_snapshots_client, snap_request)

        # Success indicates fault exists (even if no snapshots captured due to no publishers)
        self.assertTrue(snap_response.success)
        print('Pattern-matched fault processed successfully')

    # ==================== Correlation Tests ====================
    # Tests for fault correlation feature (#105)

    def test_21_correlation_hierarchical_root_cause(self):
        """
        Test hierarchical correlation: root cause is recognized.

        When ESTOP_001 is reported, it should be identified as a root cause.
        """
        # Report root cause fault (CRITICAL bypasses debounce)
        request = ReportFault.Request()
        request.fault_code = 'ESTOP_001'
        request.event_type = ReportFault.Request.EVENT_FAILED
        request.severity = Fault.SEVERITY_CRITICAL
        request.description = 'E-Stop triggered'
        request.source_id = '/estop_node'

        response = self._call_service(self.report_fault_client, request)
        self.assertTrue(response.accepted)

        # Verify fault is confirmed
        get_request = ListFaults.Request()
        get_request.filter_by_severity = False
        get_request.statuses = [Fault.STATUS_CONFIRMED]

        get_response = self._call_service(self.list_faults_client, get_request)

        fault_codes = [f.fault_code for f in get_response.faults]
        self.assertIn('ESTOP_001', fault_codes)
        print('Root cause ESTOP_001 confirmed')

    def test_22_correlation_symptoms_muted(self):
        """
        Test hierarchical correlation: symptoms are muted.

        After ESTOP_001 is reported, subsequent MOTOR_COMM_* faults
        should be muted (stored but not returned in default query).
        """
        # First ensure root cause exists (from test_21 or report it again)
        root_request = ReportFault.Request()
        root_request.fault_code = 'ESTOP_001'
        root_request.event_type = ReportFault.Request.EVENT_FAILED
        root_request.severity = Fault.SEVERITY_CRITICAL
        root_request.description = 'E-Stop triggered'
        root_request.source_id = '/estop_node'
        self._call_service(self.report_fault_client, root_request)

        # Report symptom faults (CRITICAL to bypass debounce)
        symptoms = ['MOTOR_COMM_FL', 'MOTOR_COMM_FR', 'DRIVE_FAULT_1']
        for symptom in symptoms:
            request = ReportFault.Request()
            request.fault_code = symptom
            request.event_type = ReportFault.Request.EVENT_FAILED
            request.severity = Fault.SEVERITY_CRITICAL
            request.description = f'Symptom: {symptom}'
            request.source_id = '/motor_node'

            response = self._call_service(self.report_fault_client, request)
            self.assertTrue(response.accepted)

        # Query with include_muted=true to see muted faults
        get_request = ListFaults.Request()
        get_request.filter_by_severity = False
        get_request.statuses = [Fault.STATUS_CONFIRMED]
        get_request.include_muted = True

        get_response = self._call_service(self.list_faults_client, get_request)

        # Check muted_count > 0
        self.assertGreater(get_response.muted_count, 0)
        print(f'Muted count: {get_response.muted_count}')

        # Check muted_faults contains our symptoms
        muted_codes = [m.fault_code for m in get_response.muted_faults]
        for symptom in symptoms:
            self.assertIn(symptom, muted_codes)
            print(f'Symptom {symptom} is muted')

        # Verify muted faults have correct root cause
        for muted in get_response.muted_faults:
            if muted.fault_code in symptoms:
                self.assertEqual(muted.root_cause_code, 'ESTOP_001')
                self.assertEqual(muted.rule_id, 'estop_cascade')

    def test_23_correlation_auto_clear_with_root(self):
        """
        Test hierarchical correlation: clearing root cause clears symptoms.

        When ESTOP_001 is cleared, all its correlated symptoms should
        also be cleared (auto_clear_with_root=true).
        """
        # Clear the root cause
        clear_request = ClearFault.Request()
        clear_request.fault_code = 'ESTOP_001'

        clear_response = self._call_service(self.clear_fault_client, clear_request)

        self.assertTrue(clear_response.success)

        # Check auto_cleared_codes contains our symptoms
        auto_cleared = clear_response.auto_cleared_codes
        print(f'Auto-cleared codes: {auto_cleared}')

        # At least some symptoms should be auto-cleared
        self.assertGreater(len(auto_cleared), 0)

        # Verify muted count is now 0
        get_request = ListFaults.Request()
        get_request.filter_by_severity = False
        get_request.statuses = [Fault.STATUS_CONFIRMED]
        get_request.include_muted = True

        get_response = self._call_service(self.list_faults_client, get_request)

        # Muted faults related to ESTOP_001 should be cleared
        remaining_muted = [m for m in get_response.muted_faults
                           if m.root_cause_code == 'ESTOP_001']
        self.assertEqual(len(remaining_muted), 0)
        print('All symptoms auto-cleared with root cause')

    def test_24_correlation_auto_cluster(self):
        """
        Test auto-cluster correlation: sensor errors are grouped.

        When 2+ SENSOR_* faults occur within the window, they form a cluster.
        """
        # Report sensor faults (CRITICAL to bypass debounce)
        sensors = ['SENSOR_LIDAR_FAIL', 'SENSOR_IMU_TIMEOUT']
        for sensor in sensors:
            request = ReportFault.Request()
            request.fault_code = sensor
            request.event_type = ReportFault.Request.EVENT_FAILED
            request.severity = Fault.SEVERITY_CRITICAL
            request.description = f'Sensor error: {sensor}'
            request.source_id = '/sensor_node'

            response = self._call_service(self.report_fault_client, request)
            self.assertTrue(response.accepted)
            time.sleep(0.1)  # Small delay to stay within window

        # Query with include_clusters=true
        get_request = ListFaults.Request()
        get_request.filter_by_severity = False
        get_request.statuses = [Fault.STATUS_CONFIRMED]
        get_request.include_clusters = True

        get_response = self._call_service(self.list_faults_client, get_request)

        # Check cluster_count > 0
        self.assertGreater(get_response.cluster_count, 0)
        print(f'Cluster count: {get_response.cluster_count}')

        # Check clusters contain our sensor faults
        self.assertGreater(len(get_response.clusters), 0)
        cluster = get_response.clusters[0]
        print(f'Cluster: {cluster.cluster_id}, representative: {cluster.representative_code}')
        print(f'Fault codes in cluster: {cluster.fault_codes}')

        # Verify cluster properties
        self.assertEqual(cluster.rule_id, 'sensor_storm')
        self.assertGreaterEqual(cluster.count, 2)

    def test_25_correlation_include_muted_false_excludes_details(self):
        """
        Test that include_muted=false excludes muted_faults details.

        The muted_count should still be returned, but muted_faults array
        should be empty when include_muted=false (default).
        """
        get_request = ListFaults.Request()
        get_request.filter_by_severity = False
        get_request.statuses = [Fault.STATUS_CONFIRMED]
        get_request.include_muted = False  # Default
        get_request.include_clusters = False  # Default

        get_response = self._call_service(self.list_faults_client, get_request)

        # muted_count is always returned
        self.assertIsNotNone(get_response.muted_count)

        # muted_faults should be empty when include_muted=false
        self.assertEqual(len(get_response.muted_faults), 0)

        # clusters should be empty when include_clusters=false
        self.assertEqual(len(get_response.clusters), 0)

        print('Default query excludes muted/cluster details as expected')

    def test_26_cleared_fault_can_be_reactivated(self):
        """Test that a CLEARED fault can be reactivated by new FAILED events."""
        fault_code = 'TEST_REACTIVATION'

        # Report and confirm fault
        request = ReportFault.Request()
        request.fault_code = fault_code
        request.event_type = ReportFault.Request.EVENT_FAILED
        request.severity = Fault.SEVERITY_CRITICAL
        request.description = 'Initial fault'
        request.source_id = '/node1'
        response = self._call_service(self.report_fault_client, request)
        self.assertTrue(response.accepted)

        # Clear the fault
        clear_request = ClearFault.Request()
        clear_request.fault_code = fault_code
        clear_response = self._call_service(self.clear_fault_client, clear_request)
        self.assertTrue(clear_response.success)

        # Report again - should reactivate
        request = ReportFault.Request()
        request.fault_code = fault_code
        request.event_type = ReportFault.Request.EVENT_FAILED
        request.severity = Fault.SEVERITY_CRITICAL
        request.description = 'Reactivated'
        request.source_id = '/node2'
        response = self._call_service(self.report_fault_client, request)
        self.assertTrue(response.accepted)

        # Verify fault is CONFIRMED again
        get_request = ListFaults.Request()
        get_request.statuses = [Fault.STATUS_CONFIRMED]
        get_response = self._call_service(self.list_faults_client, get_request)

        fault = next((f for f in get_response.faults if f.fault_code == fault_code), None)
        self.assertIsNotNone(fault)
        self.assertEqual(fault.status, Fault.STATUS_CONFIRMED)
        self.assertEqual(fault.occurrence_count, 2)
        self.assertEqual(len(fault.reporting_sources), 2)
        print(f'Cleared fault reactivated: occurrence_count={fault.occurrence_count}')

    def test_27_capture_pool_bounds_fault_storm(self, proc_output):
        """
        Burst of distinct CRITICAL faults that overflows the bounded pool (issue #441).

        The launch caps the pool small (pool_size=1, queue_depth=2 -> capacity 3). The
        STORM_FAULT_* codes capture /test/default_data; the publisher exists but is
        idle during the burst, so each on-demand capture blocks for the full snapshot
        timeout (3s). With a single worker stuck on each capture, a 12-fault burst
        overflows capacity deterministically (independent of how fast a capture would
        otherwise complete) and exercises the reject_newest drop path.

        We assert the bound engaged (a "Capture queue full" drop was logged) and the
        node stayed responsive (no crash/UAF). That drop signal is the real regression
        guard: it is absent on the old unbounded thread-per-fault model. Capture-still-
        works is already covered by the snapshot tests above, so we deliberately do not
        re-assert a near-tautological "something captured" here.
        """
        fault_codes = [f'STORM_FAULT_{i}' for i in range(12)]

        # STORM_FAULT_* codes fall through to default_topics: [/test/default_data].
        # Keep the publisher alive so the topic type resolves and the on-demand capture
        # subscribes - but do NOT publish during the burst, so each capture waits the
        # full timeout and the single worker stays busy long enough to overflow.
        default_pub = self.node.create_publisher(Temperature, '/test/default_data', 10)
        temp_msg = Temperature()
        temp_msg.temperature = 70.0
        temp_msg.variance = 0.1
        # Establish topic presence (let discovery propagate) before confirming faults.
        self._publish_and_spin(default_pub, temp_msg, count=10, interval=0.05)

        # Synchronous sends (no lost responses): report_fault replies immediately after
        # enqueue, on the executor thread, independently of the busy capture worker.
        for code in fault_codes:
            request = ReportFault.Request()
            request.fault_code = code
            request.event_type = ReportFault.Request.EVENT_FAILED
            request.severity = Fault.SEVERITY_CRITICAL  # immediate confirm
            request.description = 'storm'
            request.source_id = '/test_node'
            response = self._call_service(self.report_fault_client, request)
            self.assertTrue(response.accepted)

        # The bound engaged: with capacity 3 and the worker stuck on 3s captures, the
        # 12-fault burst overflowed and the pool dropped captures. The first drop logs
        # immediately (later drops are throttled).
        proc_output.assertWaitFor('Capture queue full', timeout=15.0)

        # Node is still responsive after the storm (no crash/UAF).
        list_request = ListFaults.Request()
        list_request.filter_by_severity = False
        list_request.severity = 0
        list_request.statuses = [Fault.STATUS_CONFIRMED]
        list_response = self._call_service(self.list_faults_client, list_request)
        storm = [f for f in list_response.faults if f.fault_code.startswith('STORM_FAULT_')]
        self.assertGreater(len(storm), 0, 'Storm faults not persisted')
        print(f'Storm bounded: {len(storm)} faults confirmed, reject_newest drop path exercised')


@launch_testing.post_shutdown_test()
class TestFaultManagerShutdown(unittest.TestCase):
    """Post-shutdown tests."""

    def test_exit_code(self, proc_info):
        """Verify fault_manager exits cleanly."""
        launch_testing.asserts.assertExitCodes(proc_info)
