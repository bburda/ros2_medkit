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
Debounce and healing contract for an edge-triggered fault reporter.

A reporter is edge-triggered when it sends one FAILED event as a condition
appears and one clear as it goes away, instead of repeating FAILED on every
sample. The count-based debounce cannot filter a noisy sample for such a
reporter: the second FAILED that would move the counter never arrives. The
time-based lever does the filtering instead, so these tests pin the pair.

Every case sends the number of events an edge-triggered reporter really sends
(one), never the number the counter would need.
"""

import os
import tempfile
import time
import unittest

from launch import LaunchDescription
import launch_ros.actions
import launch_testing.actions
import launch_testing.markers
import rclpy
from rclpy.node import Node
from ros2_medkit_msgs.msg import Fault
from ros2_medkit_msgs.srv import ListFaults, ReportFault

DATABASE_PATH = os.path.join(tempfile.mkdtemp(prefix='debounce_healing_'), 'faults.db')

# Seconds a fault stays PREFAILED before the timer confirms it. The node runs
# that timer once a second, so a confirmation lands within AUTO_CONFIRM_SEC + 1.
AUTO_CONFIRM_SEC = 3.0

# Every status, so a test can see a fault the default CONFIRMED-only filter hides.
ALL_STATUSES = ['PREFAILED', 'PREPASSED', 'CONFIRMED', 'HEALED', 'CLEARED']


def generate_test_description():
    """Launch fault_manager with the appliance debounce and healing settings."""
    fault_manager_node = launch_ros.actions.Node(
        package='ros2_medkit_fault_manager',
        executable='fault_manager_node',
        name='fault_manager',
        output='screen',
        parameters=[{
            'storage_type': 'sqlite',
            'database_path': DATABASE_PATH,
            # Below -1, so the first FAILED lands in PREFAILED instead of
            # confirming. An edge-triggered reporter never sends the second
            # event, so the counter stays here and the timer below decides.
            'confirmation_threshold': -2,
            'auto_confirm_after_sec': AUTO_CONFIRM_SEC,
            # A clear arrives as one PASSED event, so healing has to finish on
            # that one event. Threshold 0 is what makes it reachable.
            'healing_enabled': True,
            'healing_threshold': 0,
        }],
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


class TestDebounceAndHealing(unittest.TestCase):
    """One failed read must not confirm, and a de-assert must heal unattended."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = Node('test_debounce_healing_client')
        cls.report_client = cls.node.create_client(ReportFault, '/fault_manager/report_fault')
        cls.list_client = cls.node.create_client(ListFaults, '/fault_manager/list_faults')

        assert cls.report_client.wait_for_service(timeout_sec=10.0), \
            'report_fault service not available'
        assert cls.list_client.wait_for_service(timeout_sec=10.0), \
            'list_faults service not available'

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def _call(self, client, request):
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)
        self.assertIsNotNone(future.result(), 'Service call timed out')
        return future.result()

    def _report(self, fault_code, event_type, severity=Fault.SEVERITY_ERROR):
        """Send one ReportFault event, the way an edge-triggered reporter does."""
        request = ReportFault.Request()
        request.fault_code = fault_code
        request.event_type = event_type
        request.severity = severity
        request.description = 'debounce and healing contract test'
        request.source_id = '/test_plc'
        response = self._call(self.report_client, request)
        self.assertTrue(response.accepted, f'ReportFault rejected for {fault_code}')

    def _status_of(self, fault_code):
        """Return the status of one fault, or None when the store has no such fault."""
        request = ListFaults.Request()
        request.statuses = ALL_STATUSES
        response = self._call(self.list_client, request)
        for fault in response.faults:
            if fault.fault_code == fault_code:
                return fault.status
        return None

    def _default_filter_codes(self):
        """Fault codes an operator sees with no status filter (CONFIRMED only)."""
        response = self._call(self.list_client, ListFaults.Request())
        return [fault.fault_code for fault in response.faults]

    def _wait_for_status(self, fault_code, expected, timeout_sec):
        """Poll until the fault reaches expected, returning the last status seen."""
        deadline = time.time() + timeout_sec
        status = self._status_of(fault_code)
        while time.time() < deadline and status != expected:
            time.sleep(0.25)
            status = self._status_of(fault_code)
        return status

    def test_01_single_failed_read_does_not_confirm(self):
        """One bad sample must not raise a confirmed fault."""
        code = 'PLC_SINGLE_READ'
        self._report(code, ReportFault.Request.EVENT_FAILED)

        status = self._status_of(code)
        self.assertIsNotNone(status, 'fault was not recorded at all')
        self.assertNotEqual(
            status, Fault.STATUS_CONFIRMED,
            'a single failed read confirmed the fault immediately'
        )
        self.assertEqual(status, Fault.STATUS_PREFAILED)

    def test_02_prefailed_fault_is_hidden_from_the_default_list(self):
        """A not-yet-confirmed fault must not reach an operator listing."""
        code = 'PLC_HIDDEN_WHILE_PENDING'
        self._report(code, ReportFault.Request.EVENT_FAILED)

        self.assertNotIn(
            code, self._default_filter_codes(),
            'an unconfirmed fault is already visible in the default fault list'
        )

    def test_03_sustained_condition_confirms_with_nobody_acting(self):
        """
        A real fault must still surface.

        The reporter sends its one FAILED and never repeats it, so only the
        time-based lever can promote this. Without it the fault would stay
        PREFAILED forever and the appliance would go quiet.
        """
        code = 'PLC_SUSTAINED'
        self._report(code, ReportFault.Request.EVENT_FAILED)

        status = self._wait_for_status(
            code, Fault.STATUS_CONFIRMED, AUTO_CONFIRM_SEC + 5.0
        )
        self.assertEqual(
            status, Fault.STATUS_CONFIRMED,
            f'a sustained fault never confirmed, it is still {status}'
        )
        self.assertIn(code, self._default_filter_codes())

    def test_04_transient_that_clears_in_time_never_confirms(self):
        """
        The falsifying case for the whole setting.

        A glitch that goes away before the window closes must never confirm.
        If it does, the configuration only delays a false alarm rather than
        filtering it.
        """
        code = 'PLC_TRANSIENT'
        self._report(code, ReportFault.Request.EVENT_FAILED)
        self._report(code, ReportFault.Request.EVENT_PASSED)

        # Sit past the auto-confirm window and the timer tick behind it.
        time.sleep(AUTO_CONFIRM_SEC + 3.0)

        status = self._status_of(code)
        self.assertNotEqual(
            status, Fault.STATUS_CONFIRMED,
            'a transient that already cleared was confirmed by the timer'
        )
        self.assertNotIn(code, self._default_filter_codes())

    def test_05_deasserted_alarm_heals_with_nobody_acting(self):
        """
        A confirmed fault must return to healed on the reporter's single clear.

        The reporter sends exactly one PASSED, so healing has to complete on
        that one event. With healing off, or with a threshold above zero, the
        fault stays CONFIRMED until a human clears it.
        """
        code = 'PLC_DEASSERTED'
        self._report(code, ReportFault.Request.EVENT_FAILED)

        status = self._wait_for_status(
            code, Fault.STATUS_CONFIRMED, AUTO_CONFIRM_SEC + 5.0
        )
        self.assertEqual(
            status, Fault.STATUS_CONFIRMED, 'fault never confirmed, cannot test healing'
        )

        self._report(code, ReportFault.Request.EVENT_PASSED)

        status = self._status_of(code)
        self.assertEqual(
            status, Fault.STATUS_HEALED,
            f'a de-asserted alarm did not heal on its single clear, it is {status}'
        )
        self.assertNotIn(code, self._default_filter_codes())


@launch_testing.post_shutdown_test()
class TestDebounceAndHealingShutdown(unittest.TestCase):
    """Check the node exited cleanly."""

    def test_exit_code(self, proc_info, fault_manager_node):
        launch_testing.asserts.assertExitCodes(
            proc_info, allowable_exit_codes=[0, -2, -15], process=fault_manager_node
        )
