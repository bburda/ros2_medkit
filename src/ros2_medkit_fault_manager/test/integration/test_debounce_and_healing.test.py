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
Healing contract for a reporter that sends one event per transition.

Such a reporter raises a fault with a single FAILED and de-asserts it with a
single PASSED, never repeating either while the condition holds. Both debounce
directions count events, so for this reporter only the counts reachable in one
event are usable.

The suite runs twice, once per healing_threshold, because the whole contract
turns on that value: at 0 the single PASSED heals, at the default 3 it does not
and the fault stays CONFIRMED with nobody able to clear it but a human. Each
test states which outcome it expects for the threshold it runs under, so the
run at 3 is a falsifying control rather than a skipped case.

Every case sends the number of events such a reporter really sends, never the
number the counter would need.
"""

import os
import shutil
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

# Only 0 lets a single PASSED reach the healing threshold. 3 is the parameter
# default and is exercised to show it leaves the fault confirmed.
HEALING_THRESHOLDS = [0, 3]

# Long enough that any timer-driven promotion would have fired. No test sets
# auto_confirm_after_sec, so a status observed after this period is stable.
QUIET_PERIOD_SEC = 5.0

# Every status, so a test can see a fault the default CONFIRMED-only filter hides.
ALL_STATUSES = ['PREFAILED', 'PREPASSED', 'CONFIRMED', 'HEALED', 'CLEARED']

_temp_dirs = []


# keep_alive below parametrize: parametrize rebuilds the description with
# functools.update_wrapper against the undecorated function, so a marker set on
# the outer wrapper never reaches the per-parameter run.
@launch_testing.parametrize('healing_threshold', HEALING_THRESHOLDS)
@launch_testing.markers.keep_alive
def generate_test_description(healing_threshold):
    """Launch fault_manager with healing on and the threshold under test."""
    temp_dir = tempfile.mkdtemp(prefix='debounce_healing_')
    _temp_dirs.append(temp_dir)

    fault_manager_node = launch_ros.actions.Node(
        package='ros2_medkit_fault_manager',
        executable='fault_manager_node',
        name='fault_manager',
        output='screen',
        parameters=[{
            # SQLite rather than the in-memory store: the counter that decides
            # healing is persisted, and both backends have to agree on it.
            'storage_type': 'sqlite',
            'database_path': os.path.join(temp_dir, 'faults.db'),
            'healing_enabled': True,
            'healing_threshold': healing_threshold,
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
            'healing_threshold': healing_threshold,
        },
    )


class TestHealingOnASingleClear(unittest.TestCase):
    """A de-asserted alarm must reach healed on the one PASSED it gets."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        try:
            cls._setup_clients()
        except Exception:
            rclpy.shutdown()
            raise

    @classmethod
    def _setup_clients(cls):
        cls.node = Node('test_debounce_healing_client')
        cls.report_client = cls.node.create_client(ReportFault, '/fault_manager/report_fault')
        cls.list_client = cls.node.create_client(ListFaults, '/fault_manager/list_faults')

        assert cls.report_client.wait_for_service(timeout_sec=30.0), \
            'report_fault service not available'
        assert cls.list_client.wait_for_service(timeout_sec=30.0), \
            'list_faults service not available'

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def _call(self, client, request):
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=10.0)
        self.assertIsNotNone(future.result(), 'Service call timed out')
        return future.result()

    def _report(self, fault_code, event_type, severity=Fault.SEVERITY_ERROR):
        """Send one ReportFault event, the way a one-event reporter does."""
        request = ReportFault.Request()
        request.fault_code = fault_code
        request.event_type = event_type
        request.severity = severity
        request.description = 'healing contract test'
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

    def test_01_a_de_asserted_alarm_heals_on_its_single_clear(self, healing_threshold):
        """
        The fix this suite exists for.

        One FAILED raises the fault, one PASSED de-asserts it, and nobody
        clears anything by hand. At threshold 0 the counter reaches the
        threshold on that one event; at 3 it cannot, and the fault latches.
        """
        code = 'PLC_DEASSERTED'
        self._report(code, ReportFault.Request.EVENT_FAILED)
        self.assertEqual(
            self._status_of(code), Fault.STATUS_CONFIRMED,
            'the raise did not confirm, so healing cannot be under test'
        )

        self._report(code, ReportFault.Request.EVENT_PASSED)
        status = self._status_of(code)

        if healing_threshold == 0:
            self.assertEqual(
                status, Fault.STATUS_HEALED,
                f'a de-asserted alarm did not heal on its single clear, it is {status}'
            )
            self.assertNotIn(code, self._default_filter_codes())
        else:
            self.assertEqual(
                status, Fault.STATUS_CONFIRMED,
                'threshold 3 unexpectedly healed on one PASSED, so threshold 0 '
                'is not what makes healing reachable'
            )
            self.assertIn(code, self._default_filter_codes())

    def test_02_a_healed_fault_confirms_again_when_the_condition_returns(self, healing_threshold):
        """
        The guard that a healed fault is not a dead fault.

        HEALED is latched, and escaping the latch costs
        healing_threshold - confirmation_threshold FAILED events. A one-event
        reporter sends one, so any healing_threshold above 0 leaves the second
        occurrence of a fault code permanently invisible.
        """
        code = 'PLC_RERAISE'
        self._report(code, ReportFault.Request.EVENT_FAILED)
        self._report(code, ReportFault.Request.EVENT_PASSED)

        expected_after_clear = (
            Fault.STATUS_HEALED if healing_threshold == 0 else Fault.STATUS_CONFIRMED
        )
        self.assertEqual(self._status_of(code), expected_after_clear)

        self._report(code, ReportFault.Request.EVENT_FAILED)
        status = self._status_of(code)
        self.assertEqual(
            status, Fault.STATUS_CONFIRMED,
            f'a returning condition did not leave the fault confirmed, it is {status}'
        )
        self.assertIn(code, self._default_filter_codes())

        # At 0 the fault genuinely left CONFIRMED and came back, which is the
        # case the latch could swallow. At 3 it never left, so this run only
        # shows that a repeat FAILED does not disturb a confirmed fault.
        if healing_threshold == 0:
            self.assertEqual(expected_after_clear, Fault.STATUS_HEALED)

    def test_03_a_healed_fault_stays_healed_while_the_condition_is_gone(self, healing_threshold):
        """A settled fault must not change status without an event."""
        code = 'PLC_STAYS_HEALED'
        self._report(code, ReportFault.Request.EVENT_FAILED)
        self._report(code, ReportFault.Request.EVENT_PASSED)
        settled = self._status_of(code)

        time.sleep(QUIET_PERIOD_SEC)
        self.assertEqual(
            self._status_of(code), settled,
            'the fault changed status with no event to cause it'
        )
        if healing_threshold == 0:
            self.assertEqual(settled, Fault.STATUS_HEALED)
            self.assertNotIn(code, self._default_filter_codes())
        else:
            self.assertEqual(settled, Fault.STATUS_CONFIRMED)
            self.assertIn(code, self._default_filter_codes())

    def test_04_one_failed_read_confirms_immediately(self, healing_threshold):
        """
        The limitation, pinned so nobody assumes otherwise.

        confirmation_threshold defaults to -1, so the first FAILED confirms.
        Filtering a single noisy read is not reachable from this node's
        configuration for a one-event reporter: raising the threshold means the
        second event that would confirm never arrives.
        """
        code = 'PLC_SINGLE_READ'
        self._report(code, ReportFault.Request.EVENT_FAILED)

        self.assertEqual(
            self._status_of(code), Fault.STATUS_CONFIRMED,
            'the documented immediate-confirmation behaviour changed'
        )
        self.assertIn(code, self._default_filter_codes())


@launch_testing.post_shutdown_test()
class TestHealingShutdown(unittest.TestCase):
    """Check the node exited cleanly and clean up the databases."""

    def test_exit_code(self, proc_info, fault_manager_node):
        launch_testing.asserts.assertExitCodes(proc_info, process=fault_manager_node)

    def test_temp_dirs_removed(self):
        # Idempotent: the parametrized suite runs this once per launch and the
        # list is module-level, so a directory may already be gone. The contract
        # is the end state, not that this call did the removing.
        for path in _temp_dirs:
            shutil.rmtree(path, ignore_errors=True)
            self.assertFalse(os.path.exists(path), f'{path} survived cleanup')
