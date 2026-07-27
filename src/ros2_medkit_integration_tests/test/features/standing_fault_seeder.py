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

"""Seed a confirmed fault into the fault manager, then exit.

Companion process for test_standing_fault_freeze_frame.test.py: it runs
before the gateway is launched, reports a fault under a plugin entity id and
exits 0 only once ListFaults shows the fault CONFIRMED. The launch file
starts the gateway on this process's exit, so the gateway provably comes up
with the fault already standing.
"""

import sys
import time

import rclpy
from rclpy.node import Node
from ros2_medkit_msgs.msg import Fault
from ros2_medkit_msgs.srv import ListFaults, ReportFault

FAULT_CODE = 'PLC_STANDING_AT_BOOT'
SOURCE_ID = 'test_plc_app'


def main():
    rclpy.init()
    node = Node('standing_fault_seeder')
    report = node.create_client(ReportFault, '/fault_manager/report_fault')
    lister = node.create_client(ListFaults, '/fault_manager/list_faults')
    if not report.wait_for_service(timeout_sec=30.0):
        print('report_fault service not available', file=sys.stderr)
        return 1
    if not lister.wait_for_service(timeout_sec=30.0):
        print('list_faults service not available', file=sys.stderr)
        return 1

    deadline = time.monotonic() + 30.0
    while time.monotonic() < deadline:
        req = ReportFault.Request()
        req.fault_code = FAULT_CODE
        req.event_type = ReportFault.Request.EVENT_FAILED
        req.severity = Fault.SEVERITY_ERROR
        req.description = 'standing fault seeded before gateway start'
        req.source_id = SOURCE_ID
        future = report.call_async(req)
        rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)

        list_req = ListFaults.Request()
        list_req.statuses = [Fault.STATUS_CONFIRMED]
        list_future = lister.call_async(list_req)
        rclpy.spin_until_future_complete(node, list_future, timeout_sec=5.0)
        response = list_future.result()
        if response is not None and any(
                f.fault_code == FAULT_CODE for f in response.faults):
            print(f'{FAULT_CODE} confirmed; gateway may start now')
            node.destroy_node()
            rclpy.shutdown()
            return 0
        time.sleep(0.5)

    print(f'{FAULT_CODE} never reached CONFIRMED', file=sys.stderr)
    return 1


if __name__ == '__main__':
    sys.exit(main())
