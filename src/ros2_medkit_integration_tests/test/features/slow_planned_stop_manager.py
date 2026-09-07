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

"""A stand-in fault manager whose planned-stop service misbehaves on purpose.

The absent case proves nothing about timeouts: with no service advertised the
gateway short-circuits and answers in microseconds. The case that costs anything
is a fault manager that IS on the graph and takes its time, and that is what this
stands in for. It advertises the four core fault services so the gateway
considers it available, answers them immediately, and sleeps inside
list_planned_stops.

Named `fault_manager` inside the namespace the gateway is pointed at, because
the gateway addresses its fault manager as `<namespace>/fault_manager`.

Three shapes, chosen by argument:

``--delay SEC``       how long ~/list_planned_stops sleeps before answering.
``--window``          answer with one window covering the next hour, so a fault
                      reads as expected until the service goes away.
``--drop-after SEC``  destroy the planned-stop services after SEC, leaving a
                      fault manager that still serves faults - a pre-planned-stop
                      release, or one that was replaced under the gateway.
"""

import argparse
import sys
import time

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from ros2_medkit_msgs.msg import FaultEvent, PlannedStop
from ros2_medkit_msgs.srv import (
    ClearFault,
    DeclarePlannedStop,
    EndPlannedStop,
    GetFault,
    ListFaults,
    ListPlannedStops,
    ReportFault,
)

# Long enough that a formatting pass paying it under the queue lock is
# unmistakable next to the assertion's budget, short enough to stay inside the
# gateway's 5 s service timeout so the call SUCCEEDS - a failure would trip the
# 30 s back-off and the slowness would stop happening after the first frame.
LIST_PLANNED_STOPS_DELAY_SEC = 3.0


class SlowPlannedStopManager(Node):

    def __init__(self, delay, serve_window, drop_after):
        super().__init__('fault_manager', namespace='slow')
        self._delay = delay
        self._serve_window = serve_window
        group = ReentrantCallbackGroup()

        self._planned_stop_services = [
            self.create_service(ListPlannedStops, '~/list_planned_stops',
                                self._slow_list_planned_stops, callback_group=group),
        ]
        self.create_service(ListFaults, '~/list_faults',
                            self._empty_fault_list, callback_group=group)
        self.create_service(ReportFault, '~/report_fault',
                            self._accept_report, callback_group=group)
        self.create_service(GetFault, '~/get_fault',
                            self._no_such_fault, callback_group=group)
        self.create_service(ClearFault, '~/clear_fault',
                            self._no_such_fault, callback_group=group)
        self._planned_stop_services.append(
            self.create_service(DeclarePlannedStop, '~/declare_planned_stop',
                                self._refuse_declare, callback_group=group))
        self._planned_stop_services.append(
            self.create_service(EndPlannedStop, '~/end_planned_stop',
                                self._no_such_fault, callback_group=group))

        if drop_after is not None:
            self.create_timer(drop_after, self._drop_planned_stop_services,
                              callback_group=group)

        self._events = self.create_publisher(FaultEvent, '~/events', 10)
        self.get_logger().info('slow planned-stop manager up')

    def _drop_planned_stop_services(self):
        if not self._planned_stop_services:
            return
        for service in self._planned_stop_services:
            self.destroy_service(service)
        self._planned_stop_services = []
        self.get_logger().info('planned-stop services dropped')

    def _slow_list_planned_stops(self, _request, response):
        if self._delay > 0.0:
            self.get_logger().info('list_planned_stops: sleeping')
            time.sleep(self._delay)
            self.get_logger().info('list_planned_stops: answering')
        if self._serve_window:
            window = PlannedStop()
            window.id = 'stub-window'
            window.starts_at.sec = int(time.time()) - 60
            window.ends_at.sec = int(time.time()) + 3600
            window.reason = 'served by the stub'
            window.declared_by = 'stub'
            window.declared_at.sec = window.starts_at.sec
            response.stops = [window]
        return response

    @staticmethod
    def _empty_fault_list(_request, response):
        return response

    @staticmethod
    def _accept_report(_request, response):
        response.accepted = True
        return response

    @staticmethod
    def _no_such_fault(_request, response):
        response.success = False
        response.message = 'stub'
        return response

    @staticmethod
    def _refuse_declare(_request, response):
        response.success = False
        response.message = 'stub'
        return response


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--delay', type=float, default=LIST_PLANNED_STOPS_DELAY_SEC)
    parser.add_argument('--window', action='store_true')
    parser.add_argument('--drop-after', type=float, default=None)
    args, _ = parser.parse_known_args()

    rclpy.init()
    node = SlowPlannedStopManager(args.delay, args.window, args.drop_after)
    # Multi-threaded so the sleep in one call does not also stall the services
    # the gateway checks for readiness - the point is a slow ANSWER, not a node
    # that has stopped serving.
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.remove_node(node)
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
