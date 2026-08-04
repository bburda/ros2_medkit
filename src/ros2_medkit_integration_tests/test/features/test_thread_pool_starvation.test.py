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

"""Starvation-guard tests for the bounded HTTP pool (issue #440).

`test_thread_pool_bounds` runs a *well-sized* pool, so it can only ever pass.
This file launches a pool deliberately BELOW the documented budget
(`http_thread_pool_size` < `sse.max_clients` + `data_provider.cold_wait_cap`)
and asserts the guards the PR's safety actually rests on:

- the gateway emits the startup warning that flags the misconfiguration, and
- the bounded `executor_threads` value is actually applied (the gateway logs the
  thread count, so it is observable rather than merely set).

It additionally sweeps the documented ``server.executor_threads`` clamp range
``[1, 256]`` (issue #575): three extra gateways launch with the range floor,
the range ceiling, and an out-of-range value, and each must log the resolved
(clamped) count. Without the endpoint sweep, a regression in the clamp (or in
the parameter read) would only surface for mid-range values.

All checks read the gateways' own process output, so they are deterministic and
do not depend on request timing.
"""

import time
import unittest

from launch import LaunchDescription
import launch_testing
import launch_testing.actions
import requests

from ros2_medkit_test_utils.constants import (
    ALLOWED_EXIT_CODES,
    API_BASE_PATH,
    get_test_port,
)
from ros2_medkit_test_utils.gateway_test_case import GatewayTestCase
from ros2_medkit_test_utils.launch_helpers import create_gateway_node

# Pool below the shipped budget: 2 < sse.max_clients(2) + cold_wait_cap(4) = 6.
# executor_threads is set to a non-default value so the "applied" assertion is
# meaningful (a regression ignoring the parameter would log a different count).
HTTP_THREAD_POOL_SIZE = 2
SSE_MAX_CLIENTS = 2
EXECUTOR_THREADS = 3

# Clamp sweep for server.executor_threads (documented range [1, 256]).
# The out-of-range value must clamp to the nearest endpoint; 300 clamps to the
# ceiling, so its expected log line differs from the floor gateway's and the
# three assertions stay unambiguous even without per-process scoping.
EXECUTOR_THREADS_FLOOR = 1
EXECUTOR_THREADS_CEILING = 256
EXECUTOR_THREADS_INVALID = 0  # below the floor -> must clamp to 1


def generate_test_description():
    gateway_node = create_gateway_node(
        extra_params={
            'server.http_thread_pool_size': HTTP_THREAD_POOL_SIZE,
            'server.executor_threads': EXECUTOR_THREADS,
            'sse.max_clients': SSE_MAX_CLIENTS,
        },
    )

    # Clamp-endpoint gateways (issue #575): distinct node names and ports so
    # they can share the launch. Only their startup logs are asserted.
    gw_floor = create_gateway_node(
        port=get_test_port(1),
        name='gateway_threads_floor',
        extra_params={'server.executor_threads': EXECUTOR_THREADS_FLOOR},
    )
    gw_ceiling = create_gateway_node(
        port=get_test_port(2),
        name='gateway_threads_ceiling',
        extra_params={'server.executor_threads': EXECUTOR_THREADS_CEILING},
    )
    gw_invalid = create_gateway_node(
        port=get_test_port(3),
        name='gateway_threads_invalid',
        extra_params={'server.executor_threads': EXECUTOR_THREADS_INVALID},
    )

    return (
        LaunchDescription([
            gateway_node,
            gw_floor,
            gw_ceiling,
            gw_invalid,
            launch_testing.actions.ReadyToTest(),
        ]),
        {
            'gateway_node': gateway_node,
            'gw_floor': gw_floor,
            'gw_ceiling': gw_ceiling,
            'gw_invalid': gw_invalid,
        },
    )


class TestThreadPoolStarvationGuard(GatewayTestCase):
    """An under-budget pool is flagged, and the executor bound is observable."""

    MIN_EXPECTED_APPS = 0  # skip discovery waiting; the gateways only need to start

    def test_startup_warns_when_pool_below_budget(self, proc_output, gateway_node):
        """The gateway warns when http_thread_pool_size < max_clients + cold_wait_cap.

        Without this warning the misconfiguration is silent: a burst of SSE
        streams plus cold /data waits can pin every worker. The warning is the
        operator-facing guard, so we assert it is actually emitted.
        """
        proc_output.assertWaitFor(
            'is below sse.max_clients', process=gateway_node, timeout=15,
        )

    def test_executor_thread_bound_is_applied(self, proc_output, gateway_node):
        """The configured executor_threads value is honoured (and observable).

        The reviewer noted executor_threads was set in tests but never observed.
        The gateway logs the resolved count at startup, so assert it matches the
        value we launched with - a regression that dropped the parameter would
        log a different number.
        """
        proc_output.assertWaitFor(
            f'Main executor bounded to {EXECUTOR_THREADS} threads',
            process=gateway_node, timeout=15,
        )

    def test_executor_threads_floor_applied(self, proc_output, gw_floor):
        """The documented range floor (1) is accepted and applied as-is."""
        proc_output.assertWaitFor(
            'Main executor bounded to 1 threads', process=gw_floor, timeout=15,
        )

    def test_executor_threads_ceiling_applied(self, proc_output, gw_ceiling):
        """The documented range ceiling (256) is accepted and applied as-is."""
        proc_output.assertWaitFor(
            'Main executor bounded to 256 threads', process=gw_ceiling, timeout=15,
        )

    def test_executor_threads_ceiling_gateway_serves_requests(self):
        """The 256-thread gateway actually serves, not just logs its bound.

        Every other assertion in the clamp sweep reads a log line, so a
        regression that logged the clamped count and then failed to bring the
        executor up would stay green. One real request over HTTP pins that
        the clamped value was applied to a working gateway.
        """
        url = f'http://localhost:{get_test_port(2)}{API_BASE_PATH}/health'
        deadline = time.monotonic() + 30.0
        last_error = None
        while time.monotonic() < deadline:
            try:
                response = requests.get(url, timeout=5)
                if response.status_code == 200:
                    self.assertEqual(response.json().get('status'), 'healthy')
                    return
                last_error = f'HTTP {response.status_code}: {response.text}'
            except requests.RequestException as exc:
                last_error = repr(exc)
            time.sleep(0.5)
        self.fail(
            f'the executor_threads=256 gateway never served /health: {last_error}'
        )

    def test_executor_threads_invalid_clamps_to_floor(self, proc_output, gw_invalid):
        """An out-of-range value (0) clamps to the floor instead of breaking.

        0 would mean "all host cores" to rclcpp (footprint regression) or an
        empty pool to a naive reader; the documented behaviour is a clamp to
        the [1, 256] range, observable through the same startup log line.
        """
        proc_output.assertWaitFor(
            'Main executor bounded to 1 threads', process=gw_invalid, timeout=15,
        )


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        """All processes exit cleanly."""
        for info in proc_info:
            self.assertIn(
                info.returncode, ALLOWED_EXIT_CODES,
                f'{info.process_name} exited with code {info.returncode}',
            )
