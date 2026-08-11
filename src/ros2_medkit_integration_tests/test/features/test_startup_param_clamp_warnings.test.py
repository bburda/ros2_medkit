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

"""Every clamped startup parameter reports the clamp.

A mis-set startup parameter is coerced into its documented range so the gateway
cannot be broken by a typo. Until this test existed the coercion was silent for
most of those parameters: the config file said one thing, the process ran
another, and nothing in between said so. The symptom of a wrong value (a slow
or jittery gateway) shows up far from the cause (one line in a YAML file), so
the warning is the only thing that connects them.

Two gateways launch with deliberately mis-set values and the test reads their
own process output:

- ``gateway_node`` (the default port) sets every clamped parameter BELOW its
  documented floor. It also has to serve ``/health``: the clamp exists so a
  typo cannot break request serving, and ``GatewayTestCase.setUpClass`` polls
  ``/health`` before any test runs, so a floor value that killed the gateway
  would fail the file rather than pass quietly.
- ``gw_ceiling`` sets them ABOVE their documented ceiling.

COVERAGE LIMIT, stated so nobody reads more into a green run than it has:
``PARAMS_FLOOR_ONLY`` lists three parameters whose above-ceiling case is NOT
exercised - ``server.executor_threads`` (256), ``server.http_thread_pool_size``
(1024) and ``entity_cache.capacity`` (1,000,000). Clamping means the gateway
then RUNS at the ceiling, and that is not free. Measured, not estimated: one
gateway launched past all three ceilings serves ``/health`` correctly and warns
for each parameter, at **1299 threads and 1486 MB RSS**. Paying that (times a
runner already hosting the two gateways below, and again under a sanitizer) buys
one log line per parameter from the same four-line compare-and-warn the floor
case already drives, and that the other 11 parameters already drive in the
ceiling direction. Their below-floor case IS exercised, on the floor gateway.
"""

import unittest

from launch import LaunchDescription
import launch_testing
import launch_testing.actions

from ros2_medkit_test_utils.constants import (
    ALLOWED_EXIT_CODES,
    get_test_port,
    get_time_scale,
)
from ros2_medkit_test_utils.gateway_test_case import GatewayTestCase
from ros2_medkit_test_utils.launch_helpers import create_gateway_node

# Every startup parameter that is clamped on read, with its documented range and
# the out-of-range value each gateway launches with.
#
#   (parameter, floor, ceiling, below_floor_value, above_ceiling_value)
#
# Ranges are the ones in docs/config: keep this table and the docs in step, a
# range that moves in code without moving here is exactly the regression this
# file is meant to catch. ``above_ceiling_value`` is None for the parameters in
# the coverage limit described in the module docstring.
CLAMPED_PARAMS = [
    # server.* - main.cpp and http/rest_server.cpp
    ('server.executor_threads', 1, 256, 0, None),
    ('server.http_thread_pool_size', 1, 1024, 0, None),
    ('server.keep_alive_timeout_sec', 1, 3600, 0, 7200),
    # sse.* / node-level knobs - gateway_node.cpp
    ('sse.max_clients', 1, 1024, 0, 4096),
    ('service_call_timeout_sec', 1, 3600, 0, 7200),
    ('logs.buffer_size', 1, 100000, 0, 200000),
    ('entity_cache.capacity', 16, 1000000, 8, None),
    # subscription_executor.* - main.cpp declare_executor_config
    ('subscription_executor.max_queue_depth', 16, 4096, 8, 8192),
    ('subscription_executor.watchdog_threshold_ms', 100, 60000, 50, 120000),
    ('subscription_executor.watchdog_tick_ms', 10, 10000, 5, 20000),
    ('subscription_executor.graph_poll_tick_ms', 10, 10000, 5, 20000),
    # data_provider.* - main.cpp declare_data_provider_config
    ('data_provider.max_pool_size', 1, 4096, 0, 8192),
    ('data_provider.cold_wait_cap', 0, 1024, -1, 2048),
    ('data_provider.max_parallel_samples', 1, 256, 0, 512),
    ('data_provider.idle_safety_net_sec', 0, 86400, -1, 172800),
    ('data_provider.idle_sweep_tick_sec', 0, 3600, -1, 7200),
]

# The three whose above-ceiling case is skipped (see the module docstring).
PARAMS_FLOOR_ONLY = {
    name for name, _floor, _ceiling, _below, above in CLAMPED_PARAMS if above is None
}

FLOOR_PARAMS = {name: below for name, _f, _c, below, _a in CLAMPED_PARAMS}
CEILING_PARAMS = {
    name: above for name, _f, _c, _below, above in CLAMPED_PARAMS if above is not None
}


# The gateway logs one line per clamped parameter, in the shape the three
# pre-existing sites already used: "<parameter> <requested> clamped to <effective>".
def expected_warning(parameter, requested, effective):
    """Build the log line the gateway must emit for one clamped parameter."""
    return f'{parameter} {requested} clamped to {effective}'


# Warnings are emitted during construction, so they are already in the buffer by
# the time /health answers. The timeout only covers a slow runner's startup.
WARNING_TIMEOUT = 20

# Logged by GatewayNode::set_topic_data_provider, which main.cpp calls after the
# last clamped parameter has been read (node constructor -> RESTServer -> the
# executor and data-provider configs). The two tests that SCAN the output buffer
# rather than wait for one line need this: without it they would read a buffer
# that is still filling and pass on an empty one.
STARTUP_COMPLETE = 'TopicDataProvider attached'


def generate_test_description():
    # Floor gateway on the default port: GatewayTestCase polls /health here, so
    # this gateway also proves the clamped-to-floor values still serve requests.
    gateway_node = create_gateway_node(extra_params=dict(FLOOR_PARAMS))

    gw_ceiling = create_gateway_node(
        port=get_test_port(1),
        name='gateway_clamp_ceiling',
        extra_params=dict(CEILING_PARAMS),
    )

    return (
        LaunchDescription([
            gateway_node,
            gw_ceiling,
            launch_testing.actions.ReadyToTest(),
        ]),
        {
            'gateway_node': gateway_node,
            'gw_ceiling': gw_ceiling,
        },
    )


class TestStartupParamClampWarnings(GatewayTestCase):
    """A clamped startup parameter is never absorbed without a trace."""

    MIN_EXPECTED_APPS = 0  # no demo nodes; the gateways only need to start

    def test_below_floor_values_are_reported(self, proc_output, gateway_node):
        """Every parameter set below its floor names both values in the log.

        The requested value has to appear too. A line that reported only the
        effective value would still leave the operator unable to see that the
        config file and the process disagree, which is the whole defect.
        """
        reported = _clamp_lines(proc_output, gateway_node)
        for parameter, floor, _ceiling, below, _above in CLAMPED_PARAMS:
            with self.subTest(parameter=parameter, direction='floor'):
                self.assertIn(expected_warning(parameter, below, floor), reported)

    def test_above_ceiling_values_are_reported(self, proc_output, gw_ceiling):
        """Every parameter set above its ceiling names both values in the log.

        The floor gateway alone would pass against a clamp that only guarded
        the lower bound, so the upper endpoint gets its own gateway.
        """
        reported = _clamp_lines(proc_output, gw_ceiling)
        for parameter, _floor, ceiling, _below, above in CLAMPED_PARAMS:
            if above is None:
                continue
            with self.subTest(parameter=parameter, direction='ceiling'):
                self.assertIn(expected_warning(parameter, above, ceiling), reported)

    def test_in_range_values_stay_quiet(self, proc_output, gw_ceiling):
        """An in-range parameter is not reported as clamped.

        A "warn on every read" regression would satisfy both tests above while
        burying the real misconfigurations in noise. The ceiling gateway leaves
        the floor-only parameters at their defaults, so their names must not
        appear in a clamp line at all.
        """
        reported = _clamp_lines(proc_output, gw_ceiling)
        for parameter in sorted(PARAMS_FLOOR_ONLY):
            with self.subTest(parameter=parameter):
                self.assertFalse(
                    [line for line in reported if line.startswith(f'{parameter} ')],
                    f'{parameter} was left at its default but still reported a clamp',
                )

    def test_repeated_reads_warn_once_per_parameter(self, proc_output, gateway_node):
        """A parameter read twice at startup is reported once, not twice.

        server.http_thread_pool_size and sse.max_clients are read a second time
        in main.cpp to compare the pool against the SSE + cold-wait budget. That
        re-read deliberately uses the silent clamp: two identical warnings for
        one typo trains operators to skim past them.
        """
        reported = _clamp_lines(proc_output, gateway_node)
        for parameter in ('server.http_thread_pool_size', 'sse.max_clients'):
            with self.subTest(parameter=parameter):
                matching = [
                    line for line in reported if line.startswith(f'{parameter} ')
                ]
                self.assertEqual(
                    len(matching), 1,
                    f'{parameter} reported its clamp {len(matching)} times: {matching}',
                )


def _clamp_lines(proc_output, process):
    """Return every 'X clamped to Y' fragment this process logged during startup.

    Waits for STARTUP_COMPLETE first, so the buffer is whole before it is read.
    Every check in this file scans rather than waits per parameter: waiting
    would cost one timeout per missing warning, and a regression across all of
    them would blow the test's own time budget and report "no result file"
    instead of naming the parameter that went quiet.
    """
    proc_output.assertWaitFor(
        STARTUP_COMPLETE, process=process, timeout=WARNING_TIMEOUT * get_time_scale(),
    )
    lines = []
    for output in proc_output[process]:
        for line in output.text.decode(errors='replace').splitlines():
            marker = line.find(' clamped to ')
            if marker == -1:
                continue
            # Strip the rclcpp prefix ("[WARN] [stamp] [logger]: ") so the
            # fragment starts at the parameter name.
            start = line.rfind(': ', 0, marker)
            lines.append(line[start + 2:] if start != -1 else line)
    return lines


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        """Both gateways exit cleanly.

        A clamped value must leave a working gateway behind. An exit code here
        would mean the coercion turned a typo into a crash.
        """
        for info in proc_info:
            self.assertIn(
                info.returncode, ALLOWED_EXIT_CODES,
                f'{info.process_name} exited with code {info.returncode}',
            )
