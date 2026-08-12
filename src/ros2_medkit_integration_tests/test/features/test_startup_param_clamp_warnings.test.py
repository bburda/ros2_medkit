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

Four gateways launch with deliberately mis-set values and the test reads their
own process output:

- ``gateway_node`` (the default port) sets every clamped parameter BELOW its
  documented floor. It also has to serve ``/health``: the clamp exists so a
  typo cannot break request serving, and ``GatewayTestCase.setUpClass`` polls
  ``/health`` before any test runs, so a floor value that killed the gateway
  would fail the file rather than pass quietly.
- ``gw_ceiling`` sets them ABOVE their documented ceiling.
- ``gw_in_range`` sets every one of them to a legal value and must report
  nothing at all. Without it, a site that lost its "did the clamp actually
  fire" guard would warn at every boot of a correctly configured gateway, and
  no other assertion in this file would notice.
- ``gw_rejected`` carries the parameters that are REFUSED rather than clamped
  (half a port number is not a port), including three values above INT_MAX
  that used to wrap back into the legal band and be accepted in silence.

COVERAGE LIMIT, stated so nobody reads more into a green run than it has:
``PARAMS_FLOOR_ONLY`` lists three parameters whose above-ceiling case is NOT
exercised - ``server.executor_threads`` (256), ``server.http_thread_pool_size``
(1024) and ``entity_cache.capacity`` (1,000,000). Clamping means the gateway
then RUNS at the ceiling, and that is not free. Measured, not estimated: one
gateway launched past all three ceilings serves ``/health`` correctly and warns
for each parameter, at **1299 threads and 1486 MB RSS**. Paying that (on a
runner already hosting the three other gateways, and again under a sanitizer)
buys the same four-line compare-and-warn that the floor case already drives,
that the in-range case pins the guard of, and that the other 15 parameters
already drive in the ceiling direction.

Be precise about what that costs, because it is more than a log line: for those
three, and only those three, the CEILING CONSTANT ITSELF is unpinned. Widening
``server.http_thread_pool_size`` from 1024 to 8, or ``server.executor_threads``
from 256 to 4096, leaves every test in this file green while the config docs go
on advertising the old range. The floor row still warns and the in-range value
is still silent under either bound. The table comment below says a range that
moves in code without moving here gets caught - that holds for the 15 rows with
a ceiling value, not for these three.
"""

import os
import unittest

from ament_index_python.packages import get_package_prefix
from launch import LaunchDescription
import launch_testing
import launch_testing.actions

from ros2_medkit_test_utils.constants import (
    ALLOWED_EXIT_CODES,
    get_test_domain_id,
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
# Ranges are the ones in docs/config: keep this table and the docs in step. A
# range that moves in code without moving here is caught for every row that has
# an ``above_ceiling_value``; for the three rows where it is None only the floor
# is pinned, which the module docstring spells out.
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
    # aggregation.* - gateway_node.cpp, only read when aggregation is enabled
    ('aggregation.max_discovered_peers', 1, 1000, 0, 2000),
    # fault_triggers.* - gateway_node.cpp, only read once a plugin is loaded.
    # The ceiling is INT_MAX because the value is narrowed to int for the timer;
    # it is a type boundary, not a tuning choice.
    ('fault_triggers.poll_interval_ms', 50, 2147483647, 0, 4294967296),
]

# In-range values, one per clamped parameter. A gateway launched with these must
# report NO clamp at all. Without this, no gateway in the file ever leaves most
# of these parameters in range, and a site that dropped its `if (used !=
# requested)` guard would warn on every boot of a correctly configured gateway
# while every other assertion here stayed green.
IN_RANGE_PARAMS = {
    'server.executor_threads': 2,
    'server.http_thread_pool_size': 6,
    'server.keep_alive_timeout_sec': 2,
    'sse.max_clients': 2,
    'service_call_timeout_sec': 10,
    'logs.buffer_size': 200,
    'entity_cache.capacity': 256,
    'subscription_executor.max_queue_depth': 256,
    'subscription_executor.watchdog_threshold_ms': 5000,
    'subscription_executor.watchdog_tick_ms': 1000,
    'subscription_executor.graph_poll_tick_ms': 100,
    'data_provider.max_pool_size': 256,
    'data_provider.cold_wait_cap': 4,
    'data_provider.max_parallel_samples': 8,
    'data_provider.idle_safety_net_sec': 900,
    'data_provider.idle_sweep_tick_sec': 60,
    'aggregation.max_discovered_peers': 50,
    'fault_triggers.poll_interval_ms': 1000,
}

# Parameters whose out-of-range value is REFUSED and replaced by the documented
# default instead of being clamped. Clamping makes no sense for them: half a
# port number is not a port. The warning still has to name the refused value.
#
#   (parameter, refused_value, expected log fragment)
#
# The first three carry a value above INT_MAX on purpose. Each of these was read
# into an int BEFORE its range check, so the value wrapped back into the legal
# band and was accepted in silence: port 4294976296 narrows to 9000.
REJECTED_PARAMS = [
    ('server.port', 4294976296,
     'Invalid port 4294976296. Must be between 1024-65535. Using default 8080.'),
    ('refresh_interval_ms', 4294967396,
     'Invalid backstop refresh interval 4294967396ms'),
    ('sse.max_duration_sec', 4294967300,
     'sse.max_duration_sec 4294967300 must be in [1, 2147483647]'),
    ('discovery.refresh_debounce_ms', 70000,
     'Invalid discovery.refresh_debounce_ms 70000ms'),
    ('bulk_data.max_upload_size', -1,
     'bulk_data.max_upload_size -1 is negative'),
    ('sse.max_subscriptions', -1,
     'sse.max_subscriptions -1 must be >= 1'),
    # Above INT_MAX on purpose: read as int this narrows to 0, which trips the
    # same < 1 branch and logs "0", so a negative alone would not notice the
    # read-before-narrowing order being reverted.
    ('triggers.max_triggers', 4294967296,
     'triggers.max_triggers 4294967296 out of range. Using default 1000.'),
    # Two float checks that a "below or above the range" test would not catch,
    # because every comparison against NaN is false. NaN does survive the launch
    # path: launch_ros hands the parameter through as a double, verified against
    # a running gateway. Both sites are written as isfinite plus a positive
    # range test for this reason.
    ('topic_sample_timeout_sec', float('nan'),
     'topic_sample_timeout_sec nan is outside 0.1-30.0'),
    # +inf, not NaN: the check is isfinite && > 0, and NaN already fails "> 0"
    # on its own. Infinity is the only value that passes the comparison and is
    # caught solely by isfinite - and the one that would reach wait_for_service
    # and pin an HTTP worker for the life of the process.
    ('fault_manager.service_timeout_sec', float('inf'),
     'fault_manager.service_timeout_sec inf must be finite and > 0'),
    # parameter_service_timeout_sec deliberately has NO row here. It carries a
    # FloatingPointRange descriptor, and what rclcpp does with NaN against that
    # descriptor differs by distro: Jazzy accepts it as in-range (verified on a
    # running gateway, which logged "timeout=nans") while Lyrical refuses it and
    # the process exits 1 before reaching our check. Asserting either outcome
    # pins rclcpp's behaviour rather than ours. The gateway's own isfinite check
    # stays, because it is the only thing standing between NaN and a duration
    # cast on the distros that let it through.
]

# Ordinary out-of-range values for the same two floats. They cannot ride on the
# gateway above - one parameter takes one value - and they are worth keeping
# separate anyway: NaN pins the isfinite guard, these pin the range itself, and
# a mutation that dropped either would leave the other green.
FLOOR_EXTRA_REJECTED = [
    ('topic_sample_timeout_sec', 0.05,
     'topic_sample_timeout_sec 0.05 is outside 0.1-30.0'),
    ('fault_manager.service_timeout_sec', -1.0,
     'fault_manager.service_timeout_sec -1.00 must be finite and > 0'),
    ('triggers.max_triggers', -1,
     'triggers.max_triggers -1 out of range. Using default 1000.'),
]

# The upper end of the same float range. Split across gateways because one
# parameter takes one value per process, and both ends have to be pinned: with
# only 60.0, deleting the ">= 0.1" term from the check is a green mutation.
CEILING_EXTRA_REJECTED = [
    ('topic_sample_timeout_sec', 60.0,
     'topic_sample_timeout_sec 60.00 is outside 0.1-30.0'),
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


# Do NOT read this as "the warnings are there once /health answers".
# start_rest_server() runs inside the GatewayNode constructor, so /health goes
# live before main.cpp reads server.executor_threads, the subscription_executor
# and data_provider groups, and fault_triggers - eleven of the eighteen clamp
# warnings come after that. STARTUP_COMPLETE below is what actually bounds them.
WARNING_TIMEOUT = 20

# Logged by GatewayNode::init_fault_trigger_engine, the LAST thing main.cpp does
# before it starts spinning, and after every clamped parameter has been read
# (node constructor -> RESTServer -> executor and data-provider configs ->
# fault-trigger engine). Every check here scans the output buffer, so the anchor
# has to sit after the last assertion target or a scan can read a buffer that is
# still filling and pass on what is missing from it.
#
# 'TopicDataProvider attached' is NOT good enough and was the first choice:
# main.cpp emits it at line 212 but reads fault_triggers.poll_interval_ms at
# line 228, with a synchronous subscription creation in between. Every gateway
# in this file loads a plugin, so the engine always starts and the line always
# comes.
STARTUP_COMPLETE = 'fault-trigger engine active'


def generate_test_description():
    # Two parameters in the table are only read when something else is on:
    # aggregation.max_discovered_peers needs aggregation, and the
    # fault_triggers.* group needs at least one loaded plugin. With no peers,
    # mDNS off and the plugin idle, neither does anything beyond making its
    # parameter reachable.
    plugin_path = os.path.join(
        get_package_prefix('ros2_medkit_gateway'),
        'lib', 'ros2_medkit_gateway', 'libtest_gateway_plugin.so',
    )
    # One DDS domain per gateway. Nothing here is a peer of anything else, and
    # four gateways on one domain means each one runs its discovery over the
    # other three's nodes for no reason - and four sets of participants expire
    # on that domain at teardown, which the next test to lease it inherits.

    def _domain(offset):
        return {'ROS_DOMAIN_ID': str(get_test_domain_id(offset))}

    reachable = {
        'aggregation.enabled': True,
        'plugins': ['test_plugin'],
        'plugins.test_plugin.path': plugin_path,
    }

    # Floor gateway on the default port: GatewayTestCase polls /health here, so
    # this gateway also proves the clamped-to-floor values still serve requests.
    gateway_node = create_gateway_node(
        extra_env=_domain(0),
        extra_params={
            **FLOOR_PARAMS,
            **{name: value for name, value, _fragment in FLOOR_EXTRA_REJECTED},
            **reachable,
        },
    )

    gw_ceiling = create_gateway_node(
        port=get_test_port(1),
        name='gateway_clamp_ceiling',
        extra_env=_domain(1),
        extra_params={
            **CEILING_PARAMS,
            **{name: value for name, value, _fragment in CEILING_EXTRA_REJECTED},
            **reachable,
        },
    )

    gw_in_range = create_gateway_node(
        port=get_test_port(2),
        name='gateway_clamp_in_range',
        extra_env=_domain(2),
        extra_params={**IN_RANGE_PARAMS, **reachable},
    )

    # The refused values include server.port itself, so this gateway falls back
    # to the default 8080 rather than the port it was given. Nothing here talks
    # to it over HTTP; only its startup log is read. If 8080 is already taken the
    # gateway logs that it could not start serving and keeps running, so the
    # assertions and the exit code hold either way.
    gw_rejected = create_gateway_node(
        port=get_test_port(3),
        name='gateway_param_rejected',
        extra_env=_domain(3),
        extra_params={
            **{name: value for name, value, _fragment in REJECTED_PARAMS},
            **reachable,
        },
    )

    return (
        LaunchDescription([
            gateway_node,
            gw_ceiling,
            gw_in_range,
            gw_rejected,
            launch_testing.actions.ReadyToTest(),
        ]),
        {
            'gateway_node': gateway_node,
            'gw_ceiling': gw_ceiling,
            'gw_in_range': gw_in_range,
            'gw_rejected': gw_rejected,
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

        text = _output_text(proc_output, gateway_node)
        for parameter, _value, fragment in FLOOR_EXTRA_REJECTED:
            with self.subTest(parameter=parameter, direction='out-of-range'):
                self.assertIn(fragment, text)

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

        text = _output_text(proc_output, gw_ceiling)
        for parameter, _value, fragment in CEILING_EXTRA_REJECTED:
            with self.subTest(parameter=parameter, direction='out-of-range'):
                self.assertIn(fragment, text)

    def test_in_range_values_stay_quiet(self, proc_output, gw_in_range):
        """A gateway configured entirely in range reports no clamp at all.

        A "warn on every read" regression - one site losing its
        `if (effective != requested)` guard - would satisfy the floor and
        ceiling tests while making a correctly configured gateway shout at
        every boot. Only an in-range observation catches it, and every one of
        the parameters needs its own, because a dropped guard at one site says
        nothing about the others.
        """
        reported = _clamp_lines(proc_output, gw_in_range)
        self.assertEqual(
            reported, [],
            f'a gateway with every parameter in range still reported: {reported}',
        )

    def test_ceiling_coverage_is_not_dropped_by_accident(self):
        """Only the three documented parameters may skip the ceiling direction.

        The ceiling loop skips any row whose above-value is None. Without this,
        setting one more row to None removes its ceiling coverage silently - no
        failure, no skip marker, nothing in the output to notice.
        """
        self.assertEqual(
            PARAMS_FLOOR_ONLY,
            {'server.executor_threads', 'server.http_thread_pool_size',
             'entity_cache.capacity'},
            'the set of parameters exempt from the ceiling direction changed; '
            'the module docstring explains why exactly these three are exempt',
        )

    def test_refused_values_take_the_documented_default(self, proc_output, gw_rejected):
        """The value the gateway ends up using is the documented default.

        Every other assertion here reads a warning. A warning is a claim: the
        site could name the default in its message and then assign something
        else, and nothing would notice. These three parameters log what they
        actually installed, so they can be checked rather than believed.
        """
        text = _output_text(proc_output, gw_rejected)
        for fragment in (
            'Subscription manager: max_subscriptions=100',
            'max_upload=104857600B',
            'Trigger subsystem: enabled (max=1000',
        ):
            with self.subTest(fragment=fragment):
                self.assertIn(fragment, text)

    def test_rejected_values_name_the_refused_value(self, proc_output, gw_rejected):
        """A refused value is named in the log, not just replaced.

        These parameters fall back to their documented default instead of being
        clamped. Three of them are set above INT_MAX here: each used to be
        narrowed to int before its range check, so the value wrapped back into
        the legal band and was taken as valid without a word.
        """
        text = _output_text(proc_output, gw_rejected)
        for parameter, _value, fragment in REJECTED_PARAMS:
            with self.subTest(parameter=parameter):
                self.assertIn(fragment, text)

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


def _output_text(proc_output, process):
    """Return this process's whole startup output as one string.

    Waits for STARTUP_COMPLETE first, so the buffer is whole before it is read.
    Every check in this file scans rather than waits per parameter: waiting
    would cost one timeout per missing warning, and a regression across all of
    them would blow the test's own time budget and report "no result file"
    instead of naming the parameter that went quiet.
    """
    proc_output.assertWaitFor(
        STARTUP_COMPLETE, process=process, timeout=WARNING_TIMEOUT * get_time_scale(),
    )
    # Concatenated with NO separator on purpose. proc_output yields raw stream
    # chunks, not lines: a chunk boundary can fall in the middle of a log line,
    # and joining with '\n' then splices a newline into the message so a
    # substring match on it fails while the line is plainly there in the log.
    # That is not hypothetical - it is how this file first failed under a
    # sanitizer, where the timing moved the boundary. The stream carries its own
    # newlines, so plain concatenation reconstructs it.
    return ''.join(
        output.text.decode(errors='replace') for output in proc_output[process]
    )


def _clamp_lines(proc_output, process):
    """Return every 'X clamped to Y' fragment this process logged during startup."""
    lines = []
    for line in _output_text(proc_output, process).splitlines():
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
        """Every gateway exits cleanly.

        A coerced or refused value must leave a working gateway behind. An exit
        code here would mean a mis-set parameter turned a typo into a crash.
        """
        for info in proc_info:
            self.assertIn(
                info.returncode, ALLOWED_EXIT_CODES,
                f'{info.process_name} exited with code {info.returncode}',
            )
