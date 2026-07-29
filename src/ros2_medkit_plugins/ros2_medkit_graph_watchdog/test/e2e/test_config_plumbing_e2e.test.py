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
"""Config-plumbing e2e: proves NESTED plugin config reaches a live detector
through the REAL gateway.

A C++ unit test cannot prove this - it hand-builds already-nested JSON and
calls a detector's configure() directly, bypassing the real config-delivery
path (extract_plugin_config -> detector_config.hpp ->
ParamDriftConfig::from_json) entirely. That bypass is exactly what hid the
pre-existing bug this test guards against a regression of: detector_config.hpp
used to read a FLAT dotted-key shape while the gateway delivers NESTED, and
even after fixing that, param_drift's own `expect` sub-object was read via a
flat "expect." prefix scan and silently dropped through the real gateway.

Uses a param_drift `expect` rule, which fires on a STATIC graph (no runtime
drift needed - expect rules are absolute, see param_drift_policy.hpp and
test_param_drift_integration.cpp's ExpectRuleFlagsWrongFromStart). This
exercises BOTH config layers at once: the `expect` field (extract_detector_config
-> ParamDriftConfig::from_json) and `mode` (detector_config.hpp).

Runs as THREE separate CTest targets (see CMakeLists.txt), each launching its
OWN gateway+fault_manager+demo-node stack (config is read once at plugin
set_context() time, so proving both layers needs two distinct launches, not
two test methods sharing one). The WATCHDOG_E2E_SCENARIO env var selects
which launch + assertion runs in this process:

- "positive": plugins.graph_watchdog.detectors.param_drift.expect.use_sim_time
  = True, with NO mode override. The demo node runs with its real default
  use_sim_time=False, an immediate mismatch -> GRAPH_PARAM_DRIFT must raise.
  If the `expect` nested reader (layer 2) were still broken, `expect` would
  be silently dropped and this would never fire.
- "mode_off": same expect mismatch, PLUS
  plugins.graph_watchdog.detectors.param_drift.mode = "off" -> GRAPH_PARAM_DRIFT
  must never appear. If the `mode` nested reader (layer 1, detector_config.hpp)
  were still broken, "off" would never be recognized and this detector would
  keep raising despite the operator disabling it.
- "mode_off_yaml_bool": same, but `mode` is the YAML 1.1 boolean a BARE `off`
  actually parses to. This is the form operators write, and it reaches the
  plugin as a JSON bool, never a string.

Both "off" scenarios assert an ABSENCE, so both first gate on the plugin's own
GET /x-medkit-watchdog route (harness.wait_until_watchdog_armed). Without that
gate the assertion passes just as well on a stack that never came up: the
gateway survives a plugin that fails to dlopen and a REST server that cannot
bind, both still exit 0, and poll_faults returns None on any transport error.
"""

import os
import sys
import unittest

import launch_testing

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from harness import (  # noqa: E402
    create_watchdog_test_launch,
    poll_faults,
    wait_until_watchdog_armed,
)

from ros2_medkit_test_utils.constants import ALLOWED_EXIT_CODES, get_test_port  # noqa: E402

# No default on purpose. A default makes this file FAIL OPEN: if the CTest ENVIRONMENT property
# never reaches the process - an edit that drops the variable from one of the three targets,
# someone running launch_test on the file directly - then generate_test_description() also skips
# its `mode` override, so the launch and the assertions degrade together into a second copy of the
# positive scenario and CTest still reports 1/1 passed. A KeyError here is loud and immediate.
SCENARIO = os.environ['WATCHDOG_E2E_SCENARIO']
PORT = get_test_port()

# Fast tick cadence + short warmup so the whole story (gateway/fault_manager
# startup, demo-node discovery, per-app arm, global bringup grace - see the
# plugin README's "Reliability (bringup-quiesce)") comfortably fits inside
# the poll_faults() timeouts below without any hand-tuned pre-sleep: polling
# itself absorbs demo_delay + discovery + warmup, retrying every interval
# until the assertion's own timeout.
TICK_INTERVAL_MS = 200
WARMUP_CYCLES = 3

# The two shapes `mode` actually arrives in. A QUOTED "off" in YAML stays a
# STRING; a BARE `off` is YAML 1.1, which rcl_yaml_param_parser types as BOOL
# False (as do `Off`, `OFF`, `no`, `n`). `off` bare is what operators write, so
# both shapes have to disable the detector - see parse_detector_mode().
_MODE_OFF_STRING = 'off'
_MODE_OFF_YAML_BOOL = False


def generate_test_description():
    detector_params = {
        'plugins.graph_watchdog.tick_interval_ms': TICK_INTERVAL_MS,
        'plugins.graph_watchdog.warmup_cycles': WARMUP_CYCLES,
        'plugins.graph_watchdog.detectors.param_drift.baseline': False,
        'plugins.graph_watchdog.detectors.param_drift.expect.use_sim_time': True,
    }
    if SCENARIO == 'mode_off':
        detector_params['plugins.graph_watchdog.detectors.param_drift.mode'] = _MODE_OFF_STRING
    elif SCENARIO == 'mode_off_yaml_bool':
        detector_params['plugins.graph_watchdog.detectors.param_drift.mode'] = _MODE_OFF_YAML_BOOL

    return create_watchdog_test_launch(
        detector_params=detector_params,
        demo_nodes=['temp_sensor'],
        port=PORT,
    )


class TestConfigPlumbingPositive(unittest.TestCase):
    """Positive control: the nested `expect` config reaches param_drift."""

    def test_expect_rule_raises_from_nested_config(self):
        fault = poll_faults(PORT, 'GRAPH_PARAM_DRIFT', timeout=60.0)
        self.assertIsNotNone(
            fault,
            'GRAPH_PARAM_DRIFT never raised - the nested `expect` config was '
            'not delivered to param_drift through the real gateway',
        )


class TestConfigPlumbingModeOff(unittest.TestCase):
    """Mode suppression: the nested `mode` config disables the detector."""

    def test_mode_off_suppresses_detector(self):
        # Gate on the plugin's own route BEFORE asserting absence. Without this the
        # assertion is vacuous by construction: a stack that never came up produces
        # exactly the same "no fault" result, and every failure mode on this launch
        # (plugin fails to dlopen, REST server cannot bind) still exits 0. This runs
        # in its own process against its own gateway on its own port, so nothing in
        # the positive scenario covers it either.
        self.assertTrue(
            wait_until_watchdog_armed(PORT, timeout=60.0),
            'graph_watchdog never reported an armed gate - the plugin did not load '
            'or its tick loop never ran, so an absent fault proves nothing',
        )
        # Proving absence means waiting out the full window: a detector that
        # (regression) ignored `mode` would have raised well within it.
        fault = poll_faults(PORT, 'GRAPH_PARAM_DRIFT', timeout=20.0)
        self.assertIsNone(
            fault,
            'GRAPH_PARAM_DRIFT raised despite '
            'detectors.param_drift.mode="off" - the nested `mode` config was '
            'not delivered to (or not honored by) the plugin',
        )


class TestConfigPlumbingModeOffYamlBool(unittest.TestCase):
    """A BARE `off` disables the detector, not just a quoted one."""

    def test_yaml_boolean_off_suppresses_detector(self):
        # `mode: off` written bare in YAML never reaches the plugin as a string:
        # rcl_yaml_param_parser applies YAML 1.1 and types it BOOL False. A reader
        # that only accepts strings therefore leaves the detector in its `raise`
        # default, and the detector the operator just disabled keeps pushing
        # GRAPH_PARAM_DRIFT at the robot with nothing in the log to explain it.
        # This is the shape operators actually write, so it gets its own launch.
        self.assertTrue(
            wait_until_watchdog_armed(PORT, timeout=60.0),
            'graph_watchdog never reported an armed gate - the plugin did not load '
            'or its tick loop never ran, so an absent fault proves nothing',
        )
        fault = poll_faults(PORT, 'GRAPH_PARAM_DRIFT', timeout=20.0)
        self.assertIsNone(
            fault,
            'GRAPH_PARAM_DRIFT raised despite a bare '
            'detectors.param_drift.mode: off - the YAML boolean form of "off" '
            'was not honored by the plugin',
        )


# Each CTest target launches this file with one scenario, so only that scenario's case may
# run. Marking the other two as skipped would report them as skipped on every single run,
# which is exactly how a scenario that has genuinely stopped working would look. Removing
# them from the module instead means each run reports one case, and a missing result is a
# real failure rather than an expected line of output.
_SCENARIO_CASES = {
    'positive': 'TestConfigPlumbingPositive',
    'mode_off': 'TestConfigPlumbingModeOff',
    'mode_off_yaml_bool': 'TestConfigPlumbingModeOffYamlBool',
}
if SCENARIO not in _SCENARIO_CASES:
    raise RuntimeError(
        f'WATCHDOG_E2E_SCENARIO={SCENARIO!r} is not one of {sorted(_SCENARIO_CASES)}; '
        'the CTest target and this file disagree about which scenarios exist')
for _scenario, _case_name in _SCENARIO_CASES.items():
    if _scenario != SCENARIO:
        del globals()[_case_name]


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):
    """Verify the gateway/fault_manager/demo stack exits cleanly."""

    def test_exit_codes(self, proc_info):
        for info in proc_info:
            self.assertIn(
                info.returncode,
                ALLOWED_EXIT_CODES,
                f'Process {info.process_name} exited with {info.returncode}',
            )
