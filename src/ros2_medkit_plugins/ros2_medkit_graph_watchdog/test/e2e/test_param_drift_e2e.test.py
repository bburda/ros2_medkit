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
"""param_drift e2e for the mode operators actually get: self-capturing BASELINE drift.

The three config-plumbing scenarios all set ``baseline: false`` and assert only that a fault
appears or does not, so until this file existed the detector's DEFAULT and headline behaviour -
capture a parameter's value, notice when it changes at runtime - had no end-to-end coverage, and
no scenario asserted a clear at all. The whole "Closing the loop" section of the README rests on
the clear reaching the fault_manager and healing there.

What this drives, all real: a gateway process with the plugin ``.so`` loaded, a real
fault_manager, a real ROS node whose parameter this test changes over the real parameter service,
and the operator-visible ``GET /api/v1/faults`` surface.

Unlike test_orphan_e2e.test.py, the node here MUST be spun: the detector reads its parameters over
a service, and an unspun node answers nothing, so the detector would report it as never read
instead of capturing a baseline.

Everything below is anchored to the plugin's own GET /x-medkit-watchdog route
(harness.wait_until_watchdog_armed) rather than to wall-clock time from process start. This file
both asserts an ABSENCE and then perturbs a parameter, and each needs the gate for its own reason:
an absence proves nothing about a stack that never came up, and the detector may not read a node
before the gate arms it, so a perturbation that lands first turns the DRIFTED value into the
baseline and no fault can ever raise.
"""

import os
import sys
import threading
import unittest

import launch_testing
import rclpy
from rclpy.node import Node

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
# I100 as well as E402: `harness` is only importable because of the sys.path line above, so this
# import cannot be moved up to where the alphabetical order would put it.
from harness import (  # noqa: E402, I100
    assert_fault_absent_throughout,
    create_watchdog_test_launch,
    poll_cleared,
    poll_faults,
    wait_until_watchdog_armed,
)

from ros2_medkit_test_utils.constants import ALLOWED_EXIT_CODES, get_test_port  # noqa: E402

PORT = get_test_port()

# Fast tick + short warmup so the whole story fits inside the poll timeouts without a hand-tuned
# pre-sleep - same rationale as the other e2e files here. max_reads_per_tick is raised because the
# real graph carries the gateway, the fault_manager and this test's node, and the sweep is
# round-robin: at the default budget the node under test waits several ticks for its turn.
TICK_INTERVAL_MS = 200
WARMUP_CYCLES = 3
FAULT_CODE = 'GRAPH_PARAM_DRIFT'
# The ROS node name, which is also the App id the gateway derives for a node in the root
# namespace - so it is both what this test spins up and what the watchdog gate reports as an
# entity. Used for the arming gate and for the "the fault names the node" assertion, which must
# not be able to drift apart.
TARGET_NODE = 'pd_e2e_target'
PARAM_NAME = 'speed_limit'
BASELINE_VALUE = 1.0
DRIFTED_VALUE = 0.2

# How long the detector gets to capture a baseline, measured from the moment the gate armed the
# target (NOT from process start - bringup is absorbed by the gate, which is the point of it).
# Sized to exceed the parameter transport's 10 s negative-cache TTL, so a first read that timed
# out and backed the node off is retried and still lands inside the window. The C++ integration
# suite sizes its own poll bound off the same constant for the same reason.
SILENT_CAPTURE_SEC = 15.0


def generate_test_description():
    detector_params = {
        'plugins.graph_watchdog.tick_interval_ms': TICK_INTERVAL_MS,
        'plugins.graph_watchdog.warmup_cycles': WARMUP_CYCLES,
        'plugins.graph_watchdog.detectors.param_drift.max_reads_per_tick': 32,
    }
    return create_watchdog_test_launch(
        detector_params=detector_params,
        demo_nodes=None,
        port=PORT,
        # The clear must reach HEALED, i.e. leave the default active-fault query, which the
        # fault_manager only does when healing is enabled - see harness.py and the README's
        # "Closing the loop".
        healing_enabled=True,
    )


class TestParamDriftE2e(unittest.TestCase):
    """Baseline drift raises through the real stack, and restoring the value heals it."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls._node = Node(TARGET_NODE)
        cls._node.declare_parameter(PARAM_NAME, BASELINE_VALUE)
        cls._stop = threading.Event()
        cls._spin = threading.Thread(target=cls._spin_until_stopped, daemon=True)
        cls._spin.start()

    @classmethod
    def _spin_until_stopped(cls):
        # The node has to answer parameter-service calls for the detector to read it at all.
        while not cls._stop.is_set() and rclpy.ok():
            rclpy.spin_once(cls._node, timeout_sec=0.1)

    @classmethod
    def tearDownClass(cls):
        cls._stop.set()
        cls._spin.join(timeout=5.0)
        cls._node.destroy_node()
        rclpy.shutdown()

    def test_01_runtime_change_raises_and_names_the_parameter(self):
        # Gate on the target being ARMED before anything below runs. Two distinct things go wrong
        # without it, and neither is visible in the result:
        #
        # - The absence assertion is vacuous against a stack that never came up. A plugin that
        #   fails to dlopen, a REST server that cannot bind - the gateway survives both and exits
        #   0, and poll_faults swallows every transport error and returns None, which is exactly
        #   what assertIsNone wants to see.
        # - Worse, the flip below is only a drift if a baseline was captured first, and the
        #   detector may not target a node until the gate arms it. On a slow runner a fixed
        #   pre-window elapses during bringup, set_parameters lands before the first read, the
        #   DRIFTED value silently becomes the baseline, and no fault can ever raise - the run
        #   then dies at the 60 s poll below blaming the detector for a race in this test.
        #
        # Anchoring both to the gate makes bringup cost time here, where the failure message
        # names the real cause, instead of eating the window that is supposed to cover the read.
        self.assertTrue(
            wait_until_watchdog_armed(PORT, timeout=60.0, app_id=TARGET_NODE),
            f'graph_watchdog never reported {TARGET_NODE} as an armed entity - the plugin did not '
            f'load, its tick loop never ran, or the gateway never discovered the node. The '
            f'detector reads only apps the gate arms, so nothing below would mean anything',
        )

        # Give the detector time to capture the baseline BEFORE changing anything. Capturing
        # must be silent, so a fault appearing here at all would mean the detector reports
        # its own first read as drift. assert_fault_absent_throughout, not
        # assertIsNone(poll_faults(...)): the latter swallows every non-200 response and
        # transport error into the same None a genuine absence produces, so a /faults that
        # died partway through this window would still pass - assert_fault_absent_throughout
        # instead fails naming which poll could not even ask.
        assert_fault_absent_throughout(self, PORT, FAULT_CODE, SILENT_CAPTURE_SEC)

        self._node.set_parameters(
            [rclpy.parameter.Parameter(PARAM_NAME, rclpy.Parameter.Type.DOUBLE, DRIFTED_VALUE)])

        fault = poll_faults(PORT, FAULT_CODE, timeout=60.0)
        self.assertIsNotNone(
            fault,
            f'{FAULT_CODE} never raised after {PARAM_NAME} changed from {BASELINE_VALUE} to '
            f'{DRIFTED_VALUE} at runtime',
        )
        description = fault.get('description', '')
        self.assertIn(PARAM_NAME, description)
        self.assertIn(TARGET_NODE, description)

    def test_02_restoring_the_value_heals_the_fault(self):
        # "Cleared" is an absence too, and it is the default state of a fault that was never
        # raised: on the stack test_01 just failed against, poll_cleared answers True on its
        # first request. Confirm the fault is actually THERE first, so what this measures is a
        # heal and not a fault that never happened. test_01 raised it and the drifted value is
        # still set, so it is present now or the level-triggered raise is not level-triggered.
        self.assertIsNotNone(
            poll_faults(PORT, FAULT_CODE, timeout=30.0),
            f'{FAULT_CODE} is not active at the start of the heal test, so there is nothing here '
            f'that could heal and the clear below would pass without measuring anything',
        )

        # This is the path the README tells operators to rely on: put the value back and the fault
        # goes away on its own. It only works if the detector keeps emitting a level-triggered
        # PASSED and the fault_manager counts those toward healing.
        self._node.set_parameters(
            [rclpy.parameter.Parameter(PARAM_NAME, rclpy.Parameter.Type.DOUBLE, BASELINE_VALUE)])

        self.assertTrue(
            poll_cleared(PORT, FAULT_CODE, timeout=60.0),
            f'{FAULT_CODE} did not heal after {PARAM_NAME} was restored to {BASELINE_VALUE}',
        )


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):
    """Verify the gateway/fault_manager stack exits cleanly."""

    def test_exit_codes(self, proc_info):
        for info in proc_info:
            self.assertIn(
                info.returncode,
                ALLOWED_EXIT_CODES,
                f'Process {info.process_name} exited with {info.returncode}',
            )
