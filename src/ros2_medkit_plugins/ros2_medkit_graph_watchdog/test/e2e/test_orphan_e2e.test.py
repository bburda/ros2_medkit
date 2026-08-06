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
"""Orphan / topic-name-mismatch e2e: proves GRAPH_ORPHAN raises and clears through the REAL stack.

The real stack is the gateway process, the orphan_detector, and the fault_manager. This is the
acceptance gate for its design issue.

Unlike test_orphan_integration.cpp (which drives the detector directly against a fake
ReportFault service and a bare rclcpp::Node), this launches the REAL gateway process with
the graph_watchdog plugin (.so) loaded and a real fault_manager, and polls the
operator-visible ``GET /api/v1/faults`` surface - the only way to prove the detector's
raise/clear actually reaches a SOVD fault through the real tick timer and config-delivery
path (see harness.py's module docstring for why a C++ test alone cannot give this proof).

Two real rclpy nodes are created directly in this test process (mirrors the pattern in
test_qos_e2e.test.py and ros2_medkit_integration_tests'
test_external_component_fault_rollup.test.py):

- a publisher on ``/scam`` (LaserScan) - the typo, pub-only, never changes until removed.
- a publisher on ``/robott/odom`` and a subscriber on ``/robot/odom`` - the same mistake made
  in the namespace instead of the leaf, reported only because this launch opts into
  ``namespace_edit_distance``. Both typo publishers sit on the same node, so destroying it
  leaves nothing orphaned and the clear story below still holds.
- a subscriber on ``/scan`` (LaserScan) - the intended target, sub-only. It never matches
  the typo publisher (different topic name), so no data ever flows between the two nodes
  even though both are alive - the silent failure signature this detector exists to catch.

Endpoint discovery (participant/publication/subscription builtin-topic data) is populated
by DDS's own background discovery, independent of executor spinning - the same rationale
as test_orphan_integration.cpp - so these nodes are never spun; they only need to stay
alive so their advertised pub/sub counts remain visible to the gateway.
"""

import os
import sys
import unittest

import launch_testing
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import LaserScan

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
# I100 as well as E402: `harness` is only importable because of the sys.path line above, so this
# import cannot be moved up to where the alphabetical order would put it.
from harness import create_watchdog_test_launch, poll_cleared, poll_faults  # noqa: E402, I100

from ros2_medkit_test_utils.constants import ALLOWED_EXIT_CODES, get_test_port  # noqa: E402

PORT = get_test_port()

# Fast tick cadence + short warmup so the whole story (gateway/fault_manager startup, real
# DDS endpoint discovery, per-app arm, global bringup grace) comfortably fits inside the
# poll_faults()/poll_cleared() timeouts below without a hand-tuned pre-sleep - see the
# identical rationale in test_qos_e2e.test.py.
TICK_INTERVAL_MS = 200
WARMUP_CYCLES = 3

TYPO_TOPIC = '/scam'
TARGET_TOPIC = '/scan'
FAULT_CODE = 'GRAPH_ORPHAN'

# A second near-miss pair, identical in shape to the one above (same parent namespace, edit
# distance 1, same type, one-sided on each side) and only distinguished by being allowlisted.
# It is the negative control for the allowlist: the detector must report the first pair and
# stay silent about this one, which is also the only proof that a string ARRAY survives the
# YAML -> ROS parameter -> nested-config delivery path into configure(). No C++ test can give
# that proof - both the unit and the integration test hand configure() a JSON object directly.
ALLOWLISTED_TYPO_TOPIC = '/hokuyo_scam'
ALLOWLISTED_TARGET_TOPIC = '/hokuyo_scan'

# A misspelled NAMESPACE with an identical leaf. At the default namespace_edit_distance of 0
# this pair is invisible, so seeing it reported is what proves an integer key also survives
# the parameter path - and it is the mistake the exact-namespace guard otherwise hides.
NS_TYPO_TOPIC = '/robott/odom'
NS_TARGET_TOPIC = '/robot/odom'
NAMESPACE_EDIT_DISTANCE = 1


def generate_test_description():
    detector_params = {
        'plugins.graph_watchdog.tick_interval_ms': TICK_INTERVAL_MS,
        'plugins.graph_watchdog.warmup_cycles': WARMUP_CYCLES,
        'plugins.graph_watchdog.detectors.orphan.allowlist': [ALLOWLISTED_TYPO_TOPIC],
        'plugins.graph_watchdog.detectors.orphan.namespace_edit_distance': NAMESPACE_EDIT_DISTANCE,
    }
    return create_watchdog_test_launch(
        detector_params=detector_params,
        demo_nodes=None,
        port=PORT,
        # Clearing GRAPH_ORPHAN after the typo publisher is removed must reach HEALED
        # (absent from the default active-fault query) - see harness.py's healing_enabled
        # doc.
        healing_enabled=True,
    )


class TestOrphanE2e(unittest.TestCase):
    """GRAPH_ORPHAN raises on a real pub-only/sub-only topic-name near-miss pair.

    Clears once the typo publisher is destroyed.
    """

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls._pub_node = Node('orphan_e2e_typo_pub')
        cls._pub = cls._pub_node.create_publisher(LaserScan, TYPO_TOPIC, QoSProfile(depth=10))
        cls._ns_pub = cls._pub_node.create_publisher(
            LaserScan, NS_TYPO_TOPIC, QoSProfile(depth=10))

        cls._sub_node = Node('orphan_e2e_target_sub')
        cls._sub = cls._sub_node.create_subscription(
            LaserScan, TARGET_TOPIC, lambda _msg: None, QoSProfile(depth=10))
        cls._ns_sub = cls._sub_node.create_subscription(
            LaserScan, NS_TARGET_TOPIC, lambda _msg: None, QoSProfile(depth=10))

        cls._allowed_node = Node('orphan_e2e_allowlisted')
        cls._allowed_pub = cls._allowed_node.create_publisher(
            LaserScan, ALLOWLISTED_TYPO_TOPIC, QoSProfile(depth=10))
        cls._allowed_sub = cls._allowed_node.create_subscription(
            LaserScan, ALLOWLISTED_TARGET_TOPIC, lambda _msg: None, QoSProfile(depth=10))

    @classmethod
    def tearDownClass(cls):
        cls._allowed_node.destroy_node()
        cls._sub_node.destroy_node()
        if cls._pub_node is not None:
            cls._pub_node.destroy_node()
        rclpy.shutdown()

    def test_01_typo_pair_raises_naming_both(self):
        fault = poll_faults(PORT, FAULT_CODE, timeout=60.0)
        self.assertIsNotNone(
            fault,
            f'{FAULT_CODE} never raised for a pub-only {TYPO_TOPIC} / sub-only '
            f'{TARGET_TOPIC} near-miss pair',
        )
        description = fault.get('description', '')
        self.assertIn(TYPO_TOPIC, description)
        self.assertIn(TARGET_TOPIC, description)

    def test_01a_namespace_typo_pair_is_reported_too(self):
        fault = poll_faults(PORT, FAULT_CODE, timeout=60.0)
        self.assertIsNotNone(fault, f'{FAULT_CODE} is no longer raised')
        description = fault.get('description', '')
        self.assertIn(
            NS_TYPO_TOPIC,
            description,
            f'{NS_TYPO_TOPIC} / {NS_TARGET_TOPIC} differ only in the namespace, so they are '
            f'reported only if namespace_edit_distance={NAMESPACE_EDIT_DISTANCE} reached the '
            f'detector: {description}',
        )

    def test_01b_allowlisted_pair_is_absent_from_the_same_fault(self):
        # The aggregated fault enumerates every orphaned pair, so an allowlist that never
        # reached the detector would show up here as a second pair in the same description.
        fault = poll_faults(PORT, FAULT_CODE, timeout=60.0)
        self.assertIsNotNone(fault, f'{FAULT_CODE} is no longer raised')
        description = fault.get('description', '')
        self.assertNotIn(
            ALLOWLISTED_TYPO_TOPIC,
            description,
            f'{ALLOWLISTED_TYPO_TOPIC} is allowlisted in the gateway parameters but was '
            f'still reported: {description}',
        )

    def test_02_removing_typo_publisher_clears(self):
        # Clear: DESTROY the typo publisher (not "add a publisher on /scan" - that would
        # leave /scam permanently orphaned) - same rationale as
        # test_orphan_integration.cpp.
        type(self)._pub_node.destroy_node()
        type(self)._pub_node = None
        type(self)._pub = None

        cleared = poll_cleared(PORT, FAULT_CODE, timeout=60.0)
        self.assertTrue(
            cleared,
            f'{FAULT_CODE} did not clear after the typo publishers {TYPO_TOPIC} and '
            f'{NS_TYPO_TOPIC} were removed',
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
