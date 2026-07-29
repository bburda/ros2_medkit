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
"""QoS-mismatch e2e: proves GRAPH_QOS_MISMATCH raises and clears through the REAL
gateway + qos_mismatch_detector + fault_manager stack. This is the acceptance gate
for its design issue.

Unlike test_qos_mismatch_integration.cpp (which drives the detector directly against
a fake ReportFault service and a bare rclcpp::Node), this launches the REAL gateway
process with the graph_watchdog plugin (.so) loaded and a real fault_manager, and
polls the operator-visible ``GET /api/v1/faults`` surface - the only way to prove the
detector's raise/clear actually reaches a SOVD fault through the real tick-timer and
config-delivery path (see harness.py's module docstring for why a C++ test alone
cannot give this proof).

Two real rclpy nodes are created directly in this test process (mirrors the pattern
in ros2_medkit_integration_tests' test_external_component_fault_rollup.test.py, which
creates a plain rclpy Node in-process rather than launching a separate executable):

- a publisher on ``/probe`` with BEST_EFFORT reliability (never changes)
- a subscriber on ``/probe``, first RELIABLE (raises the mismatch), then replaced
  with a BEST_EFFORT one (clears it)

QoS endpoint discovery (participant/publication/subscription builtin-topic data) is
populated by DDS's own background discovery, independent of executor spinning - the
same rationale as test_qos_mismatch_integration.cpp - so these nodes are never spun;
they only need to stay alive so their advertised QoS remains visible to the gateway.

test_04/test_05 extend the same launch with two more RxO-incompatibility dimensions
(deadline, liveliness lease duration - see qos_policy.hpp's qos_incompatibility()):
each uses its OWN topic and its OWN pub/sub node pair, created and destroyed within the
test method itself, so the shared GRAPH_QOS_MISMATCH aggregate (one fault_code covering
every currently-mismatched topic, same rationale as node_death's aggregate) returns to
empty before the next scenario - proving each dimension raises and is named on its own,
not merely riding along on the /probe reliability mismatch above.
"""

import os
import sys
import unittest

import launch_testing
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from harness import (  # noqa: E402
    create_watchdog_test_launch,
    poll_cleared,
    poll_entity_faults,
    poll_faults,
)

from ros2_medkit_test_utils.constants import ALLOWED_EXIT_CODES, get_test_port  # noqa: E402

PORT = get_test_port()

# Fast tick cadence + short warmup so the whole story (gateway/fault_manager startup,
# real DDS endpoint discovery, per-app arm, global bringup grace) comfortably fits
# inside the poll_faults()/poll_cleared() timeouts below without a hand-tuned
# pre-sleep - see the identical rationale in test_config_plumbing_e2e.test.py.
TICK_INTERVAL_MS = 200
WARMUP_CYCLES = 3

TOPIC = '/probe'
TOPIC_DEADLINE = '/probe_deadline'
TOPIC_LEASE = '/probe_lease'
FAULT_CODE = 'GRAPH_QOS_MISMATCH'


def generate_test_description():
    detector_params = {
        'plugins.graph_watchdog.tick_interval_ms': TICK_INTERVAL_MS,
        'plugins.graph_watchdog.warmup_cycles': WARMUP_CYCLES,
    }
    return create_watchdog_test_launch(
        detector_params=detector_params,
        demo_nodes=None,
        port=PORT,
        # Clearing GRAPH_QOS_MISMATCH after the QoS fix must reach HEALED (absent
        # from the default active-fault query) - see harness.py's healing_enabled doc.
        healing_enabled=True,
    )


class TestQosMismatchE2e(unittest.TestCase):
    """GRAPH_QOS_MISMATCH raises on a real RxO-incompatible pub/sub pair, clears on fix."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls._pub_node = Node('qos_e2e_pub')
        pub_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        cls._pub = cls._pub_node.create_publisher(String, TOPIC, pub_qos)

        cls._sub_node = Node('qos_e2e_sub')
        reliable_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        cls._sub = cls._sub_node.create_subscription(String, TOPIC, lambda _msg: None, reliable_qos)

    @classmethod
    def tearDownClass(cls):
        cls._sub_node.destroy_node()
        cls._pub_node.destroy_node()
        rclpy.shutdown()

    def test_01_mismatch_raises_naming_topic(self):
        fault = poll_faults(PORT, FAULT_CODE, timeout=60.0)
        self.assertIsNotNone(
            fault,
            f'{FAULT_CODE} never raised for a BEST_EFFORT publisher / RELIABLE '
            f'subscriber pair on {TOPIC}',
        )
        self.assertIn(TOPIC, fault.get('description', ''))

    def test_02_fault_is_reachable_from_its_entity(self):
        # The acceptance criterion for scoping. The flat /faults list carries a fault
        # whatever its source_id is, so it cannot show whether an operator can actually
        # OPEN the fault on an entity. Scoped to the host Component - the obvious
        # choice, and what this used to do - the answer is no: the gateway only puts a
        # Component's own id in its fault scope when `external` is set, and a runtime
        # host Component never sets it, so the fault existed in exactly one place, the
        # global list. Only a real gateway can prove the fix; a C++ test on
        # introspect() proves the plugin RETURNS an entity, not that the fault is
        # reachable through it.
        fault = poll_entity_faults(PORT, 'apps/graph_watchdog', FAULT_CODE, timeout=60.0)
        self.assertIsNotNone(
            fault,
            f'{FAULT_CODE} is not reachable at /apps/graph_watchdog/faults - the '
            'entity the plugin publishes does not own the fault it raises',
        )

    def test_03_matching_qos_clears(self):
        # Replace the RELIABLE subscriber with a BEST_EFFORT one on a fresh node
        # (distinct node name - avoids racing the old node's DDS teardown, same
        # rationale as test_qos_mismatch_integration.cpp).
        type(self)._sub_node.destroy_node()
        best_effort_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        type(self)._sub_node = Node('qos_e2e_sub2')
        type(self)._sub = type(self)._sub_node.create_subscription(
            String, TOPIC, lambda _msg: None, best_effort_qos)

        cleared = poll_cleared(PORT, FAULT_CODE, timeout=60.0)
        self.assertTrue(
            cleared,
            f'{FAULT_CODE} did not clear after the subscriber QoS was fixed to '
            'BEST_EFFORT',
        )

    def test_04_deadline_mismatch_raises_naming_deadline(self):
        # Publisher: no deadline set (defaults to RMW_DURATION_UNSPECIFIED - the most
        # permissive offer). Subscriber: a finite requested deadline. RxO-incompatible -
        # see qos_policy.hpp's qos_incompatibility() deadline branch.
        pub_node = Node('qos_e2e_deadline_pub')
        pub_qos = QoSProfile(depth=10)
        pub = pub_node.create_publisher(String, TOPIC_DEADLINE, pub_qos)

        sub_node = Node('qos_e2e_deadline_sub')
        sub_qos = QoSProfile(depth=10, deadline=Duration(seconds=0.1))
        sub = sub_node.create_subscription(String, TOPIC_DEADLINE, lambda _msg: None, sub_qos)

        try:
            fault = poll_faults(PORT, FAULT_CODE, timeout=60.0)
            self.assertIsNotNone(
                fault,
                f'{FAULT_CODE} never raised for a no-deadline publisher / 0.1s-deadline '
                f'subscriber pair on {TOPIC_DEADLINE}',
            )
            self.assertIn('deadline', fault.get('description', ''))
        finally:
            del sub
            del pub
            sub_node.destroy_node()
            pub_node.destroy_node()

        # Both endpoints gone -> the topic itself drops out of the aggregate, returning
        # GRAPH_QOS_MISMATCH to empty before the next scenario (see module docstring).
        cleared = poll_cleared(PORT, FAULT_CODE, timeout=60.0)
        self.assertTrue(
            cleared,
            f'{FAULT_CODE} did not clear after both {TOPIC_DEADLINE} endpoints were destroyed',
        )

    def test_05_liveliness_lease_mismatch_raises_naming_lease(self):
        # Publisher: no liveliness lease duration set (defaults to unspecified).
        # Subscriber: a finite requested lease duration. RxO-incompatible - see
        # qos_policy.hpp's qos_incompatibility() liveliness-lease branch.
        pub_node = Node('qos_e2e_lease_pub')
        pub_qos = QoSProfile(depth=10)
        pub = pub_node.create_publisher(String, TOPIC_LEASE, pub_qos)

        sub_node = Node('qos_e2e_lease_sub')
        sub_qos = QoSProfile(depth=10, liveliness_lease_duration=Duration(seconds=0.1))
        sub = sub_node.create_subscription(String, TOPIC_LEASE, lambda _msg: None, sub_qos)

        try:
            fault = poll_faults(PORT, FAULT_CODE, timeout=60.0)
            self.assertIsNotNone(
                fault,
                f'{FAULT_CODE} never raised for a no-lease publisher / 0.1s-lease '
                f'subscriber pair on {TOPIC_LEASE}',
            )
            self.assertIn('lease', fault.get('description', ''))
        finally:
            del sub
            del pub
            sub_node.destroy_node()
            pub_node.destroy_node()

        cleared = poll_cleared(PORT, FAULT_CODE, timeout=60.0)
        self.assertTrue(
            cleared,
            f'{FAULT_CODE} did not clear after both {TOPIC_LEASE} endpoints were destroyed',
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
