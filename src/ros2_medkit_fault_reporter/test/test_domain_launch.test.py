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
A launch test's children run on the launch test's domain.

A launch test is the case with no upstream isolated wrapper: the test process
holds the domain and everything the launch description starts has to end up on
it. Echoing an environment variable would only show that the variable was
inherited, so the child here creates a real ROS publisher and this test receives
from it on the domain it was allocated - the DDS layer answers, not os.environ.

It lives in this package rather than in ros2_medkit_cmake because that project
is declared NONE. With no language enabled CMake leaves CMAKE_LIBRARY_ARCHITECTURE
and CMAKE_SIZEOF_VOID_P empty, and launch_testing_ament_cmake reaches the
deprecated FindPythonLibs through python_cmake_module on Humble, which then
cannot find libpython however the headers are installed.
"""

import os
import sys
import unittest

import launch
import launch.actions
import launch_testing
import launch_testing.actions
import pytest
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

TOPIC = '/medkit_launch_domain_probe'

# Spinning under a bare `rclpy.spin` lets CPython re-raise SIGINT and die from
# the signal, which skips the participant's destructor. A DDS participant that
# is never destroyed leaves its shared-memory segments behind, and a CI
# container gets 64 MB of /dev/shm for the whole job, so the child shuts itself
# down instead.
CHILD = (
    'import os, signal, sys, rclpy;'
    'from rclpy.node import Node;'
    'from std_msgs.msg import String;'
    "print('CHILD_ROS_DOMAIN_ID=' + str(os.environ.get('ROS_DOMAIN_ID')), flush=True);"
    'rclpy.init();'
    "node = Node('medkit_launch_domain_child');"
    f"pub = node.create_publisher(String, '{TOPIC}', 10);"
    "msg = String(data='from-the-child');"
    'timer = node.create_timer(0.1, lambda: pub.publish(msg));'
    # launch escalates to SIGTERM when SIGINT is not answered in time, and
    # Python's default action for that one is to die on the spot. Routing it
    # through the same handler SIGINT uses keeps both paths ending in the
    # shutdown below.
    'signal.signal(signal.SIGTERM, signal.default_int_handler)\n'
    'try:\n'
    '    rclpy.spin(node)\n'
    'except KeyboardInterrupt:\n'
    '    pass\n'
    'finally:\n'
    '    node.destroy_node()\n'
    '    rclpy.try_shutdown()\n'
)


@pytest.mark.launch_test
def generate_test_description():
    child = launch.actions.ExecuteProcess(
        cmd=[sys.executable, '-u', '-c', CHILD],
        name='domain_probe_child',
        output='screen',
    )
    return (
        launch.LaunchDescription([child, launch_testing.actions.ReadyToTest()]),
        {'child': child},
    )


class TestLaunchTestDomain(unittest.TestCase):

    def test_the_launch_test_itself_got_a_domain(self):
        domain = os.environ.get('ROS_DOMAIN_ID')
        self.assertIsNotNone(domain, 'the launch test ran without an allocated domain')
        self.assertNotEqual(domain, '0', 'domain 0 is shared with everything on the machine')
        band = tuple(range(1, 101)) + tuple(range(215, 232))
        self.assertIn(int(domain), band)

    def test_the_child_process_reports_the_same_domain(self, proc_output, child):
        # stream='stdout': waitFor listens on stderr by default, and the child
        # prints this on stdout.
        proc_output.assertWaitFor(
            f"CHILD_ROS_DOMAIN_ID={os.environ['ROS_DOMAIN_ID']}",
            process=child,
            timeout=60,
            stream='stdout',
        )

    def test_the_child_node_is_reachable_on_that_domain(self):
        rclpy.init()
        try:
            node = Node('medkit_launch_domain_listener')
            received = []
            node.create_subscription(String, TOPIC, received.append, 10)
            end = node.get_clock().now().nanoseconds + 60 * 1_000_000_000
            while not received and node.get_clock().now().nanoseconds < end:
                rclpy.spin_once(node, timeout_sec=0.1)
            node.destroy_node()
        finally:
            rclpy.shutdown()
        self.assertTrue(
            received,
            "nothing arrived from the child, so it did not join this test's DDS domain",
        )


@launch_testing.post_shutdown_test()
class TestChildShutdown(unittest.TestCase):

    def test_the_child_was_stopped(self, proc_info, child):
        # Zero, not merely "not a crash": the child handles the shutdown signal
        # and destroys its participant. Dying from the signal instead would
        # strand this participant's shared-memory segments for the rest of the
        # job, which is what this assertion exists to catch.
        self.assertEqual(
            proc_info[child].returncode,
            0,
            'the child did not shut down cleanly, so its DDS participant leaked',
        )
