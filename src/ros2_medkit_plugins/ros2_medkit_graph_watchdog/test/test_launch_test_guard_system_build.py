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
Proves the launch-test guard against a real configure of this package.

The six-package sibling of this test,
`ros2_medkit_integration_tests/test/test_launch_test_guard_system_build.py`,
cannot cover ros2_medkit_graph_watchdog: this package already test_depends on
ros2_medkit_integration_tests for its own e2e harness, and colcon confirms
the reverse edge is a real build-order cycle, not a theoretical one - see the
comment in ros2_medkit_integration_tests/package.xml. So this package proves
the same property about itself instead, using the same configure/compare
helpers (`ros2_medkit_test_utils.cmake_guard`) the sibling test uses - it
already reaches that module through the same test_depend that creates the
cycle in the other direction.

Unlike the sibling test, this one needs no repository root: it configures
only its own source directory, which is always right where this test file
is, in the ROS build farm's isolated binarydeb chroot exactly as much as in
a full workspace checkout. It registers unconditionally.
"""

import os
import pathlib
import sys

import pytest
from ros2_medkit_test_utils.cmake_guard import (
    assert_guard_suppressed_launch_tests,
    configure,
    registered_test_names,
)

PACKAGE_ROOT = pathlib.Path(__file__).resolve().parent.parent


@pytest.fixture(scope='session')
def ros_distro():
    """
    Return ROS_DISTRO.

    Unconditional, not a skip: colcon test always sources a ROS distribution
    before running a test, so ROS_DISTRO is guaranteed to be set wherever
    this test runs at all. Its absence is a broken environment, not a reason
    to decline the check the environment promised it could run.
    """
    distro = os.environ.get('ROS_DISTRO')
    assert distro, 'ROS_DISTRO is unset; colcon test always sources a ROS distribution first'
    return distro


def test_guard_does_not_break_a_real_configure(ros_distro, tmp_path):
    """
    A system-package configure of this package still succeeds, and suppresses the right tests.

    Configures this package's own source directory twice: once as an
    ordinary workspace-style build, once under system-package conditions
    (CMAKE_INSTALL_PREFIX=/opt/ros/$ROS_DISTRO, CMAKE_INSTALL_LOCALSTATEDIR=
    /var - what bloom's debian/rules passes), and compares what each run
    actually registered. See ros2_medkit_test_utils.cmake_guard for what
    "suppresses the right tests" asserts.
    """
    baseline_build = tmp_path / 'baseline_build'
    baseline_install = tmp_path / 'baseline_install'
    baseline = configure(PACKAGE_ROOT, baseline_build, baseline_install)
    assert baseline.returncode == 0, (
        f'ros2_medkit_graph_watchdog: baseline configure failed, so the system-package '
        f'configure below cannot be compared against anything:\n{baseline.stderr}'
    )
    baseline_tests = registered_test_names(baseline_build)

    guarded_build = tmp_path / 'guarded_build'
    guarded = configure(
        PACKAGE_ROOT, guarded_build, f'/opt/ros/{ros_distro}', localstatedir='/var'
    )
    assert_guard_suppressed_launch_tests(
        'ros2_medkit_graph_watchdog', baseline_tests, guarded, guarded_build
    )


if __name__ == '__main__':
    sys.exit(pytest.main([__file__, '-v', '-s']))
