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

The system-package prefix the configure below uses is SYSTEM_INSTALL_PREFIX,
a literal - not this environment's own ROS_DISTRO. The guard's install-prefix
check (see ROS2MedkitTestDomain.cmake) is a wildcard, `/opt/ros/<anything>`,
not a comparison against the environment, so any distro name proves the same
thing. An earlier version of the guard compared against `$ENV{ROS_DISTRO}`
instead and never fired in the ROS build farm's binarydeb chroot, which has
no ROS_DISTRO in its environment at all - see
ros2_medkit_cmake/test/test_launch_test_guard.py's
test_launch_test_is_not_registered_with_ros_distro_absent for the probe that
catches a regression back to that.
"""

import pathlib
import sys

import pytest
from ros2_medkit_test_utils.cmake_guard import (
    assert_guard_suppressed_launch_tests,
    configure,
    registered_test_names,
)

PACKAGE_ROOT = pathlib.Path(__file__).resolve().parent.parent

# A literal, not this environment's own ROS_DISTRO - see the module docstring
# above for why. Any name under /opt/ros/ proves the same thing against the
# guard's wildcard prefix check, and humble is the one distro the guard exists
# for.
SYSTEM_INSTALL_PREFIX = '/opt/ros/humble'


def test_guard_does_not_break_a_real_configure(tmp_path):
    """
    A system-package configure of this package still succeeds, and suppresses the right tests.

    Configures this package's own source directory twice: once as an
    ordinary workspace-style build, once under system-package conditions
    (CMAKE_INSTALL_PREFIX=SYSTEM_INSTALL_PREFIX, CMAKE_INSTALL_LOCALSTATEDIR=
    /var - what bloom's debian/rules passes, with a literal distro name in
    place of the real one), and compares what each run actually registered.
    See ros2_medkit_test_utils.cmake_guard for what "suppresses the right
    tests" asserts.
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
        PACKAGE_ROOT, guarded_build, SYSTEM_INSTALL_PREFIX, localstatedir='/var'
    )
    assert_guard_suppressed_launch_tests(
        'ros2_medkit_graph_watchdog', baseline_tests, guarded, guarded_build
    )


if __name__ == '__main__':
    sys.exit(pytest.main([__file__, '-v', '-s']))
