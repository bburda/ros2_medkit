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
Proves the launch-test guard against a real configure, not a string sweep.

A regex sweep over CMakeLists.txt text was tried first, and failed three
times running: it read one physical line and only the first captured name,
so a second name on the same `set_tests_properties(...)` call, a name on a
continuation line, and (found in the round after that one was fixed) a
paren balanced across a quoted string or comment, and a mixed-case command
spelling (CMake command names are case-insensitive; a regex with a literal
lowercase alternative is not) all slipped past it. Every fix bought one more
evasion. The property this task promises - "the guard does not turn a
launch-test package's configure into a hard error" - is exactly what CMake
itself evaluates, so this asks CMake instead of a proxy for it: for each
package that registers a launch test, configure it for real, once as an
ordinary workspace build and once under system-package conditions
(`CMAKE_INSTALL_PREFIX=/opt/ros/$ROS_DISTRO`, `CMAKE_INSTALL_LOCALSTATEDIR=/var`
- exactly what bloom's `debian/rules` passes), and compare what each run
actually registered. No regex, on either side, decides what counts as a
launch test. The configure/compare mechanics live in
`ros2_medkit_test_utils.cmake_guard`, shared with the one package this file
cannot cover - see below.

This lives in ros2_medkit_integration_tests rather than ros2_medkit_cmake on
purpose. ros2_medkit_cmake cannot depend on the packages that consume it -
that dependency would be circular - so a test that configures
ros2_medkit_fault_reporter, ros2_medkit_diagnostic_bridge and the rest for
real needs to run somewhere whose own dependency chain already reaches them.
All six packages this file covers are declared test_depends of this package
(two - ros2_medkit_gateway, ros2_medkit_fault_manager - for the demo nodes
this package already builds; four more - ros2_medkit_action_status_bridge,
ros2_medkit_diagnostic_bridge, ros2_medkit_fault_reporter,
ros2_medkit_log_bridge - declared for this test specifically, since a
package used at test time must be a test_depend even when nothing here
find_package()s it at build time).

A seventh package registers launch tests too: ros2_medkit_graph_watchdog. It
cannot be covered from here. It already test_depends on this package for its
own e2e harness, and colcon confirms the reverse edge is a real build-order
cycle, not a theoretical one - see the comment in this package's package.xml.
Its own guarded-configure coverage lives in its own package instead, at
src/ros2_medkit_plugins/ros2_medkit_graph_watchdog/test/test_launch_test_guard_system_build.py,
built on the same ros2_medkit_test_utils.cmake_guard helpers this file uses,
where its own dependencies are already declared and guaranteed rather than
ridden on someone else's. The invariant - for every package that registers
launch tests, a guarded configure succeeds and suppresses exactly its launch
tests - is therefore split across two packages, not weakened for either one.

Confirmed empirically, not assumed: see the `resolvable_packages` fixture
below, which baseline-configures every candidate once, and
`test_all_launch_test_packages_resolve`, which fails if that ever regresses
below all six - every one of them is a package this file's own package.xml
declares a dependency on, so anything less is a real break, not a
degradation to shrug off.
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

# The six packages ros2_medkit_integration_tests actually declares as
# test_depend (directly, or - for ros2_medkit_gateway and
# ros2_medkit_fault_manager - for the demo nodes this package already
# builds), and their source directory relative to the repository root.
# Listed explicitly rather than discovered by scanning CMakeLists.txt for the
# macro call: that scan is exactly the kind of text-based proxy this test
# replaces, and a package that forgot to update this list would rather be
# caught by a reviewer than by this file quietly not exercising it.
#
# ros2_medkit_graph_watchdog also calls medkit_add_launch_test but is
# deliberately not listed here - see the module docstring for the real
# build-order cycle that rules out declaring it, and for where its own
# coverage lives instead.
LAUNCH_TEST_PACKAGES = {
    'ros2_medkit_action_status_bridge': 'src/ros2_medkit_action_status_bridge',
    'ros2_medkit_diagnostic_bridge': 'src/ros2_medkit_diagnostic_bridge',
    'ros2_medkit_fault_manager': 'src/ros2_medkit_fault_manager',
    'ros2_medkit_fault_reporter': 'src/ros2_medkit_fault_reporter',
    'ros2_medkit_log_bridge': 'src/ros2_medkit_log_bridge',
    'ros2_medkit_integration_tests': 'src/ros2_medkit_integration_tests',
}

# All six configure under colcon test's own dependency-derived environment
# (verified empirically while writing this test). The floor is therefore the
# exact count, not a conservative guess under it: an environment that starts
# resolving fewer than all six is a real regression in a dependency this
# package actually declares, and test_all_launch_test_packages_resolve
# exists to fail on that rather than quietly accept less coverage.
MINIMUM_RESOLVABLE_PACKAGES = len(LAUNCH_TEST_PACKAGES)


def _repository_root():
    """Return the repository root, or None when only this package is present."""
    for candidate in PACKAGE_ROOT.parents:
        if (candidate / 'src' / 'ros2_medkit_cmake' / 'package.xml').is_file():
            return candidate
    return None


@pytest.fixture(scope='session')
def repository_root():
    """
    Return the repository root.

    Unconditional, not a skip: this test is registered in CMakeLists.txt only
    when the same signal medkit_add_launch_test's own guard uses to detect a
    system-package build (CMAKE_INSTALL_PREFIX=/opt/ros/$ROS_DISTRO plus
    CMAKE_INSTALL_LOCALSTATEDIR) says otherwise - the ROS build farm's
    binarydeb chroot builds this package alone, with no sibling packages
    beside it, which is exactly when no repository root exists above it. A
    missing repository root here means that registration guard itself broke,
    which must fail rather than skip.
    """
    root = _repository_root()
    assert root is not None, (
        'no repository root found above this package, but this test is only registered '
        '(see CMakeLists.txt) when one is expected to exist'
    )
    return root


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


@pytest.fixture(scope='session')
def resolvable_packages(repository_root, ros_distro, tmp_path_factory):
    """
    Baseline-configure every candidate exactly once; record which ones this environment reaches.

    This is the only baseline (ordinary workspace-style) configure each
    package gets. `test_guard_does_not_break_a_real_configure` below reuses
    the build directory recorded here instead of configuring the baseline
    again per parametrized case, which is what keeps the total configure
    count at twelve (one baseline plus one guarded configure per package)
    rather than eighteen.

    A package whose baseline configure fails has a dependency this
    environment cannot resolve. That is data, not a verdict on the guard by
    itself - `test_guard_does_not_break_a_real_configure` and
    `test_all_launch_test_packages_resolve` are what turn it into a pass or
    a fail; this fixture only collects it and prints it (pytest shows the
    print with -s or on failure) so a failure is traceable, not silent.

    Returns {name: (resolvable, build_dir_or_None, stderr_or_None)}.
    """
    baselines = {}
    for name, rel_path in sorted(LAUNCH_TEST_PACKAGES.items()):
        source_dir = repository_root / rel_path
        build_dir = tmp_path_factory.mktemp(f'baseline_{name}_build')
        install_dir = tmp_path_factory.mktemp(f'baseline_{name}_install')
        result = configure(source_dir, build_dir, install_dir)
        if result.returncode == 0:
            print(f'RESOLVABLE {name}: baseline configure succeeded')
            baselines[name] = (True, build_dir, None)
        else:
            print(
                f'UNRESOLVED {name}: baseline configure failed in this environment:\n'
                f'{result.stderr}'
            )
            baselines[name] = (False, None, result.stderr)
    return baselines


@pytest.mark.parametrize('name', sorted(LAUNCH_TEST_PACKAGES))
def test_guard_does_not_break_a_real_configure(
    name, repository_root, ros_distro, resolvable_packages, tmp_path
):
    """
    A system-package configure still succeeds, and suppresses the right tests.

    Asserts unconditionally, including that the baseline itself resolved:
    every package in LAUNCH_TEST_PACKAGES is a declared dependency of this
    package (see the module docstring), so a baseline that fails to
    configure is this test's problem to report, not a reason to decline it.
    """
    resolvable, baseline_build, baseline_stderr = resolvable_packages[name]
    assert resolvable, (
        f'{name}: baseline configure failed, so the system-package configure below cannot be '
        f'compared against anything - this package is a declared dependency of '
        f'ros2_medkit_integration_tests and must resolve:\n{baseline_stderr}'
    )
    source_dir = repository_root / LAUNCH_TEST_PACKAGES[name]
    # Reuses the session-scoped baseline from the fixture rather than
    # configuring it again here - see that fixture's docstring for why.
    baseline_tests = registered_test_names(baseline_build)

    guarded_build = tmp_path / 'guarded_build'
    guarded = configure(source_dir, guarded_build, f'/opt/ros/{ros_distro}', localstatedir='/var')
    assert_guard_suppressed_launch_tests(name, baseline_tests, guarded, guarded_build)


def test_all_launch_test_packages_resolve(resolvable_packages):
    """
    A floor so the suite above cannot pass by having configured nothing.

    Every package listed in LAUNCH_TEST_PACKAGES is a declared dependency of
    this package, so this requires every one of them to resolve, not merely
    most - a package failing to resolve is a real regression in a dependency
    this package actually declares. test_guard_does_not_break_a_real_configure
    already fails per-package on this; this is the aggregate view of the
    same fact, for a run that has configured nothing to be unable to hide
    behind individually plausible-looking per-package messages.
    """
    resolved = sorted(name for name, (ok, _, _) in resolvable_packages.items() if ok)
    unresolved = sorted(name for name, (ok, _, _) in resolvable_packages.items() if not ok)
    assert len(resolved) >= MINIMUM_RESOLVABLE_PACKAGES, (
        f'only {len(resolved)} of {len(LAUNCH_TEST_PACKAGES)} launch-test packages could be '
        f'configured in this environment (resolved={resolved}, unresolved={unresolved}), below '
        f'the floor of {MINIMUM_RESOLVABLE_PACKAGES} - this run proves too little to trust'
    )


if __name__ == '__main__':
    sys.exit(pytest.main([__file__, '-v', '-s']))
