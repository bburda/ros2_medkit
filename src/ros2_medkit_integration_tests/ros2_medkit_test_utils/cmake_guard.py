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
Real-`cmake`-configure checks for `medkit_add_launch_test`'s system-package guard.

Shared by every package that proves, for itself, that a system-package
configure (`CMAKE_INSTALL_PREFIX=/opt/ros/<distro>`,
`CMAKE_INSTALL_LOCALSTATEDIR=/var` - what bloom's `debian/rules` passes)
suppresses exactly its own launch tests without failing the configure. A
regex sweep over `CMakeLists.txt` text was tried first and evaded three
times running (a second name on one call, a name on a continuation line, a
paren balanced across a comment); this asks CMake itself instead, which has
no such blind spot.

Two call sites use this: `ros2_medkit_integration_tests`, which reaches the
six sibling packages it declares as `test_depend`, and
`ros2_medkit_graph_watchdog`, which configures only itself - it already
`test_depend`s on `ros2_medkit_integration_tests` for its own e2e harness
(that is how it reaches this module), so the reverse edge would be a real
build-order cycle and it cannot be covered from the other side.
"""

import shutil
import subprocess


def configure(source_dir, build_dir, install_prefix, *, localstatedir=None):
    """Run a real `cmake` configure and return the completed process."""
    args = [
        shutil.which('cmake') or 'cmake',
        '-S', str(source_dir),
        '-B', str(build_dir),
        f'-DCMAKE_INSTALL_PREFIX={install_prefix}',
    ]
    if localstatedir is not None:
        args.append(f'-DCMAKE_INSTALL_LOCALSTATEDIR={localstatedir}')
    return subprocess.run(args, capture_output=True, text=True, timeout=600, check=False)


def registered_test_names(build_dir):
    """Return the set of test names ctest would run for a configured build directory."""
    testfile = build_dir / 'CTestTestfile.cmake'
    if not testfile.is_file():
        return set()
    return {
        line.split('(', 1)[1].split(' ', 1)[0].strip('"')
        for line in testfile.read_text().splitlines()
        if line.lstrip().startswith('add_test(')
    }


def assert_guard_suppressed_launch_tests(name, baseline_tests, guarded, guarded_build_dir):
    """
    Assert a system-package configure suppressed something without failing.

    *baseline_tests* is the set of test names an ordinary workspace-style
    configure of the same package registered. *guarded* is the completed
    `cmake` process from configuring the same source directory under
    system-package conditions, and *guarded_build_dir* is the build directory
    that configure was pointed at.

    This is the CRITICAL regression the guard exists to prevent: a
    `set_tests_properties` / `set_property(TEST ...)` call naming a launch
    test the guard did not register makes CMake fail the configure outright.
    Comparing the two real `CTestTestfile.cmake` outputs - not a name pattern
    - is what this checks: the configure did not fail, the guard's own
    "launch tests are not registered" message printed, something the
    baseline registered is gone from the guarded run (the guard fired on at
    least one test), what remains is a subset of the baseline (nothing new
    appeared), and something remains at all (it did not remove everything).

    It does NOT prove the removed set is exactly the launch tests and
    nothing else, or that the surviving set specifically includes gtests and
    pytest tests as opposed to some other kind - a guard that also swallowed
    a package's gtests would still pass every one of these assertions for a
    package whose pytest coverage alone kept the surviving set non-empty.
    That narrower claim, that a gtest and a pytest test each individually
    survive the guard, is proven in isolation by
    `test_gtest_is_still_registered_in_a_system_build` and
    `test_other_test_kinds_are_still_registered_in_a_system_build` in
    `ros2_medkit_cmake/test/test_launch_test_guard.py`, not by this function.
    """
    assert guarded.returncode == 0, (
        f'{name}: configure failed under system-package conditions even though the same '
        f'package configures fine otherwise. This is the CRITICAL regression the guard exists '
        f'to prevent - a property call naming a launch test the guard did not register:\n'
        f'{guarded.stderr}'
    )
    assert 'launch tests are not registered' in guarded.stdout, (
        f'{name}: configured cleanly but the guard never printed its explanation - either it '
        f'did not fire, or the message changed:\n{guarded.stdout}'
    )

    guarded_tests = registered_test_names(guarded_build_dir)
    suppressed = baseline_tests - guarded_tests
    assert suppressed, (
        f'{name}: nothing was suppressed under system-package conditions - the guard did not '
        f'remove any test that the baseline configure registered:\n'
        f'baseline={sorted(baseline_tests)}'
    )
    assert guarded_tests, (
        f'{name}: every test the baseline registered was suppressed - the guard is removing '
        f'more than launch tests:\nbaseline={sorted(baseline_tests)}'
    )
    assert guarded_tests <= baseline_tests, (
        f'{name}: the guarded configure registered a test the baseline never did, which this '
        f'comparison cannot explain:\nonly guarded={sorted(guarded_tests - baseline_tests)}'
    )
