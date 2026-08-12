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

# `add_launch_test()` (from `launch_testing_ament_cmake`) always generates a
# COMMAND that runs `python3 -m launch_testing.launch_test <file>` - that is
# how launch_testing_ament_cmake executes a launch test, not something any
# macro in this tree spells or could get out of sync with, and every
# generated `add_test(...)` call lands on a single physical line in
# `CTestTestfile.cmake` (checked against every launch-test package in this
# tree - the longest line is ~93 launch tests deep in
# ros2_medkit_integration_tests and every one of them is still one line), so
# a per-line substring check is exact rather than a heuristic. `LABELS`
# cannot be used for this instead: `add_launch_test()` only defaults it to
# `"launch_test"`, and every real launch test in this tree overrides it
# (`"integration;feature"`, `"integration;probe"`, ...), so a label sweep
# would undercount to whatever callers happened to leave unlabelled.
LAUNCH_TEST_COMMAND_MARKER = 'launch_testing.launch_test'


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


def launch_test_names(build_dir):
    """
    Return the set of test names in *build_dir* whose command is a launch test.

    Identified by `LAUNCH_TEST_COMMAND_MARKER` in the test's own `add_test(
    ...)` line - see that constant's comment for why this, and not a label
    or a name pattern, is the reliable signal. gtest and pytest tests never
    contain it: `ament_add_gtest` invokes the compiled test binary directly
    and `ament_add_pytest_test` invokes `pytest` on the test file, neither
    goes through the `launch_testing.launch_test` module. A pytest test
    whose own file path happens to contain the bare substring "launch_test"
    (e.g. this package's own `test_launch_test_guard_system_build`) does not
    match: the marker is the full dotted module name, not the bare word.
    """
    testfile = build_dir / 'CTestTestfile.cmake'
    if not testfile.is_file():
        return set()
    names = set()
    for line in testfile.read_text().splitlines():
        stripped = line.lstrip()
        if stripped.startswith('add_test(') and LAUNCH_TEST_COMMAND_MARKER in stripped:
            names.add(stripped.split('(', 1)[1].split(' ', 1)[0].strip('"'))
    return names


def assert_guard_suppressed_launch_tests(
    name, baseline_tests, guarded, guarded_build_dir, baseline_build_dir, *, also_suppressed=()
):
    """
    Assert a system-package configure suppressed exactly the launch tests.

    *baseline_tests* is the set of test names an ordinary workspace-style
    configure of the same package registered, from *baseline_build_dir*.
    *guarded* is the completed `cmake` process from configuring the same
    source directory under system-package conditions, and
    *guarded_build_dir* is the build directory that configure was pointed
    at. *also_suppressed* names tests, besides the launch tests identified
    from the baseline, that are known and documented to also not register
    under system-package conditions for a reason unrelated to being a
    launch test - see the call site for what that reason is; there is
    exactly one such test in this tree today, and it is not this function's
    place to special-case it silently.

    This is the CRITICAL regression the guard exists to prevent: a
    `set_tests_properties` / `set_property(TEST ...)` call naming a launch
    test the guard did not register makes CMake fail the configure outright.
    Comparing the two real `CTestTestfile.cmake` outputs - not a name
    pattern - is what this checks: the configure did not fail, the guard's
    own "launch tests are not registered" message printed, and the guarded
    run's test set is exactly the baseline's test set minus the baseline's
    own launch tests (`launch_test_names(baseline_build_dir)`) minus
    *also_suppressed* - nothing more suppressed, nothing less, and nothing
    new. A guard that also swallowed a package's gtests, or that only
    suppressed some of its launch tests, fails this even if a package's
    pytest coverage alone would have kept the surviving set non-empty - the
    earlier, weaker version of this function (non-empty-suppressed,
    non-empty-survivors, survivors-subset-of-baseline) could not tell that
    case apart from a correct guard.
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

    expected_launch_tests = launch_test_names(baseline_build_dir)
    assert expected_launch_tests, (
        f'{name}: the baseline configure registered no launch test at all (identified by the '
        f'"{LAUNCH_TEST_COMMAND_MARKER}" marker in its own add_test(...) command), so this '
        f'comparison cannot tell a working guard from a broken one:\n'
        f'baseline={sorted(baseline_tests)}'
    )

    guarded_tests = registered_test_names(guarded_build_dir)
    expected_survivors = baseline_tests - expected_launch_tests - set(also_suppressed)
    missing = sorted(expected_survivors - guarded_tests)
    unexpected = sorted(guarded_tests - expected_survivors)
    assert guarded_tests == expected_survivors, (
        f'{name}: the guarded configure did not register exactly the non-launch, '
        f'non-also_suppressed tests. missing (should have survived but did not)={missing}, '
        f'unexpected (should have been suppressed, or never existed in the baseline, but is '
        f'registered)={unexpected}. Expected exactly {sorted(expected_launch_tests)} '
        f'(plus {sorted(also_suppressed)} named explicitly) to be suppressed and nothing else.'
    )
