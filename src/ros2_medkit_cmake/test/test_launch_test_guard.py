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
The guard that keeps launch tests out of a build that cannot run them.

A launch test resolves the executables and the Python helpers of the package
under test through the ament index of an installed prefix. The ROS build farm's
binarydeb job runs ``make test`` before the install step, so the package is
absent from ``AMENT_PREFIX_PATH`` and every launch test fails on resolution
rather than on the thing it set out to check. Humble is the only distro that
still runs those tests, and because bloom runs the step as
``dh_auto_test || true`` the failures cannot stop the build and nobody has to
see them.

Guarding registration is not enough by itself: a caller that names its launch
test afterwards, with a ``set_tests_properties`` or a ``set_property(TEST ...)``,
is naming a test a guarded build never registered, and CMake fails the
configure outright rather than doing nothing. So ``medkit_add_launch_test`` owns
``LABELS`` and ``ENV`` itself now.

The regression test for "does this actually stay true across every
``CMakeLists.txt`` in the tree" does not live in this file. A first attempt at
it was a regex sweep over the tree's ``CMakeLists.txt`` text, and it was
patched three times running for the same reason each time: a string proxy for
"does this configure cleanly" always has one more shape it cannot see (a
second name on one call, a name on a continuation line, a paren balanced
across a comment, a mixed-case command spelling CMake itself does not care
about). ``ros2_medkit_integration_tests/test/test_launch_test_guard_system_build.py``
replaces it with a real ``cmake`` configure of every package that registers a
launch test, under system-package conditions, which is exactly the property
being promised and needs no proxy for it. It lives there rather than here
because this package cannot depend on the packages that consume it - that
would be circular - so it needs a package whose own dependency chain already
reaches them.

The guard itself fires on two independent signals, both read from CMake
variables rather than the environment - the install prefix matches
``/opt/ros/<anything>``, AND ``CMAKE_INSTALL_LOCALSTATEDIR`` equals ``/var`` -
because either alone has a false positive: the prefix alone would also catch
``colcon build --install-base /opt/ros/<distro>``, whose test step runs after
its own install step and whose launch tests work fine, and checking only
``DEFINED`` would also catch ``include(GNUInstallDirs)``, which defines it
itself as the relative ``var`` rather than the absolute ``/var`` debhelper
passes on the command line. An earlier version of the prefix check compared
against ``$ENV{ROS_DISTRO}`` instead of a wildcard, and never fired in the ROS
build farm's binarydeb chroot as a result: that environment has no
``ROS_DISTRO`` set at all, so the old guard always returned TRUE (register)
there - see ``test_launch_test_is_not_registered_with_ros_distro_absent``
below, which is the probe that would have caught it.

The guard covers launch tests only. gtests run fine in that chroot and are the
only place our packages meet a minimal dependency closure, which is how the
missing rosbag2 sqlite3 storage plugin was found.
``test_other_test_kinds_are_still_registered_in_a_system_build`` is what stops
the guard from growing into them.
"""

import os
import pathlib
import re
import shutil
import subprocess
import sys

import pytest

PACKAGE_ROOT = pathlib.Path(__file__).resolve().parent.parent
DOMAIN_MODULE = PACKAGE_ROOT / 'cmake' / 'ROS2MedkitTestDomain.cmake'

# Every probe that find_package(launch_testing_ament_cmake)s is declared with
# the C language, not NONE. On Humble, launch_testing_ament_cmake reaches the
# deprecated FindPythonLibs through python_cmake_module, and a project with no
# language enabled leaves CMAKE_LIBRARY_ARCHITECTURE and CMAKE_SIZEOF_VOID_P
# empty, so find_library never searches the multiarch directory and libpython
# is not found however the development headers are installed - see
# design/index.rst ("The one case that cannot live here...") and
# ros2_medkit_fault_reporter/CMakeLists.txt for the same reasoning applied to
# why that package's own launch test lives there instead of in this one. This
# package's real CMakeLists.txt is unaffected (declared NONE on purpose, and
# never find_package(launch_testing_ament_cmake)s), but a throwaway probe
# project that does both has the identical failure mode on Humble, and this
# suite is labelled "unit", which CI runs on every distro.
#
# The pytest probe does not find_package(launch_testing_ament_cmake), so it
# stays NONE.

# One macro call, given LABELS and ENV, so the workspace-build probe can pin
# the trap this migration removes: the domain the macro sets for itself has to
# survive alongside a caller's own properties.
LAUNCH_PROBE = """cmake_minimum_required(VERSION 3.14)
project(medkit_launch_guard_probe C)
find_package(ament_cmake REQUIRED)
find_package(launch_testing_ament_cmake REQUIRED)
include("{module}")
enable_testing()
medkit_add_launch_test(probe_launch_test probe_launch.test.py TIMEOUT 10
  LABELS "integration;probe"
  ENV "PROBE_FOO=bar" "PROBE_BAZ=qux")
"""

# Two calls in one project, so the "reported once" probe below can tell a flag
# that is never set from one that is set and then wrongly cleared between calls
# - a single-call probe cannot tell those apart.
DOUBLE_LAUNCH_PROBE = """cmake_minimum_required(VERSION 3.14)
project(medkit_double_launch_guard_probe C)
find_package(ament_cmake REQUIRED)
find_package(launch_testing_ament_cmake REQUIRED)
include("{module}")
enable_testing()
medkit_add_launch_test(probe_launch_test_one probe_launch_one.test.py TIMEOUT 10)
medkit_add_launch_test(probe_launch_test_two probe_launch_two.test.py TIMEOUT 10)
"""

PYTEST_PROBE = """cmake_minimum_required(VERSION 3.14)
project(medkit_pytest_guard_probe NONE)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_pytest REQUIRED)
include("{module}")
enable_testing()
medkit_add_pytest_test(probe_pytest_test probe_pytest TIMEOUT 10)
"""

# A CXX project, not C or NONE: ament_cmake_gtest's own find_package chain
# needs a C++ compiler to locate GTest with, and medkit_add_gtest's whole
# point is a compiled test binary. cmake_guard.py's
# assert_guard_suppressed_launch_tests backstops "the guard did not remove
# everything" with a bare `assert guarded_tests`, and its docstring names
# gtests explicitly - but nothing before this probe actually exercised a
# gtest through the guard: a guard that also swallowed gtests would still
# pass every real-package check, because ros2_medkit_log_bridge's own pytest
# test would still be there to satisfy the assertion.
GTEST_PROBE = """cmake_minimum_required(VERSION 3.14)
project(medkit_gtest_guard_probe CXX)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
include("{module}")
enable_testing()
medkit_add_gtest(probe_gtest_test probe_gtest.cpp)
"""

GTEST_PROBE_SOURCE = """#include <gtest/gtest.h>

TEST(ProbeGtestGuard, Passes)
{
  EXPECT_TRUE(true);
}
"""

# LABELS is a single word here on purpose: the point of this probe is ARGS,
# not the LABELS multi-value splitting LAUNCH_PROBE already covers. If ARGS
# leaked into LABELS the way it did before ARGS got its own keyword, LABELS
# would read "integration;ARGS;probe_arg:=probe_value" instead of plain
# "integration".
ARGS_PROBE = """cmake_minimum_required(VERSION 3.14)
project(medkit_launch_guard_args_probe C)
find_package(ament_cmake REQUIRED)
find_package(launch_testing_ament_cmake REQUIRED)
include("{module}")
enable_testing()
medkit_add_launch_test(probe_launch_test probe_launch.test.py TIMEOUT 10
  LABELS "integration"
  ARGS "probe_arg:=probe_value")
"""

# A caller that tries to set MEDKIT_TEST_DOMAINS through ENV instead of
# DOMAINS - this must fail the configure, not silently double the entry.
DOMAINS_COLLISION_PROBE = """cmake_minimum_required(VERSION 3.14)
project(medkit_launch_guard_domains_collision_probe C)
find_package(ament_cmake REQUIRED)
find_package(launch_testing_ament_cmake REQUIRED)
include("{module}")
enable_testing()
medkit_add_launch_test(probe_launch_test probe_launch.test.py TIMEOUT 10
  ENV "MEDKIT_TEST_DOMAINS=99")
"""


def configure_probe(
    tmp_path, template, install_prefix, extra_files, *, localstatedir=None, ros_distro='jazzy'
):
    """
    Configure a throwaway project and return the completed cmake process.

    ROS_DISTRO is set to *ros_distro* in the subprocess environment (jazzy by
    default, overridable) unless *ros_distro* is None, in which case it is
    deleted from the environment instead of set - the actual state of the ROS
    build farm's binarydeb chroot, which has no `ros_environment` package (the
    only in-tree source of that export) and no Docker `ENV` setting it either.

    The guard no longer reads ROS_DISTRO at all (see
    ROS2MedkitTestDomain.cmake), so this parameter no longer decides the
    guard's outcome the way it used to - but it did decide it, silently,
    before this file gained `ros_distro=None` as a real case: every probe here
    used to force ROS_DISTRO into the child environment unconditionally,
    which is exactly the one state that does not hold in the target job, and
    is how a guard that never fired there kept passing this whole file. The
    default stays jazzy rather than humble because that is the distro this
    suite otherwise runs under; a guard hard-coded to jazzy would still pass
    every case here if every case also used jazzy, which is why one case pins
    humble and another deletes ROS_DISTRO entirely.
    """
    source = tmp_path / 'probe'
    source.mkdir()
    (source / 'CMakeLists.txt').write_text(template.format(module=DOMAIN_MODULE.as_posix()))
    for name, content in extra_files.items():
        target = source / name
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_text(content)
    build = tmp_path / 'build'
    cmake_args = [
        shutil.which('cmake') or 'cmake',
        '-S', str(source),
        '-B', str(build),
        f'-DCMAKE_INSTALL_PREFIX={install_prefix}',
    ]
    if localstatedir is not None:
        cmake_args.append(f'-DCMAKE_INSTALL_LOCALSTATEDIR={localstatedir}')
    # Override, not replace: cmake still needs the inherited AMENT_PREFIX_PATH /
    # CMAKE_PREFIX_PATH to resolve find_package(ament_cmake) and
    # find_package(launch_testing_ament_cmake) at all. Only ROS_DISTRO itself is
    # touched, so every probe imitates a chosen distro - or its absence -
    # regardless of who runs it. Those two stay present even in the ROS build
    # farm's binarydeb chroot: the farm's test step runs
    # `if [ -f "/opt/ros/humble/setup.sh" ]; then . "/opt/ros/humble/setup.sh";
    # fi && dh_auto_test`, and sourcing setup.sh is what sets both - the same
    # reason ROS_DISTRO is the one thing missing there: that export comes from
    # the ros_environment package specifically, which the chroot does not
    # install, not from setup.sh itself. A probe never needs to fake either
    # path for that reason.
    env = dict(os.environ)
    if ros_distro is None:
        env.pop('ROS_DISTRO', None)
    else:
        env['ROS_DISTRO'] = ros_distro
    return subprocess.run(
        cmake_args,
        capture_output=True,
        text=True,
        timeout=300,
        check=False,
        env=env,
    ), build


def read_ctest_testfile(build_dir):
    """Return the raw CTestTestfile.cmake text, or '' when configure produced none."""
    testfile = build_dir / 'CTestTestfile.cmake'
    if not testfile.is_file():
        return ''
    return testfile.read_text()


def registered_tests(build_dir):
    """Return the names ctest would run for a configured probe."""
    return [
        line.split('(', 1)[1].split(' ', 1)[0].strip('"')
        for line in read_ctest_testfile(build_dir).splitlines()
        if line.lstrip().startswith('add_test(')
    ]


def test_launch_test_is_registered_in_a_workspace_build(tmp_path):
    """
    A colcon build installs into the workspace, so the launch test is registered.

    No CMAKE_INSTALL_LOCALSTATEDIR here, matching an ordinary colcon build,
    which never passes it. The caller's LABELS and ENV must both reach the
    generated CTestTestfile.cmake alongside MEDKIT_TEST_DOMAINS - the domain
    the macro set for itself - which is the trap this migration removes: a
    plain set_tests_properties/set_property after the call used to be able to
    drop it.
    """
    result, build = configure_probe(
        tmp_path,
        LAUNCH_PROBE,
        tmp_path / 'install',
        {'probe_launch.test.py': '# empty launch test file\n'},
    )
    assert result.returncode == 0, result.stderr
    assert 'probe_launch_test' in registered_tests(build), result.stdout
    testfile = read_ctest_testfile(build)
    assert 'MEDKIT_TEST_DOMAINS=1' in testfile, testfile
    assert 'PROBE_FOO=bar' in testfile, testfile
    assert 'PROBE_BAZ=qux' in testfile, testfile
    assert 'integration;probe' in testfile, testfile


def test_launch_tests_are_not_registered_in_a_system_package_build(tmp_path):
    """
    A system package build registers no launch test and says why exactly once.

    This is the case the guard exists for. Without it the build farm publishes
    a log in which every launch test of the package failed, none of them for a
    reason that says anything about the package. Two calls in one project, so
    that "exactly once" is a real assertion about the report flag rather than
    a coincidence of there being only one call to not print twice.
    """
    result, build = configure_probe(
        tmp_path,
        DOUBLE_LAUNCH_PROBE,
        '/opt/ros/jazzy',
        {
            'probe_launch_one.test.py': '# empty launch test file\n',
            'probe_launch_two.test.py': '# empty launch test file\n',
        },
        localstatedir='/var',
    )
    assert result.returncode == 0, result.stderr
    names = registered_tests(build)
    assert 'probe_launch_test_one' not in names, result.stdout
    assert 'probe_launch_test_two' not in names, result.stdout
    assert result.stdout.count('launch tests are not registered') == 1, result.stdout


def test_launch_test_is_registered_with_the_distribution_prefix_but_no_localstatedir(tmp_path):
    """
    `colcon build --install-base /opt/ros/jazzy` still gets its launch tests.

    Its test step runs after its own install step, unlike a system package
    build's, so the install prefix by itself is not a safe signal - only
    CMAKE_INSTALL_LOCALSTATEDIR, which debhelper's cmake buildsystem always
    passes and colcon never does, tells the two apart.
    """
    result, build = configure_probe(
        tmp_path,
        LAUNCH_PROBE,
        '/opt/ros/jazzy',
        {'probe_launch.test.py': '# empty launch test file\n'},
    )
    assert result.returncode == 0, result.stderr
    assert 'probe_launch_test' in registered_tests(build), result.stdout


def test_launch_test_is_registered_when_localstatedir_is_relative(tmp_path):
    """
    A relative CMAKE_INSTALL_LOCALSTATEDIR does not arm the guard.

    `include(GNUInstallDirs)` defines CMAKE_INSTALL_LOCALSTATEDIR itself, to
    the relative "var" rather than the absolute "/var" debhelper passes on
    the command line. Checking only DEFINED cannot tell those apart; this
    pins that the guard checks the value. No in-tree package includes
    GNUInstallDirs today, so this is a regression guard for a real default,
    not a reproduction of an observed failure.
    """
    result, build = configure_probe(
        tmp_path,
        LAUNCH_PROBE,
        '/opt/ros/jazzy',
        {'probe_launch.test.py': '# empty launch test file\n'},
        localstatedir='var',
    )
    assert result.returncode == 0, result.stderr
    assert 'probe_launch_test' in registered_tests(build), result.stdout


def test_launch_test_is_not_registered_with_a_trailing_slash_on_localstatedir(tmp_path):
    """
    A trailing slash on CMAKE_INSTALL_LOCALSTATEDIR does not disarm the guard.

    This file used to probe a trailing slash on the install prefix instead,
    and that probe could not fail however the guard's own code was written:
    CMake itself strips a trailing slash from CMAKE_INSTALL_PREFIX before
    project code ever sees it - confirmed on CMake 3.22, 3.28 and 4.4 - so
    the guard's prefix match needs no normalisation of its own, and by the
    time the guard runs the question was already settled upstream.
    CMAKE_INSTALL_LOCALSTATEDIR is an ordinary cache variable nothing else
    normalises, so the guard's own `string(REGEX REPLACE "/+$" ...)` on it is
    the only thing standing between a trailing slash here and a failed
    STREQUAL "/var" comparison - this is what actually needs a test, and this
    pins that it works.
    """
    result, build = configure_probe(
        tmp_path,
        LAUNCH_PROBE,
        '/opt/ros/jazzy',
        {'probe_launch.test.py': '# empty launch test file\n'},
        localstatedir='/var/',
    )
    assert result.returncode == 0, result.stderr
    assert 'probe_launch_test' not in registered_tests(build), result.stdout


def test_other_test_kinds_are_still_registered_in_a_system_build(tmp_path):
    """
    The guard covers launch tests only, so a pytest test still registers.

    Under the full system-package condition (prefix AND
    CMAKE_INSTALL_LOCALSTATEDIR both set): gtests and pytest tests run in the
    binarydeb chroot, and they are the only coverage our packages get against
    a minimal dependency closure. A guard that swallowed them would take that
    away.
    """
    result, build = configure_probe(
        tmp_path,
        PYTEST_PROBE,
        '/opt/ros/jazzy',
        {'probe_pytest/test_probe.py': 'def test_probe():\n    assert True\n'},
        localstatedir='/var',
    )
    assert result.returncode == 0, result.stderr
    assert 'probe_pytest_test' in registered_tests(build), result.stdout


def test_gtest_is_still_registered_in_a_system_build(tmp_path):
    """
    The guard covers launch tests only, so a gtest still registers too.

    The pytest half of this claim is
    `test_other_test_kinds_are_still_registered_in_a_system_build` above;
    this is the gtest half, which `cmake_guard.py`'s
    `assert_guard_suppressed_launch_tests` docstring names but which no probe
    here actually drove through the guard before this test: its own
    backstop, a bare `assert guarded_tests`, would still pass for a guard
    that also swallowed gtests, as long as some pytest test in the same
    package survived to keep the set non-empty. This closes that gap
    directly rather than relying on a real package always happening to carry
    both kinds.
    """
    result, build = configure_probe(
        tmp_path,
        GTEST_PROBE,
        '/opt/ros/jazzy',
        {'probe_gtest.cpp': GTEST_PROBE_SOURCE},
        localstatedir='/var',
    )
    assert result.returncode == 0, result.stderr
    assert 'probe_gtest_test' in registered_tests(build), result.stdout


def test_launch_test_is_registered_with_a_non_distribution_prefix_and_localstatedir_set(tmp_path):
    """
    CMAKE_INSTALL_LOCALSTATEDIR alone does not suppress; the prefix still has to match.

    Fills the hole in the truth table every other probe leaves open: every
    case that expects suppression sets both signals, and every case that
    expects registration sets neither. A guard that suppressed whenever
    CMAKE_INSTALL_LOCALSTATEDIR is defined, ignoring the prefix, would still
    pass the whole rest of this file. This is the prefix-does-not-match,
    localstatedir-set corner that catches it.
    """
    result, build = configure_probe(
        tmp_path,
        LAUNCH_PROBE,
        tmp_path / 'install',
        {'probe_launch.test.py': '# empty launch test file\n'},
        localstatedir='/var',
    )
    assert result.returncode == 0, result.stderr
    assert 'probe_launch_test' in registered_tests(build), result.stdout


def test_launch_test_is_not_registered_on_humble(tmp_path):
    """
    The guard's prefix match is a wildcard, not hard-coded to one distro.

    Humble is the only distro whose binarydeb job still runs package tests -
    the one this whole task exists for - so a guard that happened to work only
    for jazzy (every other case in this file) would not fire on the build that
    matters and would still pass the suite. ros_distro='humble' here only
    controls what the probe's subprocess environment looks like, matching the
    real chroot's distro; it plays no part in the guard's own decision - see
    test_launch_test_is_not_registered_with_ros_distro_absent below for the
    case that proves the guard does not need it at all.
    """
    result, build = configure_probe(
        tmp_path,
        LAUNCH_PROBE,
        '/opt/ros/humble',
        {'probe_launch.test.py': '# empty launch test file\n'},
        localstatedir='/var',
        ros_distro='humble',
    )
    assert result.returncode == 0, result.stderr
    assert 'probe_launch_test' not in registered_tests(build), result.stdout


def test_launch_test_is_not_registered_with_ros_distro_absent(tmp_path):
    """
    The guard suppresses launch tests even when ROS_DISTRO is not in the environment at all.

    This is the actual state of the ROS build farm's binarydeb chroot, not a
    hypothetical: no in-tree package.xml declares ros_environment (the only
    source of the ROS_DISTRO export outside a Docker image's own ENV), and
    ros-humble-ros-environment does not appear in the farm's chroot install
    log, so ROS_DISTRO is genuinely unset there. An earlier version of this
    guard read ENV{ROS_DISTRO} and returned TRUE (register) the instant it
    was undefined - which is exactly this case - so the guard never fired in
    the one job it exists for, and every launch test kept failing on
    resolution exactly as before the guard was written. Every other probe in
    this file sets ROS_DISTRO in the child environment (see
    configure_probe's docstring for why that was not incidental); this one
    deletes it instead, which is what makes this the probe that would have
    caught the regression before it shipped.
    """
    result, build = configure_probe(
        tmp_path,
        LAUNCH_PROBE,
        '/opt/ros/humble',
        {'probe_launch.test.py': '# empty launch test file\n'},
        localstatedir='/var',
        ros_distro=None,
    )
    assert result.returncode == 0, result.stderr
    assert 'probe_launch_test' not in registered_tests(build), (
        f'the guard did not suppress the launch test with ROS_DISTRO absent from the '
        f'environment - this is the exact condition of the ROS build farm binarydeb '
        f'chroot the guard exists for:\n{result.stdout}'
    )


def test_args_reaches_add_launch_test_and_is_not_absorbed_by_labels(tmp_path):
    """
    ARGS still reaches add_launch_test, and does not leak into LABELS.

    Before ARGS had its own keyword, LABELS being multi-value (to survive its
    own semicolon-split values) meant an ARGS that cmake_parse_arguments did
    not recognise as a keyword would be swallowed as more LABELS values
    instead of forwarding to add_launch_test or erroring. No in-tree caller
    passes ARGS today, so this was latent, not live - this probe is the only
    coverage for it.
    """
    result, build = configure_probe(
        tmp_path,
        ARGS_PROBE,
        tmp_path / 'install',
        {'probe_launch.test.py': '# empty launch test file\n'},
    )
    assert result.returncode == 0, result.stderr
    assert 'probe_launch_test' in registered_tests(build), result.stdout
    testfile = read_ctest_testfile(build)
    assert 'probe_arg:=probe_value' in testfile, testfile
    labels_match = re.search(r'LABELS "([^"]*)"', testfile)
    assert labels_match, testfile
    assert labels_match.group(1) == 'integration', (
        f'ARGS leaked into LABELS: {labels_match.group(1)!r}\n{testfile}'
    )


def test_env_cannot_set_medkit_test_domains_directly(tmp_path):
    """
    An ENV entry that sets MEDKIT_TEST_DOMAINS fails the configure.

    Without this, a caller writing ENV "MEDKIT_TEST_DOMAINS=99" would append a
    second, later ENVIRONMENT entry that wins over the one the macro sets from
    DOMAINS at run time - the comment above the ENV foreach promises the
    domain can never be overwritten, and this is what makes that true instead
    of merely documented.
    """
    result, build = configure_probe(
        tmp_path,
        DOMAINS_COLLISION_PROBE,
        tmp_path / 'install',
        {'probe_launch.test.py': '# empty launch test file\n'},
    )
    assert result.returncode != 0, (
        'configuring succeeded with ENV setting MEDKIT_TEST_DOMAINS directly, so a '
        f'caller could silently override the domain the macro allocated:\n{result.stdout}'
    )
    assert 'MEDKIT_TEST_DOMAINS' in result.stderr, result.stderr
    assert 'DOMAINS' in result.stderr, result.stderr
    assert not (build / 'CTestTestfile.cmake').is_file(), (
        'a failed configure left a CTestTestfile.cmake behind'
    )


if __name__ == '__main__':
    sys.exit(pytest.main([__file__, '-v']))
