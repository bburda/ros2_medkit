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

The guard itself fires on two independent signals - the install prefix is the
ROS distribution root, AND ``CMAKE_INSTALL_LOCALSTATEDIR`` equals ``/var`` -
because either alone has a false positive: the prefix alone would also catch
``colcon build --install-base /opt/ros/<distro>``, whose test step runs after
its own install step and whose launch tests work fine, and checking only
``DEFINED`` would also catch ``include(GNUInstallDirs)``, which defines it
itself as the relative ``var`` rather than the absolute ``/var`` debhelper
passes on the command line.

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

    ROS_DISTRO is always set in the subprocess environment (jazzy by default,
    overridable). A probe that skipped itself when ROS_DISTRO happened to be
    unset in whoever runs this suite would let a deleted guard pass here right
    along with a working one - that used to be true of this file and is
    exactly what let the CRITICAL finding through unnoticed. The default is
    jazzy rather than humble because that is the distro this suite otherwise
    runs under; a guard hard-coded to jazzy would still pass every case here
    if every case also used jazzy, which is why one case pins humble instead.
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
    # pinned, so every probe imitates a chosen distro regardless of who runs it.
    env = dict(os.environ)
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


def test_launch_test_is_not_registered_with_a_trailing_slash_on_the_prefix(tmp_path):
    """
    A trailing slash on the install prefix does not disarm the guard.

    The comparison is STREQUAL, which is exact, so the guard normalises the
    prefix before comparing it rather than relying on the caller to spell it
    exactly as bloom does.
    """
    result, build = configure_probe(
        tmp_path,
        LAUNCH_PROBE,
        '/opt/ros/jazzy/',
        {'probe_launch.test.py': '# empty launch test file\n'},
        localstatedir='/var',
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
    The guard reads ROS_DISTRO rather than being hard-coded to one distro.

    Humble is the only distro whose binarydeb job still runs package tests -
    the one this whole task exists for - so a guard that happened to work only
    for jazzy (every other case in this file) would not fire on the build that
    matters and would still pass the suite.
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
