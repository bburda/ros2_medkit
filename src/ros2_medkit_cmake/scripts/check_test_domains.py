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

"""Catch a test that never goes through the DDS domain wrapper.

Domains are allocated at run time now, by a wrapper that sits in the test's
command line. A test registered without it does not fail - it runs on the ROS 2
default domain 0, where it sees every node on the machine and every node on the
machine sees it. That is a silent failure, so it needs a gate.

This runs on the machine that TESTS, against the CTest properties CMake actually
generated, which is the only place the truth is visible: the wrapper is part of
the command, and no configure-time check can tell whether a later
``set_tests_properties`` dropped the count that goes with it.

The same "on the machine that tests" argument applies to the band itself, so
this also checks the band against that machine's live
``net.ipv4.ip_local_port_range``. The allocator refuses to hand out a domain
that the kernel may also hand to an unrelated process, but it refuses one test
at a time and in the middle of a run; this says it once, up front, in the words
of the sysctl that has to change. See ``verify_band`` in ``medkit_domain``.

A test passes when any of these holds:

* its command goes through one of the wrapper scripts and it carries
  ``MEDKIT_TEST_DOMAINS`` >= 1
* it carries the ``no_ros_domain`` label, the explicit statement written by
  ``medkit_test_needs_no_domain()`` that it creates no ROS entities at all
* it is a linter, which runs no ROS code and is registered by ament, not by us

Two modes, because one package cannot see the hole the other half covers:

``--build-dir``
    One package, registered automatically for every package that finds
    ros2_medkit_cmake (see its extras hook).

``--workspace-build-dir``
    Every package build directory in the workspace, including packages that
    never found ros2_medkit_cmake and therefore carry no per-package guard.
    Those are reported by name: a package with tests and no guard is how a
    package leaves the scheme, and it is the case a per-package gate can never
    see.
"""

import argparse
import json
import os
import subprocess
import sys

# Installed next to this script, so an import by name resolves under a plain
# install, a --symlink-install and a source tree alike. Kept as an import rather
# than a copy of the constants: the previous check carried its own copy of the
# port arithmetic and the two drifted apart.
import medkit_domain

#: Command basenames that acquire and hold a domain. Kept as names rather than
#: paths because the same scripts are referenced from the source tree under
#: --symlink-install and from the install tree otherwise.
WRAPPER_SCRIPTS = ('medkit_domain_runner.py', 'medkit_run_with_domain.py')

DOMAIN_COUNT_ENV = 'MEDKIT_TEST_DOMAINS'

WAIT_ENV = 'MEDKIT_TEST_DOMAIN_WAIT'

#: CTest's own default when a test carries no TIMEOUT property.
CTEST_DEFAULT_TIMEOUT = 1500.0

#: Written by medkit_test_needs_no_domain().
NO_DOMAIN_LABEL = 'no_ros_domain'

#: Tests we never expect to hold a domain. Linters are registered by
#: ament_lint_auto and run no ROS code.
EXEMPT_LABELS = ('linter',)

#: The gates themselves, which only read CTest properties.
SELF_TEST_NAME = 'test_dds_domain_allocation'
COVERAGE_TEST_NAME = 'test_dds_domain_coverage'


def load_tests(build_dir):
    """Return the CTest test list with its properties, straight from ctest."""
    completed = subprocess.run(
        ['ctest', '--show-only=json-v1'],
        cwd=build_dir,
        capture_output=True,
        text=True,
        check=False,
    )
    if completed.returncode != 0:
        raise SystemExit(
            f'ctest --show-only=json-v1 failed in {build_dir} '
            f'(exit {completed.returncode}):\n{completed.stderr.strip()}'
        )
    return json.loads(completed.stdout).get('tests', [])


def test_properties(test):
    """Return {property name: list of values} for one test entry.

    CTest reports TIMEOUT as a JSON number rather than a string, so scalars are
    wrapped rather than discarded. Dropping them silently turned every timeout
    into the default and made the wait-budget check pass on tests it should
    have caught.
    """
    result = {}
    for prop in test.get('properties', []):
        value = prop.get('value')
        if isinstance(value, list):
            pass
        elif value is None:
            value = []
        else:
            value = [value]
        result[prop['name']] = value
    return result


def env_value(env_entries, name):
    """Return the last value CTest would apply for *name*, or None."""
    for entry in reversed(env_entries):
        key, _, value = entry.partition('=')
        if key == name:
            return value
    return None


def domain_count(env_entries):
    """Return the MEDKIT_TEST_DOMAINS a test carries, or None.

    CTest applies ENVIRONMENT entries in order and the last one wins, so scan in
    reverse: the helpers APPEND, and a caller may append its own on top.
    """
    for entry in reversed(env_entries):
        name, _, value = entry.partition('=')
        if name == DOMAIN_COUNT_ENV:
            try:
                return int(value)
            except ValueError:
                return value
    return None


def goes_through_wrapper(command):
    return any(os.path.basename(part) in WRAPPER_SCRIPTS for part in command)


def audit_package(build_dir, package):
    """Apply the rule to one package. Returns (failures, wrapped, waived, extras)."""
    failures = []
    wrapped = 0
    waived = []
    extra_domains = []

    for test in load_tests(build_dir):
        name = test.get('name', '<unnamed>')
        if name in (SELF_TEST_NAME, COVERAGE_TEST_NAME):
            continue
        props = test_properties(test)
        labels = props.get('LABELS', [])
        count = domain_count(props.get('ENVIRONMENT', []))

        if goes_through_wrapper(test.get('command', [])):
            if count is None:
                failures.append(
                    f'{package}: {name} goes through the domain wrapper but carries no '
                    f'{DOMAIN_COUNT_ENV}. Its ENVIRONMENT was replaced after '
                    'registration, most likely by a set_tests_properties(... PROPERTIES '
                    'ENVIRONMENT ...) call; use set_property(TEST ... APPEND PROPERTY '
                    'ENVIRONMENT ...) instead.'
                )
            elif not isinstance(count, int) or count < 1:
                failures.append(
                    f'{package}: {name} goes through the domain wrapper with '
                    f'{DOMAIN_COUNT_ENV}={count!r}, which it cannot honour.'
                )
            else:
                wrapped += 1
                if count > 1:
                    extra_domains.append(f'{name} ({count})')
                # The wrapper waits for a free domain, then reports if it never
                # gets one. That report is only reachable if the wait ends
                # before CTest kills the test, so the budget has to fit inside
                # the test's own timeout. Left to a fixed default it did not,
                # on almost every test in the tree, and the failure the
                # allocator is careful to explain never printed.
                timeout = props.get('TIMEOUT')
                timeout = float(timeout[0]) if timeout else CTEST_DEFAULT_TIMEOUT
                wait = env_value(props.get('ENVIRONMENT', []), WAIT_ENV)
                if wait is None:
                    failures.append(
                        f'{package}: {name} goes through the domain wrapper but carries '
                        f'no {WAIT_ENV}, so it would wait the built-in default. Register '
                        'it with one of the medkit_add_* helpers, which derive the wait '
                        "budget from the test's own TIMEOUT."
                    )
                elif float(wait) >= timeout:
                    failures.append(
                        f'{package}: {name} waits up to {wait}s for a domain but CTest '
                        f'kills it at {timeout:.0f}s, so it would be killed mid-wait and '
                        'the reason it could not get a domain would never print. Lower '
                        f'{WAIT_ENV} for this test.'
                    )
            continue

        if NO_DOMAIN_LABEL in labels:
            waived.append(name)
            continue
        if any(label in EXEMPT_LABELS for label in labels):
            continue

        failures.append(
            f'{package}: {name} is registered without the domain wrapper, so it runs on '
            'the ROS 2 default domain 0 and shares it with every other process on the '
            'machine. Register it with medkit_add_gtest / medkit_add_gmock / '
            'medkit_add_launch_test / medkit_add_wrapped_test, or declare it ROS-free '
            'with medkit_test_needs_no_domain().'
        )

    return failures, wrapped, waived, extra_domains


def check_one_package(build_dir, package):
    failures, wrapped, waived, extra_domains = audit_package(build_dir, package)
    print(f'package              : {package}')
    print(f'through the wrapper  : {wrapped}')
    print(f'declared ROS-free    : {len(waived)}{" " + str(sorted(waived)) if waived else ""}')
    if extra_domains:
        print(f'more than one domain : {sorted(extra_domains)}')
    return failures


def source_directory_of(build_dir):
    """Return the source directory a package build directory was generated from.

    CMake writes it into the second line of every generated CTestTestfile.cmake.
    """
    try:
        with open(os.path.join(build_dir, 'CTestTestfile.cmake')) as handle:
            for line in handle:
                if line.startswith('# Source directory:'):
                    return line.split(':', 1)[1].strip()
                if not line.startswith('#'):
                    break
    except OSError:
        return None
    return None


def package_build_dirs(workspace_build_dir, repo_root=None):
    """Yield (name, path) for every package build directory that is ours.

    Filtered on the source directory rather than on the package name. An
    overlay workspace that source-builds one of our dependencies next to us
    produces build directories that look exactly like ours, and telling that
    maintainer to rewrite their package would be both wrong and rude.
    """
    try:
        entries = sorted(os.listdir(workspace_build_dir))
    except OSError as error:
        raise SystemExit(f'cannot read {workspace_build_dir}: {error}')
    root = os.path.realpath(repo_root) + os.sep if repo_root else None
    for name in entries:
        path = os.path.join(workspace_build_dir, name)
        if not os.path.isfile(os.path.join(path, 'CTestTestfile.cmake')):
            continue
        if root is not None:
            source = source_directory_of(path)
            if source is None or not os.path.realpath(source).startswith(root):
                continue
        yield name, path


def check_workspace(workspace_build_dir, repo_root=None):
    failures = []
    swept = 0
    ungated = []

    for package, path in package_build_dirs(workspace_build_dir, repo_root):
        swept += 1
        tests = load_tests(path)
        names = {test.get('name') for test in tests}
        failures.extend(audit_package(path, package)[0])

        # A package with tests of its own and no per-package guard never found
        # ros2_medkit_cmake, so nothing armed the gate for it. That is the hole
        # this sweep exists to close, and it cannot be seen from inside the
        # packages that do carry the guard.
        gateable = [
            test for test in tests
            if test.get('name') not in (SELF_TEST_NAME, COVERAGE_TEST_NAME)
            and not any(
                label in EXEMPT_LABELS
                for label in test_properties(test).get('LABELS', [])
            )
        ]
        if gateable and SELF_TEST_NAME not in names:
            ungated.append(package)
            failures.append(
                f'{package} has {len(gateable)} test(s) but no {SELF_TEST_NAME}, so no '
                'gate is watching it. Every package gets one from '
                'find_package(ros2_medkit_cmake); this one does not do that, and its '
                'tests are outside the domain scheme entirely.'
            )

    print(f'workspace build dir  : {workspace_build_dir}')
    print(f'packages from this repository: {swept}')
    print(f'packages without a gate: {sorted(ungated) if ungated else "none"}')
    if swept == 0:
        failures.append(
            f'no package build directory from this repository found under '
            f'{workspace_build_dir}. The sweep proved nothing, which is not the same '
            'as passing.'
        )
    return failures


def check_band_is_safe():
    """Check the band this machine would draw from against its own kernel.

    Reported here as well as refused in the allocator, because the allocator
    refuses per test, mid-run, in whichever test happened to start first. A
    machine whose ip_local_port_range was changed needs one message, before the
    run, naming the sysctl.
    """
    band = medkit_domain.default_band()
    try:
        low, high, source = medkit_domain.verify_band(band)
    except medkit_domain.DomainAllocationError as error:
        print(f'DDS domain band  : {medkit_domain.format_band(band)} (NOT usable here)')
        return [str(error)]
    print(f'ephemeral ports  : {low}-{high} ({source})')
    print(
        f'DDS domain band  : {medkit_domain.format_band(band)}, '
        f'{len(band)} domains, every slice outside that range'
    )
    return []


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--build-dir', help='package build directory')
    parser.add_argument('--package', help='package being checked')
    parser.add_argument(
        '--workspace-build-dir',
        help='sweep every package build directory below this one',
    )
    parser.add_argument(
        '--repo-root',
        help='only sweep packages whose source directory is under this path',
    )
    args = parser.parse_args()
    if not args.workspace_build_dir and not (args.build_dir and args.package):
        parser.error('pass either --workspace-build-dir, or --build-dir with --package')

    failures = check_band_is_safe()
    if args.workspace_build_dir:
        failures += check_workspace(args.workspace_build_dir, args.repo_root)
    else:
        failures += check_one_package(args.build_dir, args.package)

    if failures:
        print('')
        for failure in failures:
            print(f'FAIL: {failure}')
        return 1

    print(
        'OK: the band is safe on this machine, and every test either holds a domain of '
        'its own or says why it needs none'
    )
    return 0


if __name__ == '__main__':
    sys.exit(main())
